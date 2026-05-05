/*
 * Hybrid PCU custom app — VESC acts as active rectifier on the engine bus.
 *   RX 0x101 (SendCurrDem)         -> mc_interface_set_current()
 *   RX 0x102 (SendOmegaDem)        -> mc_interface_set_pid_speed()  (eRPM -> mech RPM)
 *   RX 0x103 (SendDutyDem)         -> mc_interface_set_duty()
 *   TX 0x201 (GetRectStateConcise) @ 10 Hz to PCU
 *
 * The PCU emits exactly one of {0x101, 0x102, 0x103} at any time, picked by
 * its rect_ctrl_mode. Last-write-wins on this side: each ID maps to a
 * different mc_interface call, the most recent one wins. The single 50 ms
 * RX watchdog covers all three — on stale, set_current(0) brings the motor
 * back to a safe stop regardless of which mode was last active.
 *
 * Wire format mirrors PCU's vesc_proto.c — see hybrid_pcu_proto.h.
 *
 * Sign convention (current): I_rect_cmd_cA negative = regen (PMSG -> DC bus).
 * PCU emits in VESC motor-frame, so we pass through without negating.
 */
#include "conf_general.h"

#include "ch.h"
#include "hal.h"

#include "app.h"
#include "mc_interface.h"
#include "comm_can.h"
#include "timeout.h"
#include "datatypes.h"
#include "utils_math.h"
#include "terminal.h"
#include "commands.h"

#include "hybrid_pcu_proto.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

#define HYBRID_TX_PERIOD_MS   100      /* 10 Hz state telemetry */
#define HYBRID_RX_WD_MS       50       /* spec: 50 ms stale-cmd watchdog */

/* Hardware-of-last-resort clamps. Mirror powertrain_state.h. */
#define HYBRID_I_MAX_A          60.0f      /*  60.00 A         */
#define HYBRID_OMEGA_MAX_ERPM  100000.0f   /* 100k electrical RPM */
#define HYBRID_DUTY_MAX         0.95f      /*  95.00 % duty    */

typedef enum {
	HYBRID_LAST_NONE    = 0,
	HYBRID_LAST_CURRENT = 1,
	HYBRID_LAST_OMEGA   = 2,
	HYBRID_LAST_DUTY    = 3,
} hybrid_last_ctrl_t;

static THD_FUNCTION(hybrid_thread, arg);
static THD_WORKING_AREA(hybrid_thread_wa, 1024);

static volatile bool      stop_now    = true;
static volatile bool      is_running  = false;

static volatile systime_t last_rx_time = 0;
static volatile bool      have_rx      = false;
static volatile bool      rx_stale     = false;

/* Diagnostic snapshots — the most recent valid frame of each type. */
static volatile hybrid_last_ctrl_t last_ctrl   = HYBRID_LAST_NONE;
static volatile int16_t            last_I_cA      = 0;
static volatile int32_t            last_omega_erpm = 0;
static volatile int16_t            last_duty_x10000 = 0;
static volatile uint8_t            last_mode    = 0;
static volatile uint8_t            last_rx_seq  = 0;

static volatile uint32_t           rx_count_curr  = 0;
static volatile uint32_t           rx_count_omega = 0;
static volatile uint32_t           rx_count_duty  = 0;
static volatile uint32_t           rx_bad_count   = 0;

static bool    hybrid_can_sid_rx(uint32_t id, uint8_t *data, uint8_t len);
static uint8_t map_fault_bits(mc_fault_code f, bool stale);
static void    terminal_hybrid_status(int argc, const char **argv);

static inline void note_rx(uint8_t mode, uint8_t seq, hybrid_last_ctrl_t which) {
	last_rx_time = chVTGetSystemTimeX();
	last_mode    = mode;
	last_rx_seq  = seq;
	last_ctrl    = which;
	have_rx      = true;
	rx_stale     = false;
	timeout_reset();
}

void app_custom_start(void) {
	stop_now     = false;
	have_rx      = false;
	rx_stale     = false;
	last_rx_time = 0;

	comm_can_set_sid_rx_callback(hybrid_can_sid_rx);

	terminal_register_command_callback(
			"hybrid_status",
			"Print hybrid PCU link diagnostics",
			0,
			terminal_hybrid_status);

	chThdCreateStatic(hybrid_thread_wa, sizeof(hybrid_thread_wa),
			NORMALPRIO, hybrid_thread, NULL);
}

void app_custom_stop(void) {
	comm_can_set_sid_rx_callback(0);
	terminal_unregister_callback(terminal_hybrid_status);
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

/* eRPM -> mechanical RPM via VESC's configured motor pole count.
 * mc_interface_set_pid_speed expects mechanical RPM. */
static float erpm_to_mech_rpm(int32_t erpm) {
	const volatile mc_configuration *mcc = mc_interface_get_configuration();
	float poles = (mcc != 0) ? (float)mcc->si_motor_poles : 2.0f;
	if (poles < 2.0f) poles = 2.0f;          /* 2 poles = 1 pole-pair minimum */
	const float pole_pairs = poles * 0.5f;
	return (float)erpm / pole_pairs;
}

/* CAN RX callback — runs in comm_can dispatch thread context.
 * Return true so VESC core/LispBM skip further processing of these IDs. */
static bool hybrid_can_sid_rx(uint32_t id, uint8_t *data, uint8_t len) {
	switch (id) {
	case HYBRID_ID_SEND_CURR_DEM: {
		hybrid_curr_dem_t cmd;
		if (hybrid_proto_decode_curr_dem(data, len, &cmd) != HYBRID_DECODE_OK) {
			rx_bad_count++;
			return true;
		}
		float i_a = (float)cmd.I_rect_cmd_cA * 0.01f;
		utils_truncate_number(&i_a, -HYBRID_I_MAX_A, HYBRID_I_MAX_A);
		mc_interface_set_current(i_a);
		last_I_cA = cmd.I_rect_cmd_cA;
		rx_count_curr++;
		note_rx((uint8_t)cmd.mode, cmd.seq, HYBRID_LAST_CURRENT);
		return true;
	}
	case HYBRID_ID_SEND_OMEGA_DEM: {
		hybrid_omega_dem_t cmd;
		if (hybrid_proto_decode_omega_dem(data, len, &cmd) != HYBRID_DECODE_OK) {
			rx_bad_count++;
			return true;
		}
		float erpm_f = (float)cmd.omega_e_cmd_erpm;
		utils_truncate_number(&erpm_f, -HYBRID_OMEGA_MAX_ERPM, HYBRID_OMEGA_MAX_ERPM);
		mc_interface_set_pid_speed(erpm_to_mech_rpm((int32_t)erpm_f));
		last_omega_erpm = cmd.omega_e_cmd_erpm;
		rx_count_omega++;
		note_rx((uint8_t)cmd.mode, cmd.seq, HYBRID_LAST_OMEGA);
		return true;
	}
	case HYBRID_ID_SEND_DUTY_DEM: {
		hybrid_duty_dem_t cmd;
		if (hybrid_proto_decode_duty_dem(data, len, &cmd) != HYBRID_DECODE_OK) {
			rx_bad_count++;
			return true;
		}
		float duty = (float)cmd.duty_cmd_x10000 * 0.0001f;
		utils_truncate_number(&duty, -HYBRID_DUTY_MAX, HYBRID_DUTY_MAX);
		mc_interface_set_duty(duty);
		last_duty_x10000 = cmd.duty_cmd_x10000;
		rx_count_duty++;
		note_rx((uint8_t)cmd.mode, cmd.seq, HYBRID_LAST_DUTY);
		return true;
	}
	default:
		return false;
	}
}

/* "hybrid_status" terminal command — type it in VESC Tool's Terminal tab. */
static void terminal_hybrid_status(int argc, const char **argv) {
	(void)argc; (void)argv;
	uint32_t age_ms = have_rx ? ST2MS(chVTTimeElapsedSinceX(last_rx_time)) : 0;
	const char *ctrl_name = "NONE";
	switch (last_ctrl) {
	case HYBRID_LAST_CURRENT: ctrl_name = "CURRENT"; break;
	case HYBRID_LAST_OMEGA:   ctrl_name = "OMEGA";   break;
	case HYBRID_LAST_DUTY:    ctrl_name = "DUTY";    break;
	default: break;
	}
	commands_printf("hybrid PCU link:");
	commands_printf("  RX OK curr/omega/duty: %lu / %lu / %lu",
			(unsigned long)rx_count_curr,
			(unsigned long)rx_count_omega,
			(unsigned long)rx_count_duty);
	commands_printf("  RX frames bad:  %lu", (unsigned long)rx_bad_count);
	commands_printf("  active control: %s", ctrl_name);
	commands_printf("  last I_cmd:     %d cA   (%.2f A)",
			(int)last_I_cA, (double)last_I_cA * 0.01);
	commands_printf("  last omega:     %ld eRPM",
			(long)last_omega_erpm);
	commands_printf("  last duty:      %d /10000 (%.2f %%)",
			(int)last_duty_x10000, (double)last_duty_x10000 * 0.01);
	commands_printf("  last mode:      %u", (unsigned)last_mode);
	commands_printf("  last seq:       %u", (unsigned)last_rx_seq);
	commands_printf("  age:            %lu ms %s",
			(unsigned long)age_ms, rx_stale ? "(STALE)" : "");
	commands_printf("  fault code:     %d",
			(int)mc_interface_get_fault());
}

static THD_FUNCTION(hybrid_thread, arg) {
	(void)arg;
	chRegSetThreadName("Hybrid PCU");
	is_running = true;

	uint8_t   tx_seq = 0;
	systime_t next   = chVTGetSystemTimeX();

	for (;;) {
		if (stop_now) {
			is_running = false;
			return;
		}

		/* Stale-RX watchdog — covers all three setpoint frame types.
		 * set_current(0) is safe regardless of which mode was active. */
		if (have_rx &&
		    ST2MS(chVTTimeElapsedSinceX(last_rx_time)) > HYBRID_RX_WD_MS) {
			mc_interface_set_current(0.0f);
			rx_stale = true;
		}

		/* Build 0x201 payload */
		const float v_dc  = mc_interface_get_input_voltage_filtered();
		const float i_dc  = mc_interface_get_tot_current_in_filtered();
		const float rpm   = mc_interface_get_rpm();
		const float t_fet = mc_interface_temp_fet_filtered();

		float v_cv  = v_dc * 100.0f;
		float i_cA  = i_dc * 100.0f;
		float r_abs = fabsf(rpm);
		float t_c   = t_fet;
		utils_truncate_number(&v_cv,  0.0f,       65535.0f);
		utils_truncate_number(&i_cA,  -32768.0f,  32767.0f);
		utils_truncate_number(&r_abs, 0.0f,       65535.0f);
		utils_truncate_number(&t_c,   -128.0f,    127.0f);

		hybrid_rect_state_t s;
		s.V_dc_cV     = (uint16_t)v_cv;
		s.I_dc_cA     = (int16_t)i_cA;
		s.gen_rpm     = (uint16_t)r_abs;
		s.igbt_temp_C = (int8_t)t_c;
		s.fault_bits  = map_fault_bits(mc_interface_get_fault(), rx_stale);
		s.seq         = tx_seq++ & 0x0F;

		uint8_t pl[8];
		hybrid_proto_encode_rect_state_concise(&s, pl);
		comm_can_transmit_sid(HYBRID_ID_GET_RECT_STATE_CONCISE, pl, 8);

		next += MS2ST(HYBRID_TX_PERIOD_MS);
		chThdSleepUntil(next);
	}
}

/* Map mc_fault_code -> 4-bit fault_bits per link spec.
 *   bit0=OV, bit1=OC, bit2=OT, bit3=DRV/STALE/other */
static uint8_t map_fault_bits(mc_fault_code f, bool stale) {
	uint8_t b = 0;
	switch (f) {
	case FAULT_CODE_NONE:                                  break;
	case FAULT_CODE_OVER_VOLTAGE:        b |= 0x1;         break;
	case FAULT_CODE_ABS_OVER_CURRENT:    b |= 0x2;         break;
	case FAULT_CODE_OVER_TEMP_FET:       b |= 0x4;         break;
	case FAULT_CODE_OVER_TEMP_MOTOR:     b |= 0x4;         break;
	case FAULT_CODE_DRV:                 b |= 0x8;         break;
	case FAULT_CODE_UNDER_VOLTAGE:       b |= 0x8;         break;
	default:                             b |= 0x8;         break;
	}
	if (stale) b |= 0x8;
	return b;
}
