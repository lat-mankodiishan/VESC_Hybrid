# VESC_Hybrid

Custom VESC firmware for the LAT series-hybrid EGU. The VESC acts as the active
rectifier on the engine bus: takes current / speed / duty setpoints from the PCU
over CAN, broadcasts rectifier state back, with a 50 ms watchdog that pulls
current to zero on link loss.

Custom code lives entirely in **[applications/hybrid/](applications/hybrid/)**
plus two one-line wires into the rest of the tree.

## File-by-file

### [hybrid_pcu_proto.h](applications/hybrid/hybrid_pcu_proto.h) / [.c](applications/hybrid/hybrid_pcu_proto.c) — wire format

Pure framing/CRC code, no HAL or VESC dependencies. Mirrors the PCU's
`vesc_proto.c` byte-for-byte (cross-board contract).

- `hybrid_crc8()` — CRC-8/SMBUS (poly 0x07, init 0x00, no reflection).
- `hybrid_proto_decode_curr_dem()` → unpacks **0x101** `(I_cA int16, mode, seq)`.
- `hybrid_proto_decode_omega_dem()` → unpacks **0x102** `(omega_e_erpm int32, mode, seq)`.
- `hybrid_proto_decode_duty_dem()` → unpacks **0x103** `(duty_x10000 int16, mode, seq)`.
- `hybrid_proto_encode_rect_state_concise()` → packs **0x201**
  `(V_dc, I_dc, gen_rpm, igbt_temp, fault_bits, seq)`.

All multi-byte fields little-endian. CRC over bytes 0..6, byte 7 = CRC.

### [hybrid_pcu.c](applications/hybrid/hybrid_pcu.c) — the actual app

Three jobs:

**1. RX dispatcher.** `hybrid_can_sid_rx()` is registered via
`comm_can_set_sid_rx_callback`. Switches on the CAN ID and routes:
- 0x101 → `mc_interface_set_current(I_A)` after ±60 A clamp.
- 0x102 → `mc_interface_set_pid_speed(eRPM / pole_pairs)` — pole pairs read live
  from `mc_configuration.si_motor_poles / 2`.
- 0x103 → `mc_interface_set_duty(duty)` after ±0.95 clamp.

Each path resets the watchdog (`timeout_reset()` + `last_rx_time`) and bumps a
per-mode counter. Returns `true` so VESC core / LispBM ignore the frame.

**2. 10 Hz state TX.** `hybrid_thread` (ChibiOS thread, NORMALPRIO) runs the
loop:
- Watchdog: if no valid RX in 50 ms → `mc_interface_set_current(0)` and set
  `rx_stale=true`.
- Read `V_dc`, `I_dc`, `RPM`, `T_fet` from `mc_interface`.
- Pack into `hybrid_rect_state_t`, encode, transmit `0x201` via
  `comm_can_transmit_sid`.
- `fault_bits[3:0]` mapped from `mc_interface_get_fault()`: bit0=OV, bit1=OC,
  bit2=OT (FET∨motor), bit3=DRV/UV/STALE.

**3. Terminal diagnostic.** `hybrid_status` command (registered via
`terminal_register_command_callback`) prints to VESC Tool's Terminal:
- Per-mode RX OK counters (curr / omega / duty)
- Bad-frame count (CRC / length failures)
- Active control type (whichever ID was last received)
- Last decoded I / omega / duty values
- Last `mode`, `seq`, frame age, current `mc_fault_code`, STALE flag

### [hybrid_pcu_conf.h](applications/hybrid/hybrid_pcu_conf.h) — build-time config

- `APP_CUSTOM_TO_USE "hybrid/hybrid_pcu.c"` → tells `app_custom.c` to `#include`
  our app.
- `APPCONF_APP_TO_USE = APP_CUSTOM` → bakes "use custom app" into firmware
  defaults so a mass-erased unit boots ready (no VESC Tool config write needed).
- `APPCONF_CAN_BAUD_RATE = CAN_BAUD_500K` → 500k engine bus to coexist with
  Loweheiser ECU.

## Wires into the rest of the tree (only two lines)

- [conf_general.h](conf_general.h) — `#include "hybrid/hybrid_pcu_conf.h"`.
  Activates the custom app slot.
- [applications/applications.mk](applications/applications.mk) — adds
  `hybrid/hybrid_pcu_proto.c` to `APPSRC` and `applications/hybrid` to `APPINC`.
  (`hybrid_pcu.c` is *not* in APPSRC — it's pulled in via
  `#include APP_CUSTOM_TO_USE` from `app_custom.c`. Adding it would
  double-define.)

## In one sentence

VESC firmware pretends to be a current / speed / duty-controlled active
rectifier with a 10 Hz heartbeat back to the PCU and a 50 ms safety watchdog —
everything else is stock VESC.

## Build target

Target hardware: Trampa VESC 6 MK5 (`HW_NAME = "60_MK5"`).

```
make fw_60_mk5
```

Produces `build/60_mk5/60_mk5.hex`. Flash via SWD with STM32CubeProgrammer or
the bundled `make fw_60_mk5_flash` (needs OpenOCD on PATH).

Upstream Vedder firmware is preserved as the `upstream` git remote — pull
updates via `git fetch upstream && git merge upstream/master`.
