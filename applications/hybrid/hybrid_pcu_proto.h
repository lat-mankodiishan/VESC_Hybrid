/*
 * Wire-format mirror of PCU's vesc_proto.h. Byte layout, CRC, and enum
 * MUST match exactly — this is the cross-board contract. Do not edit
 * one side without the other.
 */
#ifndef HYBRID_PCU_PROTO_H_
#define HYBRID_PCU_PROTO_H_

#include <stdint.h>
#include <stdbool.h>

#define HYBRID_ID_SEND_CURR_DEM           0x101u
#define HYBRID_ID_GET_RECT_STATE_CONCISE  0x201u

typedef enum {
	HYBRID_MODE_IDLE    = 0,
	HYBRID_MODE_TAKEOFF = 1,
	HYBRID_MODE_CLIMB   = 2,
	HYBRID_MODE_CRUISE  = 3,
	HYBRID_MODE_LAND    = 4,
	HYBRID_MODE_FAULT   = 5,
} hybrid_mode_t;

typedef struct {
	int16_t       I_rect_cmd_cA;   /* 0.01 A/LSB, signed; regen = negative */
	hybrid_mode_t mode;
	uint8_t       seq;
} hybrid_curr_dem_t;

typedef struct {
	uint16_t V_dc_cV;       /* 0.01 V/LSB */
	int16_t  I_dc_cA;       /* 0.01 A/LSB, signed (matches PCU sign) */
	uint16_t gen_rpm;       /* 1 rpm/LSB */
	int8_t   igbt_temp_C;   /* 1 degC/LSB */
	uint8_t  fault_bits;    /* low 4 bits used; bit0=OV, bit1=OC, bit2=OT, bit3=DRV/STALE */
	uint8_t  seq;           /* low 4 bits used, wraps every 16 frames */
} hybrid_rect_state_t;

typedef enum {
	HYBRID_DECODE_OK = 0,
	HYBRID_DECODE_BAD_LEN,
	HYBRID_DECODE_BAD_CRC,
} hybrid_decode_t;

uint8_t          hybrid_crc8(const uint8_t *buf, uint8_t len);

hybrid_decode_t  hybrid_proto_decode_curr_dem(const uint8_t *data, uint8_t len,
                                              hybrid_curr_dem_t *out);

void             hybrid_proto_encode_rect_state_concise(const hybrid_rect_state_t *in,
                                                        uint8_t out8[8]);

#endif /* HYBRID_PCU_PROTO_H_ */
