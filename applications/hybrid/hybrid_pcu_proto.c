/*
 * Wire-format mirror of PCU's vesc_proto.c. Keep in lock-step.
 */
#include "hybrid_pcu_proto.h"
#include <string.h>

/* CRC-8/SMBUS — poly 0x07, init 0x00, no reflection, no XOR-out.
 * Test vector: hybrid_crc8("123456789", 9) == 0xF4. */
uint8_t hybrid_crc8(const uint8_t *buf, uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; ++i) {
		crc ^= buf[i];
		for (uint8_t b = 0; b < 8; ++b) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
			                   : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

static inline void put_u16_le(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline uint16_t get_u16_le(const uint8_t *p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

hybrid_decode_t hybrid_proto_decode_curr_dem(const uint8_t *data, uint8_t len,
                                             hybrid_curr_dem_t *out) {
	if (len != 8)                            return HYBRID_DECODE_BAD_LEN;
	if (hybrid_crc8(data, 7) != data[7])     return HYBRID_DECODE_BAD_CRC;

	out->I_rect_cmd_cA = (int16_t)get_u16_le(&data[0]);
	out->mode          = (hybrid_mode_t)data[2];
	out->seq           = data[3];
	return HYBRID_DECODE_OK;
}

void hybrid_proto_encode_rect_state_concise(const hybrid_rect_state_t *in,
                                            uint8_t out8[8]) {
	memset(out8, 0, 8);
	put_u16_le(&out8[0], in->V_dc_cV);
	put_u16_le(&out8[2], (uint16_t)in->I_dc_cA);
	put_u16_le(&out8[4], in->gen_rpm);
	out8[6] = (uint8_t)in->igbt_temp_C;
	out8[7] = (uint8_t)(((in->fault_bits & 0x0F) << 4) | (in->seq & 0x0F));
}
