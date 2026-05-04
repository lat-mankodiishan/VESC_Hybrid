/*
 * Selects hybrid_pcu.c as the active custom app.
 * Included from conf_general.h.
 */
#ifndef HYBRID_PCU_CONF_H_
#define HYBRID_PCU_CONF_H_

#define APP_CUSTOM_TO_USE          "hybrid/hybrid_pcu.c"

/* Bake hybrid defaults into the firmware so the unit boots correctly
 * without depending on VESC Tool to write app config. After a mass-erase
 * (which clears the EEPROM-emu sectors) the unit comes up with our
 * custom app active and the right CAN bitrate. */
#define APPCONF_APP_TO_USE         APP_CUSTOM
#define APPCONF_CAN_BAUD_RATE      CAN_BAUD_250K

#endif /* HYBRID_PCU_CONF_H_ */
