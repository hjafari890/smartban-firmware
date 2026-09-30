/*
 * ============================================================================
 * hal_optical.h
 * Hardware Abstraction Layer - TI OPT4041 ALS & Vishay VCNL4040 Proximity/ALS
 * ============================================================================
 */

#ifndef HAL_OPTICAL_H_
#define HAL_OPTICAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* OPT4041 I2C Definitions */
#define OPT4041_I2C_ADDR            0x44
#define OPT4041_REG_RESULT_MSB      0x00
#define OPT4041_REG_RESULT_LSB      0x01
#define OPT4041_REG_CONFIG          0x0A
#define OPT4041_REG_DEVICE_ID       0x11
#define OPT4041_CONFIG_CONTINUOUS   0x3238 /* 100ms continuous auto-range */
#define OPT4041_LUX_PER_COUNT       0.000585f

/* VCNL4040 I2C Definitions */
#define VCNL4040_I2C_ADDR           0x60
#define VCNL4040_REG_ALS_CONF       0x00
#define VCNL4040_REG_PS_CONF1_2     0x03
#define VCNL4040_REG_PS_CONF3_MS    0x04
#define VCNL4040_REG_PS_DATA        0x08
#define VCNL4040_REG_ALS_DATA       0x09
#define VCNL4040_REG_WHITE_DATA     0x0A
#define VCNL4040_REG_ID             0x0C

#define VCNL4040_VAL_ID             0x0186
#define VCNL4040_SKIN_CONTACT_THRESH 6000  /* Skin contact confirmed when PS >= 6000 */

/**
 * @brief Initialize OPT4041 Ambient Light Sensor and VCNL4040 Proximity Sensor.
 * @return true if at least one sensor initializes successfully.
 */
bool hal_optical_init(void);

/**
 * @brief Read ambient light illuminance from OPT4041 in lux.
 * @param lux Output lux value.
 * @return true on success.
 */
bool hal_optical_read_lux(float *lux);

/**
 * @brief Pure math function: convert OPT4041 raw register pair to lux.
 * @param reg0 16-bit word from reg 0x00 (MSB).
 * @param reg1 8-bit byte from reg 0x01 (LSB).
 * @return Calculated lux.
 */
float hal_optical_calc_lux(uint16_t reg0, uint8_t reg1);

/**
 * @brief Read raw proximity and ambient counts from VCNL4040.
 * @param prox Output proximity raw count (0..65535).
 * @param als Output ALS raw count.
 * @param skin_contact Output boolean, true if prox >= 6000.
 * @return true on success.
 */
bool hal_optical_read_prox(uint16_t *prox, uint16_t *als, bool *skin_contact);

/**
 * @brief High-level combined optical acquisition for SmartBAN telemetry.
 * @param lux Output lux value from OPT4041 (or ALS fallback).
 * @param prox Output proximity count from VCNL4040.
 * @param skin_contact Output skin contact detection flag (prox >= 6000).
 * @return true on success.
 */
bool hal_optical_read(uint32_t *lux, uint16_t *prox, bool *skin_contact);

/**
 * @brief Query online status of optical sensors.
 */
bool hal_optical_opt4041_is_online(void);
bool hal_optical_vcnl4040_is_online(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_OPTICAL_H_ */
