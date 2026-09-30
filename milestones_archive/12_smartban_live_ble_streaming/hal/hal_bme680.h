/*
 * ============================================================================
 * hal_bme680.h
 * Hardware Abstraction Layer - Bosch BME680 / BME690 Environmental Sensor
 * (Temperature, Barometric Pressure, Relative Humidity, Gas IAQ)
 * ============================================================================
 */

#ifndef HAL_BME680_H_
#define HAL_BME680_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BME680_I2C_ADDR         0x76

typedef struct {
    float temperature_c;    /* Ambient Temperature in degrees Celsius */
    float pressure_hpa;     /* Barometric Pressure in hPa */
    float humidity_pct;     /* Relative Humidity in % RH */
    float gas_res_kohm;     /* Gas Resistance in kOhms */
    float iaq;              /* Indoor Air Quality index (0 - 500) */
    float co2_equivalent;   /* Estimated CO2 equivalent (ppm) */
    float bvoc_equivalent;  /* Estimated bVOC equivalent (ppm) */
    bool  valid;
} hal_bme680_data_t;

/**
 * @brief Initialize BME680/BME690 sensor, verify communication, and load factory calibration coeffs.
 * @return true on success, false otherwise.
 */
bool hal_bme680_init(void);

/**
 * @brief Trigger a measurement cycle and read compensated temperature, pressure, humidity, and gas resistance.
 * @param data Output data structure.
 * @return true on success, false otherwise.
 */
bool hal_bme680_read(hal_bme680_data_t *data);

/**
 * @brief Query if BME680 is detected and online.
 */
bool hal_bme680_is_online(void);

/**
 * @brief Query if sensor variant is BME690.
 */
bool hal_bme680_is_bme690(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_BME680_H_ */
