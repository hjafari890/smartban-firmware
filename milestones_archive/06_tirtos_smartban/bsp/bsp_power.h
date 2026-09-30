/*
 * ============================================================================
 * bsp_power.h
 * Board Support Package - SmartBAN Power Rail Management
 * ============================================================================
 */

#ifndef BSP_POWER_H_
#define BSP_POWER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and sequence all power rails for SmartBAN Shield Rev 3.5.
 *        - DIO 21 (1.8V LDO EN) -> HIGH
 *        - DIO 30 (I2C Level Shifter EN) -> HIGH
 *        - DIO 18 (IMU Q1 Switch Gate) -> LOW (active operation)
 * @return true on success, false on error.
 */
bool bsp_power_init(void);

/**
 * @brief Control the 1.8V LDO regulator rail (AP2112K-1.8 via DIO 21).
 * @param enable true = rail ON, false = rail OFF.
 */
void bsp_power_set_1v8(bool enable);

/**
 * @brief Control the PCA9306 bidirectional I2C level translator (DIO 30).
 * @param enable true = translator ON, false = translator OFF.
 */
void bsp_power_set_i2c_shifter(bool enable);

/**
 * @brief Control the IMU discharge switch (Q1 FET via DIO 18).
 *        WARNING: Must remain LOW (false) during active sampling.
 * @param discharge_on true = FET ON (pulls IMU rail to GND), false = FET OFF (normal operation).
 */
void bsp_power_set_imu_discharge(bool discharge_on);

/**
 * @brief Check current state of the 1.8V power rail.
 * @return true if enabled, false otherwise.
 */
bool bsp_power_is_1v8_enabled(void);

/**
 * @brief Check current state of the I2C level shifter.
 * @return true if enabled, false otherwise.
 */
bool bsp_power_is_i2c_shifter_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_H_ */
