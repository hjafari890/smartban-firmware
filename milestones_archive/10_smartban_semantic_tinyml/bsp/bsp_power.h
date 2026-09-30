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

/**
 * @brief Power operation modes for SmartBAN node.
 */
typedef enum {
    BSP_POWER_STATE_ACTIVE = 0,
    BSP_POWER_STATE_SLEEP = 1,        /* Standby mode: rails alive, rapid 14us wake */
    BSP_POWER_STATE_DEEP_SLEEP = 2    /* Deep Sleep: sub-5uA rail shutdown */
} bsp_power_state_t;

/**
 * @brief Dynamic run-time energy and power metrics (Method 1).
 */
typedef struct {
    bsp_power_state_t state;
    float             current_ma;          /* Instantaneous estimated current (mA) */
    float             power_mw;            /* Instantaneous power (mW, Vdd=3.3V) */
    float             accum_energy_mj;     /* Accumulated energy (mJ) */
    float             battery_remain_pct;  /* Remaining battery capacity (%) */
    float             battery_hours_left;  /* Estimated battery hours remaining (500 mAh cell) */
} bsp_power_metrics_t;

/**
 * @brief Retrieve latest dynamic power metrics.
 */
bsp_power_metrics_t bsp_power_get_metrics(void);

/**
 * @brief Update run-time energy accumulation and metrics.
 * @param dt_sec Delta time in seconds since last update.
 */
void bsp_power_update_metrics(float dt_sec);

/**
 * @brief Enter Tier 1 Sleep (Standby).
 *        Keeps power rails alive for instant 14us wake. Blank display and put sensors in standby.
 * @param timer_sec Auto-wake duration in seconds (0 = indefinite until button/GUI wake).
 */
void bsp_power_enter_sleep(uint32_t timer_sec);

/**
 * @brief Enter Tier 2 Deep Sleep (Sub-5uA Shutdown).
 *        Shuts off 1.8V LDO, I2C level shifter, displays, and sensors.
 *        Registers AON GPIO interrupt on SW3 (DIO 29) and UART RX (DIO 2).
 * @param timer_sec Auto-wake duration in seconds (0 = indefinite until button/GUI wake).
 */
void bsp_power_enter_deepsleep(uint32_t timer_sec);

/**
 * @brief Restore active operational state after waking from sleep or deep sleep.
 */
void bsp_power_restore_active(void);

/**
 * @brief Query current power operating mode.
 */
bsp_power_state_t bsp_power_get_state(void);

/**
 * @brief Reset accumulated energy and restore battery model to 100%.
 */
void bsp_power_reset_energy(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_H_ */
