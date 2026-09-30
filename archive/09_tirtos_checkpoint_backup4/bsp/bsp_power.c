/*
 * ============================================================================
 * bsp_power.c
 * Board Support Package - SmartBAN Power Rail Management Implementation
 * ============================================================================
 */

#include "bsp_power.h"
#include "bsp_pins.h"
#include <unistd.h>

static bool s_1v8_enabled = false;
static bool s_i2c_shifter_enabled = false;
static bool s_imu_discharge_active = false;

bool bsp_power_init(void)
{
    /* 1. Ensure IMU discharge switch FET Q1 gate is LOW (prevent supply ground shunt) */
    GPIO_write(CONFIG_GPIO_IMU_SW, 0);
    s_imu_discharge_active = false;

    /* 2. Enable 1.8V ultra-low-dropout regulator (AP2112K-1.8) for ADS1292R and sensors */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    s_1v8_enabled = true;

    /* 3. Enable PCA9306 bidirectional I2C level translator */
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    s_i2c_shifter_enabled = true;

    /* 4. Settle time for voltage domains (50 ms) */
    usleep(50000);

    return true;
}

void bsp_power_set_1v8(bool enable)
{
    GPIO_write(CONFIG_GPIO_1V8_EN, enable ? 1 : 0);
    s_1v8_enabled = enable;
}

void bsp_power_set_i2c_shifter(bool enable)
{
    GPIO_write(CONFIG_GPIO_I2C_EN, enable ? 1 : 0);
    s_i2c_shifter_enabled = enable;
}

void bsp_power_set_imu_discharge(bool discharge_on)
{
    GPIO_write(CONFIG_GPIO_IMU_SW, discharge_on ? 1 : 0);
    s_imu_discharge_active = discharge_on;
}

bool bsp_power_is_1v8_enabled(void)
{
    return s_1v8_enabled;
}

bool bsp_power_is_i2c_shifter_enabled(void)
{
    return s_i2c_shifter_enabled;
}

/* ============================================================================
 * Dynamic Run-Time Power Profiler & Method 1 Energy Estimation
 * Based on CC2652R1 LaunchPad & SmartBAN Shield Electrical Specifications:
 * - Active: CPU (3.4 mA) + 7-Seg LEDs (24 mA) + BME680 (2.2 mA) + AFEs (2.2 mA) = 31.8 mA
 * - Sleep (Standby): Rails ON, display off, sensors standby = 0.020 mA (20 uA)
 * - Deep Sleep: Rails OFF, LDO off, sub-5uA shutdown = 0.004 mA (4 uA)
 * ============================================================================ */
static bsp_power_state_t s_power_state = BSP_POWER_STATE_ACTIVE;
static bsp_power_metrics_t s_metrics = {
    .state = BSP_POWER_STATE_ACTIVE,
    .current_ma = 31.8f,
    .power_mw = 104.94f,     /* 31.8 mA * 3.3V */
    .accum_energy_mj = 0.0f,
    .battery_remain_pct = 100.0f,
    .battery_hours_left = 15.7f /* 500 mAh / 31.8 mA */
};

bsp_power_metrics_t bsp_power_get_metrics(void)
{
    return s_metrics;
}

bsp_power_state_t bsp_power_get_state(void)
{
    return s_power_state;
}

void bsp_power_update_metrics(float dt_sec)
{
    if (dt_sec <= 0.0f) return;

    if (s_power_state == BSP_POWER_STATE_ACTIVE) {
        s_metrics.current_ma = 31.8f;
    } else if (s_power_state == BSP_POWER_STATE_SLEEP) {
        s_metrics.current_ma = 0.020f;
    } else {
        s_metrics.current_ma = 0.004f;
    }

    s_metrics.state = s_power_state;
    s_metrics.power_mw = s_metrics.current_ma * 3.3f;
    s_metrics.accum_energy_mj += s_metrics.power_mw * dt_sec;

    /* Total battery capacity: 500 mAh @ 3.7V nominal = 1850 mWh = 6,660,000 mJ */
    const float total_capacity_mj = 6660000.0f;
    float used_pct = (s_metrics.accum_energy_mj / total_capacity_mj) * 100.0f;
    s_metrics.battery_remain_pct = (used_pct < 100.0f) ? (100.0f - used_pct) : 0.0f;

    if (s_metrics.current_ma > 0.0001f) {
        float rem_mah = 500.0f * (s_metrics.battery_remain_pct / 100.0f);
        s_metrics.battery_hours_left = rem_mah / s_metrics.current_ma;
    } else {
        s_metrics.battery_hours_left = 99999.0f;
    }
}

void bsp_power_enter_sleep(uint32_t timer_sec)
{
    (void)timer_sec;
    s_power_state = BSP_POWER_STATE_SLEEP;
    s_metrics.state = BSP_POWER_STATE_SLEEP;
    s_metrics.current_ma = 0.020f;
    s_metrics.power_mw = 0.066f;
}

void bsp_power_enter_deepsleep(uint32_t timer_sec)
{
    (void)timer_sec;
    s_power_state = BSP_POWER_STATE_DEEP_SLEEP;
    s_metrics.state = BSP_POWER_STATE_DEEP_SLEEP;
    s_metrics.current_ma = 0.004f;
    s_metrics.power_mw = 0.0132f;

    /* Shut down high-current AFEs, LEDs and sensor peripherals for sub-5uA quiescent draw
     * Keep 1V8 and I2C level shifter active so PCAL6408A can wake the MCU on SW3 press */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    GPIO_write(CONFIG_GPIO_LED_0, 0);
    GPIO_write(CONFIG_GPIO_LED_1, 0);
}

void bsp_power_restore_active(void)
{
    /* Restore peripheral rails */
    bsp_power_set_1v8(true);
    bsp_power_set_i2c_shifter(true);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);

    /* Settle rails */
    usleep(30000);

    s_power_state = BSP_POWER_STATE_ACTIVE;
    s_metrics.state = BSP_POWER_STATE_ACTIVE;
    s_metrics.current_ma = 31.8f;
    s_metrics.power_mw = 104.94f;
}

void bsp_power_reset_energy(void)
{
    s_metrics.accum_energy_mj = 0.0f;
    s_metrics.battery_remain_pct = 100.0f;
    s_metrics.battery_hours_left = 500.0f / (s_metrics.current_ma > 0.0f ? s_metrics.current_ma : 31.8f);
}

