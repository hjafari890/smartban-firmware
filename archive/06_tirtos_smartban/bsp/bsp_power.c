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
