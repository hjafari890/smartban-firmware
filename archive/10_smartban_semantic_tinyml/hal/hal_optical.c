/*
 * ============================================================================
 * hal_optical.c
 * Hardware Abstraction Layer - TI OPT4041 ALS & Vishay VCNL4040 Implementation
 * ============================================================================
 */

#include "hal_optical.h"
#include "../bsp/bsp_pins.h"
#include "../bsp/bsp_power.h"
#include "../bsp/bsp_i2c.h"
#include <unistd.h>

static bool s_opt4041_online = false;
static bool s_vcnl4040_online = false;

static float    s_last_lux = 0.0f;
static uint16_t s_last_prox = 0;
static bool     s_last_contact = false;

bool hal_optical_init(void)
{
    bsp_power_set_i2c_shifter(true);
    bsp_i2c_init();

    /* 1. Initialize TI OPT4041 ALS */
    uint16_t dev_id = 0;
    if (bsp_i2c_read_reg16(OPT4041_I2C_ADDR, OPT4041_REG_DEVICE_ID, &dev_id, true)) {
        /* Configure OPT4041: Continuous 100ms conversion auto-range */
        bsp_i2c_write_reg16(OPT4041_I2C_ADDR, OPT4041_REG_CONFIG, OPT4041_CONFIG_CONTINUOUS, true);
        s_opt4041_online = true;
    } else {
        s_opt4041_online = false;
    }

    /* 2. Initialize Vishay VCNL4040 Proximity / ALS */
    uint16_t vcnl_id = 0;
    if (bsp_i2c_read_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_ID, &vcnl_id, false)) {
        if (vcnl_id == VCNL4040_VAL_ID) {
            /* Enable ALS: 80ms integration */
            bsp_i2c_write_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_ALS_CONF, 0x0000, false);
            /* Enable PS: 1/80 duty cycle, 8T integration, 16-bit output */
            bsp_i2c_write_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_PS_CONF1_2, 0x080E, false);
            /* PS Conf 3 / MS: 200mA LED drive, sunlight cancellation */
            bsp_i2c_write_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_PS_CONF3_MS, 0x0700, false);
            s_vcnl4040_online = true;
        } else {
            s_vcnl4040_online = false;
        }
    } else {
        s_vcnl4040_online = false;
    }

    return (s_opt4041_online || s_vcnl4040_online);
}

float hal_optical_calc_lux(uint16_t reg0, uint8_t reg1)
{
    uint32_t mantissa = ((uint32_t)(reg0 & 0x0FFF) << 8) | (uint32_t)reg1;
    uint8_t exponent = (uint8_t)((reg0 >> 12) & 0x0F);
    double adc_codes = (double)mantissa * (double)(1ULL << exponent);
    return (float)(adc_codes * (double)OPT4041_LUX_PER_COUNT);
}

bool hal_optical_read_lux(float *lux)
{
    if (!lux) return false;

    if (!s_opt4041_online) {
        *lux = s_last_lux;
        return false;
    }

    uint16_t reg0 = 0;
    uint8_t reg1 = 0;

    if (bsp_i2c_read_reg16(OPT4041_I2C_ADDR, OPT4041_REG_RESULT_MSB, &reg0, true) &&
        bsp_i2c_read_reg8(OPT4041_I2C_ADDR, OPT4041_REG_RESULT_LSB, &reg1)) {
        *lux = hal_optical_calc_lux(reg0, reg1);
        s_last_lux = *lux;
        return true;
    }

    *lux = s_last_lux;
    return false;
}

bool hal_optical_read_prox(uint16_t *prox, uint16_t *als, bool *skin_contact)
{
    if (!s_vcnl4040_online) {
        if (prox) *prox = s_last_prox;
        if (skin_contact) *skin_contact = s_last_contact;
        return false;
    }

    uint16_t ps_val = 0;
    uint16_t als_val = 0;

    bool ok = bsp_i2c_read_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_PS_DATA, &ps_val, false);
    if (als) {
        bsp_i2c_read_reg16(VCNL4040_I2C_ADDR, VCNL4040_REG_ALS_DATA, &als_val, false);
        *als = als_val;
    }

    if (ok) {
        s_last_prox = ps_val;
        s_last_contact = (ps_val >= VCNL4040_SKIN_CONTACT_THRESH);
        if (prox) *prox = ps_val;
        if (skin_contact) *skin_contact = s_last_contact;
        return true;
    }

    if (prox) *prox = s_last_prox;
    if (skin_contact) *skin_contact = s_last_contact;
    return false;
}

bool hal_optical_read(uint32_t *lux, uint16_t *prox, bool *skin_contact)
{
    float lux_f = 0.0f;
    bool lux_ok = hal_optical_read_lux(&lux_f);
    if (lux) {
        *lux = (uint32_t)(lux_f + 0.5f);
    }

    uint16_t p = 0;
    bool sc = false;
    bool prox_ok = hal_optical_read_prox(&p, NULL, &sc);
    if (prox) {
        *prox = p;
    }
    if (skin_contact) {
        *skin_contact = sc;
    }

    return (lux_ok || prox_ok);
}

bool hal_optical_opt4041_is_online(void)
{
    return s_opt4041_online;
}

bool hal_optical_vcnl4040_is_online(void)
{
    return s_vcnl4040_online;
}
