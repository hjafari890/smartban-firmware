/*
 * ============================================================================
 * hal_fir.c
 * Hardware Abstraction Layer - Melexis MLX90632 Driver Implementation
 * 14 EEPROM Calibration Coefficients & 3-Iteration Optical Non-Linear Model
 * ============================================================================
 */

#include "hal_fir.h"
#include "../bsp/bsp_pins.h"
#include "../bsp/bsp_power.h"
#include "../bsp/bsp_i2c.h"
#include <unistd.h>
#include <math.h>
#include <string.h>

static mlx90632_calib_t s_cal;
static uint16_t         s_version = 0;
static bool             s_fir_online = false;
static float            s_last_t_amb = 25.0f;
static float            s_last_t_obj = 25.0f;

static bool mlx90632_read_reg32(uint16_t addr, int32_t *val)
{
    uint16_t lsw = 0, msw = 0;
    if (!bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, addr, &lsw)) return false;
    if (!bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, addr + 1, &msw)) return false;
    *val = (int32_t)(((uint32_t)msw << 16) | lsw);
    return true;
}

bool hal_fir_init(void)
{
    bsp_power_set_i2c_shifter(true);
    bsp_i2c_init();

    /* 1. Read silicon version */
    if (!bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_VERSION, &s_version)) {
        s_fir_online = false;
        return false;
    }

    /* 2. Put sensor in sleep mode during configuration */
    uint16_t ctrl = 0;
    if (bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_CONTROL, &ctrl)) {
        ctrl &= ~(0x03 << 1);
        ctrl |= (0x01 << 1); /* MODE_SLEEP */
        bsp_i2c_write_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_CONTROL, ctrl);
    }
    usleep(20000);

    /* 3. Read 14 Factory EEPROM Calibration Coefficients */
    int32_t val32 = 0;
    uint16_t uval16 = 0;

    mlx90632_read_reg32(0x240C, &val32); s_cal.P_R = (double)val32 * pow(2.0,  -8.0);
    mlx90632_read_reg32(0x240E, &val32); s_cal.P_G = (double)val32 * pow(2.0, -20.0);
    mlx90632_read_reg32(0x2410, &val32); s_cal.P_T = (double)val32 * pow(2.0, -44.0);
    mlx90632_read_reg32(0x2412, &val32); s_cal.P_O = (double)val32 * pow(2.0,  -8.0);
    mlx90632_read_reg32(0x2424, &val32); s_cal.Ea  = (double)val32 * pow(2.0, -16.0);
    mlx90632_read_reg32(0x2426, &val32); s_cal.Eb  = (double)val32 * pow(2.0,  -8.0);
    mlx90632_read_reg32(0x2428, &val32); s_cal.Fa  = (double)val32 * pow(2.0, -46.0);
    mlx90632_read_reg32(0x242A, &val32); s_cal.Fb  = (double)val32 * pow(2.0, -36.0);
    mlx90632_read_reg32(0x242C, &val32); s_cal.Ga  = (double)val32 * pow(2.0, -36.0);

    bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, 0x242E, &uval16);
    s_cal.Gb = (double)((int16_t)uval16) * pow(2.0, -10.0);

    bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, 0x242F, &uval16);
    s_cal.Ka = (double)((int16_t)uval16) * pow(2.0, -10.0);

    bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, 0x2481, &uval16);
    s_cal.Ha = (double)((int16_t)uval16) * pow(2.0, -14.0);

    bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, 0x2482, &uval16);
    s_cal.Hb = (double)((int16_t)uval16) * pow(2.0, -10.0);

    /* 4. Switch back to continuous measurement mode */
    if (bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_CONTROL, &ctrl)) {
        ctrl &= ~(0x03 << 1);
        ctrl |= (0x03 << 1); /* MODE_CONTINUOUS */
        bsp_i2c_write_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_CONTROL, ctrl);
    }

    s_fir_online = true;
    return true;
}

bool hal_fir_calc_ambient(int16_t sixRAM, int16_t nineRAM, const mlx90632_calib_t *cal, float *ambient_c)
{
    if (!cal || !ambient_c) return false;

    double VRta = (double)nineRAM + cal->Gb * ((double)sixRAM / 12.0);
    if (fabs(VRta) < 1e-6 || fabs(cal->P_G) < 1e-6) {
        return false;
    }

    double AMB = (((double)sixRAM / 12.0) / VRta) * 524288.0;
    double amb_diff = AMB - cal->P_R;
    double sensorTemp = cal->P_O + (amb_diff / cal->P_G) + cal->P_T * (amb_diff * amb_diff);

    *ambient_c = (float)sensorTemp;
    return true;
}

bool hal_fir_calc_object(int16_t sixRAM, int16_t nineRAM, int16_t lowerRAM, int16_t upperRAM,
                         const mlx90632_calib_t *cal, float *object_c)
{
    if (!cal || !object_c) return false;

    double VRta = (double)nineRAM + cal->Gb * ((double)sixRAM / 12.0);
    if (fabs(VRta) < 1e-6 || fabs(cal->Ea) < 1e-6) {
        return false;
    }
    double AMB = (((double)sixRAM / 12.0) / VRta) * 524288.0;

    double S = (double)(lowerRAM + upperRAM) / 2.0;
    double VRto = (double)nineRAM + cal->Ka * ((double)sixRAM / 12.0);
    if (fabs(VRto) < 1e-6) {
        return false;
    }
    double Sto = ((double)S / 12.0) / VRto * 524288.0;

    double TAdut = (AMB - cal->Eb) / cal->Ea + 25.0;
    double ambientTempK = TAdut + 273.15;
    double ambientTempK4 = ambientTempK * ambientTempK * ambientTempK * ambientTempK;

    const double TO0 = 25.0;
    const double TA0 = 25.0;
    double TOdut = 25.0;
    double objTemp = 25.0;

    /* 3-iteration non-linear optical compensation with constant TO0 preventing overflow */
    for (int i = 0; i < 3; i++) {
        double denom = cal->Fa * cal->Ha * (1.0 + cal->Ga * (TOdut - TO0) + cal->Fb * (TAdut - TA0));
        double bigFraction = (fabs(denom) > 1e-12) ? (Sto / denom) : 0.0;
        double sum4 = bigFraction + ambientTempK4;
        if (sum4 > 0.0) {
            objTemp = sqrt(sqrt(sum4)) - 273.15 - cal->Hb;
        }
        TOdut = objTemp;
    }

    *object_c = (float)objTemp;
    return true;
}

bool hal_fir_read(float *ambient_c, float *object_c)
{
    if (!s_fir_online) {
        if (ambient_c) *ambient_c = s_last_t_amb;
        if (object_c) *object_c = s_last_t_obj;
        return false;
    }

    uint16_t status = 0;
    if (!bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_STATUS, &status)) {
        return false;
    }

    bool new_data = (status & (1 << 0)) != 0;
    uint8_t cycle_pos = (status >> 2) & 0x1F;

    if (new_data) {
        /* Clear new data flag */
        bsp_i2c_write_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_STATUS, status & ~(1 << 0));

        uint16_t u6 = 0, u9 = 0;
        bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_5, &u6);
        bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_8, &u9);
        int16_t sixRAM = (int16_t)u6;
        int16_t nineRAM = (int16_t)u9;

        uint16_t ul = 0, uu = 0;
        if (cycle_pos == 1) {
            bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_3, &ul);
            bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_4, &uu);
        } else {
            bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_6, &ul);
            bsp_i2c_read_reg16_addr16(MLX90632_I2C_ADDR, MLX90632_REG_RAM_7, &uu);
        }
        int16_t lowerRAM = (int16_t)ul;
        int16_t upperRAM = (int16_t)uu;

        float tamb = 25.0f;
        float tobj = 25.0f;
        if (hal_fir_calc_ambient(sixRAM, nineRAM, &s_cal, &tamb) &&
            hal_fir_calc_object(sixRAM, nineRAM, lowerRAM, upperRAM, &s_cal, &tobj)) {
            s_last_t_amb = tamb;
            s_last_t_obj = tobj;
        }
    }

    if (ambient_c) *ambient_c = s_last_t_amb;
    if (object_c) *object_c = s_last_t_obj;
    return true;
}

bool hal_fir_get_calib(mlx90632_calib_t *cal_out)
{
    if (!cal_out) return false;
    *cal_out = s_cal;
    return s_fir_online;
}

bool hal_fir_is_online(void)
{
    return s_fir_online;
}
