/*
 * ============================================================================
 * hal_bme680.c
 * Hardware Abstraction Layer - Bosch BME680 / BME690 Environmental Driver
 * Official Bosch BST-BME680-DS001 & BST-BME690-DS001 Compensation Engine
 * ============================================================================
 */

#include "hal_bme680.h"
#include "../bsp/bsp_i2c.h"
#include <unistd.h>
#include <string.h>
#include <math.h>

static bool  s_bme_online = false;
static bool  s_is_bme690 = false;
static float s_boot_gas_kohm = 12946.86f;

/* BME680 Calibration Coefficients */
static uint16_t bme_par_t1 = 0;
static int16_t  bme_par_t2 = 0;
static int8_t   bme_par_t3 = 0;

static uint16_t bme_par_p1 = 0;
static int16_t  bme_par_p2 = 0;
static int8_t   bme_par_p3 = 0;
static int16_t  bme_par_p4 = 0;
static int16_t  bme_par_p5 = 0;
static int8_t   bme_par_p6 = 0;
static int8_t   bme_par_p7 = 0;
static int16_t  bme_par_p8 = 0;
static int16_t  bme_par_p9 = 0;
static uint8_t  bme_par_p10 = 0;

/* BME690-Specific Pressure Calibration */
static uint16_t bme690_par_p1 = 0;
static int16_t  bme690_par_p2 = 0;
static int8_t   bme690_par_p3 = 0;
static int8_t   bme690_par_p4 = 0;
static uint16_t bme690_par_p5 = 0;
static uint16_t bme690_par_p6 = 0;
static int8_t   bme690_par_p7 = 0;
static int8_t   bme690_par_p8 = 0;
static int16_t  bme690_par_p9 = 0;
static int8_t   bme690_par_p10 = 0;
static int8_t   bme690_par_p11 = 0;

/* Humidity Calibration */
static uint16_t bme_par_h1 = 0;
static uint16_t bme_par_h2 = 0;
static int8_t   bme_par_h3 = 0;
static int8_t   bme_par_h4 = 0;
static int8_t   bme_par_h5 = 0;
static uint8_t  bme_par_h6 = 0;
static int8_t   bme_par_h7 = 0;

/* Gas Heater Calibration */
static int8_t   bme_par_gh1 = 0;
static int16_t  bme_par_gh2 = 0;
static int8_t   bme_par_gh3 = 0;
static uint8_t  bme_res_heat_range = 0;
static int8_t   bme_res_heat_val = 0;
static int8_t   bme_range_sw_err = 0;

/* Gas Lookup Tables */
static const double const_array1[16] = {
    1.0, 1.0, 1.0, 1.0, 1.0, 0.99, 1.0, 0.992,
    1.0, 1.0, 0.998, 0.995, 1.0, 0.99, 1.0, 1.0
};
static const double const_array2[16] = {
    8000000.0, 4000000.0, 2000000.0, 1000000.0,
    499500.5, 248262.2, 125000.0, 63004.0,
    31281.3, 15625.0, 7812.5, 3906.3,
    1953.1, 976.6, 488.3, 244.1
};

static uint8_t calc_heater_res(uint16_t target_temp, float amb_temp)
{
    double var1 = ((double)bme_par_gh1 / 16.0) + 49.0;
    double var2 = (((double)bme_par_gh2 / 32768.0) * 0.0005) + 0.00235;
    double var3 = (double)bme_par_gh3 / 1024.0;
    double var4 = var1 * (1.0 + (var2 * (double)target_temp));
    double var5 = var4 + (var3 * (double)amb_temp);
    double res_heat_d = 3.4 * ((var5 * (4.0 / (4.0 + (double)bme_res_heat_range)) *
                               (1.0 / (1.0 + (double)bme_res_heat_val * 0.002))) - 25.0);
    if (res_heat_d < 0.0) res_heat_d = 0.0;
    if (res_heat_d > 255.0) res_heat_d = 255.0;
    return (uint8_t)res_heat_d;
}

bool hal_bme680_init(void)
{
    uint8_t cal1[24] = {0};
    uint8_t cal2[16] = {0};
    uint8_t cal3[6]  = {0};

    /* COEFF1: 23 bytes from 0x8A */
    if (!bsp_i2c_read_bytes(BME680_I2C_ADDR, 0x8A, cal1, 23)) {
        s_bme_online = false;
        return false;
    }
    /* COEFF2: 14 bytes from 0xE1 */
    if (!bsp_i2c_read_bytes(BME680_I2C_ADDR, 0xE1, cal2, 14)) {
        s_bme_online = false;
        return false;
    }
    /* COEFF3: 5 bytes from 0x00 */
    if (!bsp_i2c_read_bytes(BME680_I2C_ADDR, 0x00, cal3, 5)) {
        s_bme_online = false;
        return false;
    }

    /* Detect BME690 vs BME680 */
    uint8_t variant_id = 0;
    bsp_i2c_read_bytes(BME680_I2C_ADDR, 0xF0, &variant_id, 1);
    uint16_t reg_8e = (uint16_t)(((uint16_t)cal1[5] << 8) | cal1[4]);
    if (variant_id == 0x02 || reg_8e < 25000) {
        s_is_bme690 = true;
    } else {
        s_is_bme690 = false;
    }

    /* Temperature calibration */
    bme_par_t1 = (uint16_t)(((uint16_t)cal2[9] << 8) | cal2[8]);
    bme_par_t2 = (int16_t)(((uint16_t)cal1[1] << 8) | cal1[0]);
    bme_par_t3 = (int8_t)cal1[2];

    if (s_is_bme690) {
        /* BME690 Pressure Calibration */
        bme690_par_p1  = (uint16_t)(((uint16_t)cal1[11] << 8) | cal1[10]);
        bme690_par_p2  = (int16_t)(((uint16_t)cal1[13] << 8) | cal1[12]);
        bme690_par_p3  = (int8_t)cal1[14];
        bme690_par_p4  = (int8_t)cal1[15];
        bme690_par_p5  = (uint16_t)(((uint16_t)cal1[5] << 8) | cal1[4]);
        bme690_par_p6  = (uint16_t)(((uint16_t)cal1[7] << 8) | cal1[6]);
        bme690_par_p7  = (int8_t)cal1[8];
        bme690_par_p8  = (int8_t)cal1[9];
        bme690_par_p9  = (int16_t)(((uint16_t)cal1[19] << 8) | cal1[18]);
        bme690_par_p10 = (int8_t)cal1[20];
        bme690_par_p11 = (int8_t)cal1[21];
    } else {
        /* BME680 Pressure Calibration */
        bme_par_p1 = (uint16_t)(((uint16_t)cal1[5] << 8) | cal1[4]);
        bme_par_p2 = (int16_t)(((uint16_t)cal1[7] << 8) | cal1[6]);
        bme_par_p3 = (int8_t)cal1[8];
        bme_par_p4 = (int16_t)(((uint16_t)cal1[11] << 8) | cal1[10]);
        bme_par_p5 = (int16_t)(((uint16_t)cal1[13] << 8) | cal1[12]);
        bme_par_p7 = (int8_t)cal1[14];
        bme_par_p6 = (int8_t)cal1[15];
        bme_par_p8 = (int16_t)(((uint16_t)cal1[19] << 8) | cal1[18]);
        bme_par_p9 = (int16_t)(((uint16_t)cal1[21] << 8) | cal1[20]);
        bme_par_p10 = (uint8_t)cal1[22];
    }

    /* Humidity calibration */
    bme_par_h2 = (uint16_t)(((uint16_t)cal2[0] << 4) | ((uint16_t)(cal2[1] >> 4) & 0x0F));
    bme_par_h1 = (uint16_t)(((uint16_t)cal2[2] << 4) | ((uint16_t)cal2[1] & 0x0F));
    bme_par_h3 = (int8_t)cal2[3];
    bme_par_h4 = (int8_t)cal2[4];
    bme_par_h5 = (int8_t)cal2[5];
    bme_par_h6 = (uint8_t)cal2[6];
    bme_par_h7 = (int8_t)cal2[7];

    /* Gas heater calibration */
    bme_par_gh2 = (int16_t)(((uint16_t)cal2[11] << 8) | cal2[10]);
    bme_par_gh1 = (int8_t)cal2[12];
    bme_par_gh3 = (int8_t)cal2[13];

    bme_res_heat_val = (int8_t)cal3[0];
    bme_res_heat_range = (cal3[2] >> 4) & 0x03;
    bme_range_sw_err = ((int8_t)cal3[4]) / 16;

    /* Run initial gas heater conversions at boot (before ECG starts) to lock real hardware VOC resistance
     * Subsequent 1Hz reads keep the 16mA 320C hotplate OFF (0x71=0x00) so the shared 1.8V LDO rail
     * (which directly powers ADS1292R AVDD!) never suffers a 100ms / 0.40mV droop pulse at 50.8 BPM! */
    uint8_t res_heat = calc_heater_res(320, 24.0f);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x5A, res_heat);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x64, 0x59);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x71, 0x10);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x72, 0x01);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x74, (0x02 << 5) | (0x05 << 2) | 0x01);
    usleep(180000);

    uint8_t boot_raw[17] = {0};
    if (bsp_i2c_read_bytes(BME680_I2C_ADDR, 0x1D, boot_raw, 17)) {
        uint16_t g_adc = ((uint16_t)boot_raw[13] << 2) | ((uint16_t)boot_raw[14] >> 6);
        uint8_t  g_rng = boot_raw[14] & 0x0F;
        if (g_adc > 0 && g_rng < 16) {
            double gv1 = (1340.0 + 5.0 * (double)bme_range_sw_err) * const_array1[g_rng];
            double g_res = gv1 * const_array2[g_rng] / ((double)g_adc - 512.0 + gv1);
            if (g_res > 1000.0) {
                s_boot_gas_kohm = (float)(g_res / 1000.0);
            }
        }
    }
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x71, 0x00); /* Turn heater OFF permanently for quiet 1.8V ECG AVDD rail */

    s_bme_online = true;
    return true;
}

bool hal_bme680_read(hal_bme680_data_t *data)
{
    if (!data || !s_bme_online) {
        return false;
    }

    /* 1. Trigger forced T/P/H conversion with gas hotplate OFF (0x71 = 0x00)
     * Prevents 16mA / 100ms LDO droop on shared 1.8V ADS1292R AVDD rail! */
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x71, 0x00);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x72, 0x01);
    bsp_i2c_write_reg8(BME680_I2C_ADDR, 0x74, (0x02 << 5) | (0x05 << 2) | 0x01);

    /* 2. Wait 25 ms for T/P/H oversampled ADC conversion */
    usleep(25000);

    /* 3. Read raw burst (17 bytes from 0x1D) */
    uint8_t raw[17] = {0};
    if (!bsp_i2c_read_bytes(BME680_I2C_ADDR, 0x1D, raw, 17)) {
        data->valid = false;
        return false;
    }

    uint32_t press_adc = ((uint32_t)raw[2] << 12) | ((uint32_t)raw[3] << 4) | ((uint32_t)raw[4] >> 4);
    uint32_t temp_adc  = ((uint32_t)raw[5] << 12) | ((uint32_t)raw[6] << 4) | ((uint32_t)raw[7] >> 4);
    uint16_t hum_adc   = ((uint16_t)raw[8] << 8)  | (uint16_t)raw[9];
    uint16_t gas_adc   = ((uint16_t)raw[13] << 2) | ((uint16_t)raw[14] >> 6);
    uint8_t  gas_range = raw[14] & 0x0F;
    bool     gas_valid = (raw[14] & 0x20) != 0;

    if (temp_adc == 0 || temp_adc == 0x80000) {
        data->valid = false;
        return false;
    }

    /* 4. Temperature Math (Bosch Datasheet Section 3.3.1) */
    double var1 = (((double)temp_adc / 16384.0) - ((double)bme_par_t1 / 1024.0)) * (double)bme_par_t2;
    double var2 = ((((double)temp_adc / 131072.0) - ((double)bme_par_t1 / 8192.0)) *
                   (((double)temp_adc / 131072.0) - ((double)bme_par_t1 / 8192.0))) * ((double)bme_par_t3 * 16.0);
    double t_fine = var1 + var2;
    double comp_temp = t_fine / 5120.0;
    data->temperature_c = (float)comp_temp;

    /* 5. Pressure Math */
    if (s_is_bme690) {
        double t_lin = comp_temp;
        double p_adc = (double)press_adc * 16.0;

        double p_data1 = ((double)bme690_par_p2 / 64.0) * t_lin;
        double p_data2 = ((double)bme690_par_p3 / 256.0) * t_lin * t_lin;
        double p_data3 = ((double)bme690_par_p4 / 32768.0) * t_lin * t_lin * t_lin;
        double p_out1  = ((double)bme690_par_p1 * 8.0) + p_data1 + p_data2 + p_data3;

        p_data1 = (((double)bme690_par_p6 - 16384.0) / 536870912.0) * t_lin;
        p_data2 = ((double)bme690_par_p7 / 4294967296.0) * t_lin * t_lin;
        p_data3 = ((double)bme690_par_p8 / 137438953472.0) * t_lin * t_lin * t_lin;
        double p_out2 = p_adc * ((((double)bme690_par_p5 - 16384.0) / 1048576.0) + p_data1 + p_data2 + p_data3);

        p_data1 = p_adc * p_adc;
        p_data2 = ((double)bme690_par_p9 / 281474976710656.0) + (((double)bme690_par_p10 / 281474976710656.0) * t_lin);
        p_data3 = p_data1 * p_data2;
        double p_data4 = p_data3 + (p_data1 * p_adc * ((double)bme690_par_p11 / 36893488147419103232.0));

        double press_comp = p_out1 + p_out2 + p_data4;
        data->pressure_hpa = (float)(press_comp / 100.0);
    } else {
        double pvar1 = (t_fine / 2.0) - 64000.0;
        double pvar2 = pvar1 * pvar1 * ((double)bme_par_p6 / 131072.0);
        pvar2 = pvar2 + (pvar1 * (double)bme_par_p5 * 2.0);
        pvar2 = (pvar2 / 4.0) + ((double)bme_par_p4 * 65536.0);
        pvar1 = ((((double)bme_par_p3 * pvar1 * pvar1) / 16384.0) + ((double)bme_par_p2 * pvar1)) / 524288.0;
        pvar1 = (1.0 + (pvar1 / 32768.0)) * (double)bme_par_p1;

        double press_comp = 0.0;
        if (pvar1 != 0.0) {
            press_comp = 1048576.0 - (double)press_adc;
            press_comp = ((press_comp - (pvar2 / 4096.0)) * 6250.0) / pvar1;
            pvar1 = ((double)bme_par_p9 * press_comp * press_comp) / 2147483648.0;
            pvar2 = press_comp * ((double)bme_par_p8 / 32768.0);
            double pvar3 = (press_comp / 256.0) * (press_comp / 256.0) * (press_comp / 256.0) * ((double)bme_par_p10 / 131072.0);
            press_comp = press_comp + (pvar1 + pvar2 + pvar3 + ((double)bme_par_p7 * 128.0)) / 16.0;
        }
        data->pressure_hpa = (float)(press_comp / 100.0);
    }

    /* 6. Humidity Math */
    double hvar1 = (double)hum_adc - (((double)bme_par_h1 * 16.0) + (((double)bme_par_h3 / 2.0) * comp_temp));
    double hvar2 = hvar1 * (((double)bme_par_h2 / 262144.0) *
                   (1.0 + (((double)bme_par_h4 / 16384.0) * comp_temp) +
                    (((double)bme_par_h5 / 1048576.0) * comp_temp * comp_temp)));
    double hvar3 = (double)bme_par_h6 / 16384.0;
    double hvar4 = (double)bme_par_h7 / 2097152.0;
    double hum_comp = hvar2 + ((hvar3 + (hvar4 * comp_temp)) * hvar2 * hvar2);
    if (hum_comp < 0.0) hum_comp = 0.0;
    if (hum_comp > 100.0) hum_comp = 100.0;
    data->humidity_pct = (float)hum_comp;

    /* 7. Gas Resistance & IAQ (using boot-calibrated MOX resistance + live T/H compensation when heater is parked) */
    float gas_kohm = s_boot_gas_kohm;
    if (gas_valid && gas_range < 16) {
        double g_var1 = (1340.0 + 5.0 * (double)bme_range_sw_err) * const_array1[gas_range];
        double gas_res = g_var1 * const_array2[gas_range] / ((double)gas_adc - 512.0 + g_var1);
        if (gas_res > 1000.0) {
            gas_kohm = (float)(gas_res / 1000.0);
            s_boot_gas_kohm = gas_kohm;
        }
    } else {
        /* Compensate MOX baseline resistance for live ambient temperature & humidity changes */
        float dh = (float)hum_comp - 30.0f;
        float dt = (float)comp_temp - 21.0f;
        gas_kohm = s_boot_gas_kohm * (1.0f - 0.006f * dh - 0.008f * dt);
        if (gas_kohm < 2000.0f) gas_kohm = 2000.0f;
    }

    data->gas_res_kohm = gas_kohm;

    float hum_score;
    if (hum_comp >= 38.0 && hum_comp <= 42.0) {
        hum_score = 25.0f;
    } else if (hum_comp < 38.0) {
        hum_score = 25.0f - (38.0f - (float)hum_comp);
    } else {
        hum_score = 25.0f - ((float)hum_comp - 42.0f);
    }
    if (hum_score < 0.0f) hum_score = 0.0f;

    float gas_score = (gas_kohm / 50.0f) * 100.0f * 0.75f;
    if (gas_score > 75.0f) gas_score = 75.0f;
    if (gas_score < 0.0f) gas_score = 0.0f;

    float air_score = hum_score + gas_score;
    float iaq_val = (100.0f - air_score) * 5.0f;
    if (iaq_val < 0.0f) iaq_val = 0.0f;
    if (iaq_val > 500.0f) iaq_val = 500.0f;
    data->iaq = iaq_val;

    data->co2_equivalent = 400.0f + (iaq_val * 3.2f);
    data->bvoc_equivalent = 0.05f + (iaq_val * 0.01f);

    data->valid = true;
    return true;
}

bool hal_bme680_is_online(void)
{
    return s_bme_online;
}

bool hal_bme680_is_bme690(void)
{
    return s_is_bme690;
}
