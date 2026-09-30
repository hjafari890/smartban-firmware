/*
 * ============================================================================
 * hal_imu.c
 * Hardware Abstraction Layer - ADI ADXL362 IMU Driver Implementation
 * ============================================================================
 */

#include "hal_imu.h"
#include "../bsp/bsp_pins.h"
#include "../bsp/bsp_power.h"
#include "../bsp/bsp_spi.h"
#include <unistd.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint8_t         s_part_id = 0x00;
static bool            s_imu_online = false;
static hal_imu_range_t s_current_range = HAL_IMU_RANGE_8G;
static float           s_scale_factor = 4.0f; /* 4.0 mg/LSB for +/-8g (05_imu_adxl362_test default) */

static float   s_tare_x_mg = 0.0f;
static float   s_tare_y_mg = 0.0f;
static float   s_tare_z_mg = 0.0f;
static int16_t s_offset_x = 0;
static int16_t s_offset_y = 0;
static int16_t s_offset_z = 0;
static float   s_scale_z  = 1.0f;

static bool adxl362_write_reg(uint8_t reg, uint8_t val)
{
    if (!bsp_spi_acquire(BSP_SPI_DEV_ADXL362)) {
        return false;
    }

    uint8_t tx[3] = { ADXL362_CMD_WRITE_REG, reg, val };
    uint8_t rx[3] = { 0 };

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADXL362);

    return ok;
}

static uint8_t adxl362_read_reg(uint8_t reg)
{
    if (!bsp_spi_acquire(BSP_SPI_DEV_ADXL362)) {
        return 0xFF;
    }

    uint8_t tx[3] = { ADXL362_CMD_READ_REG, reg, 0x00 };
    uint8_t rx[3] = { 0 };

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADXL362);

    return ok ? rx[2] : 0xFF;
}

static bool adxl362_read_burst(uint8_t start_reg, uint8_t *buf, size_t len)
{
    if (!buf || len == 0) {
        return false;
    }

    if (!bsp_spi_acquire(BSP_SPI_DEV_ADXL362)) {
        return false;
    }

    uint8_t tx[34] = {0};
    uint8_t rx[34] = {0};
    if (len > 32) len = 32;

    tx[0] = ADXL362_CMD_READ_REG;
    tx[1] = start_reg;

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 2 + len;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADXL362);

    if (ok) {
        memcpy(buf, &rx[2], len);
    }
    return ok;
}

bool hal_imu_init(hal_imu_range_t range)
{
    (void)range;
    /* Exact 05_imu_adxl362_test/main.c line 323 configuration:
     * Default +/-8g range (4.0 mg/LSB) to accommodate wide zero-bias offsets on BAN Shield V3.5 */
    s_current_range = HAL_IMU_RANGE_8G;
    s_scale_factor = 4.0f;

    bsp_spi_init();

    GPIO_setConfig(CONFIG_GPIO_IMU_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    GPIO_setConfig(CONFIG_GPIO_IMU_SW, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_IMU_SW, 0);
    usleep(15000);

    bool found = false;
    uint8_t devid_ad = 0;
    uint8_t partid = 0;

    for (int swap_idx = 0; swap_idx < 2 && !found; swap_idx++) {
        /* Rev 3.5 schematic: swap_idx 0 = Swapped (TX=DIO8, RX=DIO9); swap_idx 1 = Standard (TX=DIO9, RX=DIO8) */
        bsp_spi_set_imu_swap_pins(swap_idx == 0);

        for (int sw_state = 0; sw_state < 2 && !found; sw_state++) {
            GPIO_write(CONFIG_GPIO_IMU_SW, sw_state);
            usleep(15000); /* Rail settling */

            /* Soft reset ADXL362 (Reg 0x1F = 0x52 'R') */
            adxl362_write_reg(ADXL362_REG_SOFT_RESET, 0x52);
            usleep(30000);

            /* Probe DEVID_AD (0x00) - Expected: 0xAD */
            devid_ad = adxl362_read_reg(ADXL362_REG_DEVID_AD);
            if (devid_ad == ADXL362_VAL_DEVID_AD) {
                found = true;
                partid = adxl362_read_reg(ADXL362_REG_PARTID);
                break;
            }
        }
    }

    if (!found) {
        s_imu_online = false;
        return false;
    }

    s_part_id = partid;
    s_imu_online = true;

    /* Exact configuration from 05_imu_adxl362_test/main.c (adxl362_update_config):
     * FILTER_CTL (0x2C) = (2 << 6) | (1 << 4) | 0x03 = 0x93 (+/-8g, HALF_BW=1, 100 Hz ODR)
     * POWER_CTL  (0x2D) = (2 << 4) | 0x02           = 0x22 (Ultra-Low Noise + Measurement Mode) */
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, 0x93);
    adxl362_write_reg(ADXL362_REG_INTMAP1, 0x01);
    adxl362_write_reg(ADXL362_REG_INTMAP2, 0x40);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);
    usleep(25000);

    /* Calibrate zero-g tare in mg (exact 05_imu_adxl362_test calibrate_tare method) */
    hal_imu_calibrate_static(64);

    return true;
}

bool hal_imu_calibrate_static(uint8_t num_samples)
{
    if (!s_imu_online) {
        return false;
    }

    if (num_samples < 8) num_samples = 64;

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int count = 0;

    for (int i = 0; i < (int)num_samples; i++) {
        uint8_t r[6] = {0};
        if (adxl362_read_burst(ADXL362_REG_XDATA_L, r, 6)) {
            int16_t rx = (int16_t)(((uint16_t)r[1] << 8) | r[0]);
            if (rx & 0x0800) rx |= (int16_t)0xF000; else rx &= 0x0FFF;
            int16_t ry = (int16_t)(((uint16_t)r[3] << 8) | r[2]);
            if (ry & 0x0800) ry |= (int16_t)0xF000; else ry &= 0x0FFF;
            int16_t rz = (int16_t)(((uint16_t)r[5] << 8) | r[4]);
            if (rz & 0x0800) rz |= (int16_t)0xF000; else rz &= 0x0FFF;

            /* Multiply by s_scale_factor (4.0 mg/LSB) BEFORE averaging, exactly as in 05_imu_adxl362_test */
            sum_x += (float)rx * s_scale_factor;
            sum_y += (float)ry * s_scale_factor;
            sum_z += (float)rz * s_scale_factor;
            count++;
        }
        usleep(5000);
    }

    if (count > 0) {
        s_tare_x_mg = sum_x / (float)count;
        s_tare_y_mg = sum_y / (float)count;
        float avg_z = sum_z / (float)count;

        if (avg_z > 500.0f) {
            s_tare_z_mg = avg_z - 1000.0f;
        } else if (avg_z < -500.0f) {
            s_tare_z_mg = avg_z + 1000.0f;
        } else {
            s_tare_z_mg = avg_z - 1000.0f;
        }

        s_offset_x = (int16_t)s_tare_x_mg;
        s_offset_y = (int16_t)s_tare_y_mg;
        s_offset_z = (int16_t)s_tare_z_mg;
        s_scale_z = 1.0f;
        return true;
    }

    return false;
}

void hal_imu_get_calibration(int16_t *ox, int16_t *oy, int16_t *oz, float *sz)
{
    if (ox) *ox = (int16_t)s_tare_x_mg;
    if (oy) *oy = (int16_t)s_tare_y_mg;
    if (oz) *oz = (int16_t)s_tare_z_mg;
    if (sz) *sz = s_scale_z;
}

void hal_imu_set_calibration(int16_t ox, int16_t oy, int16_t oz, float sz)
{
    s_tare_x_mg = (float)ox;
    s_tare_y_mg = (float)oy;
    s_tare_z_mg = (float)oz;
    s_offset_x = ox;
    s_offset_y = oy;
    s_offset_z = oz;
    if (sz > 0.5f && sz < 2.0f) {
        s_scale_z = sz;
    }
}

bool hal_imu_read_sample(hal_imu_sample_t *sample)
{
    if (!sample || !s_imu_online) {
        return false;
    }

    uint8_t raw[8] = {0};
    if (!adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 8)) {
        return false;
    }

    /* Exact 12-bit two's complement decoding from 05_imu_adxl362_test/main.c (adxl362_read_all) */
    int16_t rx_x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    int16_t rx_y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    int16_t rx_z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    int16_t rx_t = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);

    if (rx_x & 0x0800) rx_x |= (int16_t)0xF000; else rx_x &= 0x0FFF;
    if (rx_y & 0x0800) rx_y |= (int16_t)0xF000; else rx_y &= 0x0FFF;
    if (rx_z & 0x0800) rx_z |= (int16_t)0xF000; else rx_z &= 0x0FFF;
    if (rx_t & 0x0800) rx_t |= (int16_t)0xF000; else rx_t &= 0x0FFF;

    /* Convert to mg and subtract Zero-G Tare (exact 05_imu_adxl362_test/main.c lines 843-857) */
    float ax_mg = ((float)rx_x * s_scale_factor) - s_tare_x_mg;
    float ay_mg = ((float)rx_y * s_scale_factor) - s_tare_y_mg;
    float az_mg = ((float)rx_z * s_scale_factor) - s_tare_z_mg;

    /* Instantaneous raw magnitude for high-speed Fall Detection & Step peaks (no IIR attenuation!) */
    float raw_mag_mg = sqrtf(ax_mg * ax_mg + ay_mg * ay_mg + az_mg * az_mg);
    float raw_total_g = raw_mag_mg / 1000.0f;

    /* Adaptive Deadband Anti-Jitter Filter for 3-Axis Posture & Waveform Display */
    static float s_smooth_x = 0.0f;
    static float s_smooth_y = 0.0f;
    static float s_smooth_z = 1000.0f;
    static bool  s_filt_init = false;

    if (!s_filt_init) {
        s_smooth_x = ax_mg;
        s_smooth_y = ay_mg;
        s_smooth_z = az_mg;
        s_filt_init = true;
    } else {
        float dx = fabsf(ax_mg - s_smooth_x);
        float dy = fabsf(ay_mg - s_smooth_y);
        float dz = fabsf(az_mg - s_smooth_z);
        float max_delta = dx;
        if (dy > max_delta) max_delta = dy;
        if (dz > max_delta) max_delta = dz;

        if (max_delta < 16.0f && fabsf(raw_total_g - 1.0f) < 0.06f) {
            /* Stationary / Desk Rest: Ultra-heavy IIR smoothing (93% retention) kills ±8g LSB quantization noise */
            s_smooth_x = 0.93f * s_smooth_x + 0.07f * ax_mg;
            s_smooth_y = 0.93f * s_smooth_y + 0.07f * ay_mg;
            s_smooth_z = 0.93f * s_smooth_z + 0.07f * az_mg;
        } else {
            /* Active Tilt / Motion: Fast tracking (45% new sample) for crisp 3D attitude */
            s_smooth_x = 0.55f * s_smooth_x + 0.45f * ax_mg;
            s_smooth_y = 0.55f * s_smooth_y + 0.45f * ay_mg;
            s_smooth_z = 0.55f * s_smooth_z + 0.45f * az_mg;
        }
    }

    /* Resting zero-gravity snap deadband (±12 mg around level desk rest) */
    float out_x = (fabsf(s_smooth_x) < 12.0f) ? 0.0f : s_smooth_x;
    float out_y = (fabsf(s_smooth_y) < 12.0f) ? 0.0f : s_smooth_y;
    float out_z = (fabsf(s_smooth_z - 1000.0f) < 16.0f && out_x == 0.0f && out_y == 0.0f) ? 1000.0f : s_smooth_z;

    sample->x_mg = (int16_t)out_x;
    sample->y_mg = (int16_t)out_y;
    sample->z_mg = (int16_t)out_z;

    /* Exact ADXL362 on-chip diode temp from 05_imu_adxl362_test/main.c line 367 */
    sample->temp_c = ((float)(rx_t - 79) * 0.065f) + 24.5f;

    /* Preserve unattenuated dynamic g-force during motion/drops, or smooth 1.000g at rest */
    if (fabsf(raw_total_g - 1.0f) >= 0.06f) {
        sample->total_g = raw_total_g;
    } else {
        sample->total_g = sqrtf(out_x * out_x + out_y * out_y + out_z * out_z) / 1000.0f;
    }

    sample->pitch_deg = atan2f(out_x, sqrtf(out_y * out_y + out_z * out_z)) * (180.0f / 3.14159265f);
    sample->roll_deg  = atan2f(out_y, sqrtf(out_x * out_x + out_z * out_z)) * (180.0f / 3.14159265f);

    if (fabsf(sample->pitch_deg) < 1.1f) sample->pitch_deg = 0.0f;
    if (fabsf(sample->roll_deg)  < 1.1f) sample->roll_deg  = 0.0f;

    uint8_t st = 0;
    hal_imu_read_status(&st);
    sample->status = st;

    return true;
}

bool hal_imu_run_self_test(hal_imu_self_test_result_t *res)
{
    if (!s_imu_online || !res) {
        return false;
    }

    uint8_t orig_filter = adxl362_read_reg(ADXL362_REG_FILTER_CTL);
    uint8_t orig_pwr    = adxl362_read_reg(ADXL362_REG_POWER_CTL);

    /* Exact 7-step Electrostatic MEMS Self-Test from 05_imu_adxl362_test/main.c lines 424-512 */
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, 0x83);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);
    usleep(50000);

    int32_t b_x = 0, b_y = 0, b_z = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6] = {0};
        adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 6);
        int16_t x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        int16_t y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        int16_t z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
        if (x & 0x0800) x |= (int16_t)0xF000; else x &= 0x0FFF;
        if (y & 0x0800) y |= (int16_t)0xF000; else y &= 0x0FFF;
        if (z & 0x0800) z |= (int16_t)0xF000; else z &= 0x0FFF;
        b_x += x; b_y += y; b_z += z;
        usleep(10000);
    }
    res->baseline_x_lsb = (float)b_x / 16.0f;
    res->baseline_y_lsb = (float)b_y / 16.0f;
    res->baseline_z_lsb = (float)b_z / 16.0f;

    adxl362_write_reg(ADXL362_REG_SELF_TEST, 0x01);
    usleep(50000);

    int32_t s_x = 0, s_y = 0, s_z = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6] = {0};
        adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 6);
        int16_t x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        int16_t y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        int16_t z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
        if (x & 0x0800) x |= (int16_t)0xF000; else x &= 0x0FFF;
        if (y & 0x0800) y |= (int16_t)0xF000; else y &= 0x0FFF;
        if (z & 0x0800) z |= (int16_t)0xF000; else z &= 0x0FFF;
        s_x += x; s_y += y; s_z += z;
        usleep(10000);
    }
    res->stim_x_lsb = (float)s_x / 16.0f;
    res->stim_y_lsb = (float)s_y / 16.0f;
    res->stim_z_lsb = (float)s_z / 16.0f;

    adxl362_write_reg(ADXL362_REG_SELF_TEST, 0x00);

    res->delta_x_g = (res->stim_x_lsb - res->baseline_x_lsb) / 250.0f;
    res->delta_y_g = (res->stim_y_lsb - res->baseline_y_lsb) / 250.0f;
    res->delta_z_g = (res->stim_z_lsb - res->baseline_z_lsb) / 250.0f;

    res->pass_x = (res->delta_x_g >= 0.20f && res->delta_x_g <= 2.80f);
    res->pass_y = (res->delta_y_g <= -0.20f && res->delta_y_g >= -2.80f);
    res->pass_z = (res->delta_z_g >= 0.20f && res->delta_z_g <= 2.80f);
    res->all_pass = (res->pass_x && res->pass_y && res->pass_z);

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, orig_filter);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, orig_pwr);
    usleep(20000);

    return true;
}

bool hal_imu_read_fifo(hal_imu_fifo_sample_t *samples, uint16_t max_count, uint16_t *actual_count)
{
    if (!samples || max_count == 0 || !actual_count) {
        return false;
    }
    *actual_count = 0;

    /* Read FIFO entry count */
    uint8_t l = adxl362_read_reg(ADXL362_REG_FIFO_ENTRIES_L);
    uint8_t h = adxl362_read_reg(ADXL362_REG_FIFO_ENTRIES_H);
    uint16_t entries = ((uint16_t)(h & 0x03) << 8) | l;
    if (entries == 0) {
        return true;
    }

    uint16_t count_to_read = (entries < max_count) ? entries : max_count;

    if (!bsp_spi_acquire(BSP_SPI_DEV_ADXL362)) {
        return false;
    }

    uint8_t cmd = ADXL362_CMD_READ_FIFO;
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    /* Command phase */
    trans.count = 1;
    trans.txBuf = &cmd;
    trans.rxBuf = NULL;
    bsp_spi_transfer(&trans);

    /* Stream words phase */
    for (uint16_t i = 0; i < count_to_read; i++) {
        uint8_t tx_dummy[2] = {0, 0};
        uint8_t rx_word[2] = {0, 0};
        trans.count = 2;
        trans.txBuf = tx_dummy;
        trans.rxBuf = rx_word;
        if (!bsp_spi_transfer(&trans)) {
            break;
        }

        uint16_t w = ((uint16_t)rx_word[1] << 8) | rx_word[0];
        samples[i].tag = (hal_imu_tag_t)((w >> 14) & 0x03);

        int16_t data14 = (int16_t)(w & 0x0FFF);
        if (data14 & 0x0800) {
            data14 |= (int16_t)0xF000; /* 12-bit two's complement sign-extension */
        }

        if (samples[i].tag != HAL_IMU_TAG_TEMP) {
            samples[i].value = (int16_t)((float)data14 * s_scale_factor);
        } else {
            samples[i].value = data14;
        }

        (*actual_count)++;
    }

    bsp_spi_release(BSP_SPI_DEV_ADXL362);
    return true;
}

bool hal_imu_configure_motion_wakeup(uint16_t act_mg, uint8_t act_time_samples,
                                     uint16_t inact_mg, uint16_t inact_time_samples)
{
    if (!s_imu_online) {
        return false;
    }

    /* Per ADXL362 datasheet, enter Standby (0x00) before modifying activity/filter registers */
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    usleep(5000);

    /* Convert mg to LSB based on scale factor */
    uint16_t act_lsb = (uint16_t)((float)act_mg / s_scale_factor);
    uint16_t inact_lsb = (uint16_t)((float)inact_mg / s_scale_factor);

    /* Activity threshold (11-bit) and qualification timer */
    adxl362_write_reg(ADXL362_REG_THRESH_ACT_L, (uint8_t)(act_lsb & 0xFF));
    adxl362_write_reg(ADXL362_REG_THRESH_ACT_H, (uint8_t)((act_lsb >> 8) & 0x07));
    adxl362_write_reg(ADXL362_REG_TIME_ACT, act_time_samples);

    /* Inactivity threshold (11-bit) and qualification timer */
    adxl362_write_reg(ADXL362_REG_THRESH_INACT_L, (uint8_t)(inact_lsb & 0xFF));
    adxl362_write_reg(ADXL362_REG_THRESH_INACT_H, (uint8_t)((inact_lsb >> 8) & 0x07));
    adxl362_write_reg(ADXL362_REG_TIME_INACT_L, (uint8_t)(inact_time_samples & 0xFF));
    adxl362_write_reg(ADXL362_REG_TIME_INACT_H, (uint8_t)((inact_time_samples >> 8) & 0xFF));

    /* Map Activity interrupt (0x10) to INT1 (DIO 26) */
    adxl362_write_reg(ADXL362_REG_INTMAP1, 0x10);

    /* Map AWAKE status (0x40) to INT2 (DIO 27) */
    adxl362_write_reg(ADXL362_REG_INTMAP2, 0x40);

    /* ACT_INACT_CTL: Loop mode (bits 5:4 = 11), Referenced Inact (bit 3=1),
     * Inact Enable (bit 2=1), Referenced Act (bit 1=1), Act Enable (bit 0=1) -> 0x3F */
    adxl362_write_reg(ADXL362_REG_ACT_INACT_CTL, 0x3F);

    /* Re-apply Filter Control (100 Hz ODR, 1/4 ODR filter) and Measurement + Ultra-Low Noise Mode (0x22) */
    uint8_t filter_ctl = ((uint8_t)s_current_range << 6) | 0x10 | 0x03;
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, filter_ctl);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);
    usleep(25000);

    /* Refresh static zero-g tare now that ADXL362 is cleanly in Measurement Mode */
    hal_imu_calibrate_static(16);

    return true;
}

bool hal_imu_read_status(uint8_t *status)
{
    if (!status) return false;
    *status = adxl362_read_reg(ADXL362_REG_STATUS);
    return true;
}

uint8_t hal_imu_get_part_id(void)
{
    return s_part_id;
}

bool hal_imu_is_online(void)
{
    return s_imu_online;
}

void hal_imu_set_tare(int16_t tx, int16_t ty, int16_t tz)
{
    s_offset_x = tx;
    s_offset_y = ty;
    s_offset_z = tz;
}
