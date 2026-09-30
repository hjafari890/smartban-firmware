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
static hal_imu_range_t s_current_range = HAL_IMU_RANGE_2G;
static float           s_scale_factor = 1.0f; /* 1 mg/LSB for 2g, 2 for 4g, 4 for 8g */

static int16_t s_tare_x = 0;
static int16_t s_tare_y = 0;
static int16_t s_tare_z = 0;

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
    s_current_range = range;
    if (range == HAL_IMU_RANGE_2G) {
        s_scale_factor = 1.0f;
    } else if (range == HAL_IMU_RANGE_4G) {
        s_scale_factor = 2.0f;
    } else {
        s_scale_factor = 4.0f;
    }

    bsp_power_set_imu_discharge(false);
    bsp_spi_init();

    uint8_t devid_ad = 0;
    uint8_t devid_mst = 0;
    uint8_t partid = 0;
    bool found = false;

    for (int swap = 0; swap < 2 && !found; swap++) {
        /* Try Rev 3.5 PCB swapped pins first (DIO8=TX, DIO9=RX), then standard */
        bsp_spi_set_imu_swap_pins(swap == 0);
        usleep(5000);

        for (int retry = 0; retry < 2 && !found; retry++) {
            /* 1. Software Reset */
            adxl362_write_reg(ADXL362_REG_SOFT_RESET, 0x52);
            usleep(35000); /* 35 ms reset settling time */

            /* 2. Verify Silicon Identifiers */
            devid_ad = adxl362_read_reg(ADXL362_REG_DEVID_AD);
            devid_mst = adxl362_read_reg(ADXL362_REG_DEVID_MST);
            partid = adxl362_read_reg(ADXL362_REG_PARTID);

            if (devid_ad == ADXL362_VAL_DEVID_AD) {
                found = true;
                break;
            }
            usleep(10000);
        }
    }

    if (!found) {
        s_imu_online = false;
        return false;
    }

    s_part_id = partid;
    s_imu_online = true;

    /* 3. Configure Filter Control: Range +/-2g/4g/8g, 1/4 ODR bandwidth, 100 Hz ODR */
    uint8_t filter_ctl = ((uint8_t)range << 6) | 0x10 | 0x03;
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, filter_ctl);

    /* 4. Configure Power Control: Ultra-Low Noise (0x20) + Measurement Mode (0x02) = 0x22 */
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);
    usleep(10000);

    return true;
}

bool hal_imu_read_sample(hal_imu_sample_t *sample)
{
    if (!sample || !s_imu_online) {
        return false;
    }

    uint8_t raw[8] = {0};
    /* Read 8 bytes starting at XDATA_L (0x0E): X_L, X_H, Y_L, Y_H, Z_L, Z_H, TEMP_L, TEMP_H */
    if (!adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 8)) {
        return false;
    }

    /* 12-bit two's complement sign-extension */
    int16_t rx = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    if (rx & 0x0800) rx |= (int16_t)0xF000; else rx &= 0x0FFF;

    int16_t ry = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    if (ry & 0x0800) ry |= (int16_t)0xF000; else ry &= 0x0FFF;

    int16_t rz = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    if (rz & 0x0800) rz |= (int16_t)0xF000; else rz &= 0x0FFF;

    int16_t rt = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
    if (rt & 0x0800) rt |= (int16_t)0xF000; else rt &= 0x0FFF;

    /* Apply sensitivity scale and subtract tare */
    sample->x_mg = (int16_t)((float)rx * s_scale_factor) - s_tare_x;
    sample->y_mg = (int16_t)((float)ry * s_scale_factor) - s_tare_y;
    sample->z_mg = (int16_t)((float)rz * s_scale_factor) - s_tare_z;

    /* Temperature sensor: 0.065 deg C/LSB with 25 deg C nominal bias offset (350 LSB @ 25 C) */
    sample->temp_c = ((float)(rt - 350) * 0.065f) + 25.0f;

    /* Kinematic Vector Magnitude */
    float fx = (float)sample->x_mg;
    float fy = (float)sample->y_mg;
    float fz = (float)sample->z_mg;
    sample->total_g = sqrtf(fx * fx + fy * fy + fz * fz) / 1000.0f;

    /* Dynamic Tilt Angles */
    float denom_pitch = sqrtf(fy * fy + fz * fz);
    sample->pitch_deg = atan2f(fx, (denom_pitch > 1e-4f ? denom_pitch : 1e-4f)) * (180.0f / (float)M_PI);

    float denom_roll = sqrtf(fx * fx + fz * fz);
    sample->roll_deg = atan2f(fy, (denom_roll > 1e-4f ? denom_roll : 1e-4f)) * (180.0f / (float)M_PI);

    uint8_t st = 0;
    hal_imu_read_status(&st);
    sample->status = st;

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
    s_tare_x = tx;
    s_tare_y = ty;
    s_tare_z = tz;
}
