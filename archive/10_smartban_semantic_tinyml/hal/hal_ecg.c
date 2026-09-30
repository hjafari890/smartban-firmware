/*
 * ============================================================================
 * hal_ecg.c
 * Hardware Abstraction Layer - TI ADS1292R 24-Bit ECG Driver Implementation
 * ============================================================================
 */

#include "hal_ecg.h"
#include "../bsp/bsp_pins.h"
#include "../bsp/bsp_power.h"
#include "../bsp/bsp_spi.h"
#include <unistd.h>
#include <string.h>

static uint8_t        s_ads_id = 0x00;
static bool           s_ads_online = false;
static volatile bool  s_ecg_configuring = false;
static hal_ecg_mode_t s_current_mode = HAL_ECG_MODE_LIVE_ELECTRODE;

static bool ads1292_send_cmd(uint8_t cmd)
{
    if (!bsp_spi_acquire(BSP_SPI_DEV_ADS1292)) {
        return false;
    }

    uint8_t tx = cmd;
    uint8_t rx = 0;
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 1;
    trans.txBuf = &tx;
    trans.rxBuf = &rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADS1292);
    usleep(50);
    return ok;
}

static bool ads1292_write_reg(uint8_t reg, uint8_t val)
{
    if (!bsp_spi_acquire(BSP_SPI_DEV_ADS1292)) {
        return false;
    }

    uint8_t tx[3];
    uint8_t rx[3];
    tx[0] = (uint8_t)(ADS1292_CMD_WREG | (reg & 0x1F));
    tx[1] = 0x00; /* Write 1 register */
    tx[2] = val;

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADS1292);
    usleep(50);
    return ok;
}

static bool ads1292_read_regs(uint8_t start_reg, uint8_t count, uint8_t *buf)
{
    if (!buf || count == 0) return false;

    if (!bsp_spi_acquire(BSP_SPI_DEV_ADS1292)) {
        return false;
    }

    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_RREG | (start_reg & 0x1F));
    tx[1] = (uint8_t)((count - 1) & 0x1F);

    uint32_t total = 2 + count;
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = total;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    if (ok) {
        memcpy(buf, &rx[2], count);
    }

    bsp_spi_release(BSP_SPI_DEV_ADS1292);
    usleep(50);
    return ok;
}

bool hal_ecg_init(void)
{
    s_ecg_configuring = true;

    /* 1. Assert power rails and hold START LOW during configuration (exact 08_ecg_dedicated/main.c lines 431-437) */
    bsp_power_set_1v8(true);
    bsp_spi_init();
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    GPIO_write(CONFIG_GPIO_ECG_START, 0); /* START LOW during config! */
    usleep(100000); /* 100 ms rail stabilization */

    /* Lock ADS1292R to DIO 11 CS and strict SPI Mode 1 (SPI_POL0_PHA1, CPOL=0, CPHA=1)
     * NEVER probe SPI_POL0_PHA0 (Mode 0), which clocks on the wrong SCLK edge and corrupts WREG/RDATAC! */
    bsp_spi_set_ecg_cs_pin(CONFIG_GPIO_ECG_CS);
    bsp_spi_set_ecg_frame_format(SPI_POL0_PHA1);

    /* 2. Hardware reset: pulse PWDN low for 10 ms, then high (08_ecg_dedicated/main.c lines 441-443) */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    usleep(10000);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);

    /* 3. Wait 1000 ms for POR + VCAP stabilization (08_ecg_dedicated/main.c line 447) */
    usleep(1000000);

    /* 4. Send SDATAC (0x11) while START is LOW so device exits RDATAC cleanly */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);

    /* 5. Read Chip ID (0x73 = ADS1292R) */
    uint8_t reg_id = 0;
    for (int retry = 0; retry < 5; retry++) {
        if (ads1292_read_regs(ADS1292_REG_ID, 1, &reg_id) && reg_id != 0x00 && reg_id != 0xFF) {
            break;
        }
        ads1292_send_cmd(ADS1292_CMD_SDATAC);
        usleep(2000);
    }

    if (reg_id == 0x00 || reg_id == 0xFF) {
        s_ads_online = false;
        s_ecg_configuring = false;
        return false;
    }

    s_ads_id = reg_id;
    s_ads_online = true;

    /* 6. Exact 08_ecg_dedicated/main.c lines 475-534 Register Setup + 100ms Vref Settle + 500ms OFFSETCAL */
    ads1292_write_reg(ADS1292_REG_CONFIG2,   0xE0); /* Internal 2.42V ref ON, LOFF comparators ON */
    usleep(100000);                                 /* CRITICAL: Wait 100 ms for 2.42V Vref capacitor to settle! */
    ads1292_write_reg(ADS1292_REG_CONFIG1,   0x01); /* 250 SPS */
    ads1292_write_reg(ADS1292_REG_LOFF,      0x10); /* 95%/5% threshold, 6 nA DC lead-off current */
    ads1292_write_reg(ADS1292_REG_CH1SET,    0x40); /* Gain=4, Normal input (08_ecg_dedicated line 491) */
    ads1292_write_reg(ADS1292_REG_CH2SET,    0x00); /* Gain=6, Normal input (Lead I ECG) */
    ads1292_write_reg(ADS1292_REG_RLD_SENS,  0x2C); /* RLD buffer ON, derived from CH2 */
    ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x0C); /* Lead-off sensing on CH2 */
    ads1292_write_reg(ADS1292_REG_RESP1,     0x00); /* 32kHz carrier OFF */
    ads1292_write_reg(ADS1292_REG_RESP2,     0x87); /* CALIB_ON=1, RLDREF_INT=1 */

    /* 7. Run Offset Calibration after Vref has settled (08_ecg_dedicated/main.c lines 522-533) */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000);
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_send_cmd(ADS1292_CMD_OFFSETCAL);
    usleep(500000);
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_write_reg(ADS1292_REG_RESP2,     0x07); /* CALIB_ON=0, RLDREF_INT=1 */

    /* 8. Enter Mode 4 Live ECG (08_ecg_dedicated/main.c line 744) */
    s_ecg_configuring = false;
    hal_ecg_set_mode(HAL_ECG_MODE_LIVE_ELECTRODE);

    return true;
}

bool hal_ecg_calibrate_offset(void)
{
    if (!s_ads_online && s_ads_id == 0) {
        return false;
    }

    s_ecg_configuring = true;

    /* Exact runtime offset calibration sequence from 08_ecg_dedicated/main.c lines 780-793 */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_write_reg(ADS1292_REG_RESP2, 0x87); /* CALIB_ON = 1, RLDREF_INT = 1 */
    usleep(10000);
    ads1292_send_cmd(ADS1292_CMD_OFFSETCAL);
    usleep(500000);
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_write_reg(ADS1292_REG_RESP2, 0x07); /* CALIB_ON = 0, RLDREF_INT = 1 */
    usleep(10000);
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(1000);

    s_ecg_configuring = false;
    return true;
}

bool hal_ecg_set_mode(hal_ecg_mode_t mode)
{
    if (!s_ads_online && s_ads_id == 0) {
        return false;
    }

    /* Block Task_ECG SPI reads while reconfiguring ADS1292R registers */
    s_ecg_configuring = true;

    /* 1. Stop continuous conversion mode (08_ecg_dedicated/main.c line 555) */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);

    /* 2. Common configuration: 250 SPS continuous conversion (0x01) */
    ads1292_write_reg(ADS1292_REG_CONFIG1, 0x01);

    switch (mode) {
        case HAL_ECG_MODE_SQUARE_WAVE:
            ads1292_write_reg(ADS1292_REG_CONFIG2,   0xA3);
            ads1292_write_reg(ADS1292_REG_CH1SET,    0x05);
            ads1292_write_reg(ADS1292_REG_CH2SET,    0x05);
            ads1292_write_reg(ADS1292_REG_RESP1,     0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,     0x83);
            break;

        case HAL_ECG_MODE_INPUT_SHORT:
            ads1292_write_reg(ADS1292_REG_CONFIG2,   0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,    0x01);
            ads1292_write_reg(ADS1292_REG_CH2SET,    0x01);
            ads1292_write_reg(ADS1292_REG_RESP1,     0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,     0x83);
            break;

        case HAL_ECG_MODE_TEMPERATURE:
            ads1292_write_reg(ADS1292_REG_CONFIG2,   0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,    0x04);
            ads1292_write_reg(ADS1292_REG_CH2SET,    0x04);
            ads1292_write_reg(ADS1292_REG_RESP1,     0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,     0x83);
            break;

        case HAL_ECG_MODE_LIVE_ELECTRODE:
        default:
            /* Exact 1:1 register sequence from 08_ecg_dedicated/main.c ads_set_mode(MODE_LIVE_ECG) lines 599-615: */
            ads1292_write_reg(ADS1292_REG_CONFIG2,   0xE0);
            ads1292_write_reg(ADS1292_REG_LOFF,      0x10);
            ads1292_write_reg(ADS1292_REG_CH1SET,    0x10);
            ads1292_write_reg(ADS1292_REG_CH2SET,    0x00);
            ads1292_write_reg(ADS1292_REG_RLD_SENS,  0x2C);
            ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x0C);
            ads1292_write_reg(ADS1292_REG_RESP1,     0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,     0x07);
            mode = HAL_ECG_MODE_LIVE_ELECTRODE;
            break;
    }

    /* 3. Settle reference buffer and RLD loop (100 ms, 08_ecg_dedicated/main.c line 620) */
    usleep(100000);

    /* 4. Restart conversions (08_ecg_dedicated/main.c lines 623-626) */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(10000);
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(1000);

    s_current_mode = mode;
    s_ecg_configuring = false;
    return true;
}

bool hal_ecg_read_sample(hal_ecg_sample_t *sample)
{
    static uint8_t s_loff_cnt = 0;

    if (!sample || !s_ads_online || s_ecg_configuring) {
        return false;
    }

    if (!bsp_spi_acquire(BSP_SPI_DEV_ADS1292)) {
        return false;
    }

    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 9;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);
    bsp_spi_release(BSP_SPI_DEV_ADS1292);

    if (!ok) {
        return false;
    }

    /* Verify 24-bit frame header (upper 4 bits of Byte 0 must equal 1100b = 0xC0) */
    if ((rx[0] & 0xF0) != 0xC0) {
        return false;
    }

    /* 24-bit two's complement decoding for Channel 1 (Bytes 3..5) - Exact match to 08_ecg_dedicated */
    int32_t c1 = ((int32_t)rx[3] << 16) | ((int32_t)rx[4] << 8) | (int32_t)rx[5];
    if (c1 & 0x00800000) {
        c1 |= (int32_t)0xFF000000;
    }

    /* 24-bit two's complement decoding for Channel 2 (Bytes 6..8) - Exact match to 08_ecg_dedicated */
    int32_t c2 = ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | (int32_t)rx[8];
    if (c2 & 0x00800000) {
        c2 |= (int32_t)0xFF000000;
    }

    /* Extract CH2 ECG lead-off bits (bit 2 = IN2N/RA off, bit 1 = IN2P/LA off; ignore bit 3 CH1 resp carrier) */
    bool raw_loff = ((rx[0] & 0x06) != 0);
    if (raw_loff) {
        if (s_loff_cnt < 40) s_loff_cnt++;
    } else {
        if (s_loff_cnt > 0) s_loff_cnt -= 2;
    }
    bool debounced_loff = (s_loff_cnt >= 35);
    sample->status = debounced_loff ? (0xC0 | (rx[0] & 0x06)) : 0xC0;
    sample->lead_off = debounced_loff;

    sample->raw_ch1 = c1;
    sample->microvolts_ch1 = (float)c1 * ADS1292_UV_PER_COUNT;
    sample->raw_ch2 = c2;
    sample->microvolts_ch2 = (float)c2 * ADS1292_UV_PER_COUNT;

    return true;
}

bool hal_ecg_register_drdy_callback(GPIO_CallbackFxn fxn)
{
    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, fxn);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);
    return true;
}

hal_ecg_mode_t hal_ecg_get_mode(void)
{
    return s_current_mode;
}

bool hal_ecg_is_online(void)
{
    return s_ads_online;
}

uint8_t hal_ecg_get_id(void)
{
    return s_ads_id;
}
