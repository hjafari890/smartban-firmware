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
static hal_ecg_rate_t s_current_rate = HAL_ECG_RATE_250_SPS;
static hal_ecg_mode_t s_current_mode = HAL_ECG_MODE_NORMAL;

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
    usleep(25); /* ADS1292 minimum 4 t_CLK delay between commands */
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
    tx[1] = 0x00; /* Write 1 register (count - 1 = 0) */
    tx[2] = val;

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    bool ok = bsp_spi_transfer(&trans);

    bsp_spi_release(BSP_SPI_DEV_ADS1292);
    usleep(25); /* Required inter-command delay */
    return ok;
}

static bool ads1292_read_regs(uint8_t start_reg, uint8_t count, uint8_t *buf)
{
    if (!buf || count == 0) {
        return false;
    }

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
    usleep(25);
    return ok;
}

bool hal_ecg_init(hal_ecg_rate_t rate)
{
    s_current_rate = rate;

    /* 1. Ensure 1.8V power domain is enabled and rails are stable */
    bsp_power_set_1v8(true);
    bsp_spi_init();

    /* 2. Hardware reset pulse on PWDN line (Active LOW) */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    usleep(5000);   /* 5ms PWDN low */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);

    /* 3. Mandatory POR wait: TI datasheet requires min 2^18 t_CLK (~512 ms) */
    sleep(1);

    /* 4. Pull START high to enable internal oscillator / conversions */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000);

    /* Candidate CS pins: primary DIO 11, secondary DIO 20 */
    static const uint8_t candidate_cs[] = { CONFIG_GPIO_ECG_CS, 20 };
    uint8_t id = 0;
    bool found = false;

    for (int cs_idx = 0; cs_idx < 2 && !found; cs_idx++) {
        uint8_t cs_pin = candidate_cs[cs_idx];
        
        /* Pre-configure candidate pin as HIGH output so SPI transfer doesn't see a floating line */
        GPIO_setConfig(cs_pin, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
        GPIO_write(cs_pin, 1);
        usleep(50);
        
        bsp_spi_set_ecg_cs_pin(cs_pin);

        for (int attempt = 0; attempt < 2; attempt++) {
            /* 5. Send SDATAC to escape continuous mode for register read/write */
            ads1292_send_cmd(ADS1292_CMD_SDATAC);
            usleep(100);

            /* 6. Verify Device ID */
            id = 0;
            if (ads1292_read_regs(ADS1292_REG_ID, 1, &id)) {
                if (id == 0x73 || id == 0x53) {
                    found = true;
                    break;
                }
            }
            usleep(10000);
        }
    }

    if (!found) {
        s_ads_online = false;
        return false;
    }

    s_ads_id = id;
    s_ads_online = true;

    /* 7. Configure Registers:
     * CONFIG1: Sample rate (0x01 = 250 SPS, 0x02 = 500 SPS)
     * RESP2:   0x83 (Reference 2.42V enabled, internal reference buffer, oscillator enabled)
     * RLD_SENS:0x2C (RLD buffer enabled, RLD connected to Channel 1 & 2 inputs)
     * Default to 1 Hz, +/-1 mV Square Wave Internal Calibration Signal
     * (Allows QRS and HRV pipeline to run on hardware without physical electrode cables)
     */
    ads1292_write_reg(ADS1292_REG_CONFIG1, (uint8_t)rate);
    ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
    ads1292_write_reg(ADS1292_REG_RLD_SENS,0x2C);

    /* CONFIG2: 0xA3 (Enable 1 Hz square wave +/-1 mV test generator) */
    ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA3);
    ads1292_write_reg(ADS1292_REG_CH1SET,  0x05); /* Test signal input */
    ads1292_write_reg(ADS1292_REG_CH2SET,  0x05);
    s_current_mode = HAL_ECG_MODE_TEST_1HZ_SQUARE;

    /* 8. Mandatory 100 ms wait for internal 2.42V reference capacitor to stabilize */
    usleep(100000);

    /* 9. Re-enter Continuous Read Data (RDATAC) mode */
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(10000);

    return true;
}


bool hal_ecg_read_sample(hal_ecg_sample_t *sample)
{
    if (!sample || !s_ads_online) {
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

    sample->status = rx[0];
    sample->lead_off = ((rx[1] & 0x0F) != 0);

    /* 24-bit two's complement decoding for Channel 1 (Bytes 3..5) */
    int32_t c1 = ((int32_t)rx[3] << 16) | ((int32_t)rx[4] << 8) | (int32_t)rx[5];
    if (c1 & 0x00800000) {
        c1 |= (int32_t)0xFF000000;
    }
    sample->raw_ch1 = c1;
    sample->microvolts_ch1 = (float)c1 * ADS1292_UV_PER_COUNT;

    /* 24-bit two's complement decoding for Channel 2 (Bytes 6..8) */
    int32_t c2 = ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | (int32_t)rx[8];
    if (c2 & 0x00800000) {
        c2 |= (int32_t)0xFF000000;
    }
    sample->raw_ch2 = c2;
    sample->microvolts_ch2 = (float)c2 * ADS1292_UV_PER_COUNT;

    return true;
}

bool hal_ecg_set_mode(hal_ecg_mode_t mode)
{
    if (!s_ads_online) {
        return false;
    }

    /* 1. Escape continuous streaming */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(100);

    /* 2. Configure input mux based on selected mode */
    switch (mode) {
        case HAL_ECG_MODE_TEST_1HZ_SQUARE:
            /* CONFIG2: 0xA3 (Enable 1 Hz square wave +/-1 mV test generator) */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA3);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x05); /* Test signal input */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x05);
            break;

        case HAL_ECG_MODE_INPUT_SHORT:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x01); /* Input shorted */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x01);
            break;

        case HAL_ECG_MODE_TEMP:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x04); /* Internal temp diode */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x04);
            break;

        case HAL_ECG_MODE_NORMAL:
        default:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x00); /* Normal electrode input */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x00);
            break;
    }

    s_current_mode = mode;
    usleep(50000);

    /* 3. Re-enter continuous data mode */
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(10000);

    return true;
}

bool hal_ecg_register_drdy_callback(GPIO_CallbackFxn fxn)
{
    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, fxn);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);
    return true;
}

uint8_t hal_ecg_get_id(void)
{
    return s_ads_id;
}

bool hal_ecg_is_online(void)
{
    return s_ads_online;
}

float hal_ecg_counts_to_uV(int32_t counts)
{
    return (float)counts * ADS1292_UV_PER_COUNT;
}
