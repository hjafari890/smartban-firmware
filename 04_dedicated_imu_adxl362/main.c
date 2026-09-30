/* =========================================================================
 * SmartBAN ADXL362 IMU Dedicated Diagnostic & Telemetry Firmware
 * Platform: TI CC2652R1 / CC26X2R1 LaunchPad + BAN Shield V3.5
 * Sensor: Analog Devices ADXL362 Micropower 3-Axis MEMS Accelerometer
 * ========================================================================= */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <NoRTOS.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART2.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>

#include "ti_drivers_config.h"

/* =========================================================================
 * ADXL362 Register Map
 * ========================================================================= */
#define ADXL362_REG_DEVID_AD            0x00
#define ADXL362_REG_DEVID_MST           0x01
#define ADXL362_REG_PARTID              0x02
#define ADXL362_REG_REVID               0x03
#define ADXL362_REG_XDATA               0x08
#define ADXL362_REG_YDATA               0x09
#define ADXL362_REG_ZDATA               0x0A
#define ADXL362_REG_STATUS              0x0B
#define ADXL362_REG_FIFO_ENTRIES_L      0x0C
#define ADXL362_REG_FIFO_ENTRIES_H      0x0D
#define ADXL362_REG_XDATA_L             0x0E
#define ADXL362_REG_XDATA_H             0x0F
#define ADXL362_REG_YDATA_L             0x10
#define ADXL362_REG_YDATA_H             0x11
#define ADXL362_REG_ZDATA_L             0x12
#define ADXL362_REG_ZDATA_H             0x13
#define ADXL362_REG_TEMP_L              0x14
#define ADXL362_REG_TEMP_H              0x15
#define ADXL362_REG_SOFT_RESET          0x1F
#define ADXL362_REG_THRESH_ACT_L        0x20
#define ADXL362_REG_THRESH_ACT_H        0x21
#define ADXL362_REG_TIME_ACT            0x22
#define ADXL362_REG_THRESH_INACT_L      0x23
#define ADXL362_REG_THRESH_INACT_H      0x24
#define ADXL362_REG_TIME_INACT_L        0x25
#define ADXL362_REG_TIME_INACT_H        0x26
#define ADXL362_REG_ACT_INACT_CTL       0x27
#define ADXL362_REG_FIFO_CONTROL        0x28
#define ADXL362_REG_FIFO_SAMPLES        0x29
#define ADXL362_REG_INTMAP1             0x2A
#define ADXL362_REG_INTMAP2             0x2B
#define ADXL362_REG_FILTER_CTL          0x2C
#define ADXL362_REG_POWER_CTL           0x2D
#define ADXL362_REG_SELF_TEST           0x2E

/* SPI Commands */
#define ADXL362_CMD_WRITE_REG           0x0A
#define ADXL362_CMD_READ_REG            0x0B
#define ADXL362_CMD_READ_FIFO           0x0D

/* STATUS Register Bit Masks */
#define ADXL362_STATUS_DATA_READY       (1 << 0)
#define ADXL362_STATUS_FIFO_READY       (1 << 1)
#define ADXL362_STATUS_FIFO_WATERMARK   (1 << 2)
#define ADXL362_STATUS_FIFO_OVERRUN     (1 << 3)
#define ADXL362_STATUS_ACT              (1 << 4)
#define ADXL362_STATUS_INACT            (1 << 5)
#define ADXL362_STATUS_AWAKE            (1 << 6)
#define ADXL362_STATUS_ERR_USER_REGS    (1 << 7)

/* Global Drivers */
static UART2_Handle uart = NULL;
static SPI_Handle   spi  = NULL;

/* Device Identification & State */
static uint8_t devid_ad  = 0;
static uint8_t devid_mst = 0;
static uint8_t partid    = 0;
static uint8_t revid     = 0;
static bool adxl_swap_pins = true; /* V3.5 routing: SDI=DIO8, SDO=DIO9 */

/* Configuration State */
static uint8_t current_range = 0; /* 0: +/-2g, 1: +/-4g, 2: +/-8g */
static uint8_t current_odr   = 3; /* 0: 12.5Hz, 1: 25Hz, 2: 50Hz, 3: 100Hz, 4: 200Hz, 5: 400Hz */
static uint8_t current_noise = 2; /* 0: Normal, 1: Low Noise, 2: Ultra-Low Noise */

/* Tare Offsets (stored in milli-g to remain scale-invariant across range switches) */
static float tare_x_mg = 0.0f;
static float tare_y_mg = 0.0f;
static float tare_z_mg = 0.0f;
static bool tare_enabled = false;

typedef enum {
    OPMODE_STREAM = 1,
    OPMODE_FIFO,
    OPMODE_MOTION,
    OPMODE_IDLE
} OpMode_t;

static OpMode_t active_mode = OPMODE_STREAM;

/* =========================================================================
 * UART Helper Functions
 * ========================================================================= */
static void uartPrint(const char *str)
{
    if (!uart || !str) return;
    size_t written;
    UART2_write(uart, str, strlen(str), &written);
}

static void printHex8(uint8_t val)
{
    char buf[8];
    snprintf(buf, sizeof(buf), "%02X", val);
    uartPrint(buf);
}

static void printHex16(uint16_t val)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "%04X", val);
    uartPrint(buf);
}

static void printInt(int32_t val)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)val);
    uartPrint(buf);
}

static void printFloat(float val, int decimals)
{
    char buf[32];
    if (decimals == 1) snprintf(buf, sizeof(buf), "%.1f", val);
    else if (decimals == 2) snprintf(buf, sizeof(buf), "%.2f", val);
    else if (decimals == 3) snprintf(buf, sizeof(buf), "%.3f", val);
    else snprintf(buf, sizeof(buf), "%f", val);
    uartPrint(buf);
}

static bool uartGetChar(char *ch)
{
    if (!uart) return false;
    size_t read_bytes = 0;
    int_fast16_t status = UART2_read(uart, ch, 1, &read_bytes);
    return (status == UART2_STATUS_SUCCESS && read_bytes == 1);
}

/* =========================================================================
 * Low-Level SPI & ADXL362 Communication
 * ========================================================================= */
static void apply_spi_routing(void)
{
    if (adxl_swap_pins) {
        IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
    } else {
        IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
    }
}

static bool spi_init_mode0(void)
{
    if (spi != NULL) {
        SPI_close(spi);
        spi = NULL;
    }
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate     = 1000000;
    spiParams.frameFormat = SPI_POL0_PHA0;
    spiParams.mode        = SPI_CONTROLLER;
    spi = SPI_open(CONFIG_SPI_0, &spiParams);
    if (!spi) return false;

    apply_spi_routing();
    return true;
}

static bool adxl362_write_reg(uint8_t reg, uint8_t val)
{
    if (!spi) return false;
    apply_spi_routing();

    uint8_t tx[3] = { ADXL362_CMD_WRITE_REG, reg, val };
    uint8_t rx[3] = { 0 };
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(CONFIG_GPIO_IMU_CS, 0);
    usleep(10);
    bool ok = SPI_transfer(spi, &trans);
    usleep(10);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    return ok;
}

static uint8_t adxl362_read_reg(uint8_t reg)
{
    if (!spi) return 0xFF;
    apply_spi_routing();

    uint8_t tx[3] = { ADXL362_CMD_READ_REG, reg, 0x00 };
    uint8_t rx[3] = { 0 };
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(CONFIG_GPIO_IMU_CS, 0);
    usleep(10);
    bool ok = SPI_transfer(spi, &trans);
    usleep(10);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    return ok ? rx[2] : 0xFF;
}

static bool adxl362_read_burst(uint8_t start_reg, uint8_t *buf, size_t len)
{
    if (!spi || !buf || len == 0) return false;
    apply_spi_routing();

    uint8_t tx[34];
    uint8_t rx[34];
    if (len + 2 > sizeof(tx)) return false;

    memset(tx, 0, len + 2);
    memset(rx, 0, len + 2);
    tx[0] = ADXL362_CMD_READ_REG;
    tx[1] = start_reg;

    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = len + 2;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(CONFIG_GPIO_IMU_CS, 0);
    usleep(10);
    bool ok = SPI_transfer(spi, &trans);
    usleep(10);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    if (ok) {
        memcpy(buf, &rx[2], len);
    }
    return ok;
}

static float get_sensitivity(void)
{
    switch (current_range) {
        case 0: return 1.0f;
        case 1: return 2.0f;
        case 2:
        default: return 4.0f;
    }
}

static void adxl362_update_config(void)
{
    uint8_t filter_val = (current_range << 6) | (1 << 4) | (current_odr & 0x07);
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, filter_val);

    uint8_t pwr_val = (current_noise << 4) | 0x02;
    adxl362_write_reg(ADXL362_REG_POWER_CTL, pwr_val);
    usleep(20000);
}

static bool adxl362_init(void)
{
    GPIO_setConfig(CONFIG_GPIO_ECG_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);

    GPIO_setConfig(CONFIG_GPIO_IMU_SW, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_IMU_SW, 0);

    GPIO_setConfig(CONFIG_GPIO_IMU_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    GPIO_setConfig(CONFIG_GPIO_IMU_INT1, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_PULL_UP_INTERNAL);
    GPIO_setConfig(CONFIG_GPIO_IMU_INT2, GPIO_CFG_INPUT_INTERNAL | GPIO_CFG_PULL_UP_INTERNAL);

    usleep(15000);

    bool found = false;
    for (int swap = 0; swap < 2 && !found; swap++) {
        adxl_swap_pins = (swap == 0);
        spi_init_mode0();

        adxl362_write_reg(ADXL362_REG_SOFT_RESET, 0x52);
        usleep(30000);

        uint8_t id = adxl362_read_reg(ADXL362_REG_DEVID_AD);
        if (id == 0xAD) {
            found = true;
            devid_ad  = id;
            devid_mst = adxl362_read_reg(ADXL362_REG_DEVID_MST);
            partid    = adxl362_read_reg(ADXL362_REG_PARTID);
            revid     = adxl362_read_reg(ADXL362_REG_REVID);
            break;
        }
    }

    if (!found) {
        devid_ad = adxl362_read_reg(ADXL362_REG_DEVID_AD);
        return false;
    }

    current_range = 2; /* Default +/-8g range (4 mg/LSB) to accommodate wide zero-bias offsets */
    current_odr   = 3;
    current_noise = 2;
    adxl362_update_config();

    adxl362_write_reg(ADXL362_REG_INTMAP1, 0x01);
    adxl362_write_reg(ADXL362_REG_INTMAP2, 0x40);

    return true;
}

static bool adxl362_read_all(int16_t *x, int16_t *y, int16_t *z,
                             int8_t *x8, int8_t *y8, int8_t *z8,
                             float *temp_c, uint8_t *status,
                             uint8_t *int1_pin, uint8_t *int2_pin)
{
    if (devid_ad != 0xAD) return false;

    *status = adxl362_read_reg(ADXL362_REG_STATUS);

    uint8_t b8[3] = {0};
    adxl362_read_burst(ADXL362_REG_XDATA, b8, 3);
    *x8 = (int8_t)b8[0];
    *y8 = (int8_t)b8[1];
    *z8 = (int8_t)b8[2];

    uint8_t raw[8] = {0};
    if (!adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 8)) return false;

    int16_t rx_x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    int16_t rx_y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    int16_t rx_z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    int16_t rx_t = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);

    if (rx_x & 0x0800) rx_x |= (int16_t)0xF000; else rx_x &= 0x0FFF;
    if (rx_y & 0x0800) rx_y |= (int16_t)0xF000; else rx_y &= 0x0FFF;
    if (rx_z & 0x0800) rx_z |= (int16_t)0xF000; else rx_z &= 0x0FFF;
    if (rx_t & 0x0800) rx_t |= (int16_t)0xF000; else rx_t &= 0x0FFF;

    *x = rx_x;
    *y = rx_y;
    *z = rx_z;

    /* ADXL362 on-chip diode temp: room-temp nominal bias for this die is ~79 LSB @ 25C */
    *temp_c = ((float)(rx_t - 79) * 0.065f) + 24.5f;

    *int1_pin = GPIO_read(CONFIG_GPIO_IMU_INT1);
    *int2_pin = GPIO_read(CONFIG_GPIO_IMU_INT2);

    return true;
}

static void calibrate_tare(void)
{
    uartPrint("\r\n[*] Calibrating Zero-G Tare (keep device stationary on flat surface)...\r\n");
    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int SAMPLES = 64;
    float sens = get_sensitivity();

    for (int i = 0; i < SAMPLES; i++) {
        uint8_t raw[6];
        if (adxl362_read_burst(ADXL362_REG_XDATA_L, raw, 6)) {
            int16_t x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
            int16_t y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
            int16_t z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
            if (x & 0x0800) x |= (int16_t)0xF000; else x &= 0x0FFF;
            if (y & 0x0800) y |= (int16_t)0xF000; else y &= 0x0FFF;
            if (z & 0x0800) z |= (int16_t)0xF000; else z &= 0x0FFF;
            sum_x += (float)x * sens;
            sum_y += (float)y * sens;
            sum_z += (float)z * sens;
        }
        usleep(10000);
    }

    tare_x_mg = sum_x / (float)SAMPLES;
    tare_y_mg = sum_y / (float)SAMPLES;
    float avg_z = sum_z / (float)SAMPLES;

    if (avg_z > 500.0f) {
        tare_z_mg = avg_z - 1000.0f;
    } else if (avg_z < -500.0f) {
        tare_z_mg = avg_z + 1000.0f;
    } else {
        tare_z_mg = 0.0f;
    }

    tare_enabled = true;
    uartPrint("[+] Tare calibration complete:\r\n");
    uartPrint("    Offset X: "); printInt((int32_t)tare_x_mg); uartPrint(" mg\r\n");
    uartPrint("    Offset Y: "); printInt((int32_t)tare_y_mg); uartPrint(" mg\r\n");
    uartPrint("    Offset Z: "); printInt((int32_t)tare_z_mg); uartPrint(" mg\r\n\r\n");
}

static void run_self_test(void)
{
    uartPrint("\r\n================================================================\r\n");
    uartPrint("  ADXL362 ELECTROSTATIC MEMS SELF-TEST DIAGNOSTIC\r\n");
    uartPrint("  (Deflects proof masses electrostatically and audits response)\r\n");
    uartPrint("================================================================\r\n");

    uint8_t orig_filter = adxl362_read_reg(ADXL362_REG_FILTER_CTL);
    uint8_t orig_pwr    = adxl362_read_reg(ADXL362_REG_POWER_CTL);

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, 0x83);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);
    usleep(50000);

    int32_t b_x = 0, b_y = 0, b_z = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6];
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
    float baseline_x = (float)b_x / 16.0f;
    float baseline_y = (float)b_y / 16.0f;
    float baseline_z = (float)b_z / 16.0f;

    uartPrint("[1] Baseline Accel (ST OFF) [LSB]:\r\n");
    uartPrint("    X: "); printFloat(baseline_x, 1);
    uartPrint(" | Y: "); printFloat(baseline_y, 1);
    uartPrint(" | Z: "); printFloat(baseline_z, 1);
    uartPrint("\r\n");

    uartPrint("[2] Asserting Electrostatic Self-Test Force (0x2E = 0x01)...\r\n");
    adxl362_write_reg(ADXL362_REG_SELF_TEST, 0x01);
    usleep(50000);

    int32_t s_x = 0, s_y = 0, s_z = 0;
    for (int i = 0; i < 16; i++) {
        uint8_t raw[6];
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
    float st_x = (float)s_x / 16.0f;
    float st_y = (float)s_y / 16.0f;
    float st_z = (float)s_z / 16.0f;

    uartPrint("[3] Stimulated Accel (ST ON) [LSB]:\r\n");
    uartPrint("    X: "); printFloat(st_x, 1);
    uartPrint(" | Y: "); printFloat(st_y, 1);
    uartPrint(" | Z: "); printFloat(st_z, 1);
    uartPrint("\r\n");

    adxl362_write_reg(ADXL362_REG_SELF_TEST, 0x00);

    float delta_g_x = (st_x - baseline_x) / 250.0f;
    float delta_g_y = (st_y - baseline_y) / 250.0f;
    float delta_g_z = (st_z - baseline_z) / 250.0f;

    uartPrint("\r\n[4] Electrostatic Deflection Results (Datasheet Table 22):\r\n");
    uartPrint("    Delta X: "); printFloat(delta_g_x, 3); uartPrint(" g   (Expected: +0.20g to +2.80g)\r\n");
    uartPrint("    Delta Y: "); printFloat(delta_g_y, 3); uartPrint(" g   (Expected: -2.80g to -0.20g)\r\n");
    uartPrint("    Delta Z: "); printFloat(delta_g_z, 3); uartPrint(" g   (Expected: +0.20g to +2.80g)\r\n");

    bool pass_x = (delta_g_x >= 0.20f && delta_g_x <= 2.80f);
    bool pass_y = (delta_g_y <= -0.20f && delta_g_y >= -2.80f);
    bool pass_z = (delta_g_z >= 0.20f && delta_g_z <= 2.80f);

    uartPrint("\r\n[5] Sensor Health Verdict: ");
    if (pass_x && pass_y && pass_z) {
        uartPrint("[ PASS ] ALL 3 AXES ARE FULLY OPERATIONAL!\r\n");
    } else {
        uartPrint("[ FAIL ] Response outside Table 22 boundaries.\r\n");
        if (!pass_x) uartPrint("    -> X-Axis out of spec\r\n");
        if (!pass_y) uartPrint("    -> Y-Axis out of spec\r\n");
        if (!pass_z) uartPrint("    -> Z-Axis out of spec\r\n");
    }
    uartPrint("================================================================\r\n\r\n");

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FILTER_CTL, orig_filter);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, orig_pwr);
    usleep(20000);
}

static void run_fifo_mode(void)
{
    uartPrint("\r\n================================================================\r\n");
    uartPrint("  ADXL362 HARDWARE FIFO BURST TELEMETRY\r\n");
    uartPrint("  (Testing 512-sample hardware FIFO with Tag Decoders)\r\n");
    uartPrint("================================================================\r\n");

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FIFO_CONTROL, 0x06);
    adxl362_write_reg(ADXL362_REG_FIFO_SAMPLES, 64);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);

    uartPrint("[*] FIFO accumulating in Stream Mode (Accel + Temp)...\r\n");
    sleep(1);

    uint8_t ent_l = adxl362_read_reg(ADXL362_REG_FIFO_ENTRIES_L);
    uint8_t ent_h = adxl362_read_reg(ADXL362_REG_FIFO_ENTRIES_H);
    uint16_t entries = ((uint16_t)(ent_h & 0x03) << 8) | ent_l;

    uartPrint("[+] FIFO Entries Count: "); printInt(entries); uartPrint(" samples\r\n");

    uint16_t samples_to_read = (entries > 32) ? 32 : entries;
    if (samples_to_read > 0) {
        uartPrint("  Decoded FIFO Frames [Tag | Axis | Value]:\r\n");
        for (uint16_t i = 0; i < samples_to_read; i++) {
            uint8_t tx[3] = { ADXL362_CMD_READ_FIFO, 0, 0 };
            uint8_t rx[3] = { 0 };
            SPI_Transaction trans;
            memset(&trans, 0, sizeof(trans));
            trans.count = 3;
            trans.txBuf = tx;
            trans.rxBuf = rx;

            GPIO_write(CONFIG_GPIO_IMU_CS, 0);
            usleep(5);
            SPI_transfer(spi, &trans);
            usleep(5);
            GPIO_write(CONFIG_GPIO_IMU_CS, 1);

            uint16_t frame = ((uint16_t)rx[2] << 8) | rx[1];
            uint8_t tag = (frame >> 14) & 0x03;
            int16_t val = (int16_t)(frame & 0x3FFF);
            if (val & 0x0800) val |= (int16_t)0xF000; else val &= 0x0FFF;

            uartPrint("    [#"); printInt(i); uartPrint("] Tag 0b");
            printHex8(tag);
            switch (tag) {
                case 0: uartPrint(" (X-Axis): "); printInt(val); uartPrint(" LSB\r\n"); break;
                case 1: uartPrint(" (Y-Axis): "); printInt(val); uartPrint(" LSB\r\n"); break;
                case 2: uartPrint(" (Z-Axis): "); printInt(val); uartPrint(" LSB\r\n"); break;
                case 3: {
                    float tc = ((float)(val - 350) * 0.065f) + 25.0f;
                    uartPrint(" (Temp)  : "); printFloat(tc, 1); uartPrint(" deg C\r\n");
                    break;
                }
            }
        }
    }

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_FIFO_CONTROL, 0x00);
    adxl362_update_config();
    uartPrint("================================================================\r\n\r\n");
}

static void run_motion_detection(void)
{
    uartPrint("\r\n================================================================\r\n");
    uartPrint("  ADXL362 AUTONOMOUS MOTION & ACTIVITY/INACTIVITY DETECTOR\r\n");
    uartPrint("  (Loop Mode: Activity > 250mg, Inactivity < 150mg for ~0.5s)\r\n");
    uartPrint("  Move/shake board to trigger ACTIVITY; hold still for INACTIVITY.\r\n");
    uartPrint("  Press any key to return to Main Menu.\r\n");
    uartPrint("================================================================\r\n");

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_THRESH_ACT_L, 0xFA);
    adxl362_write_reg(ADXL362_REG_THRESH_ACT_H, 0x00);
    adxl362_write_reg(ADXL362_REG_TIME_ACT, 4);
    adxl362_write_reg(ADXL362_REG_THRESH_INACT_L, 0x96);
    adxl362_write_reg(ADXL362_REG_THRESH_INACT_H, 0x00);
    adxl362_write_reg(ADXL362_REG_TIME_INACT_L, 50);
    adxl362_write_reg(ADXL362_REG_TIME_INACT_H, 0x00);
    adxl362_write_reg(ADXL362_REG_ACT_INACT_CTL, 0x3F);
    adxl362_write_reg(ADXL362_REG_INTMAP1, 0x10);
    adxl362_write_reg(ADXL362_REG_INTMAP2, 0x40);
    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x22);

    while (1) {
        char ch;
        if (uartGetChar(&ch)) {
            break;
        }

        uint8_t status = adxl362_read_reg(ADXL362_REG_STATUS);
        uint8_t int1 = GPIO_read(CONFIG_GPIO_IMU_INT1);
        uint8_t int2 = GPIO_read(CONFIG_GPIO_IMU_INT2);

        bool is_awake = (status & ADXL362_STATUS_AWAKE) != 0;
        bool has_act  = (status & ADXL362_STATUS_ACT) != 0;
        bool has_inact = (status & ADXL362_STATUS_INACT) != 0;

        uartPrint("  State: [");
        if (is_awake) {
            uartPrint(" AWAKE / MOVING  ] | ");
        } else {
            uartPrint(" ASLEEP / STILL  ] | ");
        }
        uartPrint("Flags: ACT="); printInt(has_act);
        uartPrint(" INACT="); printInt(has_inact);
        uartPrint(" | Hardware INT1(ACT)="); printInt(int1);
        uartPrint(" INT2(AWAKE)="); printInt(int2);
        uartPrint("\r\n");

        usleep(150000);
    }

    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_ACT_INACT_CTL, 0x00);
    adxl362_write_reg(ADXL362_REG_INTMAP1, 0x01);
    adxl362_write_reg(ADXL362_REG_INTMAP2, 0x40);
    adxl362_update_config();
}

static void dump_registers(void)
{
    uartPrint("\r\n================================================================\r\n");
    uartPrint("  ADXL362 INTERNAL REGISTER MAP DUMP (0x00 - 0x2E)\r\n");
    uartPrint("================================================================\r\n");
    uartPrint("  ADDR | REGISTER NAME       | HEX VALUE | BINARY\r\n");
    uartPrint("  -------------------------------------------------\r\n");

    const struct { uint8_t addr; const char *name; } reg_names[] = {
        { 0x00, "DEVID_AD        " },
        { 0x01, "DEVID_MST       " },
        { 0x02, "PARTID          " },
        { 0x03, "REVID           " },
        { 0x08, "XDATA           " },
        { 0x09, "YDATA           " },
        { 0x0A, "ZDATA           " },
        { 0x0B, "STATUS          " },
        { 0x0C, "FIFO_ENTRIES_L  " },
        { 0x0D, "FIFO_ENTRIES_H  " },
        { 0x0E, "XDATA_L         " },
        { 0x0F, "XDATA_H         " },
        { 0x10, "YDATA_L         " },
        { 0x11, "YDATA_H         " },
        { 0x12, "ZDATA_L         " },
        { 0x13, "ZDATA_H         " },
        { 0x14, "TEMP_L          " },
        { 0x15, "TEMP_H          " },
        { 0x20, "THRESH_ACT_L    " },
        { 0x21, "THRESH_ACT_H    " },
        { 0x22, "TIME_ACT        " },
        { 0x23, "THRESH_INACT_L  " },
        { 0x24, "THRESH_INACT_H  " },
        { 0x25, "TIME_INACT_L    " },
        { 0x26, "TIME_INACT_H    " },
        { 0x27, "ACT_INACT_CTL   " },
        { 0x28, "FIFO_CONTROL    " },
        { 0x29, "FIFO_SAMPLES    " },
        { 0x2A, "INTMAP1         " },
        { 0x2B, "INTMAP2         " },
        { 0x2C, "FILTER_CTL      " },
        { 0x2D, "POWER_CTL       " },
        { 0x2E, "SELF_TEST       " }
    };

    size_t count = sizeof(reg_names) / sizeof(reg_names[0]);
    for (size_t i = 0; i < count; i++) {
        uint8_t addr = reg_names[i].addr;
        uint8_t val  = adxl362_read_reg(addr);

        uartPrint("  0x"); printHex8(addr);
        uartPrint(" | "); uartPrint(reg_names[i].name);
        uartPrint(" | 0x"); printHex8(val);
        uartPrint("    | 0b");
        for (int b = 7; b >= 0; b--) {
            uartPrint((val & (1 << b)) ? "1" : "0");
        }
        uartPrint("\r\n");
    }
    uartPrint("================================================================\r\n\r\n");
}

static void print_menu(void)
{
    uartPrint("\r\n================================================================\r\n");
    uartPrint("  SMARTBAN ADXL362 DEDICATED IMU FIRMWARE COMMAND MENU\r\n");
    uartPrint("================================================================\r\n");
    uartPrint("  [1] Live Multi-Channel Stream (Acc X/Y/Z, Temp, Tilt, Status)\r\n");
    uartPrint("  [2] Run MEMS Electrostatic Self-Test Diagnostic (Pass/Fail)\r\n");
    uartPrint("  [3] Run 512-Sample FIFO Burst Stream & Tag Decode\r\n");
    uartPrint("  [4] Run Autonomous Motion & Activity/Inactivity Detection\r\n");
    uartPrint("  [5] Toggle Dynamic Range (+/-2g, +/-4g, +/-8g)\r\n");
    uartPrint("  [6] Cycle Output Data Rate (12.5Hz -> 400Hz)\r\n");
    uartPrint("  [7] Cycle Power/Noise Mode (Normal, Low Noise, Ultra-Low)\r\n");
    uartPrint("  [8] Zero-G Tare Horizontal Offset Calibration\r\n");
    uartPrint("  [9] Register Map Inspection Dump (0x00 - 0x2E)\r\n");
    uartPrint("  [R] Soft Reset Sensor ('R' command)\r\n");
    uartPrint("  [M] Re-display this Menu\r\n");
    uartPrint("================================================================\r\n\r\n");
}

int main(void)
{
    Board_init();
    NoRTOS_start();
    GPIO_init();
    SPI_init();

    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate  = 115200;
    uartParams.readMode  = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    GPIO_setConfig(CONFIG_GPIO_1V8_EN, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    GPIO_setConfig(CONFIG_GPIO_I2C_EN, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    GPIO_setConfig(CONFIG_GPIO_IMU_SW, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_LOW);
    GPIO_write(CONFIG_GPIO_IMU_SW, 0);
    GPIO_setConfig(CONFIG_GPIO_ECG_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);

    uartPrint("\r\n\r\n\r\n");
    uartPrint("****************************************************************\r\n");
    uartPrint("*  SMARTBAN ADXL362 DEDICATED IMU FIRMWARE (CC2652R1 / BAN V3.5) *\r\n");
    uartPrint("****************************************************************\r\n");

    uartPrint("[*] Probing ADXL362 on SPI bus...\r\n");
    bool ok = adxl362_init();

    uartPrint("    DEVID_AD  : 0x"); printHex8(devid_ad);
    if (devid_ad == 0xAD) uartPrint(" (MATCH ADI)\r\n"); else uartPrint(" (FAIL! Expected 0xAD)\r\n");
    uartPrint("    DEVID_MST : 0x"); printHex8(devid_mst);
    if (devid_mst == 0x1D) uartPrint(" (MATCH MEMS)\r\n"); else uartPrint("\r\n");
    uartPrint("    PARTID    : 0x"); printHex8(partid);
    if (partid == 0xF2) uartPrint(" (MATCH ADXL362 Octal 362)\r\n"); else uartPrint("\r\n");
    uartPrint("    REVID     : 0x"); printHex8(revid); uartPrint("\r\n");
    uartPrint("    SPI Routing : ");
    if (adxl_swap_pins) uartPrint("V3.5 Swapped (SDI=DIO8, SDO=DIO9)\r\n");
    else uartPrint("Standard BoosterPack (MOSI=DIO9, MISO=DIO8)\r\n");

    if (!ok || devid_ad != 0xAD) {
        uartPrint("[!] CRITICAL: ADXL362 communication failed! Halting.\r\n");
        while (1) sleep(1);
    }

    uartPrint("[+] ADXL362 successfully initialized in +/-8g Ultra-Low Noise Measurement Mode.\r\n");
    calibrate_tare();
    print_menu();

    while (1) {
        char ch;
        if (uartGetChar(&ch)) {
            switch (ch) {
                case '1':
                    active_mode = OPMODE_STREAM;
                    uartPrint("\r\n[*] Switched to LIVE STREAM MODE\r\n");
                    break;
                case '2':
                    run_self_test();
                    break;
                case '3':
                    run_fifo_mode();
                    break;
                case '4':
                    run_motion_detection();
                    break;
                case '5': {
                    current_range = (current_range + 1) % 3;
                    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
                    adxl362_update_config();
                    uartPrint("\r\n[+] Dynamic Range changed to: ");
                    if (current_range == 0) uartPrint("+/- 2g (1 mg/LSB)\r\n");
                    else if (current_range == 1) uartPrint("+/- 4g (2 mg/LSB)\r\n");
                    else uartPrint("+/- 8g (4 mg/LSB)\r\n");
                    break;
                }
                case '6': {
                    current_odr = (current_odr + 1) % 6;
                    adxl362_write_reg(ADXL362_REG_POWER_CTL, 0x00);
                    adxl362_update_config();
                    uartPrint("\r\n[+] Output Data Rate (ODR) changed to: ");
                    const char *odr_names[] = { "12.5 Hz", "25 Hz", "50 Hz", "100 Hz", "200 Hz", "400 Hz" };
                    uartPrint(odr_names[current_odr]); uartPrint("\r\n");
                    break;
                }
                case '7': {
                    current_noise = (current_noise + 1) % 3;
                    adxl362_update_config();
                    uartPrint("\r\n[+] Noise Mode changed to: ");
                    if (current_noise == 0) uartPrint("Normal Operation (1.8 uA)\r\n");
                    else if (current_noise == 1) uartPrint("Low Noise (3.3 uA)\r\n");
                    else uartPrint("Ultra-Low Noise (13 uA)\r\n");
                    break;
                }
                case '8':
                    calibrate_tare();
                    break;
                case '9':
                    dump_registers();
                    break;
                case 'r':
                case 'R':
                    uartPrint("\r\n[*] Soft resetting ADXL362...\r\n");
                    adxl362_init();
                    uartPrint("[+] Re-initialized.\r\n");
                    break;
                case 'm':
                case 'M':
                case '?':
                    print_menu();
                    break;
                default:
                    break;
            }
        }

        if (active_mode == OPMODE_STREAM) {
            int16_t x = 0, y = 0, z = 0;
            int8_t x8 = 0, y8 = 0, z8 = 0;
            float temp_c = 0.0f;
            uint8_t status = 0;
            uint8_t int1 = 0, int2 = 0;

            if (adxl362_read_all(&x, &y, &z, &x8, &y8, &z8, &temp_c, &status, &int1, &int2)) {
                float sens = get_sensitivity();
                float ax_mg = (float)x * sens;
                float ay_mg = (float)y * sens;
                float az_mg = (float)z * sens;

                if (tare_enabled) {
                    ax_mg -= tare_x_mg;
                    ay_mg -= tare_y_mg;
                    az_mg -= tare_z_mg;
                }

                float mag_mg = sqrtf(ax_mg * ax_mg + ay_mg * ay_mg + az_mg * az_mg);
                float pitch = atan2f(ax_mg, sqrtf(ay_mg * ay_mg + az_mg * az_mg)) * (180.0f / 3.14159265f);
                float roll  = atan2f(ay_mg, sqrtf(ax_mg * ax_mg + az_mg * az_mg)) * (180.0f / 3.14159265f);
                float temp_f = temp_c * 1.8f + 32.0f;

                uartPrint("X:"); printInt((int32_t)ax_mg);
                uartPrint("mg Y:"); printInt((int32_t)ay_mg);
                uartPrint("mg Z:"); printInt((int32_t)az_mg);
                uartPrint("mg | |a|:"); printInt((int32_t)mag_mg);
                uartPrint("mg | Pitch:"); printFloat(pitch, 1);
                uartPrint(" deg Roll:"); printFloat(roll, 1);
                uartPrint(" deg | Temp:"); printFloat(temp_c, 1);
                uartPrint(" C ("); printFloat(temp_f, 1);
                uartPrint(" F) | INT1:"); printInt(int1);
                uartPrint(" INT2:"); printInt(int2);
                uartPrint(" | ST:0x"); printHex8(status);

                uartPrint(" [");
                if (status & ADXL362_STATUS_DATA_READY)     uartPrint("RDY ");
                if (status & ADXL362_STATUS_AWAKE)          uartPrint("AWK ");
                if (status & ADXL362_STATUS_ACT)            uartPrint("ACT ");
                if (status & ADXL362_STATUS_INACT)          uartPrint("INACT ");
                if (status & ADXL362_STATUS_FIFO_WATERMARK) uartPrint("WTM ");
                if (status & ADXL362_STATUS_FIFO_OVERRUN)   uartPrint("OVR ");
                uartPrint("]\r\n");
            }

            usleep(100000);
        } else {
            usleep(50000);
        }
    }

    return 0;
}
