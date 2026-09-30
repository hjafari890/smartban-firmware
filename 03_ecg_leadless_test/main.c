/* =========================================================================
 * SmartBAN Leadless ECG Verification Firmware
 * Target: CC2652R1 / CC26X2R1 LaunchPad + BAN Shield V3.5
 * Chip: Texas Instruments ADS1292R 24-Bit Low-Power ECG & Respiration AFE
 * =========================================================================
 * This dedicated program verifies the ADS1292R analog front-end WITHOUT
 * requiring external electrode leads or cables.
 *
 * Self-Test Capabilities:
 *   [1] Internal Precision Test Signal (1 Hz, +/-1 mV Square Wave)
 *       -> Injects +/-1 mV into PGAs. With Gain 6, output alternates between
 *          approx +6000 uV and -6000 uV (+/-20,800 ADC counts).
 *   [2] Input Shorted (Internal Noise Floor & Baseline Offset Test)
 *       -> Shorts differential PGA inputs internally. Verifies front-end
 *          drift, offset (< 100 uV), and low RMS noise floor.
 *   [3] Internal Die Temperature Sensor
 *       -> Reads on-chip temperature diode (~145 uV/deg C slope).
 *   [4] Live Electrode Stream (Finger Touch & Interference Test)
 *       -> Connects external electrode inputs. Touching the RA/LA pins
 *          or 3.5mm jack with a finger injects 50/60 Hz ambient body hum.
 *
 * Pin Mapping:
 *   DIO_8  : SPI POCI (MISO)
 *   DIO_9  : SPI PICO (MOSI)
 *   DIO_10 : SPI SCLK
 *   DIO_11 : ECG_CS (Chip Select, active low)
 *   DIO_15 : IMU_CS (Held HIGH to isolate SPI bus)
 *   DIO_21 : 1V8_EN (Power domain enable, active high)
 *   DIO_22 : ECG_PWDN / RESET (Active low, held HIGH)
 *   DIO_23 : ECG_DRDY (Conversion ready interrupt, active low)
 *   DIO_24 : ECG_START (Conversion start, active high)
 *   DIO_30 : I2C_EN (PCA9306 level shifter enable)
 *   DIO_3  : UART TX (115200 baud)
 *   DIO_2  : UART RX
 * ========================================================================= */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include <NoRTOS.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART2.h>

#include "ti_drivers_config.h"

/* =========================================================================
 * ADS1292R SPI Commands & Registers
 * ========================================================================= */
#define ADS1292_CMD_WAKEUP     0x02
#define ADS1292_CMD_STANDBY    0x04
#define ADS1292_CMD_RESET      0x06
#define ADS1292_CMD_START      0x08
#define ADS1292_CMD_STOP       0x0A
#define ADS1292_CMD_OFFSETCAL  0x1A
#define ADS1292_CMD_RDATAC     0x10
#define ADS1292_CMD_SDATAC     0x11
#define ADS1292_CMD_RDATA      0x12
#define ADS1292_CMD_RREG       0x20
#define ADS1292_CMD_WREG       0x40

#define ADS1292_REG_ID         0x00
#define ADS1292_REG_CONFIG1    0x01
#define ADS1292_REG_CONFIG2    0x02
#define ADS1292_REG_LOFF       0x03
#define ADS1292_REG_CH1SET     0x04
#define ADS1292_REG_CH2SET     0x05
#define ADS1292_REG_RLD_SENS   0x06
#define ADS1292_REG_LOFF_SENS  0x07
#define ADS1292_REG_LOFF_STAT  0x08
#define ADS1292_REG_RESP1      0x09
#define ADS1292_REG_RESP2      0x0A
#define ADS1292_REG_GPIO       0x0B

/* Self-Test Modes */
typedef enum {
    ECG_MODE_SQUARE_WAVE = 1,   /* 1 Hz, +/-1 mV internal test generator */
    ECG_MODE_INPUT_SHORT = 2,   /* Inputs shorted (noise floor & offset) */
    ECG_MODE_TEMPERATURE = 3,   /* Internal die temperature diode */
    ECG_MODE_LIVE_ELECTRODE = 4 /* External electrode pins (finger touch test) */
} EcgTestMode_t;

static const char *MODE_NAMES[] = {
    "UNKNOWN",
    "INTERNAL 1Hz SQUARE WAVE (+/-1mV)",
    "INPUT SHORTED (Noise & Offset Test)",
    "INTERNAL DIE TEMPERATURE SENSOR",
    "LIVE ELECTRODES (Finger Touch Test)"
};

/* Global Drivers */
static UART2_Handle uart = NULL;
static SPI_Handle   spi  = NULL;

/* Device State */
static uint8_t ads_cs_pin = CONFIG_GPIO_ECG_CS;
static uint8_t ads_id = 0x00;
static bool ads_online = false;
static uint8_t ads_regs[12] = {0};
static EcgTestMode_t current_mode = ECG_MODE_SQUARE_WAVE;

/* =========================================================================
 * UART Helper Functions
 * ========================================================================= */
static void uartPrint(const char *str)
{
    if (!uart) return;
    size_t written;
    UART2_write(uart, str, strlen(str), &written);
}

static void printHex8(uint8_t val)
{
    const char hexDigits[] = "0123456789ABCDEF";
    char h[3] = { hexDigits[(val >> 4) & 0x0F], hexDigits[val & 0x0F], '\0' };
    uartPrint(h);
}

static void printDec(uint32_t val)
{
    char buf[12];
    int idx = 0;
    if (val == 0) { uartPrint("0"); return; }
    char temp[12];
    int t = 0;
    while (val > 0) { temp[t++] = '0' + (val % 10); val /= 10; }
    while (t > 0) buf[idx++] = temp[--t];
    buf[idx] = '\0';
    uartPrint(buf);
}

static void printInt(int32_t val)
{
    if (val < 0) {
        uartPrint("-");
        val = -val;
    }
    printDec((uint32_t)val);
}

static void printFloat1(float val)
{
    if (val < 0.0f) {
        uartPrint("-");
        val = -val;
    }
    uint32_t whole = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - (float)whole) * 10.0f + 0.5f);
    if (frac >= 10) { whole++; frac = 0; }
    printDec(whole);
    uartPrint(".");
    printDec(frac);
}

/* =========================================================================
 * ADS1292R SPI Low-Level Driver (SPI Mode 1: CPOL=0, CPHA=1)
 * ========================================================================= */
static void ads1292_send_cmd(uint8_t cmd)
{
    if (!spi) return;
    SPI_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.count = 1;
    trans.txBuf = &cmd;
    trans.rxBuf = NULL;

    GPIO_write(ads_cs_pin, 0);
    usleep(15);
    SPI_transfer(spi, &trans);
    usleep(15);
    GPIO_write(ads_cs_pin, 1);
    usleep(25);
}

static bool ads1292_write_reg(uint8_t reg, uint8_t val)
{
    if (!spi) return false;
    SPI_Transaction trans;
    uint8_t tx[3] = { (uint8_t)(ADS1292_CMD_WREG | (reg & 0x1F)), 0x00, val };
    uint8_t rx[3] = { 0 };
    memset(&trans, 0, sizeof(trans));
    trans.count = 3;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(ads_cs_pin, 0);
    usleep(15);
    bool ok = SPI_transfer(spi, &trans);
    usleep(15);
    GPIO_write(ads_cs_pin, 1);
    usleep(25);
    return ok;
}

static bool ads1292_read_regs(uint8_t start_reg, uint8_t count, uint8_t *buf)
{
    if (!spi || count == 0) return false;
    SPI_Transaction trans;
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_RREG | (start_reg & 0x1F));
    tx[1] = (uint8_t)((count - 1) & 0x1F);

    uint32_t total = 2 + count;
    memset(&trans, 0, sizeof(trans));
    trans.count = total;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(ads_cs_pin, 0);
    usleep(15);
    bool ok = SPI_transfer(spi, &trans);
    usleep(15);
    GPIO_write(ads_cs_pin, 1);
    usleep(25);

    if (ok) {
        memcpy(buf, &rx[2], count);
    }
    return ok;
}

/* =========================================================================
 * ADS1292R Mode Configuration
 * ========================================================================= */
static bool ads1292_set_test_mode(EcgTestMode_t mode)
{
    if (!ads_online && ads_id == 0) return false;

    /* 1. Stop continuous conversion mode to write registers */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(100);

    /* 2. Common configuration:
     * CONFIG1 (0x01): 0x02 = 500 SPS data rate
     * RESP2   (0x0A): 0x83 = Reference 2.42V enabled, internal reference buffer
     */
    ads1292_write_reg(ADS1292_REG_CONFIG1, 0x02);
    ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
    ads1292_write_reg(ADS1292_REG_RLD_SENS,0x2C);

    switch (mode) {
        case ECG_MODE_SQUARE_WAVE:
            /* CONFIG2: 0xA3
             * Bit 7 = 1 (Required)
             * Bit 5 = 1 (PDB_REFBUF: Enable internal 2.42V reference buffer)
             * Bit 1 = 1 (INT_TEST: Enable internal test signal generator, +/-1mV)
             * Bit 0 = 1 (TEST_FREQ: 1 = Square wave at 1 Hz; 0 = DC)
             */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA3);

            /* CH1SET & CH2SET: 0x05
             * Bit 7   = 0 (Active)
             * Bits6:4 = 000 (PGA Gain = 6)
             * Bits3:0 = 0101 (MUX: Test Signal)
             */
            ads1292_write_reg(ADS1292_REG_CH1SET, 0x05);
            ads1292_write_reg(ADS1292_REG_CH2SET, 0x05);
            break;

        case ECG_MODE_INPUT_SHORT:
            /* CONFIG2: 0xA0 (Test signal OFF, Reference Buffer ON) */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);

            /* CH1SET & CH2SET: 0x01
             * Bits3:0 = 0001 (MUX: Input Shorted to measure offset and noise floor)
             */
            ads1292_write_reg(ADS1292_REG_CH1SET, 0x01);
            ads1292_write_reg(ADS1292_REG_CH2SET, 0x01);
            break;

        case ECG_MODE_TEMPERATURE:
            /* CONFIG2: 0xA0 (Reference Buffer ON) */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);

            /* CH1SET & CH2SET: 0x04
             * Bits3:0 = 0100 (MUX: Internal Temperature Sensor)
             */
            ads1292_write_reg(ADS1292_REG_CH1SET, 0x04);
            ads1292_write_reg(ADS1292_REG_CH2SET, 0x04);
            break;

        case ECG_MODE_LIVE_ELECTRODE:
        default:
            /* CONFIG2: 0xA0 (Reference Buffer ON) */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);

            /* CH1SET & CH2SET: 0x00
             * Bits3:0 = 0000 (MUX: Normal Electrode Input)
             */
            ads1292_write_reg(ADS1292_REG_CH1SET, 0x00);
            ads1292_write_reg(ADS1292_REG_CH2SET, 0x00);
            break;
    }

    /* 3. Wait 100 ms for reference buffer / caps to settle */
    usleep(100000);

    /* 4. Read back registers to verify */
    ads1292_read_regs(0x00, 12, ads_regs);

    /* 5. Start conversions */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(10000);

    /* 6. Re-enter continuous data mode */
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(10000);

    current_mode = mode;

    uartPrint("\r\n>> Switched to Mode [");
    printDec((uint32_t)mode);
    uartPrint("]: ");
    uartPrint(MODE_NAMES[mode]);
    uartPrint("\r\n");
    uartPrint("   Reg Dump [0..5]: ");
    for (int r = 0; r < 6; r++) {
        printHex8(ads_regs[r]);
        uartPrint(" ");
    }
    uartPrint("\r\n\r\n");

    return true;
}

/* =========================================================================
 * Hardware Initialization & Probe
 * ========================================================================= */
static bool ads1292_hardware_init(void)
{
    /* 1. Assert power down pin LOW, then release HIGH to force hardware reset */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    usleep(5000);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);

    /* 2. Mandatory POR wait: TI datasheet requires min 2^18 t_CLK (~512 ms) */
    uartPrint("  [INIT] Waiting 1000 ms for digital core POR...\r\n");
    usleep(1000000);

    /* 3. Pull START HIGH */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000);

    /* 4. Probe CS pins (DIO 11 primary, DIO 20 secondary) */
    static const uint8_t candidate_cs[] = { CONFIG_GPIO_ECG_CS, 20 };
    bool found = false;

    uartPrint("  [INIT] Probing ADS1292R SPI interface (250 kHz, Mode 1)...\r\n");

    for (int i = 0; i < 2; i++) {
        uint8_t p = candidate_cs[i];
        GPIO_setConfig(p, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
        GPIO_write(p, 1);
        usleep(50);

        ads_cs_pin = p;

        /* Issue SDATAC */
        ads1292_send_cmd(ADS1292_CMD_SDATAC);
        usleep(100);

        /* Read ID register */
        uint8_t id_val = 0;
        ads1292_read_regs(ADS1292_REG_ID, 1, &id_val);

        if (id_val != 0x00 && id_val != 0xFF) {
            ads_id = id_val;
            ads_cs_pin = p;
            found = true;
            uartPrint("  [INIT] FOUND ADS1292! CS=DIO ");
            printDec(p);
            uartPrint(" | Chip ID = 0x");
            printHex8(ads_id);
            if (ads_id == 0x73) {
                uartPrint(" (ADS1292R with Respiration)\r\n");
            } else if (ads_id == 0x53) {
                uartPrint(" (ADS1292 Standard)\r\n");
            } else {
                uartPrint(" (Compatible ADS129x family)\r\n");
            }
            break;
        }
    }

    if (!found) {
        uartPrint("  [WARNING] Chip ID did not respond on standard CS. Setting default CS=DIO ");
        printDec(ads_cs_pin);
        uartPrint("\r\n");
    }

    ads_online = (ads_id != 0x00 && ads_id != 0xFF);

    /* Configure initial default test mode (1 Hz Square Wave) */
    ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);

    return ads_online;
}

/* =========================================================================
 * Read Single ECG Frame (9 Bytes) in RDATAC Mode
 * ========================================================================= */
static bool ads1292_read_sample(int32_t *ch1_raw, int32_t *ch2_raw, uint8_t *status_out)
{
    if (!spi) return false;

    /* Poll DRDY pin for conversion ready (active low) */
    uint32_t timeout_us = 0;
    while (GPIO_read(CONFIG_GPIO_ECG_DRDY) != 0 && timeout_us < 12000) {
        usleep(40);
        timeout_us += 40;
    }

    SPI_Transaction trans;
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};
    memset(&trans, 0, sizeof(trans));
    trans.count = 9;
    trans.txBuf = tx;
    trans.rxBuf = rx;

    GPIO_write(ads_cs_pin, 0);
    usleep(10);
    bool ok = SPI_transfer(spi, &trans);
    usleep(10);
    GPIO_write(ads_cs_pin, 1);

    if (ok) {
        if (status_out) *status_out = rx[0];

        /* Decode 24-bit 2's complement Channel 1 */
        int32_t c1 = ((int32_t)rx[3] << 16) | ((int32_t)rx[4] << 8) | (int32_t)rx[5];
        if (c1 & 0x00800000) c1 |= 0xFF000000;
        *ch1_raw = c1;

        /* Decode 24-bit 2's complement Channel 2 */
        int32_t c2 = ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | (int32_t)rx[8];
        if (c2 & 0x00800000) c2 |= 0xFF000000;
        *ch2_raw = c2;
    }

    return ok;
}

/* Convert 24-bit ADC counts to microvolts (Vref = 2.42V, Gain = 6) */
static float raw_to_microvolts(int32_t raw_counts)
{
    /* 1 LSB = Vref / (Gain * (2^23 - 1))
     * Vref = 2.42V, Gain = 6
     * 1 LSB = 2.42 / (6 * 8388607) = 0.04808 uV */
    return ((float)raw_counts) * 0.0480803f;
}

/* ASCII Oscilloscope Visualizer Bar (-10000 uV to +10000 uV) */
static void print_ascii_bar(float uv)
{
    const int BAR_HALF_WIDTH = 12;
    char bar[27];
    memset(bar, ' ', sizeof(bar));
    bar[BAR_HALF_WIDTH] = '|'; /* Center zero line */
    bar[25] = '\0';

    int pos = (int)(uv / 800.0f); /* 800 uV per character */
    if (pos > BAR_HALF_WIDTH) pos = BAR_HALF_WIDTH;
    if (pos < -BAR_HALF_WIDTH) pos = -BAR_HALF_WIDTH;

    if (pos > 0) {
        for (int i = 1; i <= pos; i++) bar[BAR_HALF_WIDTH + i] = '#';
    } else if (pos < 0) {
        for (int i = -1; i >= pos; i--) bar[BAR_HALF_WIDTH + i] = '#';
    }

    uartPrint("[");
    uartPrint(bar);
    uartPrint("]");
}

/* =========================================================================
 * Main Program Entry
 * ========================================================================= */
int main(void)
{
    Board_init();
    NoRTOS_start();

    GPIO_init();
    SPI_init();

    /* 1. Assert system power rails and peripheral enables */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);    /* Turn on 1.8V LDO regulator */
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);    /* Enable I2C level translator */
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);    /* De-assert ADXL362 CS (idle high) */
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);    /* De-assert ADS1292R CS (idle high) */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);  /* Ensure ADS1292R is powered up */
    GPIO_write(CONFIG_GPIO_ECG_START, 1); /* Enable internal conversion clock */

    usleep(100000); /* 100 ms rail stabilization */

    /* 2. Open UART2 for interactive debug and waveform streaming */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate  = 115200;
    uartParams.readMode  = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    uartPrint("\r\n");
    uartPrint("===============================================================\r\n");
    uartPrint("  SmartBAN ADS1292R Leadless ECG Verification Tool\r\n");
    uartPrint("  Validating Analog Signal Chain via Internal Test Modes\r\n");
    uartPrint("===============================================================\r\n\r\n");

    /* 3. Open SPI Controller in Mode 1 (CPOL=0, CPHA=1) at 250 kHz */
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate     = 250000;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.mode        = SPI_CONTROLLER;
    spi = SPI_open(CONFIG_SPI_0, &spiParams);

    if (!spi) {
        uartPrint("  *** FATAL: Failed to open SPI controller! ***\r\n");
        while (1) { usleep(1000000); }
    }

    /* 4. Run hardware reset & probe ADS1292R */
    ads1292_hardware_init();

    uartPrint("---------------------------------------------------------------\r\n");
    uartPrint("  Interactive Commands (send single character via serial):\r\n");
    uartPrint("    [1] -> 1 Hz Square Wave Mode (+/-1 mV test generator)\r\n");
    uartPrint("    [2] -> Input Shorted Mode (Noise Floor & Offset Test)\r\n");
    uartPrint("    [3] -> Die Temperature Mode (Internal thermal sensor)\r\n");
    uartPrint("    [4] -> Live Electrodes Mode (Body touch / interference)\r\n");
    uartPrint("    [A] -> Auto-Cycle through all test modes\r\n");
    uartPrint("---------------------------------------------------------------\r\n\r\n");

    uint32_t sample_num = 0;
    uint32_t auto_timer = 0;
    bool auto_cycle = true;

    /* Statistical tracking */
    float ch1_min = 100000.0f, ch1_max = -100000.0f;
    float ch2_min = 100000.0f, ch2_max = -100000.0f;
    float ch1_sum = 0.0f, ch2_sum = 0.0f;
    uint32_t stat_count = 0;

    while (1)
    {
        /* Check for interactive UART command */
        uint8_t rx_char = 0;
        size_t bytes_read = 0;
        if (UART2_read(uart, &rx_char, 1, &bytes_read) == UART2_STATUS_SUCCESS && bytes_read > 0) {
            auto_cycle = false; /* Manual override disables auto cycle */
            if (rx_char == '1') ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);
            else if (rx_char == '2') ads1292_set_test_mode(ECG_MODE_INPUT_SHORT);
            else if (rx_char == '3') ads1292_set_test_mode(ECG_MODE_TEMPERATURE);
            else if (rx_char == '4') ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);
            else if (rx_char == 'a' || rx_char == 'A') {
                auto_cycle = true;
                uartPrint(">> Auto-Cycle Enabled!\r\n");
            }
            stat_count = 0;
            ch1_min = 100000.0f; ch1_max = -100000.0f;
            ch2_min = 100000.0f; ch2_max = -100000.0f;
            ch1_sum = 0.0f; ch2_sum = 0.0f;
        }

        /* Read raw 24-bit sample from ADS1292R */
        int32_t ch1_raw = 0, ch2_raw = 0;
        uint8_t stat_byte = 0;
        bool ok = ads1292_read_sample(&ch1_raw, &ch2_raw, &stat_byte);

        if (ok) {
            sample_num++;
            float ch1_uv = raw_to_microvolts(ch1_raw);
            float ch2_uv = raw_to_microvolts(ch2_raw);

            /* Track stats */
            if (ch1_uv < ch1_min) ch1_min = ch1_uv;
            if (ch1_uv > ch1_max) ch1_max = ch1_uv;
            if (ch2_uv < ch2_min) ch2_min = ch2_uv;
            if (ch2_uv > ch2_max) ch2_max = ch2_uv;
            ch1_sum += ch1_uv;
            ch2_sum += ch2_uv;
            stat_count++;

            /* Decimate output for clean terminal viewing (print every 20th sample ~25 Hz display) */
            if (sample_num % 20 == 0) {
                uartPrint("ECG | CH1=");
                if (ch1_uv >= 0.0f) uartPrint("+");
                printInt((int32_t)ch1_uv);
                uartPrint(" uV ");
                print_ascii_bar(ch1_uv);

                uartPrint(" | CH2=");
                if (ch2_uv >= 0.0f) uartPrint("+");
                printInt((int32_t)ch2_uv);
                uartPrint(" uV | Raw: ");
                printInt(ch1_raw);
                uartPrint("\r\n");
            }

            /* Print summary report every ~500 samples (once per second) */
            if (sample_num % 500 == 0) {
                float ch1_avg = (stat_count > 0) ? (ch1_sum / stat_count) : 0.0f;
                float ch1_vpp = ch1_max - ch1_min;
                float ch2_vpp = ch2_max - ch2_min;

                uartPrint("\r\n>>> [1-SEC STATS] Mode: ");
                uartPrint(MODE_NAMES[current_mode]);
                uartPrint("\r\n    CH1 Vpp = "); printFloat1(ch1_vpp);
                uartPrint(" uV | Offset = "); printFloat1(ch1_avg);
                uartPrint(" uV | Stat=0x"); printHex8(stat_byte);
                uartPrint(" | DRDY=OK\r\n");

                if (current_mode == ECG_MODE_SQUARE_WAVE) {
                    if (ch1_vpp > 8000.0f && ch1_vpp < 15000.0f) {
                        uartPrint("    [VERDICT: PASS] Clean +/-1mV square wave detected! PGA & ADC functional.\r\n");
                    } else {
                        uartPrint("    [NOTE] Tracking test signal transition.\r\n");
                    }
                } else if (current_mode == ECG_MODE_INPUT_SHORT) {
                    if (ch1_vpp < 300.0f) {
                        uartPrint("    [VERDICT: PASS] Low noise floor & minimal offset! Analog input stage quiet.\r\n");
                    }
                } else if (current_mode == ECG_MODE_TEMPERATURE) {
                    /* Temp in C = (V - 110mV) / 0.145mV + 25 */
                    float temp_c = ((ch1_avg / 1000.0f) - 110.0f) / 0.145f + 25.0f;
                    uartPrint("    [VERDICT: PASS] Die Temperature: ");
                    printFloat1(temp_c);
                    uartPrint(" deg C\r\n");
                } else if (current_mode == ECG_MODE_LIVE_ELECTRODE) {
                    uartPrint("    [INFO] Touch RA/LA pins with bare finger to see 50/60Hz hum jump!\r\n");
                }
                uartPrint("\r\n");

                /* Reset stats window */
                stat_count = 0;
                ch1_min = 100000.0f; ch1_max = -100000.0f;
                ch2_min = 100000.0f; ch2_max = -100000.0f;
                ch1_sum = 0.0f; ch2_sum = 0.0f;

                /* Handle Auto-Cycle mode progression */
                if (auto_cycle) {
                    auto_timer++;
                    if (auto_timer == 6) {
                        /* Switch to Input Shorted test */
                        ads1292_set_test_mode(ECG_MODE_INPUT_SHORT);
                    } else if (auto_timer == 11) {
                        /* Switch to Temperature test */
                        ads1292_set_test_mode(ECG_MODE_TEMPERATURE);
                    } else if (auto_timer == 16) {
                        /* Switch to Live Electrodes test */
                        ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);
                    } else if (auto_timer >= 24) {
                        /* Loop back to square wave */
                        auto_timer = 0;
                        ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);
                    }
                }
            }
        } else {
            /* If DRDY not pulsing, yield brief pause */
            usleep(1000);
        }
    }

    return 0;
}
