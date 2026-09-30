/* =========================================================================
 * SmartBAN ECG & Respiration Dedicated Firmware
 * Target: CC2652R1 / CC26X2R1 LaunchPad + BAN Shield V3.5
 * Chip:   Texas Instruments ADS1292R 24-Bit Low-Power ECG & Respiration AFE
 * =========================================================================
 * Dedicated bare-metal firmware focused exclusively on:
 *   1. Heart Rate (BPM) via on-chip Pan-Tompkins QRS detection
 *   2. ECG Waveform streaming for PC GUI
 *   3. Respiration Rate (RPM) via impedance pneumography
 *
 * Channel Assignment:
 *   Channel 1 (CH1): Respiration impedance pneumography (Gain=4, 32kHz carrier)
 *   Channel 2 (CH2): ECG Lead I (Gain=6, IN2P=LA, IN2N=RA)
 *
 * Pin Mapping (BAN Shield V3.5):
 *   DIO 8  : SPI MISO (DOUT)        DIO 22 : ECG PWDN/RESET
 *   DIO 9  : SPI MOSI (DIN)         DIO 23 : ECG DRDY (active LOW)
 *   DIO 10 : SPI SCLK               DIO 24 : ECG START
 *   DIO 11 : ECG CS (active LOW)     DIO 6  : Red LED (lead-off)
 *   DIO 15 : IMU CS (held HIGH)      DIO 7  : Green LED (heartbeat)
 *   DIO 21 : 1V8 LDO enable          DIO 3  : UART TX (115200)
 *   DIO 30 : I2C level shift enable  DIO 2  : UART RX
 *
 * UART Protocol (115200 baud, 8N1):
 *   Boot:    #ECG_DEDICATED_V1,250,<chip_id_hex>\r\n
 *   Data:    D,<ts_ms>,<ch1>,<ch2>,<status>\r\n   (250 Hz)
 *   Summary: S,<bpm>,<rr_ms>,<resp_rpm>,<loff>\r\n  (every 4 sec)
 * ========================================================================= */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <NoRTOS.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>

#include "ti_drivers_config.h"
#include "edgeai_ecg.h"

/* =========================================================================
 * WCH CH455H 7-Segment & LED Driver I2C Command Addresses
 * ========================================================================= */
#define CH455_CMD_SYS       0x24    /* System command (0x71 = Display ON, 100% brightness) */
#define CH455_DIG0_ADDR     0x34    /* Hundreds digit */
#define CH455_DIG1_ADDR     0x35    /* Tens digit */
#define CH455_DIG2_ADDR     0x36    /* Ones digit (Bit 7 = Decimal Point) */
#define CH455_DIG3_ADDR     0x37    /* 4 Discrete Mode LEDs (Bits 0..3) */

/* =========================================================================
 * ADS1292R SPI Commands
 * ========================================================================= */
#define ADS_CMD_WAKEUP      0x02
#define ADS_CMD_STANDBY     0x04
#define ADS_CMD_RESET       0x06
#define ADS_CMD_START       0x08
#define ADS_CMD_STOP        0x0A
#define ADS_CMD_OFFSETCAL   0x1A
#define ADS_CMD_RDATAC      0x10
#define ADS_CMD_SDATAC      0x11
#define ADS_CMD_RDATA       0x12
#define ADS_CMD_RREG        0x20
#define ADS_CMD_WREG        0x40

/* ADS1292R Register Addresses */
#define REG_ID              0x00
#define REG_CONFIG1         0x01
#define REG_CONFIG2         0x02
#define REG_LOFF            0x03
#define REG_CH1SET          0x04
#define REG_CH2SET          0x05
#define REG_RLD_SENS        0x06
#define REG_LOFF_SENS       0x07
#define REG_LOFF_STAT       0x08
#define REG_RESP1           0x09
#define REG_RESP2           0x0A
#define REG_GPIO_REG        0x0B

/* =========================================================================
 * Configuration Constants
 * ========================================================================= */
#define ECG_SAMPLE_RATE     250     /* Hz (CONFIG1 = 0x01) */
#define ECG_VREF_UV         2420000 /* 2.42V in microvolts */
#define ECG_GAIN_CH1        4       /* Respiration channel */
#define ECG_GAIN_CH2        6       /* ECG channel */
#define ADC_FULL_SCALE      8388607 /* 2^23 - 1 */

/* Pan-Tompkins parameters for 250 Hz */
#define PT_DERIV_LEN        5       /* 5-point derivative buffer */
#define PT_MWI_LEN          38      /* ~150 ms moving window at 250 Hz */
#define PT_REFRACTORY       50      /* 200 ms refractory = 50 samples */
#define PT_SEARCHBACK_MULT  166     /* 166% of expected RR for searchback */
#define PT_RR_BUF_LEN       8      /* RR interval history */

/* Respiration estimator */
#define RESP_WINDOW_SEC     15      /* 15-second window for resp rate */
#define RESP_WINDOW_SAMPLES (ECG_SAMPLE_RATE * RESP_WINDOW_SEC) /* 3750 */
#define RESP_LP_SHIFT       6       /* Low-pass IIR: alpha ~ 1/64 */

/* Timing */
#define SUMMARY_INTERVAL    (ECG_SAMPLE_RATE)     /* Every 1 second = 250 samples */

/* =========================================================================
 * Global Driver Handles
 * ========================================================================= */
static UART2_Handle uart = NULL;
static SPI_Handle   spi  = NULL;
static I2C_Handle   i2c  = NULL;
static bool         ch455_online = false;
static uint16_t     ecg_beat_counter = 0;
static uint8_t      beat_flash_timer = 0;

/* =========================================================================
 * UART Helpers (no printf/sprintf for minimal code size)
 * ========================================================================= */
static void uart_print(const char *str)
{
    if (!uart) return;
    size_t written;
    UART2_write(uart, str, strlen(str), &written);
}

static void uart_write_bytes(const char *buf, size_t len)
{
    if (!uart) return;
    size_t written;
    UART2_write(uart, buf, len, &written);
}

static char int_buf[16];

static void print_int(int32_t val)
{
    int idx = 0;
    if (val < 0) {
        uart_print("-");
        /* Handle INT32_MIN */
        if (val == -2147483648LL) {
            uart_print("2147483648");
            return;
        }
        val = -val;
    }
    if (val == 0) { uart_print("0"); return; }
    char tmp[12];
    int t = 0;
    while (val > 0) { tmp[t++] = '0' + (val % 10); val /= 10; }
    while (t > 0) int_buf[idx++] = tmp[--t];
    int_buf[idx] = '\0';
    uart_print(int_buf);
}

static void print_uint(uint32_t val)
{
    if (val == 0) { uart_print("0"); return; }
    int idx = 0;
    char tmp[12];
    int t = 0;
    while (val > 0) { tmp[t++] = '0' + (val % 10); val /= 10; }
    while (t > 0) int_buf[idx++] = tmp[--t];
    int_buf[idx] = '\0';
    uart_print(int_buf);
}

static void print_hex8(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    char buf[3] = { hex[(val >> 4) & 0xF], hex[val & 0xF], '\0' };
    uart_print(buf);
}

/* =========================================================================
 * ADS1292R SPI Low-Level Driver (Mode 1: CPOL=0, CPHA=1, 250 kHz)
 * ========================================================================= */
static void ads_cs_low(void)
{
    GPIO_write(CONFIG_GPIO_ECG_CS, 0);
    usleep(10);
}

static void ads_cs_high(void)
{
    usleep(10);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    usleep(10);
}

static void ads_cmd(uint8_t cmd)
{
    if (!spi) return;
    SPI_Transaction t;
    memset(&t, 0, sizeof(t));
    t.count = 1;
    t.txBuf = &cmd;
    ads_cs_low();
    SPI_transfer(spi, &t);
    ads_cs_high();
}

static void ads_wreg(uint8_t reg, uint8_t val)
{
    if (!spi) return;
    SPI_Transaction t;
    uint8_t tx[3] = { (uint8_t)(ADS_CMD_WREG | (reg & 0x1F)), 0x00, val };
    memset(&t, 0, sizeof(t));
    t.count = 3;
    t.txBuf = tx;
    ads_cs_low();
    SPI_transfer(spi, &t);
    ads_cs_high();
    usleep(10);
}

static uint8_t ads_rreg(uint8_t reg)
{
    if (!spi) return 0;
    SPI_Transaction t;
    uint8_t tx[3] = { (uint8_t)(ADS_CMD_RREG | (reg & 0x1F)), 0x00, 0x00 };
    uint8_t rx[3] = {0};
    memset(&t, 0, sizeof(t));
    t.count = 3;
    t.txBuf = tx;
    t.rxBuf = rx;
    ads_cs_low();
    SPI_transfer(spi, &t);
    ads_cs_high();
    usleep(10);
    return rx[2];
}

/* Write register and verify by readback. Returns true if match. */
static bool ads_wreg_verify(uint8_t reg, uint8_t val, const char *name)
{
    ads_wreg(reg, val);
    uint8_t rb = ads_rreg(reg);

    uart_print("  ");
    uart_print(name);
    uart_print(": wrote 0x");
    print_hex8(val);
    uart_print(", read 0x");
    print_hex8(rb);

    if (rb == val) {
        uart_print(" [OK]\r\n");
        return true;
    } else {
        uart_print(" [MISMATCH]\r\n");
        return false;
    }
}

/* =========================================================================
 * 24-bit Signed Parsing
 * ========================================================================= */
static inline int32_t parse_24bit(const uint8_t *b)
{
    int32_t val = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | (int32_t)b[2];
    if (val & 0x00800000) val |= 0xFF000000; /* Sign extend */
    return val;
}

/* Convert raw ADC counts to microvolts (integer math) */
static inline int32_t raw_to_uv(int32_t raw, uint8_t gain)
{
    /* uV = raw * Vref_uV / (gain * (2^23 - 1)) */
    return (int32_t)(((int64_t)raw * ECG_VREF_UV) / ((int64_t)gain * ADC_FULL_SCALE));
}

/* =========================================================================
 * Clinical Edge-AI Pan-Tompkins QRS Engine (Cortex-M4F Integer DSP)
 * ========================================================================= */
static edgeai_ecg_state_t ecg_state;
static edgeai_ecg_result_t ecg_latest_result;

/* =========================================================================
 * WCH CH455H 3-Digit 7-Segment Display & 4-LED Bar Driver (I2C0)
 * ========================================================================= */
static const uint8_t s_seg_font[10] = {
    0x3F, /* 0 */ 0x06, /* 1 */ 0x5B, /* 2 */ 0x4F, /* 3 */ 0x66, /* 4 */
    0x6D, /* 5 */ 0x7D, /* 6 */ 0x07, /* 7 */ 0x7F, /* 8 */ 0x6F  /* 9 */
};

static bool ch455_write(uint8_t addr, uint8_t data)
{
    if (!i2c) return false;
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.targetAddress = addr;
    trans.writeBuf = &data;
    trans.writeCount = 1;
    trans.readBuf = NULL;
    trans.readCount = 0;
    return (I2C_transferTimeout(i2c, &trans, 2000) == I2C_STATUS_SUCCESS);
}

static void ch455_update_bpm(uint16_t bpm_val, bool flash_on)
{
    if (!ch455_online) return;
    uint8_t seg0, seg1, seg2;
    if (bpm_val < 30U || bpm_val > 220U) {
        /* Waiting for rhythm lock: show '---' */
        seg0 = 0x40U | (flash_on ? 0x80U : 0x00U);
        seg1 = 0x40U | (flash_on ? 0x80U : 0x00U);
        seg2 = 0x40U | (flash_on ? 0x80U : 0x00U);
    } else {
        uint16_t val = bpm_val % 1000U;
        uint8_t h = (uint8_t)(val / 100U);
        uint8_t t = (uint8_t)((val % 100U) / 10U);
        uint8_t o = (uint8_t)(val % 10U);

        /* Blank leading hundreds digit when BPM < 100 for clean clinical readout (e.g. ' 72') */
        seg0 = (h > 0U ? s_seg_font[h] : 0x00U) | (flash_on ? 0x80U : 0x00U);
        seg1 = s_seg_font[t] | (flash_on ? 0x80U : 0x00U);
        seg2 = s_seg_font[o] | (flash_on ? 0x80U : 0x00U);
    }

    ch455_write(CH455_DIG0_ADDR, seg0);
    ch455_write(CH455_DIG1_ADDR, seg1);
    ch455_write(CH455_DIG2_ADDR, seg2);
    /* Flash all 4 discrete LEDs on beat (0x0F), or keep LED0 active (0x01) */
    ch455_write(CH455_DIG3_ADDR, flash_on ? 0x0FU : 0x01U);
}

static void ch455_init(void)
{
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    usleep(5000);
    I2C_init();
    I2C_Params ip;
    I2C_Params_init(&ip);
    ip.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_0, &ip);
    if (i2c && ch455_write(CH455_CMD_SYS, 0x71U)) {
        ch455_online = true;
        ch455_update_bpm(0, false);
    }
}

/* =========================================================================
 * Dual-Source Respiration Rate Estimator (CH1 Thoracic + CH2 EDR Baseline)
 * =========================================================================
 * Extracts the 0.12 - 0.42 Hz (7 to 25 breaths/min) respiratory baseline
 * envelope from both CH1 and CH2 and measures breath-to-breath intervals.
 * ========================================================================= */
typedef struct {
    int64_t  dc_fast;           /* ~0.38 Hz low-pass state (Q8) */
    int64_t  dc_slow;           /* ~0.08 Hz baseline state (Q8) */
    bool     primed;
    bool     in_inhale;
    uint32_t samples_since_breath;
    uint16_t breath_intervals[4];
    uint8_t  breath_idx;
    uint8_t  breath_count;
    uint16_t rpm;               /* Breaths per minute (7..30 RPM) */
} resp_estimator_t;

static resp_estimator_t resp;

static void resp_init(void)
{
    memset(&resp, 0, sizeof(resp));
    resp.primed = false;
}

static void resp_process(int32_t ch1_raw, int32_t ch2_raw)
{
    /* Combine CH1 thoracic potential + CH2 ECG-Derived Respiration (EDR) baseline */
    int64_t combined = ((int64_t)ch1_raw + ((int64_t)ch2_raw >> 1)) << 8;

    if (!resp.primed) {
        resp.dc_fast = combined;
        resp.dc_slow = combined;
        resp.primed = true;
        return;
    }

    /* Fast IIR pole: alpha = 1/128 -> fc ~ 0.31 Hz */
    resp.dc_fast += (combined - resp.dc_fast) >> 7;
    /* Slow IIR pole: alpha = 1/512 -> fc ~ 0.08 Hz */
    resp.dc_slow += (combined - resp.dc_slow) >> 9;

    /* Bandpass respiratory signal (0.08 - 0.35 Hz) */
    int32_t band = (int32_t)((resp.dc_fast - resp.dc_slow) >> 8);
    resp.samples_since_breath++;

    /* Hysteresis zero-crossing breath detector (debounce = 500 samples = 2.0s -> max 30 RPM) */
    if (!resp.in_inhale && band > 180) {
        resp.in_inhale = true;
        if (resp.samples_since_breath >= 500U && resp.samples_since_breath <= 2500U) {
            /* Valid breath cycle between 2.0 s (30 RPM) and 10.0 s (6 RPM) */
            resp.breath_intervals[resp.breath_idx] = (uint16_t)resp.samples_since_breath;
            resp.breath_idx = (resp.breath_idx + 1U) & 0x03U;
            if (resp.breath_count < 4U) resp.breath_count++;

            uint32_t sum = 0;
            for (uint8_t i = 0; i < resp.breath_count; i++) {
                sum += resp.breath_intervals[i];
            }
            uint32_t mean_samples = sum / resp.breath_count;
            if (mean_samples > 0) {
                uint16_t calc_rpm = (uint16_t)((60UL * ECG_SAMPLE_RATE) / mean_samples);
                if (calc_rpm >= 6U && calc_rpm <= 32U) {
                    resp.rpm = calc_rpm;
                }
            }
        }
        resp.samples_since_breath = 0;
    } else if (resp.in_inhale && band < -180) {
        resp.in_inhale = false;
    }
}

/* =========================================================================
 * ADS1292R Initialization Sequence
 * =========================================================================
 * Critical: follows TI datasheet precisely with proper timing
 * ========================================================================= */
static uint8_t chip_id = 0;

static bool ads_init(void)
{
    uart_print("\r\n[1/6] Powering up ADS1292R...\r\n");

    /* 1. Assert power rails */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);   /* Isolate ADXL362 */
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);   /* CS idle high */
    GPIO_write(CONFIG_GPIO_ECG_START, 0);/* START LOW during config! */
    usleep(100000); /* 100 ms rail stabilization */

    /* 2. Hardware reset: pulse PWDN low for 10 ms, then high */
    uart_print("[2/6] Hardware reset...\r\n");
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    usleep(10000); /* 10 ms reset pulse (< 4 ms = reset, > 4 ms = power down) */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);

    /* 3. Wait for POR (digital core + VCAP charge) — 1 second is safe */
    uart_print("  Waiting 1000 ms for POR + VCAP stabilization...\r\n");
    usleep(1000000);

    /* 4. SDATAC — device boots in RDATAC mode, must exit before reg access */
    ads_cmd(ADS_CMD_SDATAC);
    usleep(100);

    /* 5. Read chip ID */
    uart_print("[3/6] Reading chip ID...\r\n");
    chip_id = ads_rreg(REG_ID);
    uart_print("  Chip ID = 0x");
    print_hex8(chip_id);
    if (chip_id == 0x73) {
        uart_print(" (ADS1292R with Respiration) [OK]\r\n");
    } else if (chip_id == 0x53) {
        uart_print(" (ADS1292 Standard) [OK]\r\n");
    } else {
        uart_print(" (UNEXPECTED)\r\n");
    }

    if (chip_id == 0x00 || chip_id == 0xFF) {
        uart_print("  *** ERROR: ADS1292R not responding! ***\r\n");
        return false;
    }

    /* 6. Configure registers with verification */
    uart_print("[4/6] Configuring registers...\r\n");
    int ok = 0, total = 0;

    /* CONFIG2: Internal reference ON, lead-off comparators ON
     * Bit7=1(fixed) | Bit6=1(LOFF_COMP ON) | Bit5=1(REF_BUF ON) = 0xE0 */
    total++; if (ads_wreg_verify(REG_CONFIG2, 0xE0, "CONFIG2 ")) ok++;

    /* CRITICAL: Wait 100 ms for internal reference capacitor to settle */
    uart_print("  Waiting 100 ms for Vref settling...\r\n");
    usleep(100000);

    /* CONFIG1: 250 SPS continuous conversion (DR[2:0] = 001) */
    total++; if (ads_wreg_verify(REG_CONFIG1, 0x01, "CONFIG1 ")) ok++;

    /* LOFF: Lead-off at 95%/5% threshold, 6 nA DC current (0x10) */
    total++; if (ads_wreg_verify(REG_LOFF, 0x10, "LOFF    ")) ok++;

    /* CH1SET: Gain=4 (010), Normal input (0000) = 0x40
     * Channel 1 is dedicated to respiration / auxiliary biopotential. */
    total++; if (ads_wreg_verify(REG_CH1SET, 0x40, "CH1SET  ")) ok++;

    /* CH2SET: Gain=6 (000), Normal input (0000) = 0x00
     * Channel 2 is dedicated to ECG Lead I (IN2P=LA, IN2N=RA). */
    total++; if (ads_wreg_verify(REG_CH2SET, 0x00, "CH2SET  ")) ok++;

    /* RLD_SENS: RLD buffer ON, derive from CH2 (IN2P + IN2N)
     * Bits: CHOP=00(fmod/16) | PDB_RLD=1 | RLD_LOFF_SENS=0 | RLD2N=1 | RLD2P=1 | RLD1N=0 | RLD1P=0
     * = 0b00101100 = 0x2C */
    total++; if (ads_wreg_verify(REG_RLD_SENS, 0x2C, "RLD_SENS")) ok++;

    /* LOFF_SENS: Lead-off sensing on CH2 IN2P and IN2N
     * Bits: 00|FLIP2=0|FLIP1=0|LOFF2N=1|LOFF2P=1|LOFF1N=0|LOFF1P=0 = 0x0C */
    total++; if (ads_wreg_verify(REG_LOFF_SENS, 0x0C, "LOFF_SEN")) ok++;

    /* RESP1: Disable 32kHz carrier modulation (0x00) to prevent 17mVpp carrier
     * intermodulation noise on CH2 ECG and allow DC lead-off comparators to function */
    total++; if (ads_wreg_verify(REG_RESP1, 0x00, "RESP1   ")) ok++;

    /* RESP2: CALIB_ON=1 (for offset cal), RESP_FREQ=0(32kHz), RLDREF_INT=1 (Bit2=1), Bits1:0=11
     * = 0b10000111 = 0x87 */
    total++; if (ads_wreg_verify(REG_RESP2, 0x87, "RESP2   ")) ok++;

    uart_print("  Registers: ");
    print_int(ok);
    uart_print("/");
    print_int(total);
    uart_print(" verified OK\r\n");

    /* 7. Offset calibration */
    uart_print("[5/6] Running offset calibration...\r\n");
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000); /* 20 ms */
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);
    ads_cmd(ADS_CMD_OFFSETCAL);
    /* Wait for calibration (~1152 tCLK at 512 kHz = ~2.25 ms, use 500 ms to be safe) */
    usleep(500000);
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);

    /* Turn off CALIB_ON bit for normal operation while keeping RLDREF_INT=1 (0x07) */
    ads_wreg_verify(REG_RESP2, 0x07, "RESP2(nc)");

    return true;
}

static bool ads_read_sample(int32_t *ch1_raw, int32_t *ch2_raw, uint8_t *status);

/* =========================================================================
 * Operating Modes & Dynamic Mode Switching
 * ========================================================================= */
typedef enum {
    MODE_TEST_SQUARE = 1,   /* 1 Hz, +/-1 mV internal test generator */
    MODE_INPUT_SHORT = 2,   /* Inputs shorted (noise floor & offset test) */
    MODE_TEMPERATURE = 3,   /* Internal die temperature diode */
    MODE_LIVE_ECG    = 4    /* Live electrodes (normal ECG + Respiration) */
} ecg_op_mode_t;

static ecg_op_mode_t current_mode = MODE_LIVE_ECG;

static void ads_set_mode(ecg_op_mode_t mode)
{
    /* 1. Stop continuous conversion mode */
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);

    /* 2. Common configuration: 250 SPS data rate */
    ads_wreg(REG_CONFIG1, 0x01);

    switch (mode) {
        case MODE_TEST_SQUARE:
            /* CONFIG2: 0xA3 (Test generator ON, 1 Hz square wave, ref buffer ON) */
            ads_wreg(REG_CONFIG2, 0xA3);
            /* Route test signal to both channels (MUX = 0101, Gain = 6) */
            ads_wreg(REG_CH1SET, 0x05);
            ads_wreg(REG_CH2SET, 0x05);
            ads_wreg(REG_RESP1, 0x00);
            ads_wreg(REG_RESP2, 0x83);
            current_mode = MODE_TEST_SQUARE;
            uart_print("#MODE,1,TEST_SQUARE\r\n");
            break;

        case MODE_INPUT_SHORT:
            /* CONFIG2: 0xA0 (Test signal OFF, ref buffer ON) */
            ads_wreg(REG_CONFIG2, 0xA0);
            /* Inputs shorted internally (MUX = 0001) */
            ads_wreg(REG_CH1SET, 0x01);
            ads_wreg(REG_CH2SET, 0x01);
            ads_wreg(REG_RESP1, 0x00);
            ads_wreg(REG_RESP2, 0x83);
            current_mode = MODE_INPUT_SHORT;
            uart_print("#MODE,2,INPUT_SHORT\r\n");
            break;

        case MODE_TEMPERATURE:
            ads_wreg(REG_CONFIG2, 0xA0);
            /* Temperature sensor diode (MUX = 0100) */
            ads_wreg(REG_CH1SET, 0x04);
            ads_wreg(REG_CH2SET, 0x04);
            ads_wreg(REG_RESP1, 0x00);
            ads_wreg(REG_RESP2, 0x83);
            current_mode = MODE_TEMPERATURE;
            uart_print("#MODE,3,TEMP\r\n");
            break;

        case MODE_LIVE_ECG:
        default:
            /* CONFIG2: 0xE0 (Reference buffer ON, Lead-off comparators ON) */
            ads_wreg(REG_CONFIG2, 0xE0);
            /* LOFF: 0x10 (95%/5% threshold, 6 nA DC lead-off current) */
            ads_wreg(REG_LOFF, 0x10);
            /* CH1: Normal input (Gain = 1) */
            ads_wreg(REG_CH1SET, 0x10);
            /* CH2: ECG Lead I (Gain = 6, Normal input) */
            ads_wreg(REG_CH2SET, 0x00);
            /* RLD: Buffer ON, derived from CH2 IN2P + IN2N */
            ads_wreg(REG_RLD_SENS, 0x2C);
            /* Lead-off sensing on CH2 */
            ads_wreg(REG_LOFF_SENS, 0x0C);
            /* RESP1: 0x00 (32kHz carrier OFF for ultra-clean ECG & accurate DC lead-off) */
            ads_wreg(REG_RESP1, 0x00);
            /* RESP2: 0x07 (CALIB_ON=0, Internal RLDREF=(AVDD+AVSS)/2=1.21V enabled) */
            ads_wreg(REG_RESP2, 0x07);
            current_mode = MODE_LIVE_ECG;
            uart_print("#MODE,4,LIVE_ECG\r\n");
            break;
    }

    usleep(100000); /* 100 ms settling */

    /* Restart conversions */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(10000);
    ads_cmd(ADS_CMD_RDATAC);
    usleep(1000);

    /* Discard 3 settling samples */
    for (int i = 0; i < 3; i++) {
        uint32_t wait = 0;
        while (GPIO_read(CONFIG_GPIO_ECG_DRDY) != 0 && wait < 20000) {
            usleep(10);
            wait += 10;
        }
        int32_t c1, c2;
        uint8_t st;
        ads_read_sample(&c1, &c2, &st);
    }

    /* Reset DSP states */
    edgeai_ecg_reset(&ecg_state);
    resp_init();
}

/* =========================================================================
 * Read Single Sample (9 bytes in RDATAC mode)
 * ========================================================================= */
static bool ads_read_sample(int32_t *ch1_raw, int32_t *ch2_raw, uint8_t *status)
{
    /* Poll DRDY (active low) with timeout */
    uint32_t wait = 0;
    while (GPIO_read(CONFIG_GPIO_ECG_DRDY) != 0 && wait < 20000) {
        usleep(10);
        wait += 10;
    }

    if (wait >= 20000) return false; /* Timeout */

    SPI_Transaction t;
    uint8_t tx[9] = {0}, rx[9] = {0};
    memset(&t, 0, sizeof(t));
    t.count = 9;
    t.txBuf = tx;
    t.rxBuf = rx;

    ads_cs_low();
    bool ok = SPI_transfer(spi, &t);
    ads_cs_high();

    if (ok) {
        *status = rx[0];
        *ch1_raw = parse_24bit(&rx[3]);
        *ch2_raw = parse_24bit(&rx[6]);
    }

    return ok;
}

/* =========================================================================
 * Main Entry Point
 * ========================================================================= */
int main(void)
{
    Board_init();
    NoRTOS_start();
    GPIO_init();
    SPI_init();

    /* Initial GPIO states */
    GPIO_write(CONFIG_GPIO_LED_RED, 0);
    GPIO_write(CONFIG_GPIO_LED_GREEN, 0);
    GPIO_write(CONFIG_GPIO_ECG_START, 0); /* LOW during init */

    /* Open UART */
    UART2_Params up;
    UART2_Params_init(&up);
    up.baudRate  = 115200;
    up.readMode  = UART2_Mode_NONBLOCKING;
    up.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &up);

    uart_print("\r\n");
    uart_print("============================================================\r\n");
    uart_print("  SmartBAN ECG & Respiration Dedicated Firmware V1.1\r\n");
    uart_print("  Target: CC2652R1 + BAN Shield V3.5 + ADS1292R\r\n");
    uart_print("  Interactive Modes: [1] 1Hz Test [2] Short [3] Temp [4] Live\r\n");
    uart_print("============================================================\r\n\r\n");

    /* Open SPI in Mode 1 (CPOL=0, CPHA=1) at 250 kHz */
    SPI_Params sp;
    SPI_Params_init(&sp);
    sp.bitRate     = 250000;
    sp.frameFormat = SPI_POL0_PHA1;
    sp.mode        = SPI_CONTROLLER;
    spi = SPI_open(CONFIG_SPI_0, &sp);

    if (!spi) {
        uart_print("*** FATAL: SPI open failed! ***\r\n");
        GPIO_write(CONFIG_GPIO_LED_RED, 1);
        while (1) usleep(1000000);
    }

    /* Initialize ADS1292R */
    bool ads_ok = ads_init();
    if (!ads_ok) {
        uart_print("*** FATAL: ADS1292R initialization failed! ***\r\n");
        GPIO_write(CONFIG_GPIO_LED_RED, 1);
        while (1) usleep(1000000);
    }

    /* Initialize DSP processors */
    edgeai_ecg_init(&ecg_state, NULL);
    resp_init();

    /* Initialize WCH CH455H 3-Digit 7-Segment Display & LED Bar on BAN Shield */
    ch455_init();

    /* Send boot header for GUI synchronization */
    uart_print("#ECG_DEDICATED_V1,250,");
    print_hex8(chip_id);
    uart_print("\r\n");

    /* Set default operating mode: Live ECG */
    ads_set_mode(MODE_LIVE_ECG);

    uart_print("\r\nDATA_START\r\n");

    /* ===== Main Acquisition Loop ===== */
    uint32_t sample_count = 0;
    uint32_t timestamp_ms = 0;
    uint32_t summary_timer = 0;
    uint8_t  last_lead_off = 0;

    while (1) {
        /* Check for UART interactive commands from host / GUI */
        uint8_t rx_char = 0;
        size_t bytes_read = 0;
        bool host_beat_pulse = false;
        if (UART2_read(uart, &rx_char, 1, &bytes_read) == UART2_STATUS_SUCCESS && bytes_read > 0) {
            if (rx_char == '1') {
                ads_set_mode(MODE_TEST_SQUARE);
            } else if (rx_char == '2') {
                ads_set_mode(MODE_INPUT_SHORT);
            } else if (rx_char == '3') {
                ads_set_mode(MODE_TEMPERATURE);
            } else if (rx_char == '4') {
                ads_set_mode(MODE_LIVE_ECG);
            } else if (rx_char >= 0x80U) {
                /* Host GUI sent verified BPM (encoded as 100 + BPM) on real heartbeat */
                ecg_beat_counter = (uint16_t)(rx_char - 100U);
                host_beat_pulse = true;
            } else if (rx_char == 'B') {
                /* Host GUI confirmed QRS R-peak beat pulse */
                host_beat_pulse = true;
            } else if (rx_char == 'R') {
                /* Reset BPM display on 7-segment display to '---' */
                ecg_beat_counter = 0;
                ch455_update_bpm(0, false);
            } else if (rx_char == 'C' || rx_char == 'c') {
                /* Run runtime offset calibration */
                ads_cmd(ADS_CMD_SDATAC);
                usleep(1000);
                ads_wreg(REG_RESP2, 0x87); /* CALIB_ON = 1, RLDREF_INT = 1 */
                usleep(10000);
                ads_cmd(ADS_CMD_OFFSETCAL);
                usleep(500000);
                ads_cmd(ADS_CMD_SDATAC);
                usleep(1000);
                ads_wreg(REG_RESP2, 0x07); /* CALIB_ON = 0, RLDREF_INT = 1 */
                usleep(10000);
                ads_cmd(ADS_CMD_RDATAC);
                usleep(1000);
                uart_print("#CALIBRATED\r\n");
            }
        }

        int32_t ch1_raw = 0, ch2_raw = 0;
        uint8_t status = 0;

        bool ok = ads_read_sample(&ch1_raw, &ch2_raw, &status);

        if (!ok) {
            usleep(1000);
            continue;
        }

        sample_count++;
        timestamp_ms = (sample_count * 4); /* 1000 / 250 = 4 ms per sample */
        summary_timer++;

        /* Extract lead-off flags from status byte (bit2 = IN2N/RA off, bit1 = IN2P/LA off) */
        uint8_t lead_off = status & 0x0F;
        bool both_leads_off = (current_mode == MODE_LIVE_ECG) && ((lead_off & 0x06) == 0x06);

        /* LED indicators */
        if (both_leads_off) {
            /* Electrodes disconnected in live mode */
            GPIO_write(CONFIG_GPIO_LED_RED, 1);
        } else {
            GPIO_write(CONFIG_GPIO_LED_RED, 0);
        }
        last_lead_off = lead_off;

        /* Process ECG through clinical integer Pan-Tompkins pipeline */
        bool mcu_beat = edgeai_ecg_process_sample(&ecg_state, ch2_raw, timestamp_ms, false, &ecg_latest_result);
        uint16_t disp_bpm = ecg_beat_counter;
        if (disp_bpm == 0 && ecg_latest_result.heart_rate_smooth_bpm >= 35U && ecg_latest_result.heart_rate_smooth_bpm <= 185U) {
            disp_bpm = ecg_latest_result.heart_rate_smooth_bpm;
        }
        if ((mcu_beat || host_beat_pulse) && beat_flash_timer == 0) {
            /* Flash 7-Segment BPM Display + Green LED for 140 ms (35 samples) on every real heartbeat */
            beat_flash_timer = 35U;
            GPIO_write(CONFIG_GPIO_LED_GREEN, 1);
            ch455_update_bpm(disp_bpm, true);
        } else if (beat_flash_timer > 0) {
            beat_flash_timer--;
            if (beat_flash_timer == 0) {
                GPIO_write(CONFIG_GPIO_LED_GREEN, 0);
                ch455_update_bpm(disp_bpm, false);
            }
        }

        /* Process dual-source respiration (CH1 + CH2 EDR) in Live mode */
        if (current_mode == MODE_LIVE_ECG) {
            resp_process(ch1_raw, ch2_raw);
        }

        /* Stream data packet: D,timestamp,ch1,ch2,status */
        uart_print("D,");
        print_uint(timestamp_ms);
        uart_print(",");
        print_int(ch1_raw);
        uart_print(",");
        print_int(ch2_raw);
        uart_print(",");
        print_uint((uint32_t)status);
        uart_print("\r\n");

        /* Summary packet every 1 second: S,bpm,rr_ms,resp_rpm,lead_off,sdnn_ms,rmssd_ms,flags */
        if (summary_timer >= SUMMARY_INTERVAL) {
            summary_timer = 0;

            uint32_t out_bpm = (uint32_t)ecg_latest_result.heart_rate_smooth_bpm;
            if (out_bpm == 0) {
                out_bpm = (uint32_t)ecg_latest_result.heart_rate_bpm;
            }
            if (ecg_latest_result.cardiac_flags & (CARDIAC_FLAG_ASYSTOLE | CARDIAC_FLAG_LEARNING)) {
                out_bpm = 0;
            }

            /* S,bpm,rr_ms,resp_rpm,lead_off,sdnn_ms,rmssd_ms,flags */
            uart_print("S,");
            print_uint(out_bpm);
            uart_print(",");
            print_uint((uint32_t)ecg_latest_result.rr_interval_ms);
            uart_print(",");
            print_uint(resp.rpm);
            uart_print(",");
            print_uint((uint32_t)last_lead_off);
            uart_print(",");
            print_uint((uint32_t)ecg_latest_result.hrv_sdnn_ms);
            uart_print(",");
            print_uint((uint32_t)ecg_latest_result.hrv_rmssd_ms);
            uart_print(",");
            print_uint((uint32_t)ecg_latest_result.cardiac_flags);
            uart_print("\r\n");
        }
    }

    return 0;
}
