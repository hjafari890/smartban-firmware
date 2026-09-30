/* =========================================================================
 * SmartBAN ECG & Respiration Live Monitor — V2
 * Target: CC2652R1 / CC26X2R1 LaunchPad + BAN Shield V3.5
 * Chip: Texas Instruments ADS1292R
 * =========================================================================
 * CHANGELOG vs V1:
 * - FIX: START pin held LOW during register configuration (was HIGH = BUG)
 * - FIX: Register readback verification after every write
 * - FIX: Offset calibration (OFFSETCAL) run before streaming
 * - FIX: Lead-off comparators enabled for electrode contact detection
 * - FIX: Status byte included in output for lead-off / error monitoring
 * - FIX: Removed unreliable on-chip DSP — all DSP now in Python GUI
 * - FIX: Output rate raised to 250 Hz (from 50 Hz) for proper QRS capture
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
#include <ti/drivers/UART2.h>

#include "ti_drivers_config.h"

/* =========================================================================
 * ADS1292R SPI Commands & Registers
 * ========================================================================= */
#define ADS_CMD_WAKEUP     0x02
#define ADS_CMD_STANDBY    0x04
#define ADS_CMD_RESET      0x06
#define ADS_CMD_START      0x08
#define ADS_CMD_STOP       0x0A
#define ADS_CMD_OFFSETCAL  0x1A
#define ADS_CMD_RDATAC     0x10
#define ADS_CMD_SDATAC     0x11
#define ADS_CMD_RDATA      0x12
#define ADS_CMD_RREG       0x20
#define ADS_CMD_WREG       0x40

#define REG_ID         0x00
#define REG_CONFIG1    0x01
#define REG_CONFIG2    0x02
#define REG_LOFF       0x03
#define REG_CH1SET     0x04
#define REG_CH2SET     0x05
#define REG_RLD_SENS   0x06
#define REG_LOFF_SENS  0x07
#define REG_LOFF_STAT  0x08
#define REG_RESP1      0x09
#define REG_RESP2      0x0A
#define REG_GPIO       0x0B

/* Global Drivers */
static UART2_Handle uart = NULL;
static SPI_Handle   spi  = NULL;

/* =========================================================================
 * UART Helpers
 * ========================================================================= */
static void uart_print(const char *str)
{
    if (!uart) return;
    size_t written;
    UART2_write(uart, str, strlen(str), &written);
}

static void print_int(int32_t val)
{
    char buf[16];
    int idx = 0;
    if (val < 0) { uart_print("-"); val = -val; }
    if (val == 0) { uart_print("0"); return; }
    char tmp[16];
    int t = 0;
    while (val > 0) { tmp[t++] = '0' + (val % 10); val /= 10; }
    while (t > 0) buf[idx++] = tmp[--t];
    buf[idx] = '\0';
    uart_print(buf);
}

static void print_hex8(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    char buf[5] = { '0', 'x', hex[(val >> 4) & 0xF], hex[val & 0xF], '\0' };
    uart_print(buf);
}

/* =========================================================================
 * ADS1292R SPI Driver
 * ========================================================================= */
static void ads_cs_low(void)  { GPIO_write(CONFIG_GPIO_ECG_CS, 0); usleep(10); }
static void ads_cs_high(void) { usleep(10); GPIO_write(CONFIG_GPIO_ECG_CS, 1); usleep(10); }

static void ads_cmd(uint8_t cmd)
{
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

/* Write a register and verify by reading it back.
   Returns true if the readback matches. */
static bool ads_wreg_verify(uint8_t reg, uint8_t val, const char *name)
{
    ads_wreg(reg, val);
    uint8_t rb = ads_rreg(reg);
    uart_print("  ");
    uart_print(name);
    uart_print(": wrote ");
    print_hex8(val);
    uart_print(", read ");
    print_hex8(rb);
    if (rb == val) {
        uart_print(" [OK]\r\n");
        return true;
    } else {
        uart_print(" [MISMATCH!]\r\n");
        return false;
    }
}

/* =========================================================================
 * Main System
 * ========================================================================= */
int main(void)
{
    Board_init();
    NoRTOS_start();
    GPIO_init();
    SPI_init();

    /* ---------- Power rails ---------- */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);   /* Deselect ADXL362 */
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);   /* Deselect ADS1292R */
    GPIO_write(CONFIG_GPIO_ECG_START, 0);/* START must be LOW for config */
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);
    usleep(100000);

    /* ---------- UART ---------- */
    UART2_Params up;
    UART2_Params_init(&up);
    up.baudRate  = 115200;
    up.readMode  = UART2_Mode_NONBLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &up);

    uart_print("\r\n");
    uart_print("########################################################\r\n");
    uart_print("#  SmartBAN ECG & Respiration Monitor V2                #\r\n");
    uart_print("########################################################\r\n");

    /* ---------- SPI ---------- */
    SPI_Params sp;
    SPI_Params_init(&sp);
    sp.bitRate     = 500000;
    sp.frameFormat = SPI_POL0_PHA1;
    sp.mode        = SPI_CONTROLLER;
    spi = SPI_open(CONFIG_SPI_0, &sp);
    if (!spi) { uart_print("FATAL: SPI open failed\r\n"); while (1); }

    /* ---------- ADS1292R Hardware Reset ---------- */
    uart_print("\r\n[1/5] Resetting ADS1292R...\r\n");
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 0);
    usleep(10000);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);
    usleep(1000000);  /* Wait 1 sec for POR + VCAP charge */

    /* ---------- Stop continuous mode, enter register access ---------- */
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);

    /* ---------- Check Chip ID ---------- */
    uart_print("[2/5] Chip ID: ");
    uint8_t id = ads_rreg(REG_ID);
    print_hex8(id);
    if (id == 0x73) {
        uart_print(" (ADS1292R confirmed)\r\n");
    } else {
        uart_print(" (UNEXPECTED — expected 0x73)\r\n");
    }

    /* ---------- Configure Registers ---------- */
    uart_print("[3/5] Configuring registers...\r\n");
    int ok = 0, total = 0;

    /*  CONFIG1: 500 SPS */
    total++; if (ads_wreg_verify(REG_CONFIG1, 0x02, "CONFIG1 ")) ok++;

    /*  CONFIG2: Reference ON, Lead-off comparator ON
     *  Bit 7=1(fixed), Bit 6=1(LOFF comp ON), Bit 5=1(ref buf ON),
     *  Bits 4:0=0 (no test signal) */
    total++; if (ads_wreg_verify(REG_CONFIG2, 0xE0, "CONFIG2 ")) ok++;

    /*  LOFF: Lead-off detection threshold + current
     *  Default 0x10 is fine: 95%/5% threshold, 6nA DC lead-off */
    total++; if (ads_wreg_verify(REG_LOFF, 0x10, "LOFF    ")) ok++;

    /*  CH1SET: Gain=1, Normal input (respiration channel) */
    total++; if (ads_wreg_verify(REG_CH1SET, 0x10, "CH1SET  ")) ok++;

    /*  CH2SET: Gain=6, Normal input (ECG channel) */
    total++; if (ads_wreg_verify(REG_CH2SET, 0x00, "CH2SET  ")) ok++;

    /*  RLD_SENS: PGA chop fmod/16, RLD buffer ON, route CH2 IN2P+IN2N
     *  Bits: [00][1][0][1][1][0][0] = 0x2C  */
    total++; if (ads_wreg_verify(REG_RLD_SENS, 0x2C, "RLD_SENS")) ok++;

    /*  LOFF_SENS: Enable lead-off sensing on CH2 (IN2P, IN2N)
     *  Bits: [0000][0][0][1][1] = 0x03 */
    total++; if (ads_wreg_verify(REG_LOFF_SENS, 0x03, "LOFF_SEN")) ok++;

    /*  RESP1: Demod ON, Mod ON, Phase=112.5 deg (32kHz), internal clock
     *  Bits: [1][1][1][0][1][0][1][0] = 0xEA */
    total++; if (ads_wreg_verify(REG_RESP1, 0xEA, "RESP1   ")) ok++;

    /*  RESP2: Calib ON, 32kHz resp freq, RLDREF internal, bit1=1 (req'd)
     *  Bits: [1][000][0][0][1][1] = 0x83 */
    total++; if (ads_wreg_verify(REG_RESP2, 0x83, "RESP2   ")) ok++;

    uart_print("  Registers: ");
    print_int(ok);
    uart_print("/");
    print_int(total);
    uart_print(" verified OK\r\n");

    /* ---------- Offset Calibration ---------- */
    uart_print("[4/5] Running offset calibration...\r\n");
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000);
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);
    ads_cmd(ADS_CMD_OFFSETCAL);
    /* Wait for calibration to complete — typically ~1152 cycles at 512kHz */
    usleep(500000);
    ads_cmd(ADS_CMD_SDATAC);
    usleep(1000);

    /* ---------- Start Streaming ---------- */
    uart_print("[5/5] Starting continuous data stream at 250 Hz...\r\n");
    uart_print("FORMAT: status,ch1_raw,ch2_raw\r\n");
    uart_print("STATUS byte: 0xCx = OK. Bits[3:0] = lead-off flags.\r\n");
    uart_print("  bit3=IN2N(RA) off, bit2=IN2P(LA) off\r\n");
    uart_print("DATA_START\r\n");

    ads_cmd(ADS_CMD_RDATAC);
    usleep(100);

    /* START pin HIGH to begin conversions */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);

    uint32_t sample_count = 0;

    while (1) {
        /* Wait for DRDY (active low) */
        uint32_t wait = 0;
        while (GPIO_read(CONFIG_GPIO_ECG_DRDY) != 0 && wait < 20000) {
            usleep(10);
            wait += 10;
        }

        /* Read 9 bytes: 3 status + 3 CH1 + 3 CH2 */
        SPI_Transaction t;
        uint8_t tx[9] = {0}, rx[9] = {0};
        memset(&t, 0, sizeof(t));
        t.count = 9;
        t.txBuf = tx;
        t.rxBuf = rx;

        GPIO_write(CONFIG_GPIO_ECG_CS, 0);
        SPI_transfer(spi, &t);
        GPIO_write(CONFIG_GPIO_ECG_CS, 1);

        sample_count++;

        /* Send every 2nd sample = 250 Hz output */
        if (sample_count & 1) continue;

        /* Parse raw 24-bit signed values */
        int32_t ch1 = ((int32_t)rx[3] << 16) | ((int32_t)rx[4] << 8) | rx[5];
        if (ch1 & 0x00800000) ch1 |= 0xFF000000;

        int32_t ch2 = ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | rx[8];
        if (ch2 & 0x00800000) ch2 |= 0xFF000000;

        /* Status byte: bits[7:4]=1100, bits[3:0]=lead-off flags */
        uint8_t status = rx[0];

        /* Output: status_decimal,ch1,ch2\r\n */
        print_int((int32_t)status);
        uart_print(",");
        print_int(ch1);
        uart_print(",");
        print_int(ch2);
        uart_print("\r\n");
    }

    return 0;
}
