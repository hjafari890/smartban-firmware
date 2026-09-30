/* =========================================================================
 * PPG LED Bringup Firmware — MAXM86161 via MAX32664 Sensor Hub
 * =========================================================================
 * Target: CC26X2R1 LaunchPad (CC2652R1) + BAN Shield V3.5
 * Purpose: Turn on MAXM86161 PPG LEDs and read raw optical data
 *
 * CRITICAL FIX: Uses OPEN-DRAIN GPIO for MFIO (DIO28) and RSTN (DIO19)
 * to avoid driving 3.3V into the 1.8V MAX32664 domain.
 *
 * Signal Chain:
 *   CC2652R1 (DIO4/5, 3.3V I2C) -> PCA9306 (EN=DIO30) -> 1.8V I2C
 *   -> MAX32664 (0x55) -> Private Sensor Bus -> MAXM86161 (0x62)
 *
 * Pin Map:
 *   DIO_4  = HOST_SCL (I2C Clock, 3.3V side)
 *   DIO_5  = HOST_SDA (I2C Data, 3.3V side)
 *   DIO_19 = HOST_HUB_RST  (MAX32664 RSTN, open-drain to 1.8V)
 *   DIO_21 = 1V8_EN (LDO enable for 1.8V rail)
 *   DIO_28 = HOST_HUB_MFIO (MAX32664 MFIO, open-drain to 1.8V)
 *   DIO_30 = HOST_I2C_EN (PCA9306 level shifter enable)
 * ========================================================================= */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <NoRTOS.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>

/* CC26X2 driverlib for direct IOC register access (open-drain config) */
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/gpio.h>

#include "ti_drivers_config.h"

/* =========================================================================
 * Hardware Pin Definitions
 * ========================================================================= */
#define PIN_HUB_RST     19  /* DIO_19: MAX32664 RSTN (active-low reset) */
#define PIN_HUB_MFIO    28  /* DIO_28: MAX32664 MFIO (boot mode / wake) */
#define PIN_1V8_EN      21  /* DIO_21: 1.8V LDO enable */
#define PIN_I2C_EN      30  /* DIO_30: PCA9306 I2C level shifter enable */

/* =========================================================================
 * I2C Addresses
 * ========================================================================= */
#define MAX32664_I2C_ADDR       0x55  /* MAX32664 Sensor Hub (7-bit) */
#define MAXM86161_I2C_ADDR      0x62  /* MAXM86161 Optical (7-bit), on private bus */

/* =========================================================================
 * MAXM86161 Register Definitions
 * ========================================================================= */
#define MAXM_REG_INTR_STAT_1    0x00
#define MAXM_REG_INTR_STAT_2    0x01
#define MAXM_REG_INTR_EN_1      0x02
#define MAXM_REG_FIFO_WR_PTR    0x04
#define MAXM_REG_FIFO_RD_PTR    0x05
#define MAXM_REG_OVF_COUNTER    0x06
#define MAXM_REG_FIFO_DATA_CNT  0x07
#define MAXM_REG_FIFO_DATA      0x08
#define MAXM_REG_FIFO_CFG_1     0x09
#define MAXM_REG_FIFO_CFG_2     0x0A
#define MAXM_REG_SYS_CTRL       0x0D
#define MAXM_REG_PPG_CFG_1      0x11
#define MAXM_REG_PPG_CFG_2      0x12
#define MAXM_REG_PPG_CFG_3      0x13
#define MAXM_REG_PD_BIAS        0x15
#define MAXM_REG_LED_SEQ_1      0x20
#define MAXM_REG_LED_SEQ_2      0x21
#define MAXM_REG_LED_SEQ_3      0x22
#define MAXM_REG_LED1_PA        0x23  /* Green LED */
#define MAXM_REG_LED2_PA        0x24  /* IR LED */
#define MAXM_REG_LED3_PA        0x25  /* Red LED */
#define MAXM_REG_LED_RGE        0x2A  /* LED Full-Scale Range */
#define MAXM_REG_PART_ID        0xFF  /* Expected: 0x36 */
#define MAXM_REG_REV_ID         0xFE

/* =========================================================================
 * Global Handles
 * ========================================================================= */
static UART2_Handle uart = NULL;
static I2C_Handle   i2c  = NULL;

/* =========================================================================
 * UART Print Helpers
 * ========================================================================= */
static void uartPrint(const char *str)
{
    if (!uart) return;
    size_t written;
    UART2_write(uart, str, strlen(str), &written);
}

static void printHex8(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    char h[3] = { hex[(val >> 4) & 0x0F], hex[val & 0x0F], '\0' };
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

/* =========================================================================
 * OPEN-DRAIN GPIO Control for MFIO and RST
 * =========================================================================
 * CRITICAL: The MAX32664 runs at 1.8V. DIO_19 and DIO_28 are connected
 * DIRECTLY (no level shifter) from the 3.3V CC2652R1 domain.
 *
 * Push-pull HIGH = 3.3V → EXCEEDS MAX32664 abs max of 2.1V!
 * Open-drain HIGH = floating → pulled to 1.8V by internal/external pull-ups
 * Open-drain LOW  = driven to GND → safe
 *
 * We use driverlib IOCPortConfigureSet() to set IOC_IOMODE_OPEN_DRAIN_NORMAL.
 * ========================================================================= */

/* Configure a DIO pin as open-drain output */
static void gpio_set_open_drain_output(uint32_t dio)
{
    /* Set the IOC port configuration to:
     * - GPIO port (IOC_PORT_GPIO)
     * - Open-drain output mode
     * - No pull-up (external/internal pull-up on the MAX32664 side provides 1.8V)
     * - No hysteresis
     * - Normal slew rate
     * - Medium drive strength */
    IOCPortConfigureSet(dio, IOC_PORT_GPIO,
        IOC_IOMODE_OPEN_DRAIN_NORMAL |
        IOC_NO_IOPULL |
        IOC_CURRENT_4MA |
        IOC_STRENGTH_AUTO |
        IOC_NO_WAKE_UP |
        IOC_HYST_DISABLE |
        IOC_NO_EDGE |
        IOC_INT_DISABLE |
        IOC_INPUT_ENABLE);

    /* Enable GPIO output for this DIO */
    GPIO_setOutputEnableDio(dio, GPIO_OUTPUT_ENABLE);
}

/* Drive an open-drain pin LOW (actively pull to GND) */
static void gpio_od_low(uint32_t dio)
{
    GPIO_clearDio(dio);
}

/* Release an open-drain pin HIGH (float → pulled to 1.8V by external pull-ups) */
static void gpio_od_high(uint32_t dio)
{
    GPIO_setDio(dio);
    /* In open-drain mode, setting the output to 1 releases the driver,
     * allowing the external pull-up to pull the line to 1.8V */
}

/* Set an open-drain pin to input mode (for reading MFIO interrupt state) */
static void gpio_od_input(uint32_t dio)
{
    IOCPortConfigureSet(dio, IOC_PORT_GPIO,
        IOC_IOMODE_OPEN_DRAIN_NORMAL |
        IOC_IOPULL_UP |
        IOC_CURRENT_4MA |
        IOC_STRENGTH_AUTO |
        IOC_NO_WAKE_UP |
        IOC_HYST_DISABLE |
        IOC_NO_EDGE |
        IOC_INT_DISABLE |
        IOC_INPUT_ENABLE);

    /* Disable GPIO output to become input */
    GPIO_setOutputEnableDio(dio, GPIO_OUTPUT_DISABLE);
}

/* =========================================================================
 * MAX32664 Sensor Hub Communication
 * ========================================================================= */

/*
 * Hardware reset sequence for MAX32664 using OPEN-DRAIN GPIO.
 *
 * Boot mode selection:
 *   MFIO HIGH at RSTN rising edge → Application Mode (0x00)
 *   MFIO LOW  at RSTN rising edge → Bootloader Mode (0x08)
 *
 * We want Application Mode, so MFIO must be HIGH (released, pulled to 1.8V).
 */
static void max32664_hw_reset(void)
{
    uartPrint("  [RST] Configuring MFIO & RST as open-drain...\r\n");

    /* 1. Configure both pins as open-drain outputs */
    gpio_set_open_drain_output(PIN_HUB_MFIO);
    gpio_set_open_drain_output(PIN_HUB_RST);

    /* 2. Release MFIO HIGH (1.8V via pull-up) for Application Mode boot */
    gpio_od_high(PIN_HUB_MFIO);
    usleep(10000); /* 10 ms to stabilize */

    /* 3. Assert RSTN LOW for ≥25 ms */
    uartPrint("  [RST] Asserting RSTN LOW for 25 ms...\r\n");
    gpio_od_low(PIN_HUB_RST);
    usleep(25000);

    /* 4. Verify MFIO is still HIGH before releasing reset */
    gpio_od_high(PIN_HUB_MFIO);
    usleep(10000);

    /* 5. Release RSTN HIGH (1.8V via pull-up) — boot mode latched here */
    uartPrint("  [RST] Releasing RSTN HIGH (open-drain, 1.8V pull-up)...\r\n");
    gpio_od_high(PIN_HUB_RST);
    usleep(50000); /* 50 ms for boot mode latch */

    /* 6. Wait 1600 ms for application firmware initialization */
    uartPrint("  [RST] Waiting 1600 ms for hub firmware init...\r\n");
    usleep(1600000);

    /* 7. Release MFIO to input so hub can assert data-ready interrupts */
    gpio_od_input(PIN_HUB_MFIO);

    uartPrint("  [RST] Reset sequence complete.\r\n");
}

/*
 * I2C transfer with MAX32664 MFIO Wake-Up Handshake (OPEN-DRAIN version).
 *
 * Protocol:
 * 1. Drive MFIO LOW for ≥300 µs to wake the hub from sleep
 * 2. Send I2C command while MFIO is LOW
 * 3. Wait for hub to process (delay_ms)
 * 4. Read response while MFIO is LOW
 * 5. Release MFIO back to input (1.8V pull-up)
 */
static bool max32664_transmit(const uint8_t *tx_buf, uint8_t tx_len,
                               uint8_t *rx_buf, uint32_t rx_len,
                               uint32_t delay_ms)
{
    if (!i2c) return false;

    /* 1. Wake up: configure MFIO as open-drain output, drive LOW */
    gpio_set_open_drain_output(PIN_HUB_MFIO);
    gpio_od_low(PIN_HUB_MFIO);
    usleep(500); /* 500 µs wake-up time */

    /* 2. Write command */
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.targetAddress = MAX32664_I2C_ADDR;
    trans.writeBuf = (void *)tx_buf;
    trans.writeCount = tx_len;
    trans.readBuf = NULL;
    trans.readCount = 0;

    bool ok = I2C_transfer(i2c, &trans);
    if (!ok) {
        gpio_od_input(PIN_HUB_MFIO);
        return false;
    }

    /* 3. Wait for hub to process command */
    if (delay_ms > 0) {
        usleep(delay_ms * 1000);
    }

    /* 4. Read response if requested */
    if (rx_buf && rx_len > 0) {
        memset(&trans, 0, sizeof(trans));
        trans.targetAddress = MAX32664_I2C_ADDR;
        trans.writeBuf = NULL;
        trans.writeCount = 0;
        trans.readBuf = rx_buf;
        trans.readCount = rx_len;
        ok = I2C_transfer(i2c, &trans);
    }

    /* 5. Post-command delay */
    usleep(10000);

    /* 6. Release MFIO to input (1.8V pull-up) */
    gpio_od_input(PIN_HUB_MFIO);
    usleep(300);

    return ok;
}

/* Write a single-byte command to the hub: [Family, Index, Value] */
static uint8_t max32664_write_byte(uint8_t family, uint8_t index, uint8_t val, uint32_t delay_ms)
{
    uint8_t tx[3] = { family, index, val };
    uint8_t status = 0xFF;
    if (max32664_transmit(tx, 3, &status, 1, delay_ms))
        return status;
    return 0xFF;
}

/* Read a single-byte value from the hub: [Family, Index] → [Status, Value] */
static uint8_t max32664_read_byte(uint8_t family, uint8_t index, uint8_t *val, uint32_t delay_ms)
{
    uint8_t tx[2] = { family, index };
    uint8_t rx[2] = { 0xFF, 0x00 };
    if (max32664_transmit(tx, 2, rx, 2, delay_ms)) {
        *val = rx[1];
        return rx[0];
    }
    return 0xFF;
}

/* =========================================================================
 * MAX32664 Pass-Through Register Access to MAXM86161
 * =========================================================================
 * Family 0x40 = Write sensor register:  [0x40, sensor_idx, reg, val]
 * Family 0x41 = Read sensor register:   [0x41, sensor_idx, reg] → [status, val]
 * sensor_idx = 0x00 for the optical AFE (MAXM86161)
 * ========================================================================= */

static uint8_t hub_write_sensor_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[4] = { 0x40, 0x00, reg, val };
    uint8_t status = 0xFF;
    if (max32664_transmit(tx, 4, &status, 1, 15))
        return status;
    return 0xFF;
}

static uint8_t hub_read_sensor_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[3] = { 0x41, 0x00, reg };
    uint8_t rx[2] = { 0xFF, 0x00 };
    if (max32664_transmit(tx, 3, rx, 2, 15)) {
        *val = rx[1];
        return rx[0];
    }
    return 0xFF;
}

/* =========================================================================
 * MAXM86161 Initialization — Turn on LEDs & Start Sampling
 * ========================================================================= */

static bool maxm86161_init_via_hub(void)
{
    uartPrint("\r\n--- MAXM86161 Configuration via Pass-Through ---\r\n");

    /* 1. Verify MAXM86161 Part ID via pass-through */
    uint8_t part_id = 0, rev_id = 0;
    uint8_t st;

    st = hub_read_sensor_reg(MAXM_REG_PART_ID, &part_id);
    uartPrint("  Part ID : 0x"); printHex8(part_id);
    uartPrint(" (st=0x"); printHex8(st); uartPrint(")\r\n");

    st = hub_read_sensor_reg(MAXM_REG_REV_ID, &rev_id);
    uartPrint("  Rev ID  : 0x"); printHex8(rev_id);
    uartPrint(" (st=0x"); printHex8(st); uartPrint(")\r\n");

    if (part_id != 0x36) {
        uartPrint("  *** ERROR: Expected MAXM86161 Part ID 0x36, got 0x");
        printHex8(part_id); uartPrint(" ***\r\n");
        uartPrint("  *** Check 1.8V rail, R17/R19 pull-ups, U7 solder ***\r\n");
        /* Don't return false — continue trying anyway */
    }

    /* 2. Soft Reset */
    uartPrint("  Soft reset...\r\n");
    hub_write_sensor_reg(MAXM_REG_SYS_CTRL, 0x01);
    usleep(10000);

    /* 3. Put into shutdown while configuring */
    hub_write_sensor_reg(MAXM_REG_SYS_CTRL, 0x02);

    /* 4. Clear pending interrupts */
    uint8_t dummy;
    hub_read_sensor_reg(MAXM_REG_INTR_STAT_1, &dummy);
    hub_read_sensor_reg(MAXM_REG_INTR_STAT_2, &dummy);

    /* 5. PPG Config 1: 117.3 µs integration, 16 µA ADC range (0x0B) */
    hub_write_sensor_reg(MAXM_REG_PPG_CFG_1, 0x0B);

    /* 6. PPG Config 2: 256 sps sample rate, 1x averaging
     *    PPG_SR[4:0] = 0b01000 (256 sps) → bits [7:3] = 0x08 << 3 = 0x40
     *    SMP_AVE[2:0] = 0b000 (1x) → bits [2:0] = 0x00
     *    Register value = 0x40 */
    hub_write_sensor_reg(MAXM_REG_PPG_CFG_2, 0x40);

    /* 7. PPG Config 3: LED settling 12 µs (0xC0) */
    hub_write_sensor_reg(MAXM_REG_PPG_CFG_3, 0xC0);

    /* 8. Photodiode Bias: 0-65pF (0x01) */
    hub_write_sensor_reg(MAXM_REG_PD_BIAS, 0x01);

    /* 9. LED Current Range: 124 mA range for all 3 LEDs
     *    LED1_RGE=11, LED2_RGE=11, LED3_RGE=11 → 0x3F */
    hub_write_sensor_reg(MAXM_REG_LED_RGE, 0x3F);

    /* 10. LED Drive Currents (~15 mA each for proof-of-life)
     *     With 124 mA range: LSB = 0.48 mA
     *     0x20 (32 decimal) = 32 × 0.48 = 15.4 mA
     *     This is visible for Green, and detectable for IR/Red */
    uartPrint("  Setting LED currents to ~15 mA...\r\n");
    hub_write_sensor_reg(MAXM_REG_LED1_PA, 0x20); /* Green ~15 mA */
    hub_write_sensor_reg(MAXM_REG_LED2_PA, 0x20); /* IR    ~15 mA */
    hub_write_sensor_reg(MAXM_REG_LED3_PA, 0x20); /* Red   ~15 mA */

    /* 11. LED Sequence: Green → IR → Red (3-slot sequence)
     *     Reg 0x20: LEDC2[7:4]=0x2(IR) | LEDC1[3:0]=0x1(Green) → 0x21
     *     Reg 0x21: LEDC4[7:4]=0x0(None) | LEDC3[3:0]=0x3(Red) → 0x03
     *     Reg 0x22: LEDC6/5 = 0x00 (unused) */
    hub_write_sensor_reg(MAXM_REG_LED_SEQ_1, 0x21);
    hub_write_sensor_reg(MAXM_REG_LED_SEQ_2, 0x03);
    hub_write_sensor_reg(MAXM_REG_LED_SEQ_3, 0x00);

    /* 12. FIFO Configuration */
    hub_write_sensor_reg(MAXM_REG_FIFO_CFG_1, 0x0F); /* A_FULL watermark */
    hub_write_sensor_reg(MAXM_REG_FIFO_CFG_2, 0x12); /* Flush + Roll-over enable */

    /* 13. Enable data-ready interrupt on MAXM86161 → drives INTB → MAX32664 HR_INT */
    hub_write_sensor_reg(MAXM_REG_INTR_EN_1, 0x40); /* DATA_RDY enable */

    /* 14. Clear FIFO pointers */
    hub_write_sensor_reg(MAXM_REG_FIFO_WR_PTR, 0x00);
    hub_write_sensor_reg(MAXM_REG_FIFO_RD_PTR, 0x00);
    hub_write_sensor_reg(MAXM_REG_OVF_COUNTER, 0x00);

    /* 15. EXIT SHUTDOWN — Start sampling!
     *     Bit 3 (SINGLE_PPG) = 1 (required for MAXM86161)
     *     Bit 2 (LP_MODE) = 1 (low power between samples)
     *     Bit 1 (SHDN) = 0 (active)
     *     Bit 0 (RESET) = 0
     *     Register value = 0x0C */
    uartPrint("  Starting MAXM86161 sampling (SysCtrl=0x0C)...\r\n");
    hub_write_sensor_reg(MAXM_REG_SYS_CTRL, 0x0C);

    /* 16. Verify configuration by reading back key registers */
    uint8_t rb_led1 = 0, rb_led2 = 0, rb_led3 = 0, rb_sys = 0;
    hub_read_sensor_reg(MAXM_REG_LED1_PA, &rb_led1);
    hub_read_sensor_reg(MAXM_REG_LED2_PA, &rb_led2);
    hub_read_sensor_reg(MAXM_REG_LED3_PA, &rb_led3);
    hub_read_sensor_reg(MAXM_REG_SYS_CTRL, &rb_sys);

    uartPrint("  Readback: LED1(Green)=0x"); printHex8(rb_led1);
    uartPrint(" LED2(IR)=0x"); printHex8(rb_led2);
    uartPrint(" LED3(Red)=0x"); printHex8(rb_led3);
    uartPrint(" SysCtrl=0x"); printHex8(rb_sys);
    uartPrint("\r\n");

    bool success = (rb_led1 == 0x20 && rb_sys == 0x0C);
    if (success) {
        uartPrint("  *** SUCCESS: MAXM86161 LEDs are ON! ***\r\n");
        uartPrint("  *** You should see a GREEN glow from the sensor ***\r\n");
        uartPrint("  *** Use phone camera to see IR LED (purple/white) ***\r\n");
    } else {
        uartPrint("  *** WARNING: Register readback mismatch ***\r\n");
        uartPrint("  *** LEDs may not be on — check connections ***\r\n");
    }

    return success;
}

/* =========================================================================
 * Read raw PPG samples from MAXM86161 FIFO via hub pass-through
 * ========================================================================= */

static void read_ppg_fifo(void)
{
    /* Read FIFO data count */
    uint8_t count = 0;
    hub_read_sensor_reg(MAXM_REG_FIFO_DATA_CNT, &count);

    if (count == 0) return;
    if (count > 36) count = 36; /* Limit to avoid long I2C transactions */

    uint32_t green = 0, ir = 0, red = 0;

    /* Each FIFO sample is 3 bytes: [TAG(5) | DATA(19)] */
    for (uint8_t s = 0; s < count; s++) {
        /* Read 3 bytes from FIFO data register 0x08 via pass-through.
         * We must read byte-by-byte since pass-through only returns 1 byte. */
        uint8_t b0 = 0, b1 = 0, b2 = 0;
        hub_read_sensor_reg(MAXM_REG_FIFO_DATA, &b0);
        hub_read_sensor_reg(MAXM_REG_FIFO_DATA, &b1);
        hub_read_sensor_reg(MAXM_REG_FIFO_DATA, &b2);

        uint8_t  tag  = (b0 >> 3) & 0x1F;
        uint32_t data = (((uint32_t)(b0 & 0x07)) << 16) |
                        ((uint32_t)b1 << 8) |
                        (uint32_t)b2;

        switch (tag) {
            case 0x01: green = data; break; /* LED1 = Green */
            case 0x02: ir    = data; break; /* LED2 = IR */
            case 0x03: red   = data; break; /* LED3 = Red */
            default: break;
        }
    }

    /* Print the latest sample set */
    if (green > 0 || ir > 0 || red > 0) {
        uartPrint("PPG| G=");
        printDec(green);
        uartPrint(" IR=");
        printDec(ir);
        uartPrint(" R=");
        printDec(red);
        uartPrint(" (");
        printDec(count);
        uartPrint(" samples)\r\n");
    }
}

/* =========================================================================
 * Alternative: Read PPG data via MAX32664 Hub FIFO (Algorithm or Sensor mode)
 * This reads from the hub's own FIFO rather than pass-through to MAXM86161
 * ========================================================================= */

static void read_hub_fifo(void)
{
    /* Read hub FIFO sample count: Family 0x12, Index 0x00 */
    uint8_t cnt = 0;
    uint8_t st = max32664_read_byte(0x12, 0x00, &cnt, 10);

    if (st != 0x00 || cnt == 0) return;

    /* Read hub FIFO data: Family 0x12, Index 0x01 */
    /* In sensor-only mode (0x01), each sample is typically 6-9 bytes */
    uint8_t tx[2] = { 0x12, 0x01 };
    uint8_t rx[64];
    memset(rx, 0, sizeof(rx));

    uint32_t rlen = 1 + (cnt * 9); /* status + (cnt × bytes_per_sample) */
    if (rlen > 64) rlen = 64;

    if (max32664_transmit(tx, 2, rx, rlen, 10)) {
        uartPrint("HUB| st=0x"); printHex8(rx[0]);
        uartPrint(" cnt="); printDec(cnt);
        uartPrint(" raw: ");
        uint32_t show = (rlen < 24) ? rlen : 24;
        for (uint32_t k = 1; k < show; k++) {
            printHex8(rx[k]); uartPrint(" ");
        }
        uartPrint("\r\n");
    }
}

/* =========================================================================
 * Main Entry Point
 * ========================================================================= */

int main(void)
{
    /* Standard TI-Drivers initialization */
    Board_init();
    NoRTOS_start();

    GPIO_init();
    I2C_init();

    /* ====================================================================
     * STEP 1: Power Up — Enable 1.8V rail and I2C level shifter
     * ==================================================================== */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);  /* Turn on 1.8V LDO */
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);  /* Enable PCA9306 level shifter */
    usleep(100000); /* 100 ms for power rails to stabilize */

    /* ====================================================================
     * STEP 2: Open UART for debug output
     * ==================================================================== */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readMode  = UART2_Mode_BLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    uartPrint("\r\n");
    uartPrint("=====================================================\r\n");
    uartPrint("  PPG LED Bringup — MAXM86161 via MAX32664 Hub\r\n");
    uartPrint("  Open-Drain Fix for MFIO/RST Voltage Mismatch\r\n");
    uartPrint("=====================================================\r\n\r\n");

    /* ====================================================================
     * STEP 3: Open I2C at 100 kHz
     * ==================================================================== */
    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (!i2c) {
        uartPrint("  *** FATAL: Failed to open I2C ***\r\n");
        while (1) { usleep(1000000); }
    }
    uartPrint("  I2C opened at 100 kHz (DIO4=SCL, DIO5=SDA)\r\n");

    /* ====================================================================
     * STEP 4: Reset MAX32664 into Application Mode (OPEN-DRAIN!)
     * ==================================================================== */
    uartPrint("\r\n--- MAX32664 Hardware Reset (Open-Drain) ---\r\n");
    max32664_hw_reset();

    /* ====================================================================
     * STEP 5: Verify hub entered Application Mode
     * ==================================================================== */
    uartPrint("\r\n--- MAX32664 Status Check ---\r\n");

    uint8_t hub_mode = 0xFF;
    uint8_t st = max32664_read_byte(0x02, 0x00, &hub_mode, 5);
    uartPrint("  Hub Mode  : 0x"); printHex8(hub_mode);
    uartPrint(" (st=0x"); printHex8(st); uartPrint(")\r\n");

    if (hub_mode == 0x00) {
        uartPrint("  *** Application Mode (0x00) — CORRECT! ***\r\n");
    } else if (hub_mode == 0x08) {
        uartPrint("  *** Bootloader Mode (0x08) — WRONG! ***\r\n");
        uartPrint("  Attempting Exit Bootloader command...\r\n");

        /* Try to exit bootloader: Family 0x01, Index 0x00, Val 0x00 */
        uint8_t exit_st = max32664_write_byte(0x01, 0x00, 0x00, 50);
        uartPrint("  Exit Bootloader st=0x"); printHex8(exit_st); uartPrint("\r\n");
        usleep(1000000); /* 1 second for app boot */

        st = max32664_read_byte(0x02, 0x00, &hub_mode, 5);
        uartPrint("  Hub Mode after exit: 0x"); printHex8(hub_mode); uartPrint("\r\n");

        if (hub_mode == 0x08) {
            uartPrint("  *** STILL in Bootloader! Trying second reset... ***\r\n");
            max32664_hw_reset();
            usleep(500000);
            st = max32664_read_byte(0x02, 0x00, &hub_mode, 5);
            uartPrint("  Hub Mode after 2nd reset: 0x"); printHex8(hub_mode); uartPrint("\r\n");
        }
    } else {
        uartPrint("  *** Unexpected mode! I2C may not be working ***\r\n");
        uartPrint("  Check: 1.8V LED on? Level shifter enabled? ***\r\n");
    }

    /* Read firmware version */
    uint8_t ver_tx[2] = { 0xFF, 0x03 };
    uint8_t ver_rx[4] = { 0xFF, 0, 0, 0 };
    if (max32664_transmit(ver_tx, 2, ver_rx, 4, 10) && ver_rx[0] == 0x00) {
        uartPrint("  Hub FW Version: ");
        printDec(ver_rx[1]); uartPrint(".");
        printDec(ver_rx[2]); uartPrint(".");
        printDec(ver_rx[3]); uartPrint("\r\n");
    }

    /* ====================================================================
     * STEP 6: Configure hub — Sensor-only output mode
     * ==================================================================== */
    if (hub_mode == 0x00) {
        uartPrint("\r\n--- Hub Configuration ---\r\n");

        /* Set output mode to sensor-only (0x01) — no algorithm needed */
        st = max32664_write_byte(0x10, 0x00, 0x01, 10);
        uartPrint("  Set Output Mode 0x01 (Sensor Only): st=0x"); printHex8(st); uartPrint("\r\n");

        /* Set FIFO threshold */
        st = max32664_write_byte(0x10, 0x01, 0x01, 10);
        uartPrint("  Set FIFO Threshold 1: st=0x"); printHex8(st); uartPrint("\r\n");

        /* Enable optical AFE (Family 0x44, Index 0x00, Enable 0x01, Mode 0x00) */
        uint8_t afe_tx[4] = { 0x44, 0x00, 0x01, 0x00 };
        uint8_t afe_st = 0xFF;
        max32664_transmit(afe_tx, 4, &afe_st, 1, 250);
        uartPrint("  Enable AFE: st=0x"); printHex8(afe_st); uartPrint("\r\n");

        /* ================================================================
         * STEP 7: Configure MAXM86161 LEDs via pass-through
         * ================================================================ */
        maxm86161_init_via_hub();

    } else {
        uartPrint("\r\n*** Cannot configure PPG — hub not in Application Mode ***\r\n");
        uartPrint("*** The open-drain fix should have resolved this. ***\r\n");
        uartPrint("*** If stuck in 0x08, check for external pull-ups on ***\r\n");
        uartPrint("*** MFIO (DIO28) and RST (DIO19) to the 1.8V rail. ***\r\n");
    }

    /* ====================================================================
     * STEP 8: Main Loop — Continuously read PPG data
     * ==================================================================== */
    uartPrint("\r\n--- Starting PPG Data Acquisition Loop ---\r\n");
    uartPrint("  Format: PPG| G=<green> IR=<ir> R=<red>\r\n\r\n");

    uint32_t loop = 0;
    while (1) {
        if (hub_mode == 0x00) {
            /* Try reading from MAXM86161 FIFO via pass-through */
            read_ppg_fifo();

            /* Also try reading from hub FIFO */
            read_hub_fifo();
        }

        /* Heartbeat every 5 seconds */
        loop++;
        if (loop % 25 == 0) {
            uartPrint("  [heartbeat] loop=");
            printDec(loop);
            uartPrint("\r\n");
        }

        usleep(200000); /* 200 ms between reads (5 Hz polling) */
    }

    /* Never reached */
    return 0;
}
