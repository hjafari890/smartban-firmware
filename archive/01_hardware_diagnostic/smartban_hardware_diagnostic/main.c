#include <stdint.h>
#include <stdbool.h>

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/gpio.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ssi.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/uart.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/prcm.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/cpu.h>

/* ============================================================
 * CC2652R1 <-> ADS1292R
 * ============================================================ */

/* Shared SPI */
#define PIN_SPI_SCLK    IOID_10
#define PIN_SPI_MOSI    IOID_9
#define PIN_SPI_MISO    IOID_8

/* ADS1292R */
#define PIN_ECG_CS      IOID_11
#define PIN_ECG_DRDY    IOID_23
#define PIN_ECG_START   IOID_24
#define PIN_ECG_PWDN    IOID_22

/* Other SPI device - keep deselected */
#define PIN_IMU_CS      IOID_15

/* LaunchPad XDS110 UART */
#define PIN_UART_TX     IOID_3
#define PIN_UART_RX     IOID_2


/* ============================================================
 * ADS1292R commands
 * ============================================================ */

#define CMD_WAKEUP      0x02
#define CMD_STANDBY     0x04
#define CMD_RESET       0x06
#define CMD_START       0x08
#define CMD_STOP        0x0A
#define CMD_RDATAC      0x10
#define CMD_SDATAC      0x11
#define CMD_RDATA       0x12

/* Registers */
#define REG_ID          0x00
#define REG_CONFIG1     0x01
#define REG_CONFIG2     0x02
#define REG_LOFF        0x03
#define REG_CH1SET      0x04
#define REG_CH2SET      0x05
#define REG_RLD_SENS    0x06
#define REG_LOFF_SENS   0x07
#define REG_LOFF_STAT   0x08
#define REG_RESP1       0x09
#define REG_RESP2       0x0A
#define REG_GPIO        0x0B


/* ============================================================
 * Delay helpers
 * CPU clock is normally 48 MHz on the LaunchPad.
 * ============================================================ */

static void delay_us(uint32_t us)
{
    uint32_t cycles = (SysCtrlClockGet() / 3000000U) * us;

    if (cycles < 1)
        cycles = 1;

    CPUdelay(cycles);
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}


/* ============================================================
 * GPIO
 * ============================================================ */

static void gpio_init(void)
{
    /* ECG CS */
    IOCPinTypeGpioOutput(PIN_ECG_CS);

    /* IMU CS */
    IOCPinTypeGpioOutput(PIN_IMU_CS);

    /* START */
    IOCPinTypeGpioOutput(PIN_ECG_START);

    /* PWDN / RESET */
    IOCPinTypeGpioOutput(PIN_ECG_PWDN);

    /* DRDY */
    IOCPinTypeGpioInput(PIN_ECG_DRDY);

    /* Safe states */
    GPIO_setDio(PIN_ECG_CS);
    GPIO_setDio(PIN_IMU_CS);

    GPIO_clearDio(PIN_ECG_START);

    /* Hold ADS in reset initially */
    GPIO_clearDio(PIN_ECG_PWDN);
}


/* ============================================================
 * SPI
 * ============================================================ */

static void spi_init(void)
{
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();

    while (PRCMLoadGet())
    {
    }

    /*
     * ADS1292R:
     * SPI mode 1
     * CPOL = 0
     * CPHA = 1
     */
    IOCPortConfigureSet(
        PIN_SPI_SCLK,
        IOC_PORT_MCU_SSI0_CLK,
        IOC_STD_OUTPUT);

    IOCPortConfigureSet(
        PIN_SPI_MOSI,
        IOC_PORT_MCU_SSI0_TX,
        IOC_STD_OUTPUT);

    IOCPortConfigureSet(
        PIN_SPI_MISO,
        IOC_PORT_MCU_SSI0_RX,
        IOC_STD_INPUT);

    SSIConfigSetExpClk(
        SSI0_BASE,
        SysCtrlClockGet(),
        SSI_FRF_MOTO_MODE_1,
        SSI_MODE_MASTER,
        1000000,
        8);

    SSIEnable(SSI0_BASE);

    /* Flush RX FIFO */
    uint32_t dummy;

    while (SSIDataGetNonBlocking(SSI0_BASE, &dummy))
    {
    }
}


static uint8_t spi_transfer(uint8_t tx)
{
    uint32_t rx;

    SSIDataPut(SSI0_BASE, tx);
    SSIDataGet(SSI0_BASE, &rx);

    return (uint8_t)(rx & 0xFF);
}


/* ============================================================
 * UART
 * XDS110 Application/User UART:
 * DIO3 TX
 * DIO2 RX
 * ============================================================ */

static void uart_init(void)
{
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);
    PRCMLoadSet();

    while (PRCMLoadGet())
    {
    }

    IOCPortConfigureSet(
        PIN_UART_TX,
        IOC_PORT_MCU_UART0_TX,
        IOC_STD_OUTPUT);

    IOCPortConfigureSet(
        PIN_UART_RX,
        IOC_PORT_MCU_UART0_RX,
        IOC_STD_INPUT);

    UARTConfigSetExpClk(
        UART0_BASE,
        SysCtrlClockGet(),
        115200,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE);

    UARTEnable(UART0_BASE);
}


static void uart_putc(char c)
{
    UARTCharPut(UART0_BASE, c);
}


static void uart_puts(const char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}


/*
 * Send signed 32-bit integer as ASCII.
 *
 * Example:
 * -123456
 * 234567
 * 987654
 */
static void uart_put_int32(int32_t value)
{
    char buf[12];
    int i = 0;
    bool negative = false;

    if (value == 0)
    {
        uart_putc('0');
        return;
    }

    if (value < 0)
    {
        negative = true;
        value = -value;
    }

    while (value > 0)
    {
        buf[i++] = '0' + (value % 10);
        value /= 10;
    }

    if (negative)
        uart_putc('-');

    while (i > 0)
        uart_putc(buf[--i]);
}


/* ============================================================
 * ADS1292R low level
 * ============================================================ */

static void ads_cs_low(void)
{
    GPIO_clearDio(PIN_ECG_CS);
}

static void ads_cs_high(void)
{
    GPIO_setDio(PIN_ECG_CS);
}


static void ads_command(uint8_t cmd)
{
    ads_cs_low();

    spi_transfer(cmd);

    ads_cs_high();

    /*
     * Give the ADS some time between commands.
     */
    delay_us(20);
}


static void ads_write_reg(uint8_t addr, uint8_t value)
{
    ads_cs_low();

    /*
     * WREG:
     * 010rrrrr
     *
     * Second byte = number of registers - 1
     *
     * We write one register.
     */
    spi_transfer(0x40 | addr);
    spi_transfer(0x00);
    spi_transfer(value);

    ads_cs_high();

    delay_us(20);
}


static uint8_t ads_read_reg(uint8_t addr)
{
    uint8_t value;

    ads_cs_low();

    /*
     * RREG
     */
    spi_transfer(0x20 | addr);

    /* One register - 1 */
    spi_transfer(0x00);

    value = spi_transfer(0x00);

    ads_cs_high();

    delay_us(20);

    return value;
}


/* ============================================================
 * 24-bit signed conversion
 * ============================================================ */

static int32_t ads_s24(
    uint8_t b0,
    uint8_t b1,
    uint8_t b2)
{
    int32_t x;

    x =
        ((int32_t)b0 << 16) |
        ((int32_t)b1 << 8)  |
        ((int32_t)b2);

    /*
     * Sign extend 24-bit two's complement.
     */
    if (x & 0x00800000)
    {
        x |= 0xFF000000;
    }

    return x;
}


/* ============================================================
 * ADS1292R initialization
 * ============================================================ */

static bool ads1292_init(void)
{
    uint8_t id;

    /*
     * Keep ADC stopped while configuring.
     */
    GPIO_clearDio(PIN_ECG_START);

    /*
     * Hardware reset:
     * PWDN/RESET is active LOW.
     */
    GPIO_clearDio(PIN_ECG_PWDN);

    delay_ms(10);

    GPIO_setDio(PIN_ECG_PWDN);

    /*
     * Conservative startup delay.
     */
    delay_ms(1000);

    /*
     * Software reset.
     */
    ads_command(CMD_RESET);

    /*
     * Datasheet specifies waiting after RESET.
     */
    delay_ms(2);

    /*
     * Device powers up in RDATAC.
     * Stop continuous data before register access.
     */
    ads_command(CMD_SDATAC);

    /*
     * Important: allow >= 4 tCLK after SDATAC.
     */
    delay_us(50);

    /* ========================================================
     * Configuration
     * ======================================================== */

    /*
     * CONFIG1 = 0x01
     *
     * 250 SPS
     */
    ads_write_reg(
        REG_CONFIG1,
        0x01);

    /*
     * CONFIG2 = 0xA0
     *
     * Internal reference enabled
     * 2.42 V reference
     * Test signal disabled
     */
    ads_write_reg(
        REG_CONFIG2,
        0xA0);

    /*
     * Lead-off disabled.
     */
    ads_write_reg(
        REG_LOFF,
        0x00);

    /*
     * CH1:
     * powered down
     * input short
     */
    ads_write_reg(
        REG_CH1SET,
        0x81);

    /*
     * CH2:
     *
     * normal electrode input
     * PGA gain = 6
     */
    ads_write_reg(
        REG_CH2SET,
        0x00);

    /*
     * RLD:
     *
     * PDB_RLD = 1
     * RLD2N = 1
     * RLD2P = 1
     *
     * 0b00101100 = 0x2C
     */
    ads_write_reg(
        REG_RLD_SENS,
        0x2C);

    /*
     * Lead-off sense disabled.
     */
    ads_write_reg(
        REG_LOFF_SENS,
        0x00);

    /*
     * Respiration disabled.
     */
    ads_write_reg(
        REG_RESP1,
        0x00);

    /*
     * RESP2 reset/default already selects
     * internal RLD reference on this device.
     *
     * Explicitly write it here.
     */
    ads_write_reg(
        REG_RESP2,
        0x02);

    /*
     * Allow internal reference to settle.
     */
    delay_ms(150);

    /*
     * Verify ID.
     */
    id = ads_read_reg(REG_ID);

    /*
     * Send ID to PC so you can see it.
     */
    uart_puts("ADS1292R ID=");
    uart_put_int32(id);
    uart_puts("\r\n");

    /*
     * Back to continuous data mode.
     */
    ads_command(CMD_RDATAC);

    delay_us(50);

    /*
     * Start conversions.
     */
    GPIO_setDio(PIN_ECG_START);

    /*
     * Wait a little for the digital filter.
     */
    delay_ms(20);

    return true;
}


/* ============================================================
 * Read one ADS1292R frame
 *
 * STATUS = 3 bytes
 * CH1    = 3 bytes
 * CH2    = 3 bytes
 * ============================================================ */

static int32_t ads_read_ch2(void)
{
    uint8_t frame[9];

    /*
     * DRDY goes LOW when conversion is ready.
     */
    while (GPIO_readDio(PIN_ECG_DRDY))
    {
    }

    /*
     * ADS1292R has a 4-tCLK update/keep-out interval.
     * Wait conservatively before starting SCLK.
     */
    delay_us(20);

    ads_cs_low();

    for (int i = 0; i < 9; i++)
    {
        frame[i] = spi_transfer(0x00);
    }

    ads_cs_high();

    /*
     * CH2 = bytes 6,7,8
     */
    return ads_s24(
        frame[6],
        frame[7],
        frame[8]);
}


/* ============================================================
 * Main
 * ============================================================ */

int main(void)
{
    /*
     * Basic board GPIO setup.
     */
    gpio_init();

    /*
     * Make sure IMU is deselected.
     */
    GPIO_setDio(PIN_IMU_CS);

    /*
     * SPI.
     */
    spi_init();

    /*
     * UART.
     */
    uart_init();

    uart_puts("\r\n");
    uart_puts("ECG START\r\n");

    /*
     * ADS1292R.
     */
    ads1292_init();

    uart_puts("STREAM\r\n");

    /*
     * Continuous ECG stream.
     *
     * One sample per line.
     */
    while (1)
    {
        int32_t ecg;

        ecg = ads_read_ch2();

        uart_put_int32(ecg);
        uart_puts("\r\n");
    }
}