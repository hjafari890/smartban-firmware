/* =========================================================================
 * ads1292.c - see ads1292.h for the public API and its assumptions.
 *
 * Written against the SimpleLink CC13XX_CC26XX SDK DriverLib
 * (ti/devices/cc13x2_cc26x2/driverlib/*.h), matching the include paths
 * confirmed for your CC2652R1 project. DriverLib function names are stable
 * within an SDK major version but do shift occasionally between releases --
 * if the build complains about an undeclared function below, it's almost
 * always a one-line rename; check the matching header in your SDK install.
 * ========================================================================= */
#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/gpio.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ssi.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/prcm.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/cpu.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/interrupt.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_ints.h>

#include "pins.h"
#include "ads1292.h"

/* ---- ADS1292R SPI command opcodes (datasheet SBAS502, Table 12) --------- */
#define CMD_WAKEUP      0x02
#define CMD_STANDBY     0x04
#define CMD_RESET       0x06
#define CMD_START       0x08
#define CMD_STOP        0x0A
#define CMD_RDATAC      0x10
#define CMD_SDATAC      0x11
#define CMD_RDATA       0x12
#define CMD_RREG        0x20
#define CMD_WREG        0x40

/* ---- Register map -------------------------------------------------------- */
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
#define REG_GPIO_REG    0x0B

static volatile bool s_drdyFlag = false;
static uint8_t s_lastId = 0x00;

/* -------------------------------------------------------------------------
 * Low-level GPIO helpers
 * ---------------------------------------------------------------------- */
static void CsLow(void)  { GPIO_writeDio(PIN_ECG_CS, 0); }
static void CsHigh(void) { GPIO_writeDio(PIN_ECG_CS, 1); }

/* Crude busy-wait delays. CPUdelay() burns roughly 3 cycles per count, so
 * this is approximate -- fine for the millisecond-scale, non-timing-critical
 * waits used here (reset pulse width, rail settling). Do NOT reuse this for
 * anything that needs accurate timing. */
static void DelayUs(uint32_t us)
{
    /* Assumes a 48 MHz system clock, typical default for CC2652R1. */
    CPUdelay((48 * us) / 3);
}
static void DelayMs(uint32_t ms) { DelayUs(ms * 1000); }

/* -------------------------------------------------------------------------
 * SPI byte transfer (blocking, polled -- no interrupts on the SPI itself)
 * ---------------------------------------------------------------------- */
static uint8_t SpiTransferByte(uint8_t out)
{
    uint32_t rxWord;
    SSIDataPut(SSI0_BASE, out);
    SSIDataGet(SSI0_BASE, &rxWord);
    return (uint8_t)(rxWord & 0xFF);
}

static void WriteReg(uint8_t reg, uint8_t value)
{
    CsLow();
    SpiTransferByte(CMD_WREG | reg);
    SpiTransferByte(0x00); /* number of registers to write, minus 1 */
    SpiTransferByte(value);
    CsHigh();
}

static uint8_t ReadReg(uint8_t reg)
{
    uint8_t value;
    CsLow();
    SpiTransferByte(CMD_RREG | reg);
    SpiTransferByte(0x00); /* number of registers to read, minus 1 */
    value = SpiTransferByte(0x00);
    CsHigh();
    return value;
}

static void SendCommand(uint8_t cmd)
{
    CsLow();
    SpiTransferByte(cmd);
    CsHigh();
}

/* -------------------------------------------------------------------------
 * DRDY interrupt: single combined GPIO vector on CC26x2, shared by all DIOs.
 * Keep this ISR to "set a flag and get out" -- all SPI I/O happens in the
 * main loop so a slow/blocked SPI transfer can never stack up inside an ISR.
 * ---------------------------------------------------------------------- */
static void GpioIsr(void)
{
    uint32_t mis = GPIO_getEventMultiDio(1 << PIN_ECG_DRDY);
    if (mis != 0) {
        GPIO_clearEventDio(PIN_ECG_DRDY);
        s_drdyFlag = true;
    }
}

bool ADS1292_DrdyAsserted(void)
{
    if (s_drdyFlag) {
        s_drdyFlag = false;
        return true;
    }
    return false;
}

/* -------------------------------------------------------------------------
 * Init
 * ---------------------------------------------------------------------- */
static void ConfigureGpios(void)
{
    /* Plain push-pull outputs, driven low initially except CS (idle high). */
    IOCPinTypeGpioOutput(PIN_1V8_EN);
    IOCPinTypeGpioOutput(PIN_IMU_CS);
    IOCPinTypeGpioOutput(PIN_ECG_CS);
    IOCPinTypeGpioOutput(PIN_ECG_START);
    IOCPinTypeGpioOutput(PIN_ECG_PWDN);

    GPIO_writeDio(PIN_1V8_EN, 0);
    GPIO_writeDio(PIN_IMU_CS, 1);   /* keep the IMU off the shared SPI bus */
    GPIO_writeDio(PIN_ECG_CS, 1);
    GPIO_writeDio(PIN_ECG_START, 0);
    GPIO_writeDio(PIN_ECG_PWDN, 0);

    /* DRDY: input, pulled up, falling-edge interrupt. */
    IOCPinTypeGpioInput(PIN_ECG_DRDY);
    IOCIOPortPullSet(PIN_ECG_DRDY, IOC_IOPULL_UP);
    IOCIOIntSet(PIN_ECG_DRDY, IOC_INT_ENABLE, IOC_FALLING_EDGE);

    IntRegister(INT_GPIO_COMB, GpioIsr);
    IntPendClear(INT_GPIO_COMB);
    IntEnable(INT_GPIO_COMB);
}

static void ConfigureSpi(void)
{
    /* SSI0 needs its own power/clock domain enabled before it will latch
     * any register writes -- this is the one PRCM step TI Drivers/SysConfig
     * normally does for you behind the scenes. */
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    IOCPinTypeSsiMaster(SSI0_BASE, PIN_SPI_MISO, PIN_SPI_MOSI,
                         IOID_UNUSED, PIN_SPI_SCLK);

    SSIDisable(SSI0_BASE);
    /* ADS1292R requires SPI mode 1: CPOL = 0, CPHA = 1. Keep the clock
     * conservative (1 MHz) for the first bring-up; the datasheet allows
     * faster, push it up once you've confirmed clean reads. */
    SSIConfigSetExpClk(SSI0_BASE, 48000000, SSI_FRF_MOTO_MODE_1,
                        SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI0_BASE);
}

void ADS1292_Init(void)
{
    ConfigureGpios();
    ConfigureSpi();

    GPIO_writeDio(PIN_1V8_EN, 1);   /* power the ADS1292R analog/digital rails */
    DelayMs(10);                     /* let the LDO rail settle */

    /* Hardware reset pulse (datasheet: PWDN/RESET# low >= 1 clock cycle;
     * a few microseconds is comfortably more than enough here). */
    GPIO_writeDio(PIN_ECG_PWDN, 0);
    DelayUs(10);
    GPIO_writeDio(PIN_ECG_PWDN, 1);
    DelayMs(20);                     /* internal oscillator start-up margin */

    SendCommand(CMD_SDATAC);         /* make sure we're not mid-stream from a previous session */

    /* Baseline register set for a single ECG channel with RLD enabled.
     * These are the commonly-used starting values from ADS1292R reference
     * designs -- cross-check the bitfield meanings against your copy of
     * SBAS502 if you plan to tune gain/data-rate/lead-off detection later. */
    WriteReg(REG_CONFIG1,   0x02); /* 500 SPS */
    WriteReg(REG_CONFIG2,   0xA0); /* internal reference, test signal off */
    WriteReg(REG_LOFF,      0x10);
    WriteReg(REG_CH1SET,    0x00); /* gain 6, normal electrode input, powered on */
    WriteReg(REG_CH2SET,    0x81); /* channel 2 (respiration) powered down -- unused */
    WriteReg(REG_RLD_SENS,  0x20); /* right-leg-drive derived from channel 1 */
    WriteReg(REG_LOFF_SENS, 0x00); /* lead-off detection disabled for v1 -- simpler to bring up */
    WriteReg(REG_GPIO_REG,  0x00);
}

bool ADS1292_SelfTest(void)
{
    s_lastId = ReadReg(REG_ID);
    return (s_lastId != 0x00) && (s_lastId != 0xFF);
}

uint8_t ADS1292_LastId(void) { return s_lastId; }

void ADS1292_StartContinuous(void)
{
    SendCommand(CMD_RDATAC);
    GPIO_writeDio(PIN_ECG_START, 1);
}

int32_t ADS1292_ReadSample(void)
{
    uint8_t frame[9];
    int32_t sample;

    CsLow();
    for (int i = 0; i < 9; i++) {
        frame[i] = SpiTransferByte(0x00);
    }
    CsHigh();

    /* frame[0..2] = status bits, frame[3..5] = channel 1, frame[6..8] = channel 2 (unused) */
    sample = ((int32_t)frame[3] << 16) | ((int32_t)frame[4] << 8) | frame[5];
    if (sample & 0x00800000) {       /* sign-extend the 24-bit two's complement value */
        sample |= 0xFF000000;
    }
    return sample;
}
