/*
 * ============================================================================
 * bsp_spi.c
 * Board Support Package - Shared SPI Bus Manager Implementation
 * Dynamic IOC Pin Remapping for ADS1292R (Mode 1) & ADXL362 (Mode 0 Swapped)
 * ============================================================================
 */

#include "bsp_spi.h"
#include "bsp_pins.h"
#include <unistd.h>
#include <string.h>

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/ssi.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/cpu.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_types.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_memmap.h>
#include <ti/devices/cc13x2_cc26x2/inc/hw_ssi.h>
#include <ti/drivers/dpl/ClockP.h>
#include <ti/drivers/spi/SPICC26X2DMA.h>

/* Global POSIX Mutex */
pthread_mutex_t spi_bus_mutex;

static SPI_Handle    s_spi_handle = NULL;
static bsp_spi_dev_t s_current_dev = BSP_SPI_DEV_NONE;
static bool          s_initialized = false;
static uint8_t       s_ecg_cs_pin = CONFIG_GPIO_ECG_CS;
static bool          s_imu_swap_pins = true;
static uint32_t      s_ecg_frame_format = SPI_POL0_PHA1;

static inline void delay_us(uint32_t us)
{
    /* 48 MHz CPU: ~12 cycles per loop iteration in flash with cache */
    CPUdelay(12 * us);
}

bool bsp_spi_init(void)
{
    if (s_initialized && s_spi_handle != NULL) {
        return true;
    }

    /* Initialize TI Drivers SPI subsystem */
    SPI_init();

    /* Initialize POSIX recursive mutex with priority inheritance */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&spi_bus_mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    /* Guarantee candidate SPI chip select lines are configured and de-asserted (HIGH) */
    GPIO_setConfig(CONFIG_GPIO_ECG_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);

    GPIO_setConfig(20, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(20, 1);

    GPIO_setConfig(CONFIG_GPIO_IMU_CS, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    /* Open SPI controller handle ONCE. It remains open permanently to avoid RTOS primitive thrashing. */
    if (s_spi_handle == NULL) {
        SPI_Params params;
        SPI_Params_init(&params);
        params.mode = SPI_CONTROLLER;
        params.bitRate = 1000000;
        params.frameFormat = SPI_POL0_PHA0;
        params.dataSize = 8;
        params.transferMode = SPI_MODE_BLOCKING;

        s_spi_handle = SPI_open(CONFIG_SPI_0, &params);
        if (!s_spi_handle) {
            return false;
        }
    }

    s_current_dev = BSP_SPI_DEV_NONE;
    s_initialized = true;

    return true;
}

static bool configure_spi_device(bsp_spi_dev_t dev)
{
    if (s_spi_handle == NULL) {
        return false;
    }

    if (s_current_dev == dev) {
        return true;
    }

    /* Wait for any in-flight transmission to completely leave the shift register (bounded timeout) */
    uint32_t busy_to = 2000;
    while (SSIBusy(SSI0_BASE) && --busy_to);

    /* Disable SSI before updating clock rate and frame format */
    SSIDisable(SSI0_BASE);

    ClockP_FreqHz freq;
    ClockP_getCpuFreq(&freq);

    uint32_t bitRate = 1000000;
    uint32_t ssiFormat = SSI_FRF_MOTO_MODE_0;

    if (dev == BSP_SPI_DEV_ADS1292) {
        bitRate = 250000; /* Exact 250 kHz from 08_ecg_dedicated/main.c line 712 */
        if (s_ecg_frame_format == SPI_POL0_PHA0) {
            ssiFormat = SSI_FRF_MOTO_MODE_0;
        } else if (s_ecg_frame_format == SPI_POL0_PHA1) {
            ssiFormat = SSI_FRF_MOTO_MODE_1;
        } else if (s_ecg_frame_format == SPI_POL1_PHA0) {
            ssiFormat = SSI_FRF_MOTO_MODE_2;
        } else {
            ssiFormat = SSI_FRF_MOTO_MODE_3;
        }
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        bitRate = 1000000;
        ssiFormat = SSI_FRF_MOTO_MODE_0;
    } else {
        return false;
    }

    /* Update driver object fields in-place (zero memory allocations) */
    SPICC26X2DMA_Object *obj = (SPICC26X2DMA_Object *)s_spi_handle->object;
    obj->bitRate = bitRate;
    obj->format = (uint8_t)ssiFormat;

    /* Reconfigure SSI clock and format in hardware while SSI0 is disabled */
    SSIConfigSetExpClk(SSI0_BASE, freq.lo, ssiFormat, SSI_MODE_MASTER, bitRate, 8);

    /* Route IOC pins for target peripheral BEFORE enabling SSI0 to prevent glitch clocks */
    if (dev == BSP_SPI_DEV_ADS1292) {
        /* ADS1292R: DIN on DIO 9, DOUT on DIO 8 */
        IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        if (s_imu_swap_pins) {
            /* Rev 3.5 schematic: ADXL362 SDI (Pin 6) on DIO 8, SDO (Pin 7) on DIO 9 */
            IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
            IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
        } else {
            /* Standard pinout: MOSI on DIO 9, MISO on DIO 8 */
            IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
            IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
        }
    }

    /* Flush any stale bytes from SSI0 RX FIFO and clear overrun/timeout status */
    while (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE) {
        (void)HWREG(SSI0_BASE + SSI_O_DR);
    }
    SSIIntClear(SSI0_BASE, SSI_RXOR | SSI_RXTO);

    /* Re-enable SSI after pin muxing and FIFO flush are complete */
    SSIEnable(SSI0_BASE);
    delay_us(15); /* Allow SCLK & MOSI/MISO pin mux to settle completely while CS is still HIGH */

    s_current_dev = dev;
    return true;
}

bool bsp_spi_acquire(bsp_spi_dev_t dev)
{
    if (!s_initialized || s_spi_handle == NULL) {
        if (!bsp_spi_init()) {
            return false;
        }
    }

    pthread_mutex_lock(&spi_bus_mutex);

    /* Keep CS lines de-asserted during bus configuration */
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    GPIO_write(20, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    if (!configure_spi_device(dev)) {
        pthread_mutex_unlock(&spi_bus_mutex);
        return false;
    }

    /* Assert selected device CS line (Active LOW) */
    if (dev == BSP_SPI_DEV_ADS1292) {
        GPIO_write(s_ecg_cs_pin, 0);
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        GPIO_write(CONFIG_GPIO_IMU_CS, 0);
    }
    delay_us(10);

    return true;
}

void bsp_spi_release(bsp_spi_dev_t dev)
{
    /* Ensure SSI hardware has completely finished transmitting all bits before de-asserting CS (bounded timeout) */
    uint32_t busy_to = 2000;
    while (SSIBusy(SSI0_BASE) && --busy_to);

    /* De-assert CS line (Active LOW -> Idle HIGH) */
    if (dev == BSP_SPI_DEV_ADS1292) {
        GPIO_write(s_ecg_cs_pin, 1);
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    }
    delay_us(10);

    /* Immediately park SPI0 back on ADS1292R Mode 1 (250 kHz) after ADXL362 finishes,
     * so when DRDY falls 3.8ms later, ADS1292R is read with zero SSI reconfiguration! */
    if (dev == BSP_SPI_DEV_ADXL362) {
        configure_spi_device(BSP_SPI_DEV_ADS1292);
    }

    pthread_mutex_unlock(&spi_bus_mutex);
}

void bsp_spi_set_ecg_cs_pin(uint8_t pin)
{
    if (s_ecg_cs_pin != pin) {
        s_ecg_cs_pin = pin;
        s_current_dev = BSP_SPI_DEV_NONE;
    }
}

uint8_t bsp_spi_get_ecg_cs_pin(void)
{
    return s_ecg_cs_pin;
}

void bsp_spi_set_imu_swap_pins(bool swap)
{
    if (s_imu_swap_pins != swap) {
        s_imu_swap_pins = swap;
        s_current_dev = BSP_SPI_DEV_NONE;
    }
}

void bsp_spi_set_ecg_frame_format(uint32_t frameFormat)
{
    if (s_ecg_frame_format != frameFormat) {
        s_ecg_frame_format = frameFormat;
        s_current_dev = BSP_SPI_DEV_NONE;
    }
}

bool bsp_spi_transfer(SPI_Transaction *trans)
{
    if (!s_spi_handle || !trans) {
        return false;
    }
    return SPI_transfer(s_spi_handle, trans);
}

SPI_Handle bsp_spi_get_handle(void)
{
    return s_spi_handle;
}

bsp_spi_dev_t bsp_spi_get_current_dev(void)
{
    return s_current_dev;
}
