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

#include <ti/devices/cc13x2_cc26x2/driverlib/ssi.h>
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

bool bsp_spi_init(void)
{
    if (s_initialized && s_spi_handle != NULL) {
        return true;
    }

    /* Initialize POSIX recursive mutex with priority inheritance */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&spi_bus_mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    /* Guarantee both candidate SPI chip select lines are de-asserted (HIGH) */
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    GPIO_setConfig(20, GPIO_CFG_OUTPUT_INTERNAL | GPIO_CFG_OUT_STR_MED | GPIO_CFG_OUT_HIGH);
    GPIO_write(20, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    /* Open shared SPI controller ONCE with baseline parameters (ADXL362 defaults) */
    SPI_Params params;
    SPI_Params_init(&params);
    params.mode = SPI_CONTROLLER;
    params.bitRate = 1000000;
    params.frameFormat = SPI_POL0_PHA0;

    s_spi_handle = SPI_open(CONFIG_SPI_0, &params);
    if (!s_spi_handle) {
        return false;
    }

    s_current_dev = BSP_SPI_DEV_NONE;
    s_initialized = true;

    return true;
}

static bool configure_spi_device(bsp_spi_dev_t dev)
{
    /* Fast path: Device is already active and bus configured */
    if (s_current_dev == dev && s_spi_handle != NULL) {
        return true;
    }

    /* Close existing handle if it's open */
    if (s_spi_handle) {
        SPI_close(s_spi_handle);
        s_spi_handle = NULL;
    }

    SPI_Params params;
    SPI_Params_init(&params);
    params.mode = SPI_CONTROLLER;

    if (dev == BSP_SPI_DEV_ADS1292) {
        /*
         * ADS1292R Configuration:
         * - SPI Mode 1 (CPOL = 0, CPHA = 1)
         * - Clock Speed: 250 kHz
         * - Standard BoosterPack Pinout (DIO9=TX, DIO8=RX)
         */
        params.bitRate = 250000;
        params.frameFormat = SPI_POL0_PHA1;

        /* Ensure standard routing before opening */
        IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
        IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);

        s_spi_handle = SPI_open(CONFIG_SPI_0, &params);
        if (s_spi_handle) {
            s_current_dev = BSP_SPI_DEV_ADS1292;
            return true;
        }
    }
    else if (dev == BSP_SPI_DEV_ADXL362) {
        /*
         * ADXL362 Configuration:
         * - SPI Mode 0 (CPOL = 0, CPHA = 0)
         * - Clock Speed: 1.0 MHz
         * - Dynamic pin swap based on s_imu_swap_pins
         */
        params.bitRate = 1000000;
        params.frameFormat = SPI_POL0_PHA0;

        if (s_imu_swap_pins) {
            /* Rev 3.5 PCB Routing Inversion */
            IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
            IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
        } else {
            /* Standard Pinout */
            IOCPortConfigureSet(IOID_9, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT);
            IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_RX, IOC_STD_INPUT);
        }

        s_spi_handle = SPI_open(CONFIG_SPI_0, &params);
        if (s_spi_handle) {
            s_current_dev = BSP_SPI_DEV_ADXL362;
            return true;
        }
    }

    s_current_dev = BSP_SPI_DEV_NONE;
    return false;
}

bool bsp_spi_acquire(bsp_spi_dev_t dev)
{
    if (!s_initialized) {
        if (!bsp_spi_init()) {
            return false;
        }
    }

    pthread_mutex_lock(&spi_bus_mutex);

    /* Keep CS lines de-asserted during bus configuration */
    GPIO_write(s_ecg_cs_pin, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);

    if (!configure_spi_device(dev)) {
        pthread_mutex_unlock(&spi_bus_mutex);
        return false;
    }

    /* Assert selected device CS line (Active LOW) */
    if (dev == BSP_SPI_DEV_ADS1292) {
        GPIO_write(CONFIG_GPIO_ECG_CS, 0);
        if (s_ecg_cs_pin != CONFIG_GPIO_ECG_CS) {
            GPIO_write(s_ecg_cs_pin, 0);
        }
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        GPIO_write(CONFIG_GPIO_IMU_CS, 0);
    }
    usleep(15);

    return true;
}

void bsp_spi_release(bsp_spi_dev_t dev)
{
    /* De-assert CS line (Active LOW -> Idle HIGH) */
    if (dev == BSP_SPI_DEV_ADS1292) {
        GPIO_write(CONFIG_GPIO_ECG_CS, 1);
        if (s_ecg_cs_pin != CONFIG_GPIO_ECG_CS) {
            GPIO_write(s_ecg_cs_pin, 1);
        }
    } else if (dev == BSP_SPI_DEV_ADXL362) {
        GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    }
    usleep(15);

    pthread_mutex_unlock(&spi_bus_mutex);
}

void bsp_spi_set_ecg_cs_pin(uint8_t pin)
{
    s_ecg_cs_pin = pin;
}

uint8_t bsp_spi_get_ecg_cs_pin(void)
{
    return s_ecg_cs_pin;
}

void bsp_spi_set_imu_swap_pins(bool swap)
{
    s_imu_swap_pins = swap;
    s_current_dev = BSP_SPI_DEV_NONE;
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
