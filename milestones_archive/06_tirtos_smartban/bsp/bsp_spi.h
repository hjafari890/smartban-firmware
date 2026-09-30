/*
 * ============================================================================
 * bsp_spi.h
 * Board Support Package - Shared SPI Bus Manager with Dynamic IOC Remapping
 * ============================================================================
 */

#ifndef BSP_SPI_H_
#define BSP_SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>
#include <ti/drivers/SPI.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_SPI_DEV_NONE = 0,
    BSP_SPI_DEV_ADS1292,    /* Mode 1, 250 kHz, CS DIO 11, Standard MOSI=DIO9/MISO=DIO8 */
    BSP_SPI_DEV_ADXL362     /* Mode 0, 1 MHz,   CS DIO 15, Swapped MOSI=DIO8/MISO=DIO9 */
} bsp_spi_dev_t;

#define DEV_ADS1292 BSP_SPI_DEV_ADS1292
#define DEV_ADXL362 BSP_SPI_DEV_ADXL362

/**
 * @brief Global POSIX mutex protecting the shared SPI bus with priority inheritance.
 */
extern pthread_mutex_t spi_bus_mutex;

/**
 * @brief Initialize shared SPI bus manager, CS pins, and POSIX mutex.
 * @return true on success, false otherwise.
 */
bool bsp_spi_init(void);

/**
 * @brief Acquire SPI bus for a specific peripheral device.
 *        Locks spi_bus_mutex, applies dynamic IOC pin remapping, reconfigures
 *        clock mode/rate if needed, and asserts target chip select (LOW).
 * @param dev Target SPI device (BSP_SPI_DEV_ADS1292 or BSP_SPI_DEV_ADXL362).
 * @return true if bus was acquired and device configured successfully.
 */
bool bsp_spi_acquire(bsp_spi_dev_t dev);

/**
 * @brief Release SPI bus for a specific peripheral device.
 *        De-asserts target chip select (HIGH) and unlocks spi_bus_mutex.
 * @param dev Target SPI device.
 */
void bsp_spi_release(bsp_spi_dev_t dev);

/**
 * @brief Execute an SPI transfer on the currently acquired device.
 * @param trans Pointer to SPI_Transaction.
 * @return true on success, false on failure.
 */
bool bsp_spi_transfer(SPI_Transaction *trans);

/**
 * @brief Get the active SPI driver handle.
 * @return Current SPI_Handle or NULL.
 */
SPI_Handle bsp_spi_get_handle(void);

/**
 * @brief Get the current owner of the SPI bus.
 * @return Current bsp_spi_dev_t.
 */
bsp_spi_dev_t bsp_spi_get_current_dev(void);

/**
 * @brief Configure the GPIO pin used for ADS1292R Chip Select.
 * @param pin GPIO pin index (e.g. CONFIG_GPIO_ECG_CS or DIO 20).
 */
void bsp_spi_set_ecg_cs_pin(uint8_t pin);

/**
 * @brief Get the currently configured ADS1292R Chip Select pin.
 * @return Active CS GPIO pin index.
 */
uint8_t bsp_spi_get_ecg_cs_pin(void);

/**
 * @brief Configure SPI pin routing for ADXL362 (swapped vs standard).
 * @param swap true for Rev 3.5 routing (DIO8=TX, DIO9=RX), false for standard.
 */
void bsp_spi_set_imu_swap_pins(bool swap);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H_ */
