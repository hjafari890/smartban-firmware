/*
 * ============================================================================
 * bsp_i2c.h
 * Board Support Package - Shared I2C0 Bus Manager with POSIX Mutex
 * ============================================================================
 */

#ifndef BSP_I2C_H_
#define BSP_I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Global POSIX mutex protecting the shared I2C0 bus with priority inheritance.
 */
extern pthread_mutex_t i2c_bus_mutex;

/**
 * @brief Initialize shared I2C0 bus driver at 400 kHz and create POSIX mutex.
 * @return true on success, false otherwise.
 */
bool bsp_i2c_init(void);

/**
 * @brief Manually acquire the I2C bus mutex for multi-step atomic sequences.
 */
void bsp_i2c_acquire(void);

/**
 * @brief Release the I2C bus mutex.
 */
void bsp_i2c_release(void);

/**
 * @brief Perform an I2C transaction with automatic mutex protection.
 * @param trans Pointer to I2C_Transaction.
 * @return true on success, false on failure.
 */
bool bsp_i2c_transfer(I2C_Transaction *trans);

/**
 * @brief Read an 8-bit register from an 8-bit address device.
 */
bool bsp_i2c_read_reg8(uint8_t dev_addr, uint8_t reg, uint8_t *val);

/**
 * @brief Write an 8-bit register on an 8-bit address device.
 */
bool bsp_i2c_write_reg8(uint8_t dev_addr, uint8_t reg, uint8_t val);

/**
 * @brief Read a 16-bit register from an 8-bit address device (e.g. VCNL4040, OPT4041).
 * @param big_endian true = MSB first (OPT4041), false = LSB first (VCNL4040).
 */
bool bsp_i2c_read_reg16(uint8_t dev_addr, uint8_t reg, uint16_t *val, bool big_endian);

/**
 * @brief Write a 16-bit register to an 8-bit address device.
 */
bool bsp_i2c_write_reg16(uint8_t dev_addr, uint8_t reg, uint16_t val, bool big_endian);

/**
 * @brief Read a 16-bit register from a 16-bit address device (specifically MLX90632).
 */
bool bsp_i2c_read_reg16_addr16(uint8_t dev_addr, uint16_t reg_addr, uint16_t *val);

/**
 * @brief Write a 16-bit register to a 16-bit address device (specifically MLX90632).
 */
bool bsp_i2c_write_reg16_addr16(uint8_t dev_addr, uint16_t reg_addr, uint16_t val);

/**
 * @brief Write a 1-byte direct command without register address (specifically CH455H).
 */
bool bsp_i2c_write_cmd(uint8_t dev_addr, uint8_t cmd);

/**
 * @brief Read arbitrary byte buffer starting from register address.
 */
bool bsp_i2c_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len);

/**
 * @brief Write arbitrary byte buffer starting with register address.
 */
bool bsp_i2c_write_bytes(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len);

/**
 * @brief Get the active I2C driver handle.
 * @return Current I2C_Handle or NULL.
 */
I2C_Handle bsp_i2c_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_I2C_H_ */
