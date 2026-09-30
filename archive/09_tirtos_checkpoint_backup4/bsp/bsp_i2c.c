/*
 * ============================================================================
 * bsp_i2c.c
 * Board Support Package - Shared I2C0 Bus Manager Implementation
 * Thread-Safe Bus Arbitration with POSIX Priority Inheritance Mutex
 * ============================================================================
 */

#include "bsp_i2c.h"
#include "bsp_pins.h"
#include <string.h>

/* Global POSIX Mutex */
pthread_mutex_t i2c_bus_mutex;

static I2C_Handle s_i2c_handle = NULL;
static bool       s_initialized = false;

bool bsp_i2c_init(void)
{
    if (s_initialized && s_i2c_handle != NULL) {
        return true;
    }

    /* Initialize POSIX recursive mutex with priority inheritance */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&i2c_bus_mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    /* Open shared I2C0 peripheral at 400 kHz (Fast Mode) */
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;

    s_i2c_handle = I2C_open(CONFIG_I2C_0, &params);
    if (!s_i2c_handle) {
        return false;
    }

    s_initialized = true;
    return true;
}

void bsp_i2c_acquire(void)
{
    pthread_mutex_lock(&i2c_bus_mutex);
}

void bsp_i2c_release(void)
{
    pthread_mutex_unlock(&i2c_bus_mutex);
}

bool bsp_i2c_transfer(I2C_Transaction *trans)
{
    if (!s_i2c_handle || !trans) {
        return false;
    }

    pthread_mutex_lock(&i2c_bus_mutex);
    int_fast16_t res = I2C_transferTimeout(s_i2c_handle, trans, 5000);
    pthread_mutex_unlock(&i2c_bus_mutex);

    return (res == I2C_STATUS_SUCCESS);
}

bool bsp_i2c_read_reg8(uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
    if (!val) return false;
    uint8_t tx = reg;
    uint8_t rx = 0;
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = &tx;
    trans.writeCount = 1;
    trans.readBuf = &rx;
    trans.readCount = 1;

    if (bsp_i2c_transfer(&trans)) {
        *val = rx;
        return true;
    }
    return false;
}

bool bsp_i2c_write_reg8(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = tx;
    trans.writeCount = 2;
    trans.readBuf = NULL;
    trans.readCount = 0;

    return bsp_i2c_transfer(&trans);
}

bool bsp_i2c_read_reg16(uint8_t dev_addr, uint8_t reg, uint16_t *val, bool big_endian)
{
    if (!val) return false;
    uint8_t tx = reg;
    uint8_t rx[2] = { 0, 0 };
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = &tx;
    trans.writeCount = 1;
    trans.readBuf = rx;
    trans.readCount = 2;

    if (bsp_i2c_transfer(&trans)) {
        if (big_endian) {
            *val = ((uint16_t)rx[0] << 8) | rx[1];
        } else {
            *val = ((uint16_t)rx[1] << 8) | rx[0];
        }
        return true;
    }
    return false;
}

bool bsp_i2c_write_reg16(uint8_t dev_addr, uint8_t reg, uint16_t val, bool big_endian)
{
    uint8_t tx[3];
    tx[0] = reg;
    if (big_endian) {
        tx[1] = (uint8_t)(val >> 8);
        tx[2] = (uint8_t)(val & 0xFF);
    } else {
        tx[1] = (uint8_t)(val & 0xFF);
        tx[2] = (uint8_t)(val >> 8);
    }

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.targetAddress = dev_addr;
    trans.writeBuf = tx;
    trans.writeCount = 3;
    trans.readBuf = NULL;
    trans.readCount = 0;

    return bsp_i2c_transfer(&trans);
}

bool bsp_i2c_read_reg16_addr16(uint8_t dev_addr, uint16_t reg_addr, uint16_t *val)
{
    if (!val) return false;
    uint8_t tx[2] = { (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF) };
    uint8_t rx[2] = { 0, 0 };
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = tx;
    trans.writeCount = 2;
    trans.readBuf = rx;
    trans.readCount = 2;

    if (bsp_i2c_transfer(&trans)) {
        *val = ((uint16_t)rx[0] << 8) | rx[1];
        return true;
    }
    return false;
}

bool bsp_i2c_write_reg16_addr16(uint8_t dev_addr, uint16_t reg_addr, uint16_t val)
{
    uint8_t tx[4];
    tx[0] = (uint8_t)(reg_addr >> 8);
    tx[1] = (uint8_t)(reg_addr & 0xFF);
    tx[2] = (uint8_t)(val >> 8);
    tx[3] = (uint8_t)(val & 0xFF);

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.targetAddress = dev_addr;
    trans.writeBuf = tx;
    trans.writeCount = 4;
    trans.readBuf = NULL;
    trans.readCount = 0;

    return bsp_i2c_transfer(&trans);
}

bool bsp_i2c_write_cmd(uint8_t dev_addr, uint8_t cmd)
{
    uint8_t tx = cmd;
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = &tx;
    trans.writeCount = 1;
    trans.readBuf = NULL;
    trans.readCount = 0;

    return bsp_i2c_transfer(&trans);
}

bool bsp_i2c_read_bytes(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    if (!buf || len == 0) return false;
    uint8_t tx = reg;
    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));

    trans.targetAddress = dev_addr;
    trans.writeBuf = &tx;
    trans.writeCount = 1;
    trans.readBuf = buf;
    trans.readCount = len;

    return bsp_i2c_transfer(&trans);
}

bool bsp_i2c_write_bytes(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    if (!buf || len == 0 || len > 32) return false;
    uint8_t tx[33];
    tx[0] = reg;
    memcpy(&tx[1], buf, len);

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.targetAddress = dev_addr;
    trans.writeBuf = tx;
    trans.writeCount = len + 1;
    trans.readBuf = NULL;
    trans.readCount = 0;

    return bsp_i2c_transfer(&trans);
}

I2C_Handle bsp_i2c_get_handle(void)
{
    return s_i2c_handle;
}
