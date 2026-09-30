/*
 * ============================================================================
 * hal_ecg.h
 * Hardware Abstraction Layer - TI ADS1292R 24-Bit ECG & Respiration AFE
 * ============================================================================
 */

#ifndef HAL_ECG_H_
#define HAL_ECG_H_

#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/GPIO.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADS1292 Commands */
#define ADS1292_CMD_WAKEUP     0x02
#define ADS1292_CMD_STANDBY    0x04
#define ADS1292_CMD_RESET      0x06
#define ADS1292_CMD_START      0x08
#define ADS1292_CMD_STOP       0x0A
#define ADS1292_CMD_OFFSETCAL  0x1A
#define ADS1292_CMD_RDATAC     0x10
#define ADS1292_CMD_SDATAC     0x11
#define ADS1292_CMD_RDATA      0x12
#define ADS1292_CMD_RREG       0x20
#define ADS1292_CMD_WREG       0x40

/* ADS1292 Registers */
#define ADS1292_REG_ID         0x00
#define ADS1292_REG_CONFIG1    0x01
#define ADS1292_REG_CONFIG2    0x02
#define ADS1292_REG_LOFF       0x03
#define ADS1292_REG_CH1SET     0x04
#define ADS1292_REG_CH2SET     0x05
#define ADS1292_REG_RLD_SENS   0x06
#define ADS1292_REG_LOFF_SENS  0x07
#define ADS1292_REG_LOFF_STAT  0x08
#define ADS1292_REG_RESP1      0x09
#define ADS1292_REG_RESP2      0x0A
#define ADS1292_REG_GPIO       0x0B

/* Conversion Constant: 2.42V Ref, Gain 6 -> 0.0480803 uV/count */
#define ADS1292_UV_PER_COUNT   0.048080327f

typedef enum {
    HAL_ECG_RATE_125_SPS = 0x00,
    HAL_ECG_RATE_250_SPS = 0x01,
    HAL_ECG_RATE_500_SPS = 0x02,
    HAL_ECG_RATE_1000_SPS = 0x03
} hal_ecg_rate_t;

typedef enum {
    HAL_ECG_MODE_NORMAL = 0,    /* Live electrode inputs */
    HAL_ECG_MODE_TEST_1HZ_SQUARE,/* 1 Hz +/-1 mV internal calibration test signal */
    HAL_ECG_MODE_INPUT_SHORT,   /* Inputs shorted internally for baseline noise test */
    HAL_ECG_MODE_TEMP           /* Internal die temperature diode */
} hal_ecg_mode_t;

typedef struct {
    uint32_t timestamp_ms;      /* Monotonic node timestamp */
    int32_t  raw_ch1;           /* 24-bit sign-extended raw count */
    int32_t  raw_ch2;           /* 24-bit sign-extended raw count */
    float    microvolts_ch1;    /* Converted biopotential voltage in uV */
    float    microvolts_ch2;    /* Converted biopotential voltage in uV */
    uint8_t  status;            /* Status header byte */
    bool     lead_off;          /* Lead-off detection flag */
} hal_ecg_sample_t;

/**
 * @brief Initialize ADS1292R AFE:
 *        - Hardware reset pulse on PWDN
 *        - Mandatory 1000 ms digital core POR delay
 *        - START high, SDATAC, register configuration
 *        - 100 ms reference stabilization delay
 *        - Continuous conversion (RDATAC) mode entry
 * @param rate Sampling rate (250 SPS or 500 SPS recommended).
 * @return true on success, false if device ID invalid.
 */
bool hal_ecg_init(hal_ecg_rate_t rate);

/**
 * @brief Read a 9-byte continuous frame (RDATAC) from ADS1292R,
 *        verifying header 0xC0, sign-extending 24-bit channels,
 *        and converting to calibrated microvolts.
 * @param sample Output sample structure.
 * @return true on valid frame, false on bus error or sync mismatch.
 */
bool hal_ecg_read_sample(hal_ecg_sample_t *sample);

/**
 * @brief Configure ADS1292R input multiplexer for self-test or live signals.
 * @param mode Target mode (Normal, 1Hz square wave, Shorted, Temp).
 * @return true on success.
 */
bool hal_ecg_set_mode(hal_ecg_mode_t mode);

/**
 * @brief Register callback for ADS1292 DRDY falling-edge GPIO interrupt (DIO 23).
 * @param fxn Callback function to invoke from ISR.
 * @return true on success.
 */
bool hal_ecg_register_drdy_callback(GPIO_CallbackFxn fxn);

/**
 * @brief Retrieve read-only device ID byte (expect 0x73 or 0x53).
 */
uint8_t hal_ecg_get_id(void);

/**
 * @brief Query online status of ADS1292R AFE.
 */
bool hal_ecg_is_online(void);

/**
 * @brief Helper to convert a 24-bit signed word to microvolts.
 */
float hal_ecg_counts_to_uV(int32_t counts);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ECG_H_ */
