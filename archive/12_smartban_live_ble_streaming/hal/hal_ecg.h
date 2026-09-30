/*
 * ============================================================================
 * hal_ecg.h
 * Hardware Abstraction Layer - TI ADS1292R 24-Bit ECG & Respiration Driver
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

/* ADS1292 SPI Commands */
#define ADS1292_CMD_WAKEUP      0x02
#define ADS1292_CMD_STANDBY     0x04
#define ADS1292_CMD_RESET       0x06
#define ADS1292_CMD_START       0x08
#define ADS1292_CMD_STOP        0x0A
#define ADS1292_CMD_OFFSETCAL   0x1A
#define ADS1292_CMD_RDATAC      0x10
#define ADS1292_CMD_SDATAC      0x11
#define ADS1292_CMD_RDATA       0x12
#define ADS1292_CMD_RREG        0x20
#define ADS1292_CMD_WREG        0x40

/* ADS1292 Register Addresses */
#define ADS1292_REG_ID          0x00
#define ADS1292_REG_CONFIG1     0x01
#define ADS1292_REG_CONFIG2     0x02
#define ADS1292_REG_LOFF        0x03
#define ADS1292_REG_CH1SET      0x04
#define ADS1292_REG_CH2SET      0x05
#define ADS1292_REG_RLD_SENS    0x06
#define ADS1292_REG_LOFF_SENS   0x07
#define ADS1292_REG_LOFF_STAT   0x08
#define ADS1292_REG_RESP1       0x09
#define ADS1292_REG_RESP2       0x0A
#define ADS1292_REG_GPIO        0x0B

/* Clinical Register Tuning Constants */
#define ADS1292_CONFIG1_500SPS          0x02    /* 500 SPS continuous conversion */
#define ADS1292_CONFIG2_CLINICAL        0xE0    /* Bit7=1, LOFF comparator ON, REF buffer ON (2.42V) */
#define ADS1292_RLD_SENS_CH2_DRIVE      0x2C    /* RLD buffer ON, PGA chop fmod/16, route CH2 (IN2P+IN2N) */
#define ADS1292_LOFF_22NA_90PCT         0x54    /* 90%/10% comp threshold, 22 nA DC lead-off current */
#define ADS1292_LOFF_SENS_CH2           0x0C    /* Lead-off sensing enabled on CH2 (IN2P, IN2N) */
#define ADS1292_RESP2_CLINICAL          0x07    /* CALIB_ON=0, 32kHz resp freq, RLDREF internal (AVDD+AVSS)/2, Bits 1:0=11 */
#define ADS1292_RESP2_CALIB_ON          0x87    /* CALIB_ON=1 for hardware offset calibration */

/* Conversion Constants: VREF = 2.42V, Gain = 6 */
#define ADS1292_UV_PER_COUNT    0.048077f

/* Self-Test & Operating Modes */
typedef enum {
    HAL_ECG_MODE_SQUARE_WAVE     = 1,   /* Mode 1: 1 Hz, +/-1 mV internal test generator */
    HAL_ECG_MODE_INPUT_SHORT     = 2,   /* Mode 2: Differential inputs shorted internally (noise/offset) */
    HAL_ECG_MODE_TEMPERATURE     = 3,   /* Mode 3: Internal silicon die temperature diode */
    HAL_ECG_MODE_LIVE_ELECTRODE  = 4    /* Mode 4: External electrode inputs (Lead I + Respiration) */
} hal_ecg_mode_t;

/* Alias to preserve compatibility */
typedef hal_ecg_mode_t EcgTestMode_t;
#define ECG_MODE_SQUARE_WAVE    HAL_ECG_MODE_SQUARE_WAVE
#define ECG_MODE_INPUT_SHORT    HAL_ECG_MODE_INPUT_SHORT
#define ECG_MODE_TEMPERATURE    HAL_ECG_MODE_TEMPERATURE
#define ECG_MODE_LIVE_ELECTRODE HAL_ECG_MODE_LIVE_ELECTRODE

typedef struct {
    uint32_t timestamp_ms;
    uint8_t  status;            /* Byte 0 status (bits 7:4 = 0xC0) */
    bool     lead_off;          /* True if RA or LA electrode disconnected */
    uint8_t  pad;               /* Alignment padding */
    int32_t  raw_ch1;           /* 24-bit signed Channel 1 (Respiration / Test) */
    int32_t  raw_ch2;           /* 24-bit signed Channel 2 (Lead I ECG / Test) */
    float    microvolts_ch1;    /* Calibrated Channel 1 in microvolts */
    float    microvolts_ch2;    /* Calibrated Channel 2 in microvolts */
} hal_ecg_sample_t;

/**
 * @brief Initialize ADS1292R:
 *        - Verifies 1.8V power domain
 *        - Hardware reset pulse on PWDN line
 *        - Mandatory POR wait (>= 512 ms)
 *        - Probes candidate CS pins (DIO 11, DIO 20)
 *        - Reads Device ID (0x73 / 0x53)
 *        - Configures default mode (Mode 4 Live Electrodes or Mode 1)
 * @return true on success, false otherwise.
 */
bool hal_ecg_init(void);

/**
 * @brief Switch ADS1292R operating / test mode:
 *        SDATAC -> Write registers -> Reference settle -> START -> RDATAC.
 * @param mode Target mode (1 to 4).
 * @return true on success.
 */
bool hal_ecg_set_mode(hal_ecg_mode_t mode);

/**
 * @brief Read a 9-byte conversion frame from the ADS1292R over SPI.
 * @param sample Output decoded sample.
 * @return true on success with valid 0xC0 sync header, false otherwise.
 */
bool hal_ecg_read_sample(hal_ecg_sample_t *sample);

/**
 * @brief Register falling-edge GPIO interrupt callback for DRDY (DIO 23).
 */
bool hal_ecg_register_drdy_callback(GPIO_CallbackFxn fxn);

/**
 * @brief Execute ADS1292R internal offset calibration (OFFSETCAL command 0x1A).
 *        Shorts inputs internally, calculates digital offset DAC compensation.
 * @return true on success.
 */
bool hal_ecg_calibrate_offset(void);

/**
 * @brief Query current active mode.
 */
hal_ecg_mode_t hal_ecg_get_mode(void);

/**
 * @brief Query if ADS1292R is online and initialized.
 */
bool hal_ecg_is_online(void);

/**
 * @brief Get device Silicon ID.
 */
uint8_t hal_ecg_get_id(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ECG_H_ */
