/*
 * ============================================================================
 * hal_imu.h
 * Hardware Abstraction Layer - ADI ADXL362 Ultra-Low Power 3-Axis MEMS IMU
 * ============================================================================
 */

#ifndef HAL_IMU_H_
#define HAL_IMU_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ADXL362 Register Map */
#define ADXL362_REG_DEVID_AD            0x00
#define ADXL362_REG_DEVID_MST           0x01
#define ADXL362_REG_PARTID              0x02
#define ADXL362_REG_REVID               0x03
#define ADXL362_REG_STATUS              0x0B
#define ADXL362_REG_FIFO_ENTRIES_L      0x0C
#define ADXL362_REG_FIFO_ENTRIES_H      0x0D
#define ADXL362_REG_XDATA_L             0x0E
#define ADXL362_REG_XDATA_H             0x0F
#define ADXL362_REG_YDATA_L             0x10
#define ADXL362_REG_YDATA_H             0x11
#define ADXL362_REG_ZDATA_L             0x12
#define ADXL362_REG_ZDATA_H             0x13
#define ADXL362_REG_TEMP_L              0x14
#define ADXL362_REG_TEMP_H              0x15
#define ADXL362_REG_SOFT_RESET          0x1F
#define ADXL362_REG_THRESH_ACT_L        0x20
#define ADXL362_REG_THRESH_ACT_H        0x21
#define ADXL362_REG_TIME_ACT            0x22
#define ADXL362_REG_THRESH_INACT_L      0x23
#define ADXL362_REG_THRESH_INACT_H      0x24
#define ADXL362_REG_TIME_INACT_L        0x25
#define ADXL362_REG_TIME_INACT_H        0x26
#define ADXL362_REG_ACT_INACT_CTL       0x27
#define ADXL362_REG_FIFO_CONTROL        0x28
#define ADXL362_REG_FIFO_SAMPLES        0x29
#define ADXL362_REG_INTMAP1             0x2A
#define ADXL362_REG_INTMAP2             0x2B
#define ADXL362_REG_FILTER_CTL          0x2C
#define ADXL362_REG_POWER_CTL           0x2D
#define ADXL362_REG_SELF_TEST           0x2E

/* SPI Commands */
#define ADXL362_CMD_WRITE_REG           0x0A
#define ADXL362_CMD_READ_REG            0x0B
#define ADXL362_CMD_READ_FIFO           0x0D

/* STATUS Register Masks */
#define ADXL362_STATUS_DATA_READY       (1 << 0)
#define ADXL362_STATUS_FIFO_READY       (1 << 1)
#define ADXL362_STATUS_FIFO_WATERMARK   (1 << 2)
#define ADXL362_STATUS_FIFO_OVERRUN     (1 << 3)
#define ADXL362_STATUS_ACT              (1 << 4)
#define ADXL362_STATUS_INACT            (1 << 5)
#define ADXL362_STATUS_AWAKE            (1 << 6)
#define ADXL362_STATUS_ERR_USER_REGS    (1 << 7)

/* Expected Silicon Identifiers */
#define ADXL362_VAL_DEVID_AD            0xAD
#define ADXL362_VAL_DEVID_MST           0x1D
#define ADXL362_VAL_PARTID              0xF2

typedef enum {
    HAL_IMU_RANGE_2G = 0,    /* +/- 2g dynamic range (1.0 mg/LSB) */
    HAL_IMU_RANGE_4G = 1,    /* +/- 4g dynamic range (2.0 mg/LSB) */
    HAL_IMU_RANGE_8G = 2     /* +/- 8g dynamic range (4.0 mg/LSB) */
} hal_imu_range_t;

typedef enum {
    HAL_IMU_TAG_X = 0,       /* Tag 00: X-axis acceleration */
    HAL_IMU_TAG_Y = 1,       /* Tag 01: Y-axis acceleration */
    HAL_IMU_TAG_Z = 2,       /* Tag 10: Z-axis acceleration */
    HAL_IMU_TAG_TEMP = 3     /* Tag 11: Temperature sample */
} hal_imu_tag_t;

typedef struct {
    uint32_t timestamp_ms;
    int16_t  x_mg;           /* X-axis acceleration in milli-g */
    int16_t  y_mg;           /* Y-axis acceleration in milli-g */
    int16_t  z_mg;           /* Z-axis acceleration in milli-g */
    float    total_g;        /* Vector magnitude: sqrt(x^2 + y^2 + z^2) / 1000.0f */
    float    pitch_deg;      /* Dynamic tilt pitch angle */
    float    roll_deg;       /* Dynamic tilt roll angle */
    float    temp_c;         /* On-chip temperature reading */
    uint8_t  status;         /* Status register reading */
} hal_imu_sample_t;

typedef struct {
    hal_imu_tag_t tag;       /* Decoded channel tag */
    int16_t       value;     /* Sign-extended value (mg for accel, LSB for temp) */
} hal_imu_fifo_sample_t;

/**
 * @brief Initialize ADXL362:
 *        - Soft reset (0x52 to 0x1F)
 *        - Verify DEVID_AD (0xAD), DEVID_MST (0x1D), PARTID (0xF2)
 *        - Configure measurement range (2g, 4g, or 8g) and 100 Hz ODR
 *        - Enable ultra-low noise and measurement mode (POWER_CTL = 0x22)
 * @param range Dynamic measurement range.
 * @return true on success, false on communication failure or ID mismatch.
 */
bool hal_imu_init(hal_imu_range_t range);

/**
 * @brief Burst-read 3-axis acceleration and temperature registers,
 *        applying 12-bit sign extension and range scaling.
 * @param sample Output sample structure.
 * @return true on success.
 */
bool hal_imu_read_sample(hal_imu_sample_t *sample);

/**
 * @brief Read hardware FIFO buffer with 2-bit tag decoding.
 * @param samples Array to receive decoded FIFO samples.
 * @param max_count Maximum number of samples to read.
 * @param actual_count Output number of samples actually read.
 * @return true on success.
 */
bool hal_imu_read_fifo(hal_imu_fifo_sample_t *samples, uint16_t max_count, uint16_t *actual_count);

/**
 * @brief Configure autonomous loop-mode activity / inactivity motion thresholds
 *        routed to INT1 (DIO 26) and INT2 (DIO 27).
 * @param act_mg Activity threshold in mg (e.g. 250 mg).
 * @param act_time_samples Activity sample duration (e.g. 4 samples).
 * @param inact_mg Inactivity threshold in mg (e.g. 150 mg).
 * @param inact_time_samples Inactivity sample duration (e.g. 50 samples).
 * @return true on success.
 */
bool hal_imu_configure_motion_wakeup(uint16_t act_mg, uint8_t act_time_samples,
                                     uint16_t inact_mg, uint16_t inact_time_samples);

/**
 * @brief Read ADXL362 STATUS register (0x0B).
 */
bool hal_imu_read_status(uint8_t *status);

/**
 * @brief Get device Part ID (expect 0xF2).
 */
uint8_t hal_imu_get_part_id(void);

/**
 * @brief Query online status of ADXL362 IMU.
 */
bool hal_imu_is_online(void);

/**
 * @brief Configure tare offset subtraction.
 */
void hal_imu_set_tare(int16_t tx, int16_t ty, int16_t tz);

/**
 * @brief Perform static multi-sample zero-g / 1g gravity calibration.
 *        Averages samples while resting flat on desk to compute precision offsets
 *        and Z-scale factor, normalizing resting magnitude to exactly 1.000g.
 * @param num_samples Number of samples to average (e.g. 64).
 * @return true on success.
 */
bool hal_imu_calibrate_static(uint8_t num_samples);

/**
 * @brief Retrieve current calibration parameters.
 */
void hal_imu_get_calibration(int16_t *ox, int16_t *oy, int16_t *oz, float *sz);

/**
 * @brief Apply explicit calibration parameters.
 */
typedef struct {
    float baseline_x_lsb;
    float baseline_y_lsb;
    float baseline_z_lsb;
    float stim_x_lsb;
    float stim_y_lsb;
    float stim_z_lsb;
    float delta_x_g;
    float delta_y_g;
    float delta_z_g;
    bool  pass_x;
    bool  pass_y;
    bool  pass_z;
    bool  all_pass;
} hal_imu_self_test_result_t;

/**
 * @brief Execute Analog Devices ADXL362 Electrostatic MEMS Self-Test (Datasheet Table 22).
 *        Actuates internal electrostatic force on X, Y, Z proof masses (0x2E = 0x01)
 *        and audits mechanical deflection against Table 22 limits.
 */
bool hal_imu_run_self_test(hal_imu_self_test_result_t *res);

#ifdef __cplusplus
}
#endif

#endif /* HAL_IMU_H_ */
