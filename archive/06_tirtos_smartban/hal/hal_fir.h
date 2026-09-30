/*
 * ============================================================================
 * hal_fir.h
 * Hardware Abstraction Layer - Melexis MLX90632 Far-IR Medical Thermometer
 * ============================================================================
 */

#ifndef HAL_FIR_H_
#define HAL_FIR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90632_I2C_ADDR           0x3A

/* Register Addresses */
#define MLX90632_REG_VERSION        0x240B
#define MLX90632_REG_CONTROL        0x3001
#define MLX90632_REG_STATUS         0x3FFF
#define MLX90632_REG_RAM_3          0x4003
#define MLX90632_REG_RAM_4          0x4004
#define MLX90632_REG_RAM_5          0x4005 /* sixRAM: Ambient sensor reference */
#define MLX90632_REG_RAM_6          0x4006
#define MLX90632_REG_RAM_7          0x4007
#define MLX90632_REG_RAM_8          0x4008 /* nineRAM: Reference voltage */

typedef struct {
    double P_R;
    double P_G;
    double P_T;
    double P_O;
    double Ea;
    double Eb;
    double Fa;
    double Fb;
    double Ga;
    double Gb;
    double Ka;
    double Ha;
    double Hb;
} mlx90632_calib_t;

/**
 * @brief Initialize MLX90632 sensor, read 14 EEPROM calibration constants,
 *        and configure continuous measurement mode.
 * @return true on success, false if communication fails.
 */
bool hal_fir_init(void);

/**
 * @brief Read calibrated ambient (sensor die) and non-contact object/skin temperatures.
 *        Executes 3-iteration non-linear optical polynomial preventing numerical overflow.
 * @param ambient_c Output ambient temperature in degrees Celsius.
 * @param object_c Output object/skin temperature in degrees Celsius.
 * @return true on valid measurement.
 */
bool hal_fir_read(float *ambient_c, float *object_c);

/**
 * @brief Pure math function: Calculate ambient sensor die temperature
 *        from sixRAM, nineRAM and calibration constants.
 * @return true on successful calculation, false on math singularity.
 */
bool hal_fir_calc_ambient(int16_t sixRAM, int16_t nineRAM, const mlx90632_calib_t *cal, float *ambient_c);

/**
 * @brief Pure math function: Calculate target/skin optical temperature
 *        from sixRAM, nineRAM, lowerRAM, upperRAM and calibration constants
 *        using 3-iteration non-linear polynomial.
 * @return true on successful calculation, false on math singularity.
 */
bool hal_fir_calc_object(int16_t sixRAM, int16_t nineRAM, int16_t lowerRAM, int16_t upperRAM,
                         const mlx90632_calib_t *cal, float *object_c);

/**
 * @brief Get copy of loaded EEPROM calibration constants.
 */
bool hal_fir_get_calib(mlx90632_calib_t *cal_out);

/**
 * @brief Check online status of MLX90632.
 */
bool hal_fir_is_online(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_FIR_H_ */
