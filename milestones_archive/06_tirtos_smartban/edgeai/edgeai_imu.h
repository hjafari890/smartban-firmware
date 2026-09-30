/*
 * ============================================================================
 * edgeai_imu.h
 * Edge-AI IMU Feature Extraction, Posture State Machine & 4-Phase Fall Detection
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * Milestone M3: Edge-AI Semantic Processing & Data Reduction (Thesis Core)
 * ============================================================================
 */

#ifndef EDGEAI_IMU_H_
#define EDGEAI_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/hal_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Window Sizing Parameters */
#define EDGEAI_IMU_WINDOW_SIZE          100U    /* 1.0 s at 100 Hz ODR */
#define EDGEAI_IMU_STEP_SIZE            50U     /* 50% overlap (0.5 s cadence) */
#define EDGEAI_IMU_BASELINE_RING_SIZE   4U      /* 4 x 0.5s = 2.0s pre-fall baseline history */

/* Fall Detection Tuning Constants */
#define EDGEAI_FALL_FREEFALL_THRESH_G   0.50f   /* Free-fall weightlessness threshold (< 0.5g) */
#define EDGEAI_FALL_FREEFALL_MIN_SAMP   6U      /* Minimum 60 ms at 100 Hz (6 samples) */
#define EDGEAI_FALL_IMPACT_THRESH_G     3.00f   /* Impact shock collision threshold (> 3.0g) */
#define EDGEAI_FALL_IMPACT_WIN_MIN_MS   100U    /* Impact must occur >= 100 ms after free-fall */
#define EDGEAI_FALL_IMPACT_WIN_MAX_MS   350U    /* Impact must occur <= 350 ms after free-fall */
#define EDGEAI_FALL_TILT_THRESH_DEG     45.0f   /* Orientation change threshold (> 45 deg) */
#define EDGEAI_FALL_REST_THRESH_SMA_G   0.10f   /* Post-fall immobility SMA threshold (< 0.1g) */
#define EDGEAI_FALL_REST_MIN_SAMPLES    200U    /* Post-fall immobility duration (2.0s = 200 samples) */

/* Activity Classification Thresholds */
#define EDGEAI_ACT_SEDENTARY_SMA_MAX    0.15f   /* Sedentary SMA upper bound (< 0.15g) */
#define EDGEAI_ACT_ACTIVE_SMA_MAX       0.60f   /* Active SMA upper bound (< 0.60g) */
#define EDGEAI_ACT_HYST_ENTER_ACTIVE    0.18f   /* Schmitt trigger enter active (> 0.18g) */
#define EDGEAI_ACT_HYST_EXIT_ACTIVE     0.12f   /* Schmitt trigger exit active (< 0.12g) */
#define EDGEAI_ACT_HYST_ENTER_HIGH_DYN  0.65f   /* Schmitt trigger enter high dyn (> 0.65g) */
#define EDGEAI_ACT_HYST_EXIT_HIGH_DYN   0.55f   /* Schmitt trigger exit high dyn (< 0.55g) */

/**
 * @brief Primary Human Posture & Locomotion State
 */
typedef enum {
    POSTURE_SEDENTARY = 0,      /* Resting, sitting, standing motionless */
    POSTURE_ACTIVE = 1,         /* Normal walking, moderate domestic activity */
    POSTURE_HIGH_DYNAMIC = 2    /* Running, sprinting, vigorous jumping */
} edgeai_posture_t;

/**
 * @brief Torso Orientation Sub-Classification (in Sedentary State)
 */
typedef enum {
    SUBPOSTURE_UPRIGHT = 0,         /* Torso vertical (standing or sitting upright) */
    SUBPOSTURE_LYING_SUPINE = 1,    /* Lying horizontally on back (chest up) */
    SUBPOSTURE_LYING_PRONE = 2,     /* Lying horizontally on abdomen (chest down) */
    SUBPOSTURE_LYING_LATERAL = 3,   /* Lying horizontally on left/right side */
    SUBPOSTURE_UNKNOWN = 4          /* Ambiguous or uncalibrated orientation */
} edgeai_sub_posture_t;

/**
 * @brief 4-Phase Fall Detection Machine Phase
 */
typedef enum {
    FALL_PHASE_IDLE = 0,                /* Normal monitoring, no fall suspected */
    FALL_PHASE_FREEFALL_DETECTED = 1,   /* Phase 1: |A| < 0.5g for >= 60 ms */
    FALL_PHASE_IMPACT_DETECTED = 2,     /* Phase 2: |A| > 3.0g within 100-350 ms of freefall */
    FALL_PHASE_ORIENTATION_CHECK = 3,   /* Phase 3: Evaluating post-impact tilt shift */
    FALL_PHASE_REST_WAIT = 4,           /* Phase 4: Monitoring 2.0s post-impact immobility */
    FALL_PHASE_CONFIRMED = 5            /* Confirmed emergency fall alarm latched */
} edgeai_fall_phase_t;

/**
 * @brief Fall Alarm Status
 */
typedef enum {
    FALL_STATUS_NONE = 0,               /* No fall event */
    FALL_STATUS_SUSPECTED = 1,          /* Impact shock observed; awaiting immobility */
    FALL_STATUS_CONFIRMED = 2,          /* 4 phases satisfied; confirmed incapacitated fall */
    FALL_STATUS_RECOVERED = 3,          /* Subject stood up / moved within 2s; stumble */
    FALL_STATUS_REJECTED_ADL = 4        /* Jumping / hopping rejected by orientation check */
} edgeai_fall_status_t;

/**
 * @brief Sliding-Window Inertial Statistical Features
 */
typedef struct {
    /* Mean Acceleration */
    float mean_x_mg;            /* Mean X-axis acceleration in mg */
    float mean_y_mg;            /* Mean Y-axis acceleration in mg */
    float mean_z_mg;            /* Mean Z-axis acceleration in mg */
    float mean_mag_g;           /* Mean Vector Magnitude in g */

    /* Variance */
    float var_x_mg2;            /* X-axis variance in mg^2 */
    float var_y_mg2;            /* Y-axis variance in mg^2 */
    float var_z_mg2;            /* Z-axis variance in mg^2 */
    float var_mag_g2;           /* Vector Magnitude variance in g^2 */
    float total_variance_g2;    /* Sum of spatial variances (X+Y+Z) in g^2 */

    /* Standard Deviation */
    float std_x_mg;             /* X-axis standard deviation in mg */
    float std_y_mg;             /* Y-axis standard deviation in mg */
    float std_z_mg;             /* Z-axis standard deviation in mg */
    float std_total_g;          /* Total dynamic standard deviation in g */

    /* Signal Magnitude Area (SMA) */
    float sma_dynamic_g;        /* Dynamic (zero-mean) SMA in g (governs classification) */
    float sma_raw_g;            /* Raw SMA including gravity in g */

    /* Dynamic Tilt Angles */
    float pitch_deg;            /* Dynamic pitch angle relative to gravity [-90, +90] deg */
    float roll_deg;             /* Dynamic roll angle relative to gravity [-180, +180] deg */
} edgeai_imu_features_t;

/**
 * @brief Comprehensive Output Result of IMU Edge-AI Engine
 */
typedef struct {
    uint32_t              timestamp_ms;          /* Timestamp of the latest processed sample */
    bool                  window_completed;      /* True if a new 1.0s window was evaluated */
    edgeai_imu_features_t features;              /* Extracted statistical feature vector */
    edgeai_posture_t      posture;               /* SEDENTARY, ACTIVE, or HIGH_DYNAMIC */
    edgeai_sub_posture_t  sub_posture;           /* UPRIGHT, SUPINE, PRONE, or LATERAL */
    edgeai_fall_status_t  fall_status;           /* NONE, SUSPECTED, CONFIRMED, RECOVERED */
    edgeai_fall_phase_t   fall_phase;            /* Current phase of 4-phase state machine */
    bool                  fall_detected;         /* Latched true upon confirmed fall alarm */
    float                 impact_peak_g;         /* Maximum impact peak acceleration (g) */
    float                 tilt_change_deg;       /* Measured angular orientation change (deg) */
    float                 immobility_duration_s; /* Current observed immobility duration (s) */
} edgeai_imu_result_t;

/**
 * @brief User-Configurable Thresholds
 */
typedef struct {
    float    freefall_thresh_g;        /* Default 0.50g */
    uint8_t  freefall_min_samples;     /* Default 6 (60 ms @ 100 Hz) */
    float    impact_thresh_g;          /* Default 3.00g */
    uint16_t impact_window_max_ms;     /* Default 350 ms */
    float    tilt_thresh_deg;          /* Default 45.0 deg */
    float    rest_thresh_sma_g;        /* Default 0.10g */
    uint16_t rest_duration_samples;    /* Default 200 (2.0 s @ 100 Hz) */
    float    act_sedentary_sma_max;    /* Default 0.15g */
    float    act_active_sma_max;       /* Default 0.60g */
} edgeai_imu_config_t;

/**
 * @brief Internal Engine State Context Block
 */
typedef struct {
    edgeai_imu_config_t config;

    /* Circular Sample Window Buffer (100 samples) */
    hal_imu_sample_t sample_window[EDGEAI_IMU_WINDOW_SIZE];
    uint16_t window_head;               /* Insertion index [0..99] */
    uint16_t window_count;              /* Total samples buffered [0..100] */
    uint16_t step_counter;              /* Counts toward 50-sample hop */

    /* Pre-Fall Baseline History Buffer (Stores 4 x 0.5s vectors = 2.0s) */
    struct {
        float mean_x;
        float mean_y;
        float mean_z;
    } baseline_history[EDGEAI_IMU_BASELINE_RING_SIZE];
    uint8_t baseline_head;
    uint8_t baseline_count;

    /* Cached Pre-Fall Reference Vector */
    float ref_base_x;
    float ref_base_y;
    float ref_base_z;

    /* Posture State Machine Context */
    edgeai_posture_t current_posture;
    edgeai_sub_posture_t current_sub_posture;
    uint8_t posture_debounce_counter;
    edgeai_posture_t candidate_posture;

    /* 4-Phase Fall Detector Context */
    edgeai_fall_phase_t  fall_phase;
    edgeai_fall_status_t fall_status;
    bool                 fall_alarm_latched;
    uint8_t              consecutive_freefall_samples;
    uint32_t             t_freefall_end_ms;
    float                impact_peak_g;
    uint32_t             t_impact_ms;
    float                measured_tilt_change_deg;
    uint16_t             immobile_sample_count;

    /* Post-impact orientation accumulator */
    float                post_impact_sum_x;
    float                post_impact_sum_y;
    float                post_impact_sum_z;
    uint16_t             post_impact_sample_count;

    /* Latest Output Result */
    edgeai_imu_result_t latest_result;
} edgeai_imu_state_t;

/* ============================================================================
 * Public Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize default tuning parameters.
 * @param config Destination configuration structure.
 */
void edgeai_imu_config_default(edgeai_imu_config_t *config);

/**
 * @brief Initialize IMU Edge-AI engine state and window buffers.
 * @param state Pointer to engine context block.
 * @param config Optional custom configuration (NULL for defaults).
 */
void edgeai_imu_init(edgeai_imu_state_t *state, const edgeai_imu_config_t *config);

/**
 * @brief Reset engine internal buffers, state machines, and latched alarms.
 * @param state Pointer to engine context block.
 */
void edgeai_imu_reset(edgeai_imu_state_t *state);

/**
 * @brief Clear a latched emergency fall alarm (e.g. following user acknowledgment).
 * @param state Pointer to engine context block.
 */
void edgeai_imu_clear_fall_alarm(edgeai_imu_state_t *state);

/**
 * @brief Ingest a single 100 Hz IMU sample into the Edge-AI pipeline.
 *        Executes sample-level free-fall/impact checking and updates sliding window.
 *        When window step completes (every 50 samples / 500 ms), updates statistical
 *        features, posture classifier, and post-fall rest validation.
 * @param state Pointer to engine context block.
 * @param sample Pointer to incoming 100 Hz ADXL362 sample.
 * @param result Destination structure populated with latest inference results.
 * @return true if a window step was completed and fresh features were calculated.
 */
bool edgeai_imu_process_sample(edgeai_imu_state_t *state,
                               const hal_imu_sample_t *sample,
                               edgeai_imu_result_t *result);

/**
 * @brief Extract the latest statistical feature vector without feeding a sample.
 * @param state Pointer to engine context block.
 * @param features Destination feature vector structure.
 */
void edgeai_imu_get_features(const edgeai_imu_state_t *state, edgeai_imu_features_t *features);

/**
 * @brief Read the current overall inference result.
 * @param state Pointer to engine context block.
 * @param result Destination result structure.
 */
void edgeai_imu_get_result(const edgeai_imu_state_t *state, edgeai_imu_result_t *result);

/**
 * @brief Standalone mathematical utility: Compute two-pass statistical features
 *        over an arbitrary buffer of IMU samples.
 * @param samples Array of IMU samples.
 * @param count Number of samples in array.
 * @param features Output feature structure.
 */
void edgeai_imu_calc_stats(const hal_imu_sample_t *samples,
                           uint16_t count,
                           edgeai_imu_features_t *features);

/**
 * @brief Standalone mathematical utility: Compute 3D angular displacement (deg)
 *        between two 3-axis acceleration vectors.
 * @param x1, y1, z1 Vector 1 components.
 * @param x2, y2, z2 Vector 2 components.
 * @return Angular difference in degrees [0.0f, 180.0f].
 */
float edgeai_imu_calc_3d_angle_diff(float x1, float y1, float z1,
                                    float x2, float y2, float z2);

#ifdef __cplusplus
}
#endif

#endif /* EDGEAI_IMU_H_ */
