/*
 * ============================================================================
 * edgeai_fusion.h
 * SmartBAN Edge-AI Multi-Modal Context Fusion & Dual-Mode Stream Controller
 * Target: CC2652R1 (ARM Cortex-M4F), SimpleLink SDK 8.33, TI-RTOS7
 *
 * Thesis Core: Cross-modal optical/thermal skin contact validation, artifact
 * rejection, on-body thermal classification, and dual-mode data reduction.
 * ============================================================================
 */

#ifndef EDGEAI_FUSION_H_
#define EDGEAI_FUSION_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Telemetry Stream Modes & Contact States
 * ============================================================================ */

typedef enum {
    STREAM_MODE_SEMANTIC = 0,   /**< Default Mode: ~85 B/s, 1 Hz periodic / anomaly tokens (>95% reduction) */
    STREAM_MODE_RAW      = 1    /**< Evaluation Mode: 2566 B/s continuous raw samples for thesis benchmark */
} smartban_stream_mode_t;

typedef enum {
    CONTACT_STATE_DISCONNECTED = 0, /**< Off-body: Proximity < 5500 counts (Alarms suppressed) */
    CONTACT_STATE_CONNECTED    = 1, /**< On-body: Proximity >= 6000 counts (Thermal analysis active) */
    CONTACT_STATE_DEBOUNCING   = 2  /**< Intermediate transition state during hysteresis debounce */
} edgeai_contact_state_t;

typedef enum {
    THERMAL_CLASS_UNKNOWN          = 0, /**< Sensor uninitialized or read error */
    THERMAL_CLASS_AMBIENT_OFFBODY  = 1, /**< Sensor detached: Reading ambient air, clinical alarms suppressed */
    THERMAL_CLASS_HYPOTHERMIA      = 2, /**< Clinical Hypothermia: T_skin < 35.0 degC */
    THERMAL_CLASS_NORMAL           = 3, /**< Normothermia: 35.0 degC <= T_skin <= 37.5 degC */
    THERMAL_CLASS_ELEVATED         = 4, /**< Sub-febrile / Elevated: 37.5 degC < T_skin <= 38.0 degC */
    THERMAL_CLASS_HYPERTHERMIA     = 5  /**< Clinical Hyperthermia / Fever: T_skin > 38.0 degC */
} edgeai_thermal_class_t;

typedef enum {
    POSTURE_STATE_SEDENTARY        = 0, /**< Rest / Sitting / Lying down */
    POSTURE_STATE_ACTIVE           = 1, /**< Walking / Moderate continuous movement */
    POSTURE_STATE_HIGH_DYNAMIC     = 2, /**< Running / Rapid athletic movement */
    POSTURE_STATE_FALL_REST        = 3  /**< Post-fall prolonged motionless state (Incapacitated) */
} edgeai_posture_state_t;

/* Combined System Alert Bitmask Flags */
#define ALERT_FLAG_NONE             0x0000U
#define ALERT_FLAG_TACHYCARDIA      (1U << 0) /**< Sustained HR > 100 bpm */
#define ALERT_FLAG_BRADYCARDIA      (1U << 1) /**< Sustained HR < 50 bpm */
#define ALERT_FLAG_ARRHYTHMIA       (1U << 2) /**< RR deviation > 25% from rolling baseline */
#define ALERT_FLAG_FALL_IMPACT      (1U << 3) /**< 4-phase confirmed fall impact shock */
#define ALERT_FLAG_HYPOTHERMIA      (1U << 4) /**< Validated on-body skin temp < 35.0 degC */
#define ALERT_FLAG_FEVER            (1U << 5) /**< Validated on-body skin temp > 38.0 degC */
#define ALERT_FLAG_LEAD_OFF         (1U << 6) /**< ECG electrode contact detachment */
#define ALERT_FLAG_OPTICAL_FAULT    (1U << 7) /**< I2C optical sensor offline */

/* Cardiac Status Flags */
#define CARDIAC_FLAG_NORMAL         0x00U
#define CARDIAC_FLAG_TACHYCARDIA    0x01U
#define CARDIAC_FLAG_BRADYCARDIA    0x02U
#define CARDIAC_FLAG_ARRHYTHMIA     0x04U
#define CARDIAC_FLAG_DISCONNECTED   0x80U

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Thread-safe snapshot cache of low-frequency environmental/optical sensors.
 */
typedef struct {
    uint32_t timestamp_ms;      /**< Monotonic timestamp of acquisition */
    float    ambient_temp_c;    /**< MLX90632 sensor die ambient temperature */
    float    object_temp_c;     /**< MLX90632 infrared optical target temperature */
    uint16_t prox_counts;       /**< VCNL4040 raw proximity backscatter counts */
    uint16_t als_counts;        /**< VCNL4040 raw ALS counts */
    float    ambient_lux;       /**< OPT4041 precision ambient light illuminance */
    bool     raw_contact_flag;  /**< Direct threshold comparison (prox >= 6000) */
    bool     fir_online;        /**< MLX90632 communication health */
    bool     optical_online;    /**< OPT4041 / VCNL4040 communication health */
} edgeai_slow_cache_t;

/**
 * @brief Unified Semantic State Token (Bandwidth: ~85 Bytes JSON or 44 Bytes Binary).
 */
typedef struct {
    uint32_t timestamp_ms;      /**< Monotonic node time in milliseconds */
    uint16_t sequence_num;      /**< Monotonic token sequence counter */
    
    /* Biopotential (ECG) Features */
    uint8_t  heart_rate_bpm;    /**< Instantaneous heart rate in beats per minute */
    uint16_t rr_interval_ms;    /**< Latest validated R-to-R interval */
    uint16_t hrv_rmssd_ms;      /**< HRV Root Mean Square of Successive Differences */
    uint16_t hrv_sdnn_ms;       /**< HRV Standard Deviation of NN intervals */
    uint8_t  cardiac_flags;     /**< Cardiac anomaly status bitmask */
    
    /* Motion (IMU) Features */
    uint8_t  posture_state;     /**< Posture / Activity state classification */
    float    activity_sma_g;    /**< Signal Magnitude Area normalized activity metric */
    int16_t  tilt_pitch_deg;    /**< Torso dynamic pitch tilt angle (-90 to +90 deg) */
    int16_t  tilt_roll_deg;     /**< Torso dynamic roll tilt angle (-180 to +180 deg) */
    bool     fall_detected;     /**< True if 4-phase fall impact confirmed */
    
    /* Cross-Modal Context & Thermal Features */
    bool     skin_contact;      /**< Validated cross-modal skin contact status */
    uint8_t  contact_confidence;/**< Coupling confidence metric (0 to 100%) */
    uint8_t  thermal_class;     /**< edgeai_thermal_class_t thermal state */
    float    skin_temp_c;       /**< Calibrated physiological skin temperature in degC */
    float    ambient_temp_c;    /**< Reference ambient temperature in degC */
    uint32_t ambient_lux;       /**< OPT4041 ambient light illuminance in lux */
    
    /* System & Telemetry Controls */
    uint16_t alert_mask;        /**< Aggregated clinical and hardware alert flags */
    bool     is_anomaly_event;  /**< True if token was dispatched as an immediate alert burst */
} smartban_semantic_token_t;

/**
 * @brief Context Fusion Engine Configuration Thresholds.
 */
typedef struct {
    uint16_t prox_thresh_on;       /**< Low-to-high contact threshold (Default: 6000) */
    uint16_t prox_thresh_off;      /**< High-to-low detach threshold (Default: 5500) */
    uint8_t  contact_debounce_max; /**< Consecutive debounce cycles at 10 Hz (Default: 2) */
    float    temp_hypothermia_c;   /**< Hypothermia threshold (Default: 35.0 degC) */
    float    temp_hypo_clear_c;    /**< Hypothermia clear hysteresis (Default: 35.3 degC) */
    float    temp_fever_c;         /**< Hyperthermia / Fever threshold (Default: 38.0 degC) */
    float    temp_fever_clear_c;   /**< Fever clear hysteresis (Default: 37.8 degC) */
    float    temp_normal_high_c;   /**< Normal upper bound (Default: 37.5 degC) */
} edgeai_fusion_config_t;

/**
 * @brief Fusion Engine Internal State Structure.
 */
typedef struct {
    edgeai_fusion_config_t   config;
    edgeai_contact_state_t   contact_state;
    uint8_t                  debounce_counter;
    edgeai_contact_state_t   candidate_contact_state;
    edgeai_thermal_class_t   thermal_class;
    bool                     hypo_alert_active;
    bool                     fever_alert_active;
    uint16_t                 sequence_counter;
    uint32_t                 last_token_time_ms;
    smartban_semantic_token_t last_token;
    uint32_t                 total_tokens_emitted;
    uint32_t                 anomaly_tokens_emitted;
} edgeai_fusion_state_t;

/* Compile-time verification of token footprint */
_Static_assert(sizeof(smartban_semantic_token_t) <= 48,
               "smartban_semantic_token_t must not exceed 48 bytes for SRAM efficiency");

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/**
 * @brief Initialize the context fusion engine and configuration parameters.
 * @param state Pointer to fusion state structure.
 * @param cfg Optional pointer to custom thresholds (Pass NULL for clinical defaults).
 */
void edgeai_fusion_init(edgeai_fusion_state_t *state, const edgeai_fusion_config_t *cfg);

/**
 * @brief Reset the fusion engine state (e.g. upon subject change or mode reset).
 */
void edgeai_fusion_reset(edgeai_fusion_state_t *state);

/**
 * @brief Thread-safe update of the environmental sensor snapshot cache.
 *        Invoked exclusively by Task_Sensors_Slow.
 */
void edgeai_slow_cache_update(float amb_c, float obj_c, uint16_t prox, uint16_t als,
                              float lux, bool fir_ok, bool opt_ok);

/**
 * @brief Thread-safe retrieval of the latest environmental sensor cache.
 *        Invoked by Task_EdgeAI during token synthesis.
 */
void edgeai_slow_cache_get(edgeai_slow_cache_t *dest);

/**
 * @brief Process cross-modal context fusion, evaluate skin contact, classify thermal
 *        state, suppress off-body false alarms, and assemble the unified semantic token.
 * @param state Pointer to fusion state structure.
 * @param ecg_state Pointer to ECG Edge-AI state structure (HR, RR, HRV, flags).
 * @param imu_state Pointer to IMU Edge-AI state structure (posture, SMA, fall).
 * @param slow_cache Pointer to latest slow sensor snapshot.
 * @param out_token Pointer to destination token structure.
 * @return true if a new token was assembled; false if processing skipped.
 */
bool edgeai_fusion_process(edgeai_fusion_state_t *state,
                           const void *ecg_state,
                           const void *imu_state,
                           const edgeai_slow_cache_t *slow_cache,
                           smartban_semantic_token_t *out_token);

/**
 * @brief Query current telemetry stream mode (Thread-safe).
 */
smartban_stream_mode_t edgeai_get_stream_mode(void);

/**
 * @brief Set telemetry stream mode (Thread-safe).
 */
void edgeai_set_stream_mode(smartban_stream_mode_t mode);

/**
 * @brief Toggle telemetry stream mode between Semantic and Raw (Thread-safe).
 * @return New active stream mode.
 */
smartban_stream_mode_t edgeai_toggle_stream_mode(void);

/**
 * @brief Format semantic token into standard compact JSON frame.
 * @param token Pointer to source semantic token.
 * @param buf Output character buffer.
 * @param max_len Size of buffer (Recommended >= 128 bytes).
 * @return Number of characters written (excluding null terminator).
 */
size_t edgeai_format_semantic_json(const smartban_semantic_token_t *token, char *buf, size_t max_len);

/**
 * @brief Format high-rate raw sample into standard JSON evaluation frame.
 * @param ts_ms Monotonic sample timestamp.
 * @param ecg_raw 24-bit ECG channel 1 raw ADC counts.
 * @param ax X-axis acceleration in mg.
 * @param ay Y-axis acceleration in mg.
 * @param az Z-axis acceleration in mg.
 * @param buf Output character buffer.
 * @param max_len Size of buffer (Recommended >= 96 bytes).
 * @return Number of characters written.
 */
size_t edgeai_format_raw_json(uint32_t ts_ms, int32_t ecg_raw, int16_t ax, int16_t ay, int16_t az,
                              char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* EDGEAI_FUSION_H_ */
