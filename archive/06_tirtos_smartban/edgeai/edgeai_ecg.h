/*
 * ============================================================================
 * edgeai_ecg.h
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * Milestone M3: Edge-AI Integer Pan-Tompkins QRS, HRV & Anomaly Pipeline
 * ============================================================================
 */

#ifndef EDGEAI_ECG_H_
#define EDGEAI_ECG_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Operational Constants for 250 Hz ADS1292 Sampling */
#define EDGEAI_ECG_SAMPLE_RATE_HZ       250U
#define EDGEAI_ECG_SAMPLE_PERIOD_MS     4U
#define EDGEAI_ECG_MWI_WINDOW_SIZE      38U  /* 38 samples @ 250 Hz = 152 ms */
#define EDGEAI_ECG_REFRACTORY_SAMPLES   50U  /* 50 samples @ 250 Hz = 200 ms */
#define EDGEAI_ECG_RR_HISTORY_SIZE      32U  /* Power of 2 for fast shifts */
#define EDGEAI_ECG_RR_HISTORY_MASK      (EDGEAI_ECG_RR_HISTORY_SIZE - 1U)
#define EDGEAI_ECG_RR_MEAN_WINDOW       8U   /* Last 8 beats for rolling mean */
#define EDGEAI_ECG_ASYSTOLE_TIMEOUT     750U /* 750 samples = 3000 ms */

/* Cardiac Anomaly Status Flags */
#define CARDIAC_FLAG_NORMAL             0x00U
#define CARDIAC_FLAG_TACHYCARDIA        0x01U /* Bit 0: Sustained HR > 100 bpm */
#define CARDIAC_FLAG_BRADYCARDIA        0x02U /* Bit 1: Sustained HR < 50 bpm */
#define CARDIAC_FLAG_ARRHYTHMIA         0x04U /* Bit 2: RR interval variation > 25% */
#define CARDIAC_FLAG_PVC                0x08U /* Bit 3: Premature beat + compensatory pause */
#define CARDIAC_FLAG_ASYSTOLE           0x10U /* Bit 4: No beat detected for > 3.0 s */
#define CARDIAC_FLAG_LEAD_OFF           0x20U /* Bit 5: ADS1292 electrode disconnected */
#define CARDIAC_FLAG_SEARCHBACK         0x40U /* Bit 6: Beat detected via search-back threshold */
#define CARDIAC_FLAG_LEARNING           0x80U /* Bit 7: Initializing thresholds (first 2s) */

/**
 * @brief Configuration parameters for Edge-AI ECG detection pipeline.
 */
typedef struct {
    uint16_t sample_rate_hz;            /* Nominal 250 */
    uint16_t mwi_window_samples;        /* Nominal 38 (152 ms) */
    uint16_t refractory_samples;        /* Nominal 50 (200 ms) */
    uint16_t tachy_threshold_bpm;       /* Nominal 100 */
    uint16_t brady_threshold_bpm;       /* Nominal 50 */
    uint8_t  arrhythmia_tolerance_pct;  /* Nominal 25 (%) */
    uint8_t  consecutive_beats_anomaly; /* Nominal 5 beats */
} edgeai_ecg_config_t;

/**
 * @brief Output result produced by processing an ECG sample or QRS event.
 */
typedef struct {
    bool     qrs_detected;              /* True strictly on sample where QRS is confirmed */
    uint32_t qrs_timestamp_ms;          /* Node monotonic timestamp of confirmed R-peak */
    uint16_t rr_interval_ms;            /* Latest RR interval in milliseconds */
    uint8_t  heart_rate_bpm;            /* Instantaneous heart rate (60000 / RR_ms) */
    uint8_t  heart_rate_smooth_bpm;     /* Smoothed rolling heart rate */
    uint16_t hrv_sdnn_ms;               /* Standard deviation of NN intervals (32 beats) */
    uint16_t hrv_rmssd_ms;              /* Root mean square of successive differences */
    uint8_t  cardiac_flags;             /* Active cardiac anomaly bitfield */
    int32_t  filtered_ecg;              /* Bandpass-filtered biopotential count */
    uint32_t mwi_signal;                /* Integrated energy level */
} edgeai_ecg_result_t;

/**
 * @brief Persistent state and delay line buffers for Pan-Tompkins pipeline.
 *        Total structure size: ~530 Bytes (statically allocated, zero malloc).
 */
typedef struct {
    /* 1. Low-Pass Filter State (M=6) */
    int32_t  lpf_x[13];                 /* Input sample delay line: x[n]..x[n-12] */
    int32_t  lpf_y1;                    /* y[n-1] */
    int32_t  lpf_y2;                    /* y[n-2] */
    uint8_t  lpf_idx;                   /* Circular index for lpf_x */

    /* 2. High-Pass Filter State (P=32, K=16) */
    int32_t  hpf_x[33];                 /* Delay line: y_lp[n]..y_lp[n-32] */
    int32_t  hpf_sum;                   /* Running sum p[n] */
    uint8_t  hpf_idx;                   /* Circular index for hpf_x */

    /* 3. 5-Point Derivative Filter State */
    int32_t  deriv_x[5];                /* Delay line: y_hp[n]..y_hp[n-4] */
    uint8_t  deriv_idx;                 /* Circular index for deriv_x */

    /* 4. Moving Window Integrator State (N=38) */
    uint32_t mwi_buf[EDGEAI_ECG_MWI_WINDOW_SIZE]; /* Squared sample window */
    uint64_t mwi_sum;                   /* Running 64-bit sum of squared samples */
    uint8_t  mwi_idx;                   /* Circular index for mwi_buf */
    uint32_t mwi_prev1;                 /* y_mwi[n-1] for peak detection */
    uint32_t mwi_prev2;                 /* y_mwi[n-2] for peak detection */

    /* 5. Adaptive Dual-Threshold State Machine */
    uint32_t spki;                      /* Signal Peak Level */
    uint32_t npki;                      /* Noise Peak Level */
    uint32_t threshold1;                /* Primary QRS threshold */
    uint32_t threshold2;                /* Secondary search-back threshold */
    uint32_t peak_candidate_val;        /* Highest peak in search-back window */
    uint32_t peak_candidate_sample;     /* Sample index of candidate peak */
    uint32_t peak_max_learning;         /* Maximum peak observed during learning phase */
    uint32_t samples_since_qrs;         /* Elapsed samples since last confirmed beat */
    uint32_t total_samples_processed;   /* Monotonic sample counter */
    bool     learning_phase;            /* True during initial 2s calibration */

    /* 6. Rolling RR Intervals & HRV Buffers */
    uint16_t rr_history[EDGEAI_ECG_RR_HISTORY_SIZE]; /* 32-beat circular buffer */
    uint8_t  rr_head;                   /* Index for rr_history */
    uint8_t  rr_count;                  /* Valid beat count in rr_history (0..32) */
    uint16_t rr_mean_samples;           /* Rolling mean of last 8 beats (in samples) */
    uint16_t rr_mean_ms;                /* Rolling mean of last 8 beats (in ms) */
    uint16_t last_rr_ms;                /* Previous RR interval for PVC evaluation */
    uint16_t last_8_rr[EDGEAI_ECG_RR_MEAN_WINDOW]; /* Circular buffer of last 8 RR intervals */
    uint8_t  last_8_idx;                /* Index into last_8_rr */
    uint8_t  last_8_count;              /* Count in last_8_rr (0..8) */

    /* 7. Cardiac Anomaly Counters */
    uint8_t  consecutive_tachy_beats;
    uint8_t  consecutive_brady_beats;
    uint8_t  consecutive_normal_tachy;
    uint8_t  consecutive_normal_brady;
    uint8_t  active_flags;

    /* 8. Cached Metrics */
    edgeai_ecg_result_t latest_result;
    edgeai_ecg_config_t config;
} edgeai_ecg_state_t;

/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/**
 * @brief Initialize default configuration parameters.
 * @param config Pointer to configuration struct to populate.
 */
void edgeai_ecg_get_default_config(edgeai_ecg_config_t *config);

/**
 * @brief Initialize Edge-AI ECG pipeline state, zeroing delay lines and setting thresholds.
 * @param state Pointer to state structure.
 * @param config Pointer to configuration (NULL to use defaults).
 * @return true on success, false on invalid pointer.
 */
bool edgeai_ecg_init(edgeai_ecg_state_t *state, const edgeai_ecg_config_t *config);

/**
 * @brief Reset running filters and thresholds without re-allocating memory.
 * @param state Pointer to state structure.
 */
void edgeai_ecg_reset(edgeai_ecg_state_t *state);

/**
 * @brief Ingest and process a single 24-bit raw ECG sample from ADS1292.
 *        Executes LPF, HPF, Derivative, Squaring, MWI, Peak Detection,
 *        Adaptive Thresholding, and Anomaly Classification.
 * @param state Pointer to persistent pipeline state.
 * @param raw_sample 24-bit sign-extended biopotential count.
 * @param timestamp_ms Monotonic sample timestamp in milliseconds.
 * @param lead_off Hardware electrode disconnect status flag.
 * @param result Pointer to output result struct (populated with latest metrics).
 * @return true if a QRS beat was detected on this sample, false otherwise.
 */
bool edgeai_ecg_process_sample(edgeai_ecg_state_t *state,
                               int32_t raw_sample,
                               uint32_t timestamp_ms,
                               bool lead_off,
                               edgeai_ecg_result_t *result);

/**
 * @brief Retrieve the most recently calculated ECG features and anomaly flags.
 * @param state Pointer to persistent pipeline state.
 * @param result Destination buffer for result structure.
 */
void edgeai_ecg_get_latest_result(const edgeai_ecg_state_t *state,
                                  edgeai_ecg_result_t *result);

/**
 * @brief Fast 16-step integer square root for HRV metrics.
 * @param val Input 32-bit unsigned value.
 * @return Integer square root of val.
 */
uint32_t ecg_isqrt(uint32_t val);

#ifdef __cplusplus
}
#endif

#endif /* EDGEAI_ECG_H_ */
