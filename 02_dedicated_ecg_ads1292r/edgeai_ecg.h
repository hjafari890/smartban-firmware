/*
 * ============================================================================
 * edgeai_ecg.h
 * Dedicated Bare-Metal ECG Firmware (CC2652R1 Cortex-M4F)
 * Edge-AI Integer Pan-Tompkins QRS, Bandpass Filter & Heart Rate Engine
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

/* Cardiac Status Flags */
#define CARDIAC_FLAG_NORMAL             0x00U
#define CARDIAC_FLAG_TACHYCARDIA        0x01U /* Bit 0: Sustained HR > 100 bpm */
#define CARDIAC_FLAG_BRADYCARDIA        0x02U /* Bit 1: Sustained HR < 50 bpm */
#define CARDIAC_FLAG_ARRHYTHMIA         0x04U /* Bit 2: RR interval variation > 25% */
#define CARDIAC_FLAG_PVC                0x08U /* Bit 3: Premature beat */
#define CARDIAC_FLAG_ASYSTOLE           0x10U /* Bit 4: No beat detected for > 3.0 s */
#define CARDIAC_FLAG_LEAD_OFF           0x20U /* Bit 5: ADS1292 electrode disconnected */
#define CARDIAC_FLAG_SEARCHBACK         0x40U /* Bit 6: Beat detected via search-back threshold */
#define CARDIAC_FLAG_LEARNING           0x80U /* Bit 7: Initializing thresholds (first 2s) */

typedef struct {
    uint16_t sample_rate_hz;
    uint16_t mwi_window_samples;
    uint16_t refractory_samples;
    uint16_t tachy_threshold_bpm;
    uint16_t brady_threshold_bpm;
    uint8_t  arrhythmia_tolerance_pct;
    uint8_t  consecutive_beats_anomaly;
} edgeai_ecg_config_t;

typedef struct {
    bool     qrs_detected;              /* True on sample where QRS is confirmed */
    uint32_t qrs_timestamp_ms;          /* Timestamp of confirmed R-peak */
    uint16_t rr_interval_ms;            /* Latest RR interval in ms */
    uint8_t  heart_rate_bpm;            /* Instantaneous heart rate */
    uint8_t  heart_rate_smooth_bpm;     /* Smoothed rolling heart rate */
    uint16_t hrv_sdnn_ms;               /* SDNN (32 beats) */
    uint16_t hrv_rmssd_ms;              /* RMSSD */
    uint8_t  cardiac_flags;             /* Active status flags */
    int32_t  filtered_ecg;              /* Bandpass-filtered biopotential count */
    uint32_t mwi_signal;                /* Integrated energy level */
} edgeai_ecg_result_t;

typedef struct {
    /* 1. Low-Pass Filter State (M=6 FIR, fc ~13 Hz) */
    int32_t  lpf_x[13];
    int32_t  lpf_y1;
    int32_t  lpf_y2;
    uint8_t  lpf_idx;

    /* 2. High-Pass Filter State (fc ~5.5 Hz) */
    int32_t  hpf_x[33];
    int32_t  hpf_sum;
    uint8_t  hpf_idx;

    /* 3. 5-Point Derivative Filter State */
    int32_t  deriv_x[5];
    uint8_t  deriv_idx;

    /* 4. Moving Window Integrator State (N=38) */
    uint32_t mwi_buf[EDGEAI_ECG_MWI_WINDOW_SIZE];
    uint64_t mwi_sum;
    uint8_t  mwi_idx;
    uint32_t mwi_prev1;                 /* y_mwi[n-1] for peak detection */
    uint32_t mwi_prev2;                 /* y_mwi[n-2] for peak detection */

    /* 5. Adaptive Dual-Threshold State Machine */
    uint32_t spki;                      /* Signal Peak Level */
    uint32_t npki;                      /* Noise Peak Level */
    uint32_t threshold1;                /* Primary QRS threshold */
    uint32_t threshold2;                /* Secondary search-back threshold */
    uint32_t peak_candidate_val;        /* Highest peak in search-back window */
    uint32_t peak_candidate_sample;     /* Sample index of candidate peak */
    uint32_t peak_max_learning;         /* Maximum peak observed during learning */
    uint32_t samples_since_qrs;         /* Elapsed samples since last confirmed beat */
    uint32_t total_samples_processed;   /* Monotonic sample counter */
    bool     learning_phase;            /* True during initial calibration */

    /* 6. Rolling RR Intervals */
    uint16_t rr_history[EDGEAI_ECG_RR_HISTORY_SIZE];
    uint8_t  rr_head;
    uint8_t  rr_count;
    uint16_t rr_mean_samples;
    uint16_t rr_mean_ms;
    uint16_t last_rr_ms;
    uint16_t last_8_rr[EDGEAI_ECG_RR_MEAN_WINDOW];
    uint8_t  last_8_idx;
    uint8_t  last_8_count;

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

void edgeai_ecg_get_default_config(edgeai_ecg_config_t *config);
bool edgeai_ecg_init(edgeai_ecg_state_t *state, const edgeai_ecg_config_t *config);
void edgeai_ecg_reset(edgeai_ecg_state_t *state);
bool edgeai_ecg_process_sample(edgeai_ecg_state_t *state,
                               int32_t raw_sample,
                               uint32_t timestamp_ms,
                               bool lead_off,
                               edgeai_ecg_result_t *result);
void edgeai_ecg_get_latest_result(const edgeai_ecg_state_t *state,
                                  edgeai_ecg_result_t *result);
uint32_t ecg_isqrt(uint32_t val);

#ifdef __cplusplus
}
#endif

#endif /* EDGEAI_ECG_H_ */
