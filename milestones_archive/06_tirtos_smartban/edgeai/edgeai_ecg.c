/*
 * ============================================================================
 * edgeai_ecg.c
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * Milestone M3: Edge-AI Integer Pan-Tompkins QRS, HRV & Anomaly Pipeline
 * ============================================================================
 */

#include "edgeai/edgeai_ecg.h"
#include <string.h>

/* Fast 16-step integer square root */
uint32_t ecg_isqrt(uint32_t val)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; /* Second-highest bit of 32-bit unsigned word */

    while (bit > val) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (val >= res + bit) {
            val -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

void edgeai_ecg_get_default_config(edgeai_ecg_config_t *config)
{
    if (!config) return;
    config->sample_rate_hz            = EDGEAI_ECG_SAMPLE_RATE_HZ;
    config->mwi_window_samples        = EDGEAI_ECG_MWI_WINDOW_SIZE;
    config->refractory_samples        = EDGEAI_ECG_REFRACTORY_SAMPLES;
    config->tachy_threshold_bpm       = 100U;
    config->brady_threshold_bpm       = 50U;
    config->arrhythmia_tolerance_pct  = 25U;
    config->consecutive_beats_anomaly = 5U;
}

bool edgeai_ecg_init(edgeai_ecg_state_t *state, const edgeai_ecg_config_t *config)
{
    if (!state) return false;
    memset(state, 0, sizeof(edgeai_ecg_state_t));

    if (config) {
        state->config = *config;
    } else {
        edgeai_ecg_get_default_config(&state->config);
    }

    edgeai_ecg_reset(state);
    return true;
}

void edgeai_ecg_reset(edgeai_ecg_state_t *state)
{
    if (!state) return;

    /* Reset delay line buffers */
    memset(state->lpf_x, 0, sizeof(state->lpf_x));
    state->lpf_y1 = 0;
    state->lpf_y2 = 0;
    state->lpf_idx = 0;

    memset(state->hpf_x, 0, sizeof(state->hpf_x));
    state->hpf_sum = 0;
    state->hpf_idx = 0;

    memset(state->deriv_x, 0, sizeof(state->deriv_x));
    state->deriv_idx = 0;

    memset(state->mwi_buf, 0, sizeof(state->mwi_buf));
    state->mwi_sum = 0;
    state->mwi_idx = 0;
    state->mwi_prev1 = 0;
    state->mwi_prev2 = 0;

    /* Initial state machine parameters */
    state->spki = 0;
    state->npki = 0;
    state->threshold1 = 0;
    state->threshold2 = 0;
    state->peak_candidate_val = 0;
    state->peak_candidate_sample = 0;
    state->peak_max_learning = 0;
    state->samples_since_qrs = 0;
    state->total_samples_processed = 0;
    state->learning_phase = true;

    /* Reset RR and HRV buffers */
    memset(state->rr_history, 0, sizeof(state->rr_history));
    state->rr_head = 0;
    state->rr_count = 0;
    state->rr_mean_samples = 200U; /* Default 800 ms = 75 bpm */
    state->rr_mean_ms = 800U;
    state->last_rr_ms = 0;
    memset(state->last_8_rr, 0, sizeof(state->last_8_rr));
    state->last_8_idx = 0;
    state->last_8_count = 0;

    /* Reset anomaly tracking */
    state->consecutive_tachy_beats = 0;
    state->consecutive_brady_beats = 0;
    state->consecutive_normal_tachy = 0;
    state->consecutive_normal_brady = 0;
    state->active_flags = CARDIAC_FLAG_LEARNING;

    memset(&state->latest_result, 0, sizeof(edgeai_ecg_result_t));
    state->latest_result.heart_rate_bpm = 75U;
    state->latest_result.heart_rate_smooth_bpm = 75U;
    state->latest_result.rr_interval_ms = 800U;
    state->latest_result.cardiac_flags = CARDIAC_FLAG_LEARNING;
}

static void update_hrv_metrics(edgeai_ecg_state_t *state, edgeai_ecg_result_t *result)
{
    uint8_t count = state->rr_count;
    if (count < 2) {
        result->hrv_sdnn_ms = 0;
        result->hrv_rmssd_ms = 0;
        return;
    }

    /* 1. Two-pass SDNN */
    uint32_t sum_rr = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum_rr += state->rr_history[i];
    }
    uint32_t mean_rr = sum_rr / count;

    uint64_t sum_sq_diff = 0;
    for (uint8_t i = 0; i < count; i++) {
        int32_t diff = (int32_t)state->rr_history[i] - (int32_t)mean_rr;
        sum_sq_diff += (uint64_t)((int64_t)diff * diff);
    }
    uint32_t variance_rr = (uint32_t)(sum_sq_diff / count);
    result->hrv_sdnn_ms = (uint16_t)ecg_isqrt(variance_rr);

    /* 2. Single-pass RMSSD (successive differences) */
    uint64_t sum_successive_sq = 0;
    for (uint8_t i = 0; i < count - 1; i++) {
        /* Circular chronological walk */
        uint8_t idx_curr = (uint8_t)((state->rr_head + EDGEAI_ECG_RR_HISTORY_SIZE - count + i) & EDGEAI_ECG_RR_HISTORY_MASK);
        uint8_t idx_next = (uint8_t)((idx_curr + 1) & EDGEAI_ECG_RR_HISTORY_MASK);
        int32_t diff = (int32_t)state->rr_history[idx_next] - (int32_t)state->rr_history[idx_curr];
        sum_successive_sq += (uint64_t)((int64_t)diff * diff);
    }
    uint32_t mean_diff_sq = (uint32_t)(sum_successive_sq / (count - 1));
    result->hrv_rmssd_ms = (uint16_t)ecg_isqrt(mean_diff_sq);
}

static void record_qrs_beat(edgeai_ecg_state_t *state,
                            uint32_t peak_val,
                            uint32_t rr_samples,
                            uint32_t timestamp_ms,
                            bool from_searchback,
                            edgeai_ecg_result_t *result)
{
    uint16_t rr_ms = (uint16_t)(rr_samples * EDGEAI_ECG_SAMPLE_PERIOD_MS);
    if (rr_ms == 0) rr_ms = 800;

    /* Instantaneous and Smoothed Heart Rate */
    uint8_t hr_inst = (uint8_t)((60000UL + (rr_ms / 2)) / rr_ms);
    if (hr_inst > 250) hr_inst = 250;

    /* Update Signal Peak Level (SPKI) */
    if (from_searchback) {
        /* Faster adaptation during search-back: 1/4 PEAK + 3/4 SPKI */
        state->spki = (peak_val >> 2) + state->spki - (state->spki >> 2);
    } else {
        /* Standard adaptation: 1/8 PEAK + 7/8 SPKI */
        state->spki = (peak_val >> 3) + state->spki - (state->spki >> 3);
    }

    /* Update adaptive dual thresholds */
    if (state->spki > state->npki) {
        state->threshold1 = state->npki + ((state->spki - state->npki) >> 2);
    } else {
        state->threshold1 = state->npki;
    }
    state->threshold2 = state->threshold1 >> 1;

    /* Rolling 8-beat RR average */
    state->last_8_rr[state->last_8_idx] = rr_ms;
    state->last_8_idx = (uint8_t)((state->last_8_idx + 1) % EDGEAI_ECG_RR_MEAN_WINDOW);
    if (state->last_8_count < EDGEAI_ECG_RR_MEAN_WINDOW) {
        state->last_8_count++;
    }
    uint32_t sum_8 = 0;
    for (uint8_t i = 0; i < state->last_8_count; i++) {
        sum_8 += state->last_8_rr[i];
    }
    state->rr_mean_ms = (uint16_t)(sum_8 / state->last_8_count);
    state->rr_mean_samples = (uint16_t)(state->rr_mean_ms / EDGEAI_ECG_SAMPLE_PERIOD_MS);

    uint8_t hr_smooth = (uint8_t)(60000UL / state->rr_mean_ms);

    /* Rolling 32-beat buffer for HRV */
    state->rr_history[state->rr_head] = rr_ms;
    state->rr_head = (uint8_t)((state->rr_head + 1) & EDGEAI_ECG_RR_HISTORY_MASK);
    if (state->rr_count < EDGEAI_ECG_RR_HISTORY_SIZE) {
        state->rr_count++;
    }

    /* Tachycardia evaluation with 5-beat latch and 3-beat clear */
    if (hr_inst > state->config.tachy_threshold_bpm) {
        state->consecutive_tachy_beats++;
        state->consecutive_normal_tachy = 0;
        if (state->consecutive_tachy_beats >= state->config.consecutive_beats_anomaly) {
            state->active_flags |= CARDIAC_FLAG_TACHYCARDIA;
        }
    } else {
        state->consecutive_normal_tachy++;
        if (state->consecutive_normal_tachy >= 3) {
            state->consecutive_tachy_beats = 0;
            state->active_flags &= ~CARDIAC_FLAG_TACHYCARDIA;
        }
    }

    /* Bradycardia evaluation with 5-beat latch and 3-beat clear */
    if (hr_inst < state->config.brady_threshold_bpm && hr_inst > 0) {
        state->consecutive_brady_beats++;
        state->consecutive_normal_brady = 0;
        if (state->consecutive_brady_beats >= state->config.consecutive_beats_anomaly) {
            state->active_flags |= CARDIAC_FLAG_BRADYCARDIA;
        }
    } else {
        state->consecutive_normal_brady++;
        if (state->consecutive_normal_brady >= 3) {
            state->consecutive_brady_beats = 0;
            state->active_flags &= ~CARDIAC_FLAG_BRADYCARDIA;
        }
    }

    /* Arrhythmia & PVC Evaluation */
    if (state->rr_count >= 4 && state->rr_mean_ms > 0) {
        int32_t rr_diff = (int32_t)rr_ms - (int32_t)state->rr_mean_ms;
        if (rr_diff < 0) rr_diff = -rr_diff;
        uint32_t tolerance = (uint32_t)(state->rr_mean_ms * state->config.arrhythmia_tolerance_pct) / 100U;

        if ((uint32_t)rr_diff > tolerance) {
            state->active_flags |= CARDIAC_FLAG_ARRHYTHMIA;
        } else {
            state->active_flags &= ~CARDIAC_FLAG_ARRHYTHMIA;
        }

        /* Premature Ventricular Contraction (PVC) Check */
        if (state->last_rr_ms > 0) {
            /* Case A: Current beat is premature (< 75% mean) */
            bool curr_premature = (rr_ms < (state->rr_mean_ms * 75U) / 100U);
            /* Case B: Previous beat was premature, current beat is compensatory pause */
            bool prev_premature = (state->last_rr_ms < (state->rr_mean_ms * 75U) / 100U);
            int32_t pair_sum = (int32_t)state->last_rr_ms + (int32_t)rr_ms;
            int32_t two_mean = (int32_t)(2U * state->rr_mean_ms);
            int32_t comp_diff = pair_sum - two_mean;
            if (comp_diff < 0) comp_diff = -comp_diff;
            bool full_compensation = (comp_diff <= (int32_t)(state->rr_mean_ms * 15U) / 100U);

            if (curr_premature || (prev_premature && full_compensation)) {
                state->active_flags |= CARDIAC_FLAG_PVC;
            } else {
                state->active_flags &= ~CARDIAC_FLAG_PVC;
            }
        }
    }

    /* Clear asystole flag upon confirmed beat */
    state->active_flags &= ~CARDIAC_FLAG_ASYSTOLE;
    if (from_searchback) {
        state->active_flags |= CARDIAC_FLAG_SEARCHBACK;
    } else {
        state->active_flags &= ~CARDIAC_FLAG_SEARCHBACK;
    }

    /* Populate output result */
    result->qrs_detected          = true;
    result->qrs_timestamp_ms      = timestamp_ms;
    result->rr_interval_ms        = rr_ms;
    result->heart_rate_bpm        = hr_inst;
    result->heart_rate_smooth_bpm = hr_smooth;
    result->cardiac_flags         = state->active_flags;

    /* Compute HRV SDNN & RMSSD */
    update_hrv_metrics(state, result);

    /* Update state memory */
    state->last_rr_ms = rr_ms;
    state->samples_since_qrs = 0;
    state->peak_candidate_val = 0;
}

bool edgeai_ecg_process_sample(edgeai_ecg_state_t *state,
                               int32_t raw_sample,
                               uint32_t timestamp_ms,
                               bool lead_off,
                               edgeai_ecg_result_t *result)
{
    if (!state || !result) return false;

    state->total_samples_processed++;
    state->samples_since_qrs++;

    /* Handle electrode detachment immediately */
    if (lead_off) {
        state->active_flags |= CARDIAC_FLAG_LEAD_OFF;
        state->active_flags &= ~(CARDIAC_FLAG_TACHYCARDIA | CARDIAC_FLAG_BRADYCARDIA |
                                 CARDIAC_FLAG_ARRHYTHMIA  | CARDIAC_FLAG_PVC |
                                 CARDIAC_FLAG_ASYSTOLE);
        result->qrs_detected = false;
        result->cardiac_flags = state->active_flags;
        result->filtered_ecg = 0;
        result->mwi_signal = 0;
        state->latest_result = *result;
        return false;
    } else {
        state->active_flags &= ~CARDIAC_FLAG_LEAD_OFF;
    }

    /* Check for Asystole (Cardiac Arrest alert after 3.0 s / 750 samples without QRS) */
    if (state->samples_since_qrs >= EDGEAI_ECG_ASYSTOLE_TIMEOUT) {
        state->active_flags |= CARDIAC_FLAG_ASYSTOLE;
    }

    /* -------------------------------------------------------------------------
     * 1. Integer Low-Pass Filter (M=6, fc ~13 Hz, DC gain = 36, delay = 5 samples)
     * Transfer Function: H_LP(z) = ((1 - z^-6) / (1 - z^-1))^2
     * Exact FIR impulse response: h = [1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1]
     * Unconditionally stable with zero DC integrator drift.
     * ------------------------------------------------------------------------- */
    state->lpf_x[state->lpf_idx] = raw_sample;

    /* Symmetric FIR folding: 10 additions, 5 products */
    uint8_t idx = state->lpf_idx;
    #define LPF_TAP(lag) state->lpf_x[(idx + 13 - (lag)) % 13]
    int32_t y_lp = (LPF_TAP(0) + LPF_TAP(10))
                 + ((LPF_TAP(1) + LPF_TAP(9)) << 1)
                 + ((LPF_TAP(2) + LPF_TAP(8)) * 3)
                 + ((LPF_TAP(3) + LPF_TAP(7)) << 2)
                 + ((LPF_TAP(4) + LPF_TAP(6)) * 5)
                 + (LPF_TAP(5) * 6);
    #undef LPF_TAP

    state->lpf_y2 = state->lpf_y1;
    state->lpf_y1 = y_lp;
    state->lpf_idx = (uint8_t)((state->lpf_idx + 1) % 13);

    /* -------------------------------------------------------------------------
     * 2. Integer High-Pass Filter (P=32, K=16, fc ~5.5 Hz, DC gain = 0)
     * p[n] = p[n-1] + y_lp[n] - y_lp[n-32]
     * y_hp[n] = y_lp[n-16] - (p[n] >> 5)
     * ------------------------------------------------------------------------- */
    uint8_t old_32_idx = (uint8_t)((state->hpf_idx + 1) % 33);
    uint8_t mid_16_idx = (uint8_t)((state->hpf_idx + 17) % 33);
    int32_t x_32 = state->hpf_x[old_32_idx];
    int32_t x_16 = state->hpf_x[mid_16_idx];

    state->hpf_sum += y_lp - x_32;
    state->hpf_x[state->hpf_idx] = y_lp;
    state->hpf_idx = old_32_idx;

    int32_t y_hp = x_16 - (state->hpf_sum >> 5);

    /* -------------------------------------------------------------------------
     * 3. 5-Point First Derivative Filter (Delay = 2 samples)
     * y_deriv[n] = (2*y_hp[n] + y_hp[n-1] - y_hp[n-3] - 2*y_hp[n-4]) / 8
     * ------------------------------------------------------------------------- */
    state->deriv_x[4] = state->deriv_x[3];
    state->deriv_x[3] = state->deriv_x[2];
    state->deriv_x[2] = state->deriv_x[1];
    state->deriv_x[1] = state->deriv_x[0];
    state->deriv_x[0] = y_hp;

    int32_t y_deriv = ((state->deriv_x[0] << 1) + state->deriv_x[1]
                      - state->deriv_x[3] - (state->deriv_x[4] << 1)) >> 3;

    /* -------------------------------------------------------------------------
     * 4. Squaring Function (Single-cycle 64-bit SMULL & >> 12 scaling shift)
     * ------------------------------------------------------------------------- */
    int64_t sq64 = (int64_t)y_deriv * (int64_t)y_deriv;
    uint32_t y_sq = (uint32_t)(sq64 >> 12);

    /* -------------------------------------------------------------------------
     * 5. Moving Window Integrator (MWI) (Window = 38 samples = 152 ms)
     * ------------------------------------------------------------------------- */
    state->mwi_sum += (uint64_t)y_sq - (uint64_t)state->mwi_buf[state->mwi_idx];
    state->mwi_buf[state->mwi_idx] = y_sq;
    state->mwi_idx = (uint8_t)((state->mwi_idx + 1) % state->config.mwi_window_samples);

    uint32_t y_mwi = (uint32_t)(state->mwi_sum / state->config.mwi_window_samples);

    /* Prepare output baseline */
    result->qrs_detected          = false;
    result->qrs_timestamp_ms      = timestamp_ms;
    result->rr_interval_ms        = state->rr_mean_ms;
    result->heart_rate_bpm        = state->latest_result.heart_rate_bpm;
    result->heart_rate_smooth_bpm = state->latest_result.heart_rate_smooth_bpm;
    result->hrv_sdnn_ms           = state->latest_result.hrv_sdnn_ms;
    result->hrv_rmssd_ms          = state->latest_result.hrv_rmssd_ms;
    result->cardiac_flags         = state->active_flags;
    result->filtered_ecg          = y_hp;
    result->mwi_signal            = y_mwi;

    /* -------------------------------------------------------------------------
     * 6. Peak Detection State Machine
     * ------------------------------------------------------------------------- */
    bool is_peak = (state->mwi_prev1 > state->mwi_prev2) && (state->mwi_prev1 >= y_mwi);
    uint32_t peak_val = state->mwi_prev1;

    state->mwi_prev2 = state->mwi_prev1;
    state->mwi_prev1 = y_mwi;

    /* Learning Phase: First 2 seconds (500 samples) */
    if (state->learning_phase) {
        if (peak_val > state->peak_max_learning) {
            state->peak_max_learning = peak_val;
        }
        if (state->total_samples_processed >= 500U) {
            state->learning_phase = false;
            state->active_flags &= ~CARDIAC_FLAG_LEARNING;
            state->spki = state->peak_max_learning >> 1;
            if (state->spki == 0) state->spki = 1000U;
            state->npki = state->spki / 10U;
            state->threshold1 = state->npki + ((state->spki - state->npki) >> 2);
            state->threshold2 = state->threshold1 >> 1;
            state->samples_since_qrs = 0;
        }
        state->latest_result = *result;
        return false;
    }

    /* Peak Evaluation */
    if (is_peak) {
        /* A. Refractory blanking check (50 samples = 200 ms) */
        if (state->samples_since_qrs < state->config.refractory_samples) {
            /* Classify as Noise Peak (e.g. T-wave or pacing spike in refractory) */
            state->npki = (peak_val >> 3) + state->npki - (state->npki >> 3);
            if (state->spki > state->npki) {
                state->threshold1 = state->npki + ((state->spki - state->npki) >> 2);
            } else {
                state->threshold1 = state->npki;
            }
            state->threshold2 = state->threshold1 >> 1;
        } else {
            /* B. Outside refractory window: check primary threshold */
            if (peak_val >= state->threshold1) {
                /* Valid QRS detected! */
                record_qrs_beat(state, peak_val, state->samples_since_qrs,
                                timestamp_ms, false, result);
            } else {
                /* Classify as Noise Peak */
                state->npki = (peak_val >> 3) + state->npki - (state->npki >> 3);
                if (state->spki > state->npki) {
                    state->threshold1 = state->npki + ((state->spki - state->npki) >> 2);
                } else {
                    state->threshold1 = state->npki;
                }
                state->threshold2 = state->threshold1 >> 1;

                /* Store highest candidate peak in search-back window */
                if (peak_val > state->peak_candidate_val) {
                    state->peak_candidate_val = peak_val;
                    state->peak_candidate_sample = state->total_samples_processed;
                }
            }
        }
    }

    /* -------------------------------------------------------------------------
     * 7. Search-Back Mechanism for Missed Beats (RR > 166% of rolling mean)
     * ------------------------------------------------------------------------- */
    if (!result->qrs_detected && state->rr_mean_samples > 0) {
        uint32_t rr_timeout = (state->rr_mean_samples * 166U) / 100U;
        if (state->samples_since_qrs > rr_timeout &&
            state->peak_candidate_val >= state->threshold2 &&
            state->peak_candidate_val > 0)
        {
            /* Candidate peak qualifies under secondary search-back threshold */
            uint32_t candidate_age = state->total_samples_processed - state->peak_candidate_sample;
            uint32_t retro_rr_samples = state->samples_since_qrs - candidate_age;

            if (retro_rr_samples >= state->config.refractory_samples) {
                uint32_t retro_ts = timestamp_ms - (candidate_age * EDGEAI_ECG_SAMPLE_PERIOD_MS);
                record_qrs_beat(state, state->peak_candidate_val, retro_rr_samples,
                                retro_ts, true, result);
                state->samples_since_qrs = candidate_age;
            }
            state->peak_candidate_val = 0;
        }
    }

    state->latest_result = *result;
    return result->qrs_detected;
}

__attribute__((used))
void edgeai_ecg_get_latest_result(const edgeai_ecg_state_t *state,
                                  edgeai_ecg_result_t *result)
{
    if (state && result) {
        *result = state->latest_result;
    }
}
