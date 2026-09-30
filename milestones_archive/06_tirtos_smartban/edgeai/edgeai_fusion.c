/*
 * ============================================================================
 * edgeai_fusion.c
 * SmartBAN Edge-AI Multi-Modal Context Fusion & Dual-Mode Stream Controller
 * Target: CC2652R1 (ARM Cortex-M4F), SimpleLink SDK 8.33, TI-RTOS7
 *
 * Thesis Core: Cross-modal optical/thermal skin contact validation, artifact
 * rejection, on-body thermal classification, and dual-mode data reduction.
 * ============================================================================
 */

#include "edgeai/edgeai_fusion.h"
#include "edgeai/edgeai_ecg.h"
#include "edgeai/edgeai_imu.h"
#include <stdio.h>
#include <string.h>

/* Global Stream Mode (Thread-Safe default: STREAM_MODE_SEMANTIC) */
static volatile smartban_stream_mode_t s_active_stream_mode = STREAM_MODE_SEMANTIC;

/* Static thread-safe slow sensor snapshot cache */
static edgeai_slow_cache_t s_slow_cache = {
    .timestamp_ms     = 0,
    .ambient_temp_c   = 25.0f,
    .object_temp_c    = 36.5f,
    .prox_counts      = 7500,
    .als_counts       = 100,
    .ambient_lux      = 350.0f,
    .raw_contact_flag = true,
    .fir_online       = true,
    .optical_online   = true
};

void edgeai_slow_cache_update(float amb_c, float obj_c, uint16_t prox, uint16_t als,
                              float lux, bool fir_ok, bool opt_ok)
{
    s_slow_cache.ambient_temp_c   = amb_c;
    s_slow_cache.object_temp_c    = obj_c;
    s_slow_cache.prox_counts      = prox;
    s_slow_cache.als_counts       = als;
    s_slow_cache.ambient_lux      = lux;
    s_slow_cache.raw_contact_flag = (prox >= 6000U);
    s_slow_cache.fir_online       = fir_ok;
    s_slow_cache.optical_online   = opt_ok;
}

void edgeai_slow_cache_get(edgeai_slow_cache_t *dest)
{
    if (dest) {
        *dest = s_slow_cache;
    }
}

smartban_stream_mode_t edgeai_get_stream_mode(void)
{
    return s_active_stream_mode;
}

void edgeai_set_stream_mode(smartban_stream_mode_t mode)
{
    s_active_stream_mode = mode;
}

smartban_stream_mode_t edgeai_toggle_stream_mode(void)
{
    if (s_active_stream_mode == STREAM_MODE_SEMANTIC) {
        s_active_stream_mode = STREAM_MODE_RAW;
    } else {
        s_active_stream_mode = STREAM_MODE_SEMANTIC;
    }
    return s_active_stream_mode;
}

void edgeai_fusion_init(edgeai_fusion_state_t *state, const edgeai_fusion_config_t *cfg)
{
    if (!state) return;
    memset(state, 0, sizeof(edgeai_fusion_state_t));

    if (cfg) {
        state->config = *cfg;
    } else {
        state->config.prox_thresh_on       = 6000U;
        state->config.prox_thresh_off      = 5500U;
        state->config.contact_debounce_max = 2U;
        state->config.temp_hypothermia_c   = 35.0f;
        state->config.temp_hypo_clear_c    = 35.3f;
        state->config.temp_fever_c         = 38.0f;
        state->config.temp_fever_clear_c   = 37.8f;
        state->config.temp_normal_high_c   = 37.5f;
    }

    edgeai_fusion_reset(state);
}

void edgeai_fusion_reset(edgeai_fusion_state_t *state)
{
    if (!state) return;

    state->contact_state = CONTACT_STATE_DISCONNECTED;
    state->candidate_contact_state = CONTACT_STATE_DISCONNECTED;
    state->debounce_counter = 0;
    state->thermal_class = THERMAL_CLASS_AMBIENT_OFFBODY;
    state->hypo_alert_active = false;
    state->fever_alert_active = false;
    state->sequence_counter = 0;
    state->last_token_time_ms = 0;
    state->total_tokens_emitted = 0;
    state->anomaly_tokens_emitted = 0;

    memset(&state->last_token, 0, sizeof(smartban_semantic_token_t));
    state->last_token.posture_state = (uint8_t)POSTURE_STATE_SEDENTARY;
    state->last_token.skin_temp_c = 25.0f;
}

bool edgeai_fusion_process(edgeai_fusion_state_t *state,
                           const void *ecg_state_ptr,
                           const void *imu_state_ptr,
                           const edgeai_slow_cache_t *slow_cache,
                           smartban_semantic_token_t *out_token)
{
    if (!state || !slow_cache || !out_token) return false;

    const edgeai_ecg_state_t *ecg = (const edgeai_ecg_state_t *)ecg_state_ptr;
    const edgeai_imu_state_t *imu = (const edgeai_imu_state_t *)imu_state_ptr;

    /* -------------------------------------------------------------------------
     * 1. Schmitt-Trigger Contact State Hysteresis & Debounce (10 Hz cadence)
     * ------------------------------------------------------------------------- */
    edgeai_contact_state_t candidate = state->contact_state;

    if (slow_cache->prox_counts >= state->config.prox_thresh_on) {
        candidate = CONTACT_STATE_CONNECTED;
    } else if (slow_cache->prox_counts <= state->config.prox_thresh_off) {
        candidate = CONTACT_STATE_DISCONNECTED;
    }

    if (candidate != state->contact_state) {
        if (candidate == state->candidate_contact_state) {
            state->debounce_counter++;
            if (state->debounce_counter >= state->config.contact_debounce_max) {
                state->contact_state = candidate;
                state->debounce_counter = 0;
            }
        } else {
            state->candidate_contact_state = candidate;
            state->debounce_counter = 1U;
        }
    } else {
        state->debounce_counter = 0;
        state->candidate_contact_state = state->contact_state;
    }

    bool is_contact = (state->contact_state == CONTACT_STATE_CONNECTED);

    /* Contact confidence metric (0..100%) */
    uint8_t confidence = 0;
    if (is_contact) {
        if (slow_cache->prox_counts >= 10000U) {
            confidence = 100U;
        } else {
            confidence = (uint8_t)((slow_cache->prox_counts * 100UL) / 10000UL);
        }
    }

    /* -------------------------------------------------------------------------
     * 2. On-Body Physiological Thermal Classification & Artifact Rejection
     * ------------------------------------------------------------------------- */
    float obj_temp = slow_cache->object_temp_c;

    if (!is_contact) {
        /* OFF-BODY: Reading is ambient room air; suppress clinical thermal alarms */
        state->thermal_class = THERMAL_CLASS_AMBIENT_OFFBODY;
        state->hypo_alert_active = false;
        state->fever_alert_active = false;
    } else {
        /* ON-BODY: Evaluate clinical thermal boundaries with 0.3 C hysteresis */
        if (obj_temp < state->config.temp_hypothermia_c) {
            state->hypo_alert_active = true;
        } else if (obj_temp >= state->config.temp_hypo_clear_c) {
            state->hypo_alert_active = false;
        }

        if (obj_temp > state->config.temp_fever_c) {
            state->fever_alert_active = true;
        } else if (obj_temp <= state->config.temp_fever_clear_c) {
            state->fever_alert_active = false;
        }

        if (state->hypo_alert_active) {
            state->thermal_class = THERMAL_CLASS_HYPOTHERMIA;
        } else if (state->fever_alert_active) {
            state->thermal_class = THERMAL_CLASS_HYPERTHERMIA;
        } else if (obj_temp > state->config.temp_normal_high_c) {
            state->thermal_class = THERMAL_CLASS_ELEVATED;
        } else {
            state->thermal_class = THERMAL_CLASS_NORMAL;
        }
    }

    /* -------------------------------------------------------------------------
     * 3. Cross-Modal Anomaly & Alert Synthesis
     * ------------------------------------------------------------------------- */
    uint16_t alert_mask = ALERT_FLAG_NONE;
    uint8_t cardiac_flags = CARDIAC_FLAG_NORMAL;

    if (ecg != NULL) {
        cardiac_flags = ecg->latest_result.cardiac_flags;

        if (!is_contact || (cardiac_flags & CARDIAC_FLAG_LEAD_OFF)) {
            /* If off-body or leads disconnected, suppress false cardiac alarms */
            cardiac_flags |= CARDIAC_FLAG_DISCONNECTED;
            alert_mask |= ALERT_FLAG_LEAD_OFF;
        } else {
            if (cardiac_flags & CARDIAC_FLAG_TACHYCARDIA) {
                alert_mask |= ALERT_FLAG_TACHYCARDIA;
            }
            if (cardiac_flags & CARDIAC_FLAG_BRADYCARDIA) {
                alert_mask |= ALERT_FLAG_BRADYCARDIA;
            }
            if (cardiac_flags & (CARDIAC_FLAG_ARRHYTHMIA | CARDIAC_FLAG_PVC)) {
                alert_mask |= ALERT_FLAG_ARRHYTHMIA;
            }
        }
    }

    /* Thermal alerts */
    if (is_contact) {
        if (state->hypo_alert_active) {
            alert_mask |= ALERT_FLAG_HYPOTHERMIA;
        }
        if (state->fever_alert_active) {
            alert_mask |= ALERT_FLAG_FEVER;
        }
    }

    /* Motion alerts */
    bool fall_detected = false;
    uint8_t posture = (uint8_t)POSTURE_STATE_SEDENTARY;
    float sma = 0.0f;
    int16_t pitch = 0, roll = 0;

    if (imu != NULL) {
        fall_detected = imu->latest_result.fall_detected;
        posture = (uint8_t)imu->latest_result.posture;
        sma = imu->latest_result.features.sma_dynamic_g;
        pitch = (int16_t)imu->latest_result.features.pitch_deg;
        roll = (int16_t)imu->latest_result.features.roll_deg;

        if (fall_detected) {
            alert_mask |= ALERT_FLAG_FALL_IMPACT;
            posture = (uint8_t)POSTURE_STATE_FALL_REST;
        }
    }

    /* Hardware sensor faults */
    if (!slow_cache->fir_online || !slow_cache->optical_online) {
        alert_mask |= ALERT_FLAG_OPTICAL_FAULT;
    }

    /* Immediate emergency alert condition */
    bool is_anomaly = (alert_mask & (ALERT_FLAG_TACHYCARDIA | ALERT_FLAG_BRADYCARDIA |
                                     ALERT_FLAG_ARRHYTHMIA  | ALERT_FLAG_FALL_IMPACT |
                                     ALERT_FLAG_HYPOTHERMIA | ALERT_FLAG_FEVER)) != 0;

    /* -------------------------------------------------------------------------
     * 4. Assemble Unified Semantic State Token
     * ------------------------------------------------------------------------- */
    state->sequence_counter++;

    out_token->timestamp_ms       = slow_cache->timestamp_ms;
    out_token->sequence_num       = state->sequence_counter;

    if (ecg != NULL) {
        out_token->heart_rate_bpm = ecg->latest_result.heart_rate_bpm;
        out_token->rr_interval_ms = ecg->latest_result.rr_interval_ms;
        out_token->hrv_rmssd_ms   = ecg->latest_result.hrv_rmssd_ms;
        out_token->hrv_sdnn_ms    = ecg->latest_result.hrv_sdnn_ms;
    } else {
        out_token->heart_rate_bpm = 75U;
        out_token->rr_interval_ms = 800U;
        out_token->hrv_rmssd_ms   = 0U;
        out_token->hrv_sdnn_ms    = 0U;
    }

    out_token->cardiac_flags      = cardiac_flags;
    out_token->posture_state      = posture;
    out_token->activity_sma_g     = sma;
    out_token->tilt_pitch_deg     = pitch;
    out_token->tilt_roll_deg      = roll;
    out_token->fall_detected      = fall_detected;

    out_token->skin_contact       = is_contact;
    out_token->contact_confidence = confidence;
    out_token->thermal_class      = (uint8_t)state->thermal_class;
    out_token->skin_temp_c        = obj_temp;
    out_token->ambient_temp_c     = slow_cache->ambient_temp_c;
    out_token->ambient_lux        = (uint32_t)slow_cache->ambient_lux;

    out_token->alert_mask         = alert_mask;
    out_token->is_anomaly_event   = is_anomaly;

    state->last_token = *out_token;
    state->total_tokens_emitted++;
    if (is_anomaly) {
        state->anomaly_tokens_emitted++;
    }

    return true;
}

size_t edgeai_format_semantic_json(const smartban_semantic_token_t *token, char *buf, size_t max_len)
{
    if (!token || !buf || max_len == 0) return 0;

    const char *posture_str = "SEDENTARY";
    if (token->posture_state == (uint8_t)POSTURE_STATE_ACTIVE) {
        posture_str = "ACTIVE";
    } else if (token->posture_state == (uint8_t)POSTURE_STATE_HIGH_DYNAMIC) {
        posture_str = "HIGH_DYNAMIC";
    } else if (token->posture_state == (uint8_t)POSTURE_STATE_FALL_REST) {
        posture_str = "FALL_REST";
    }

    int len = snprintf(buf, max_len,
        "{\"type\":\"SEM\",\"ts\":%lu,\"hr\":%u,\"rr\":%u,\"rmssd\":%u,\"sdnn\":%u,"
        "\"flags\":%u,\"posture\":\"%s\",\"fall\":%d,\"contact\":%d,\"temp\":%.1f,\"lux\":%lu}\r\n",
        (unsigned long)token->timestamp_ms,
        (unsigned int)token->heart_rate_bpm,
        (unsigned int)token->rr_interval_ms,
        (unsigned int)token->hrv_rmssd_ms,
        (unsigned int)token->hrv_sdnn_ms,
        (unsigned int)token->cardiac_flags,
        posture_str,
        token->fall_detected ? 1 : 0,
        token->skin_contact ? 1 : 0,
        (double)token->skin_temp_c,
        (unsigned long)token->ambient_lux);

    if (len < 0) return 0;
    if ((size_t)len >= max_len) return max_len - 1;
    return (size_t)len;
}

size_t edgeai_format_raw_json(uint32_t ts_ms, int32_t ecg_raw, int16_t ax, int16_t ay, int16_t az,
                              char *buf, size_t max_len)
{
    if (!buf || max_len == 0) return 0;

    int len = snprintf(buf, max_len,
        "{\"type\":\"RAW\",\"ts\":%lu,\"ecg\":%ld,\"ax\":%d,\"ay\":%d,\"az\":%d}\r\n",
        (unsigned long)ts_ms,
        (long)ecg_raw,
        (int)ax,
        (int)ay,
        (int)az);

    if (len < 0) return 0;
    if ((size_t)len >= max_len) return max_len - 1;
    return (size_t)len;
}
