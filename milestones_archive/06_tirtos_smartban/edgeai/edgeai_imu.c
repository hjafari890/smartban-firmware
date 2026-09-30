/*
 * ============================================================================
 * edgeai_imu.c
 * Edge-AI IMU Feature Extraction, Posture State Machine & 4-Phase Fall Detection
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * Milestone M3: Edge-AI Semantic Processing & Data Reduction (Thesis Core)
 * ============================================================================
 */

#include "edgeai/edgeai_imu.h"
#include <string.h>
#include <math.h>

#define PI_CONST 3.14159265f

void edgeai_imu_config_default(edgeai_imu_config_t *config)
{
    if (!config) return;
    config->freefall_thresh_g      = EDGEAI_FALL_FREEFALL_THRESH_G;
    config->freefall_min_samples   = EDGEAI_FALL_FREEFALL_MIN_SAMP;
    config->impact_thresh_g        = EDGEAI_FALL_IMPACT_THRESH_G;
    config->impact_window_max_ms   = EDGEAI_FALL_IMPACT_WIN_MAX_MS;
    config->tilt_thresh_deg        = EDGEAI_FALL_TILT_THRESH_DEG;
    config->rest_thresh_sma_g      = EDGEAI_FALL_REST_THRESH_SMA_G;
    config->rest_duration_samples  = EDGEAI_FALL_REST_MIN_SAMPLES;
    config->act_sedentary_sma_max  = EDGEAI_ACT_SEDENTARY_SMA_MAX;
    config->act_active_sma_max     = EDGEAI_ACT_ACTIVE_SMA_MAX;
}

void edgeai_imu_init(edgeai_imu_state_t *state, const edgeai_imu_config_t *config)
{
    if (!state) return;
    memset(state, 0, sizeof(edgeai_imu_state_t));

    if (config) {
        state->config = *config;
    } else {
        edgeai_imu_config_default(&state->config);
    }

    edgeai_imu_reset(state);
}

void edgeai_imu_reset(edgeai_imu_state_t *state)
{
    if (!state) return;

    state->window_head = 0;
    state->window_count = 0;
    state->step_counter = 0;

    memset(state->sample_window, 0, sizeof(state->sample_window));
    memset(state->baseline_history, 0, sizeof(state->baseline_history));
    state->baseline_head = 0;
    state->baseline_count = 0;

    state->ref_base_x = 0.0f;
    state->ref_base_y = 980.0f; /* Upright gravity default */
    state->ref_base_z = 0.0f;

    state->current_posture = POSTURE_SEDENTARY;
    state->current_sub_posture = SUBPOSTURE_UPRIGHT;
    state->posture_debounce_counter = 0;
    state->candidate_posture = POSTURE_SEDENTARY;

    state->fall_phase = FALL_PHASE_IDLE;
    state->fall_status = FALL_STATUS_NONE;
    state->fall_alarm_latched = false;
    state->consecutive_freefall_samples = 0;
    state->t_freefall_end_ms = 0;
    state->impact_peak_g = 0.0f;
    state->t_impact_ms = 0;
    state->measured_tilt_change_deg = 0.0f;
    state->immobile_sample_count = 0;
    state->post_impact_sum_x = 0.0f;
    state->post_impact_sum_y = 0.0f;
    state->post_impact_sum_z = 0.0f;
    state->post_impact_sample_count = 0;

    memset(&state->latest_result, 0, sizeof(edgeai_imu_result_t));
    state->latest_result.posture = POSTURE_SEDENTARY;
    state->latest_result.sub_posture = SUBPOSTURE_UPRIGHT;
}

void edgeai_imu_clear_fall_alarm(edgeai_imu_state_t *state)
{
    if (!state) return;
    state->fall_alarm_latched = false;
    state->fall_phase = FALL_PHASE_IDLE;
    state->fall_status = FALL_STATUS_NONE;
    state->consecutive_freefall_samples = 0;
    state->immobile_sample_count = 0;
    state->impact_peak_g = 0.0f;
    state->measured_tilt_change_deg = 0.0f;
    state->latest_result.fall_detected = false;
    state->latest_result.fall_phase = FALL_PHASE_IDLE;
    state->latest_result.fall_status = FALL_STATUS_NONE;
}

float edgeai_imu_calc_3d_angle_diff(float x1, float y1, float z1,
                                    float x2, float y2, float z2)
{
    float mag1 = sqrtf(x1 * x1 + y1 * y1 + z1 * z1);
    float mag2 = sqrtf(x2 * x2 + y2 * y2 + z2 * z2);

    if (mag1 < 1e-4f || mag2 < 1e-4f) {
        return 0.0f;
    }

    float dot = (x1 * x2 + y1 * y2 + z1 * z2) / (mag1 * mag2);
    if (dot > 1.0f) dot = 1.0f;
    if (dot < -1.0f) dot = -1.0f;

    return acosf(dot) * (180.0f / PI_CONST);
}

void edgeai_imu_calc_stats(const hal_imu_sample_t *samples,
                           uint16_t count,
                           edgeai_imu_features_t *features)
{
    if (!samples || count == 0 || !features) return;
    memset(features, 0, sizeof(edgeai_imu_features_t));

    float fcount = (float)count;

    /* -------------------------------------------------------------------------
     * Pass 1: Accumulate Means
     * ------------------------------------------------------------------------- */
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f, sum_mag = 0.0f;

    for (uint16_t i = 0; i < count; i++) {
        float x = (float)samples[i].x_mg;
        float y = (float)samples[i].y_mg;
        float z = (float)samples[i].z_mg;
        float mag = samples[i].total_g;
        if (mag <= 0.0f) {
            mag = sqrtf(x * x + y * y + z * z) / 1000.0f;
        }

        sum_x += x;
        sum_y += y;
        sum_z += z;
        sum_mag += mag;
    }

    features->mean_x_mg  = sum_x / fcount;
    features->mean_y_mg  = sum_y / fcount;
    features->mean_z_mg  = sum_z / fcount;
    features->mean_mag_g = sum_mag / fcount;

    /* -------------------------------------------------------------------------
     * Pass 2: Accumulate Two-Pass Variances and Dynamic SMA
     * ------------------------------------------------------------------------- */
    float sum_sq_x = 0.0f, sum_sq_y = 0.0f, sum_sq_z = 0.0f, sum_sq_mag = 0.0f;
    float sum_sma_dyn = 0.0f;
    float sum_sma_raw = 0.0f;

    for (uint16_t i = 0; i < count; i++) {
        float x = (float)samples[i].x_mg;
        float y = (float)samples[i].y_mg;
        float z = (float)samples[i].z_mg;
        float mag = samples[i].total_g;
        if (mag <= 0.0f) {
            mag = sqrtf(x * x + y * y + z * z) / 1000.0f;
        }

        float dx = x - features->mean_x_mg;
        float dy = y - features->mean_y_mg;
        float dz = z - features->mean_z_mg;
        float dmag = mag - features->mean_mag_g;

        sum_sq_x += dx * dx;
        sum_sq_y += dy * dy;
        sum_sq_z += dz * dz;
        sum_sq_mag += dmag * dmag;

        sum_sma_dyn += (fabsf(dx) + fabsf(dy) + fabsf(dz)) / 1000.0f;
        sum_sma_raw += (fabsf(x) + fabsf(y) + fabsf(z)) / 1000.0f;
    }

    features->var_x_mg2 = sum_sq_x / fcount;
    features->var_y_mg2 = sum_sq_y / fcount;
    features->var_z_mg2 = sum_sq_z / fcount;
    features->var_mag_g2 = sum_sq_mag / fcount;

    features->total_variance_g2 = (features->var_x_mg2 + features->var_y_mg2 + features->var_z_mg2) / 1000000.0f;

    features->std_x_mg   = sqrtf(features->var_x_mg2);
    features->std_y_mg   = sqrtf(features->var_y_mg2);
    features->std_z_mg   = sqrtf(features->var_z_mg2);
    features->std_total_g = sqrtf(features->total_variance_g2);

    features->sma_dynamic_g = sum_sma_dyn / fcount;
    features->sma_raw_g     = sum_sma_raw / fcount;

    /* Dynamic Tilt Angles relative to Gravity Vector */
    features->pitch_deg = atan2f(features->mean_x_mg,
        sqrtf(features->mean_y_mg * features->mean_y_mg + features->mean_z_mg * features->mean_z_mg)) * (180.0f / PI_CONST);

    features->roll_deg = atan2f(features->mean_y_mg,
        sqrtf(features->mean_x_mg * features->mean_x_mg + features->mean_z_mg * features->mean_z_mg)) * (180.0f / PI_CONST);
}

static void update_posture_fsm(edgeai_imu_state_t *state, const edgeai_imu_features_t *features)
{
    float sma = features->sma_dynamic_g;
    float total_std = features->std_total_g;

    /* Direct escalation on high-intensity sprint */
    if (sma >= 0.75f) {
        state->current_posture = POSTURE_HIGH_DYNAMIC;
        state->posture_debounce_counter = 0;
    } else {
        switch (state->current_posture) {
            case POSTURE_SEDENTARY:
                if (sma >= EDGEAI_ACT_HYST_ENTER_ACTIVE && total_std >= 0.05f) {
                    if (state->candidate_posture == POSTURE_ACTIVE) {
                        state->posture_debounce_counter++;
                        if (state->posture_debounce_counter >= 2U) {
                            state->current_posture = POSTURE_ACTIVE;
                            state->posture_debounce_counter = 0;
                        }
                    } else {
                        state->candidate_posture = POSTURE_ACTIVE;
                        state->posture_debounce_counter = 1U;
                    }
                } else {
                    state->posture_debounce_counter = 0;
                }
                break;

            case POSTURE_ACTIVE:
                if (sma >= EDGEAI_ACT_HYST_ENTER_HIGH_DYN && total_std >= 0.28f) {
                    if (state->candidate_posture == POSTURE_HIGH_DYNAMIC) {
                        state->posture_debounce_counter++;
                        if (state->posture_debounce_counter >= 2U) {
                            state->current_posture = POSTURE_HIGH_DYNAMIC;
                            state->posture_debounce_counter = 0;
                        }
                    } else {
                        state->candidate_posture = POSTURE_HIGH_DYNAMIC;
                        state->posture_debounce_counter = 1U;
                    }
                } else if (sma < EDGEAI_ACT_HYST_EXIT_ACTIVE && total_std < 0.035f) {
                    if (state->candidate_posture == POSTURE_SEDENTARY) {
                        state->posture_debounce_counter++;
                        if (state->posture_debounce_counter >= 4U) {
                            state->current_posture = POSTURE_SEDENTARY;
                            state->posture_debounce_counter = 0;
                        }
                    } else {
                        state->candidate_posture = POSTURE_SEDENTARY;
                        state->posture_debounce_counter = 1U;
                    }
                } else {
                    state->posture_debounce_counter = 0;
                }
                break;

            case POSTURE_HIGH_DYNAMIC:
                if (sma < EDGEAI_ACT_HYST_EXIT_HIGH_DYN || total_std < 0.22f) {
                    if (state->candidate_posture == POSTURE_ACTIVE) {
                        state->posture_debounce_counter++;
                        if (state->posture_debounce_counter >= 3U) {
                            state->current_posture = POSTURE_ACTIVE;
                            state->posture_debounce_counter = 0;
                        }
                    } else {
                        state->candidate_posture = POSTURE_ACTIVE;
                        state->posture_debounce_counter = 1U;
                    }
                } else {
                    state->posture_debounce_counter = 0;
                }
                break;

            default:
                state->current_posture = POSTURE_SEDENTARY;
                break;
        }
    }

    /* Sub-posture orientation determination in SEDENTARY state */
    if (state->current_posture == POSTURE_SEDENTARY) {
        float mx = features->mean_x_mg;
        float my = features->mean_y_mg;
        float mz = features->mean_z_mg;

        if (my > 600.0f && fabsf(mx) < 500.0f && fabsf(mz) < 500.0f) {
            state->current_sub_posture = SUBPOSTURE_UPRIGHT;
        } else if (mz > 600.0f && fabsf(my) < 500.0f) {
            state->current_sub_posture = SUBPOSTURE_LYING_SUPINE;
        } else if (mz < -600.0f && fabsf(my) < 500.0f) {
            state->current_sub_posture = SUBPOSTURE_LYING_PRONE;
        } else if (fabsf(mx) > 600.0f && fabsf(my) < 500.0f) {
            state->current_sub_posture = SUBPOSTURE_LYING_LATERAL;
        } else {
            state->current_sub_posture = SUBPOSTURE_UNKNOWN;
        }
    } else {
        state->current_sub_posture = SUBPOSTURE_UPRIGHT;
    }
}

bool edgeai_imu_process_sample(edgeai_imu_state_t *state,
                               const hal_imu_sample_t *sample,
                               edgeai_imu_result_t *result)
{
    if (!state || !sample || !result) return false;

    uint32_t ts = sample->timestamp_ms;
    float mag_g = sample->total_g;
    if (mag_g <= 0.0f) {
        float fx = (float)sample->x_mg;
        float fy = (float)sample->y_mg;
        float fz = (float)sample->z_mg;
        mag_g = sqrtf(fx * fx + fy * fy + fz * fz) / 1000.0f;
    }

    /* -------------------------------------------------------------------------
     * 1. Ingest Sample into Sliding Window Buffer
     * ------------------------------------------------------------------------- */
    state->sample_window[state->window_head] = *sample;
    state->window_head = (uint16_t)((state->window_head + 1) % EDGEAI_IMU_WINDOW_SIZE);
    if (state->window_count < EDGEAI_IMU_WINDOW_SIZE) {
        state->window_count++;
    }
    state->step_counter++;

    /* -------------------------------------------------------------------------
     * 2. Sample-Level 4-Phase Fall Detection State Machine (100 Hz cadence)
     * ------------------------------------------------------------------------- */
    if (!state->fall_alarm_latched) {
        switch (state->fall_phase) {
            case FALL_PHASE_IDLE:
                /* Phase 1: Free-Fall Detection (|A| < 0.50g for >= 60 ms / 6 samples) */
                if (mag_g < state->config.freefall_thresh_g) {
                    state->consecutive_freefall_samples++;
                    if (state->consecutive_freefall_samples >= state->config.freefall_min_samples) {
                        state->fall_phase = FALL_PHASE_FREEFALL_DETECTED;
                        state->t_freefall_end_ms = ts;
                        state->impact_peak_g = 0.0f;
                        state->consecutive_freefall_samples = 0;

                        /* Latch pre-fall baseline orientation */
                        if (state->baseline_count > 0) {
                            float bx = 0.0f, by = 0.0f, bz = 0.0f;
                            for (uint8_t i = 0; i < state->baseline_count; i++) {
                                bx += state->baseline_history[i].mean_x;
                                by += state->baseline_history[i].mean_y;
                                bz += state->baseline_history[i].mean_z;
                            }
                            state->ref_base_x = bx / (float)state->baseline_count;
                            state->ref_base_y = by / (float)state->baseline_count;
                            state->ref_base_z = bz / (float)state->baseline_count;
                        } else {
                            state->ref_base_x = (float)sample->x_mg;
                            state->ref_base_y = (float)sample->y_mg;
                            state->ref_base_z = (float)sample->z_mg;
                        }
                    }
                } else {
                    state->consecutive_freefall_samples = 0;
                }
                break;

            case FALL_PHASE_FREEFALL_DETECTED:
                /* Phase 2: Impact Shock Detection (|A| > 3.00g within 100-350 ms) */
                {
                    uint32_t dt = ts - state->t_freefall_end_ms;
                    if (mag_g > state->impact_peak_g) {
                        state->impact_peak_g = mag_g;
                    }

                    if (mag_g >= state->config.impact_thresh_g &&
                        dt >= EDGEAI_FALL_IMPACT_WIN_MIN_MS &&
                        dt <= state->config.impact_window_max_ms)
                    {
                        state->fall_phase = FALL_PHASE_IMPACT_DETECTED;
                        state->fall_status = FALL_STATUS_SUSPECTED;
                        state->t_impact_ms = ts;
                        state->post_impact_sum_x = 0.0f;
                        state->post_impact_sum_y = 0.0f;
                        state->post_impact_sum_z = 0.0f;
                        state->post_impact_sample_count = 0;
                    } else if (dt > state->config.impact_window_max_ms + 100U) {
                        /* Timeout: no qualifying impact spike observed */
                        state->fall_phase = FALL_PHASE_IDLE;
                        state->fall_status = FALL_STATUS_NONE;
                    }
                }
                break;

            case FALL_PHASE_IMPACT_DETECTED:
                /* Phase 3: Post-Impact Orientation Verification (Evaluation window 200-500 ms post-impact) */
                {
                    uint32_t dt_impact = ts - state->t_impact_ms;
                    if (dt_impact >= 200U && dt_impact <= 500U) {
                        state->post_impact_sum_x += (float)sample->x_mg;
                        state->post_impact_sum_y += (float)sample->y_mg;
                        state->post_impact_sum_z += (float)sample->z_mg;
                        state->post_impact_sample_count++;
                    }

                    if (dt_impact > 500U) {
                        float post_x = state->post_impact_sample_count > 0 ?
                            (state->post_impact_sum_x / (float)state->post_impact_sample_count) : (float)sample->x_mg;
                        float post_y = state->post_impact_sample_count > 0 ?
                            (state->post_impact_sum_y / (float)state->post_impact_sample_count) : (float)sample->y_mg;
                        float post_z = state->post_impact_sample_count > 0 ?
                            (state->post_impact_sum_z / (float)state->post_impact_sample_count) : (float)sample->z_mg;

                        float delta_deg = edgeai_imu_calc_3d_angle_diff(
                            state->ref_base_x, state->ref_base_y, state->ref_base_z,
                            post_x, post_y, post_z);

                        state->measured_tilt_change_deg = delta_deg;

                        if (delta_deg >= state->config.tilt_thresh_deg) {
                            /* Major orientation shift verified: advance to immobility check */
                            state->fall_phase = FALL_PHASE_REST_WAIT;
                            state->immobile_sample_count = 0;
                        } else {
                            /* Orientation change insufficient (e.g. landing from a jump): reject as ADL */
                            state->fall_phase = FALL_PHASE_IDLE;
                            state->fall_status = FALL_STATUS_REJECTED_ADL;
                        }
                    }
                }
                break;

            case FALL_PHASE_REST_WAIT:
                /* Phase 4: Post-Fall Immobility Confirmation (2.0 s / 200 samples) */
                {
                    float post_x = state->post_impact_sample_count > 0 ?
                        (state->post_impact_sum_x / (float)state->post_impact_sample_count) : 0.0f;
                    float post_y = state->post_impact_sample_count > 0 ?
                        (state->post_impact_sum_y / (float)state->post_impact_sample_count) : 0.0f;
                    float post_z = state->post_impact_sample_count > 0 ?
                        (state->post_impact_sum_z / (float)state->post_impact_sample_count) : 1000.0f;

                    float dev = fabsf((float)sample->x_mg - post_x) +
                                fabsf((float)sample->y_mg - post_y) +
                                fabsf((float)sample->z_mg - post_z);

                    /* If subject vigorously moves or stands up, mark as recovered stumble */
                    if (dev > 750.0f) {
                        state->fall_phase = FALL_PHASE_IDLE;
                        state->fall_status = FALL_STATUS_RECOVERED;
                    } else {
                        state->immobile_sample_count++;
                        if (state->immobile_sample_count >= state->config.rest_duration_samples) {
                            /* Confirmed Incapacitated Fall! */
                            state->fall_phase = FALL_PHASE_CONFIRMED;
                            state->fall_status = FALL_STATUS_CONFIRMED;
                            state->fall_alarm_latched = true;
                        }
                    }
                }
                break;

            case FALL_PHASE_CONFIRMED:
                state->fall_alarm_latched = true;
                break;

            default:
                state->fall_phase = FALL_PHASE_IDLE;
                break;
        }
    }

    /* -------------------------------------------------------------------------
     * 3. Sliding-Window Statistical Evaluation (Every 50 samples / 500 ms)
     * ------------------------------------------------------------------------- */
    bool window_done = false;

    if (state->step_counter >= EDGEAI_IMU_STEP_SIZE && state->window_count >= EDGEAI_IMU_WINDOW_SIZE) {
        state->step_counter = 0;
        window_done = true;

        /* Linearize samples from circular buffer */
        hal_imu_sample_t linear_samples[EDGEAI_IMU_WINDOW_SIZE];
        for (uint16_t i = 0; i < EDGEAI_IMU_WINDOW_SIZE; i++) {
            uint16_t c_idx = (uint16_t)((state->window_head + i) % EDGEAI_IMU_WINDOW_SIZE);
            linear_samples[i] = state->sample_window[c_idx];
        }

        /* Calculate statistical feature vector */
        edgeai_imu_calc_stats(linear_samples, EDGEAI_IMU_WINDOW_SIZE, &state->latest_result.features);

        /* Update Pre-Fall Baseline History Buffer (last 2.0 s) */
        state->baseline_history[state->baseline_head].mean_x = state->latest_result.features.mean_x_mg;
        state->baseline_history[state->baseline_head].mean_y = state->latest_result.features.mean_y_mg;
        state->baseline_history[state->baseline_head].mean_z = state->latest_result.features.mean_z_mg;
        state->baseline_head = (uint8_t)((state->baseline_head + 1) % EDGEAI_IMU_BASELINE_RING_SIZE);
        if (state->baseline_count < EDGEAI_IMU_BASELINE_RING_SIZE) {
            state->baseline_count++;
        }

        /* Update Posture and Activity State Machine */
        update_posture_fsm(state, &state->latest_result.features);
    }

    /* -------------------------------------------------------------------------
     * 4. Assemble Output Result
     * ------------------------------------------------------------------------- */
    result->timestamp_ms          = ts;
    result->window_completed      = window_done;
    result->features              = state->latest_result.features;
    result->posture               = state->current_posture;
    result->sub_posture           = state->current_sub_posture;
    result->fall_status           = state->fall_status;
    result->fall_phase            = state->fall_phase;
    result->fall_detected         = state->fall_alarm_latched;
    result->impact_peak_g         = state->impact_peak_g;
    result->tilt_change_deg       = state->measured_tilt_change_deg;
    result->immobility_duration_s = (float)state->immobile_sample_count * 0.01f;

    state->latest_result = *result;
    return window_done;
}

void edgeai_imu_get_features(const edgeai_imu_state_t *state, edgeai_imu_features_t *features)
{
    if (state && features) {
        *features = state->latest_result.features;
    }
}

void edgeai_imu_get_result(const edgeai_imu_state_t *state, edgeai_imu_result_t *result)
{
    if (state && result) {
        *result = state->latest_result;
    }
}
