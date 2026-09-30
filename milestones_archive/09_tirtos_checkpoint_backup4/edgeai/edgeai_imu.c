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

    edgeai_imu_reset_pdr(state);

    memset(&state->latest_result, 0, sizeof(edgeai_imu_result_t));
    state->latest_result.posture = POSTURE_SEDENTARY;
    state->latest_result.sub_posture = SUBPOSTURE_UPRIGHT;
}

void edgeai_imu_reset_pdr(edgeai_imu_state_t *state)
{
    if (!state) return;
    memset(&state->locomotion, 0, sizeof(edgeai_locomotion_t));
    state->pdr_gravity_est = 1.0f;
    state->pdr_acc_smooth = 0.0f;
    state->pdr_peak_val = 0.0f;
    state->pdr_valley_val = 0.0f;
    state->pdr_seeking_peak = true;
    state->pdr_last_step_ts = 0;
    state->pdr_last_motion_ts = 0;
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
    features->pitch_deg = atan2f(-features->mean_x_mg,
        sqrtf(features->mean_y_mg * features->mean_y_mg + features->mean_z_mg * features->mean_z_mg)) * (180.0f / PI_CONST);

    features->roll_deg = atan2f(features->mean_y_mg, features->mean_z_mg) * (180.0f / PI_CONST);
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
     * 1b. TinyML Adaptive Pedometer & Omnidirectional Step Counter
     *     (Analog Devices AN-1267 / Bosch BMA423 2-Step Rhythmic Architecture)
     * ------------------------------------------------------------------------- */
    static uint8_t s_tentative_steps = 0;
    if (state->locomotion.step_count == 0 && !state->locomotion.is_moving) {
        /* Keep tentative state clean after reset */
        if (state->pdr_last_step_ts == 0) {
            s_tentative_steps = 0;
        }
    }

    /* Compute 12-sample (480 ms) peak-to-peak dynamic energy gate (Vpp) */
    float win_min_g = mag_g;
    float win_max_g = mag_g;
    uint16_t lookback = (state->window_count < 12U) ? state->window_count : 12U;
    for (uint16_t k = 0; k < lookback; k++) {
        uint16_t idx = (uint16_t)((state->window_head + EDGEAI_IMU_WINDOW_SIZE - 1U - k) % EDGEAI_IMU_WINDOW_SIZE);
        float g_k = state->sample_window[idx].total_g;
        if (g_k > 0.1f) {
            if (g_k < win_min_g) win_min_g = g_k;
            if (g_k > win_max_g) win_max_g = g_k;
        }
    }
    float vpp_g = win_max_g - win_min_g;

    /* Dynamic gravity tracking baseline removes static DC bias in all orientations */
    if (state->pdr_gravity_est < 0.70f || state->pdr_gravity_est > 1.30f) {
        state->pdr_gravity_est = 1.0f;
    } else if (mag_g >= 0.75f && mag_g <= 1.25f) {
        state->pdr_gravity_est = 0.94f * state->pdr_gravity_est + 0.06f * mag_g;
    }

    float dyn_g = mag_g - state->pdr_gravity_est;

    /* Hard Stationary Gate: Desk rest & slow posture tilts have Vpp < 0.14g (140 mg) */
    if (vpp_g < 0.14f || fabsf(dyn_g) < 0.045f) {
        dyn_g = 0.0f;
    }

    state->pdr_acc_smooth = (vpp_g < 0.14f) ? 0.0f : (0.35f * state->pdr_acc_smooth + 0.65f * dyn_g);
    float filt_g = state->pdr_acc_smooth;

    state->locomotion.step_event = false;

    /* Peak-to-Valley Zero-Crossing Step Detector with 2-Step Rhythmic Lock */
    if (vpp_g >= 0.14f) {
        if (state->pdr_seeking_peak) {
            if (filt_g > state->pdr_peak_val) {
                state->pdr_peak_val = filt_g;
            } else if (state->pdr_peak_val >= 0.080f && filt_g <= -0.030f) {
                /* True bipedal step cycle: +80 mg heel-strike peak followed by -30 mg swing valley */
                uint32_t dt_step = (state->pdr_last_step_ts > 0) ? (ts - state->pdr_last_step_ts) : 2500U;

                if (dt_step >= 260U && dt_step <= 1800U) {
                    /* Consecutive rhythmic step within human gait window (33..230 SPM) */
                    if (s_tentative_steps == 0) {
                        s_tentative_steps = 1;
                    } else if (s_tentative_steps == 1) {
                        /* 2nd rhythmic step confirmed! Commit both steps */
                        s_tentative_steps = 2;
                        state->locomotion.step_count += 2;
                        state->locomotion.step_event = true;
                    } else {
                        /* Continuous walking: increment every step immediately */
                        state->locomotion.step_count += 1;
                        state->locomotion.step_event = true;
                    }

                    state->pdr_last_step_ts = ts;
                    state->pdr_last_motion_ts = ts;

                    if (state->locomotion.step_event) {
                        state->locomotion.is_moving = true;
                        float inst_spm = 60000.0f / (float)dt_step;
                        state->locomotion.cadence_spm = (state->locomotion.cadence_spm > 10.0f) ?
                            (0.65f * state->locomotion.cadence_spm + 0.35f * inst_spm) : inst_spm;
                        state->locomotion.step_freq_hz = state->locomotion.cadence_spm / 60.0f;

                        /* Adaptive Weinberg Stride Length Model */
                        float bounce = state->pdr_peak_val - filt_g;
                        if (bounce < 0.12f) bounce = 0.12f;
                        float stride = 0.45f * sqrtf(sqrtf(bounce * 9.81f));
                        if (stride < 0.40f) stride = 0.40f;
                        if (stride > 1.10f) stride = 1.10f;
                        state->locomotion.stride_length_m = stride;

                        uint32_t added = (s_tentative_steps == 2 && state->locomotion.step_count == 2) ? 2U : 1U;
                        state->locomotion.total_distance_m += stride * (float)added;
                        state->locomotion.speed_mps = stride / ((float)dt_step / 1000.0f);
                        state->locomotion.pos_x_m = 0.0f;
                        state->locomotion.pos_y_m = state->locomotion.total_distance_m;
                        state->locomotion.heading_deg = 0.0f;
                    }
                } else if (dt_step > 1800U) {
                    /* First step candidate after rest: arm tentative buffer without false counting */
                    s_tentative_steps = 1;
                    state->pdr_last_step_ts = ts;
                }

                state->pdr_seeking_peak = false;
                state->pdr_valley_val = filt_g;
            }
        } else {
            /* Re-arm peak seeker once acceleration rises back above +0.015g */
            if (filt_g < state->pdr_valley_val) {
                state->pdr_valley_val = filt_g;
            }
            if (filt_g >= 0.015f) {
                state->pdr_seeking_peak = true;
                state->pdr_peak_val = filt_g;
            }
        }
    } else {
        state->pdr_seeking_peak = true;
        state->pdr_peak_val = 0.0f;
        state->pdr_valley_val = 0.0f;
    }

    /* Zero-Velocity Update (ZUPT): immediately reset cadence & tentative steps when stationary */
    if (state->pdr_last_step_ts == 0 || (ts - state->pdr_last_step_ts) > 1600U || vpp_g < 0.10f) {
        if ((ts - state->pdr_last_step_ts) > 1600U) {
            s_tentative_steps = 0;
            state->locomotion.is_moving = false;
            state->locomotion.cadence_spm = 0.0f;
            state->locomotion.speed_mps = 0.0f;
            state->locomotion.step_freq_hz = 0.0f;
        }
    }

    /* -------------------------------------------------------------------------
     * 2. Sample-Level 4-Phase Fall Detection State Machine (25 Hz cadence)
     * ------------------------------------------------------------------------- */
    if (!state->fall_alarm_latched) {
        switch (state->fall_phase) {
            case FALL_PHASE_IDLE:
                /* Phase 1: Free-Fall Unloading Detection (|A| < 0.65g for >= 1 sample) */
                if (mag_g < 0.65f) {
                    state->consecutive_freefall_samples++;
                    if (state->consecutive_freefall_samples >= 1U) {
                        state->fall_phase = FALL_PHASE_FREEFALL_DETECTED;
                        state->t_freefall_end_ms = ts;
                        state->impact_peak_g = 0.0f;
                        state->consecutive_freefall_samples = 0;

                        state->ref_base_x = (float)sample->x_mg;
                        state->ref_base_y = (float)sample->y_mg;
                        state->ref_base_z = (float)sample->z_mg;
                    }
                } else if (mag_g >= 2.05f) {
                    /* Phase 1b: Direct Shock Drop Detection (|A| >= 2.05g) */
                    state->fall_phase = FALL_PHASE_REST_WAIT;
                    state->fall_status = FALL_STATUS_SUSPECTED;
                    state->impact_peak_g = mag_g;
                    state->immobile_sample_count = 0;
                    state->post_impact_sample_count = 1;
                } else {
                    state->consecutive_freefall_samples = 0;
                }
                break;

            case FALL_PHASE_FREEFALL_DETECTED:
                /* Phase 2: Impact Shock Detection (|A| >= 1.65g within 0-900 ms of freefall) */
                {
                    uint32_t dt = ts - state->t_freefall_end_ms;
                    if (mag_g > state->impact_peak_g) {
                        state->impact_peak_g = mag_g;
                    }

                    if (mag_g >= 1.65f && dt <= 900U)
                    {
                        state->fall_phase = FALL_PHASE_REST_WAIT;
                        state->fall_status = FALL_STATUS_SUSPECTED;
                        state->t_impact_ms = ts;
                        state->immobile_sample_count = 0;
                        state->post_impact_sample_count = 1;
                    } else if (dt > 950U) {
                        /* Timeout: no qualifying impact spike observed */
                        state->fall_phase = FALL_PHASE_IDLE;
                        state->fall_status = FALL_STATUS_NONE;
                    }
                }
                break;

            case FALL_PHASE_IMPACT_DETECTED:
                state->fall_phase = FALL_PHASE_REST_WAIT;
                state->immobile_sample_count = 0;
                break;

            case FALL_PHASE_REST_WAIT:
                /* Phase 4: Post-Fall Immobility Confirmation (8 samples = 320 ms) */
                {
                    float g_dev = fabsf(mag_g - 1.0f);
                    if (g_dev < 0.28f) {
                        state->immobile_sample_count++;
                        if (state->immobile_sample_count >= state->config.rest_duration_samples) {
                            /* Confirmed Incapacitated Fall! Latch for 3.0s auto-dismissal */
                            state->fall_phase = FALL_PHASE_CONFIRMED;
                            state->fall_status = FALL_STATUS_CONFIRMED;
                            state->fall_alarm_latched = true;
                            state->t_impact_ms = ts;
                        }
                    } else {
                        /* Movement detected during rest phase: subject is active, not incapacitated */
                        if (state->immobile_sample_count > 0) {
                            state->immobile_sample_count--;
                        }
                        state->post_impact_sample_count++;
                        if (state->post_impact_sample_count >= 30U) {
                            /* Abort: Subject recovered and continued moving */
                            state->fall_phase = FALL_PHASE_IDLE;
                            state->fall_status = FALL_STATUS_RECOVERED;
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
    } else {
        /* Auto-dismiss fall alarm after 3.0 seconds (3000 ms) */
        if ((ts - state->t_impact_ms) >= 3000U) {
            edgeai_imu_clear_fall_alarm(state);
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
    result->locomotion            = state->locomotion;

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
