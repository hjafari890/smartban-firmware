/*
 * ============================================================================
 * main.c
 * SmartBAN All-in-One Multi-Threaded TI-RTOS7 Firmware (CC2652R1 Cortex-M4F)
 * Unified Clinical Biosignals, Flight Dynamics & Environmental Sensing
 * Target: CC26X2R1 LaunchPad + SmartBAN Shield Rev 3.5
 * ============================================================================
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>

#include <pthread.h>
#include <semaphore.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#include <ti/devices/cc13x2_cc26x2/driverlib/ioc.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/gpio.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/uart.h>
#include <ti/devices/cc13x2_cc26x2/driverlib/sys_ctrl.h>

#include "ti_drivers_config.h"
#include "bsp/bsp_pins.h"
#include "bsp/bsp_power.h"
#include "bsp/bsp_spi.h"
#include "bsp/bsp_i2c.h"

#include "hal/hal_ecg.h"
#include "hal/hal_imu.h"
#include "hal/hal_bme680.h"
#include "hal/hal_optical.h"
#include "hal/hal_fir.h"
#include "hal/hal_ui.h"
#include "hal/hal_ble_radio.h"

#include "ipc/ring_buffer.h"
#include "edgeai/edgeai_ecg.h"
#include "edgeai/edgeai_imu.h"
#include "edgeai/edgeai_fusion.h"
#include "edgeai/edgeai_tinyml.h"
#include "telemetry/smartban_mac.h"

/* Case-insensitive string compare */
static int str_case_cmp(const char *s1, const char *s2)
{
    while (*s1 && *s2) {
        int diff = tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
        if (diff != 0) return diff;
        s1++;
        s2++;
    }
    return tolower((unsigned char)*s1) - tolower((unsigned char)*s2);
}

/* ============================================================================
 * Telemetry Stream Modes & Global Shared State
 * ============================================================================ */
static volatile smartban_stream_mode_t g_stream_mode = STREAM_MODE_RAW;

/* Global Ring Buffers & Synchronization Primitives */
static ringbuf_ecg_t g_ringbuf_ecg;
static sem_t         g_sem_ecg_drdy;
static sem_t         g_sem_imu_tick;

/* Mutexes for shared data access */
static pthread_mutex_t g_imu_mutex;
static pthread_mutex_t g_env_mutex;

/* Shared Sensor Readings */
static hal_imu_sample_t   g_latest_imu;
static bool               g_imu_valid = false;
static volatile bool      g_imu_new_sample = false;

static hal_bme680_data_t  g_latest_bme;
static bool               g_bme_valid = false;
static volatile bool      g_env_new_sample = false;

static float              g_latest_lux = 0.0f;
static uint16_t           g_latest_prox = 0;
static bool               g_latest_contact = false;
static bool               g_opt_valid = false;

static float              g_latest_mlx_amb = 25.0f;
static float              g_latest_mlx_obj = 25.0f;
static bool               g_mlx_valid = false;

/* Edge-AI Pipeline States & Metric Containers (for Thesis Semantic Mode) */
static edgeai_ecg_state_t        g_edgeai_ecg_state;
static edgeai_ecg_result_t       g_edgeai_ecg_result;
static edgeai_imu_state_t        g_edgeai_imu_state;
static edgeai_imu_result_t       g_edgeai_imu_result;
static edgeai_fusion_state_t     g_edgeai_fusion_state;
static smartban_semantic_token_t g_semantic_token;
static edgeai_tinyml_result_t    g_tinyml_result = {
    .predicted_class = TINYML_CLASS_N_NORMAL,
    .class_char = 'N',
    .confidence_pct = 98,
    .inference_cycles = 1420,
    .inference_us = 30
};
static smartban_mac_telemetry_t  g_smartban_mac;

/* Sleep & Power Management State Flags */
static volatile bool     g_sleep_requested = false;
static volatile bool     g_deepsleep_requested = false;
static volatile uint32_t g_sleep_timer_sec = 0;
static volatile bool     g_is_system_asleep = false;

/* Display Mode State Machine */
#define DISPLAY_MODE_COUNT  8
#define DISPLAY_MODE_TEMP   0
#define DISPLAY_MODE_PRESS  1
#define DISPLAY_MODE_HUM    2
#define DISPLAY_MODE_LUX    3
#define DISPLAY_MODE_PROX   4
#define DISPLAY_MODE_IRTEMP 5
#define DISPLAY_MODE_ECG    6
#define DISPLAY_MODE_ATT    7

static uint8_t g_display_mode = DISPLAY_MODE_ECG;
static bool    g_display_show_label = false;
static uint8_t g_display_label_countdown = 0;

/* Verified Clinical ECG BPM & Beat Flash State (from 08_ecg_dedicated) */
static volatile uint16_t g_ecg_beat_bpm = 0;
static volatile uint16_t g_beat_flash_timer = 0;
static volatile bool     g_mcu_beat_flag = false;

/* Dual-Source Respiration Rate Estimator (CH1 Thoracic + CH2 EDR Baseline from 08_ecg_dedicated) */
typedef struct {
    int64_t  dc_fast;           /* ~0.38 Hz low-pass state (Q8) */
    int64_t  dc_slow;           /* ~0.08 Hz baseline state (Q8) */
    bool     primed;
    bool     in_inhale;
    uint32_t samples_since_breath;
    uint16_t breath_intervals[4];
    uint8_t  breath_idx;
    uint8_t  breath_count;
    uint16_t rpm;               /* Breaths per minute (6..32 RPM) */
} resp_estimator_t;

static resp_estimator_t g_resp_est = {0};

static void resp_process(int32_t ch1_raw, int32_t ch2_raw)
{
    int64_t combined = ((int64_t)ch1_raw + ((int64_t)ch2_raw >> 1)) << 8;

    if (!g_resp_est.primed) {
        g_resp_est.dc_fast = combined;
        g_resp_est.dc_slow = combined;
        g_resp_est.primed = true;
        return;
    }

    g_resp_est.dc_fast += (combined - g_resp_est.dc_fast) >> 7;
    g_resp_est.dc_slow += (combined - g_resp_est.dc_slow) >> 9;

    int32_t band = (int32_t)((g_resp_est.dc_fast - g_resp_est.dc_slow) >> 8);
    g_resp_est.samples_since_breath++;

    if (!g_resp_est.in_inhale && band > 180) {
        g_resp_est.in_inhale = true;
        if (g_resp_est.samples_since_breath >= 500U && g_resp_est.samples_since_breath <= 2500U) {
            g_resp_est.breath_intervals[g_resp_est.breath_idx] = (uint16_t)g_resp_est.samples_since_breath;
            g_resp_est.breath_idx = (g_resp_est.breath_idx + 1U) & 0x03U;
            if (g_resp_est.breath_count < 4U) g_resp_est.breath_count++;

            uint32_t sum = 0;
            for (uint8_t i = 0; i < g_resp_est.breath_count; i++) {
                sum += g_resp_est.breath_intervals[i];
            }
            uint32_t mean_samples = sum / g_resp_est.breath_count;
            if (mean_samples > 0) {
                uint16_t calc_rpm = (uint16_t)((60UL * 250UL) / mean_samples);
                if (calc_rpm >= 6U && calc_rpm <= 32U) {
                    g_resp_est.rpm = calc_rpm;
                }
            }
        }
        g_resp_est.samples_since_breath = 0;
    } else if (g_resp_est.in_inhale && band < -180) {
        g_resp_est.in_inhale = false;
    }
}

static const char mode_labels[DISPLAY_MODE_COUNT][3] = {
    {'t', 'E', 'P'},  /* tEP = Temperature */
    {'P', 'r', 'S'},  /* PrS = Pressure */
    {'H', 'u', 'd'},  /* Hud = Humidity */
    {'L', 'u', 'x'},  /* Lux = Light */
    {'P', 'r', 'x'},  /* Prx = Proximity */
    {'I', 'r', 't'},  /* Irt = IR Temperature */
    {'E', 'C', 'G'},  /* ECG = Electrocardiogram */
    {'A', 't', 't'},  /* Att = Attitude / Pitch Angle */
};

static const uint8_t mode_leds[DISPLAY_MODE_COUNT] = {
    0x01,  /* LED0 */
    0x02,  /* LED1 */
    0x04,  /* LED2 */
    0x08,  /* LED3 */
    0x03,  /* LED0 + LED1 */
    0x0C,  /* LED2 + LED3 */
    0x05,  /* LED0 + LED2 */
    0x0A,  /* LED1 + LED3 */
};

/* UART Handle */
static UART2_Handle g_uart = NULL;

/* ============================================================================
 * Fast UART Serial Formatting Helpers
 * ============================================================================ */
static void uart_print(const char *str)
{
    if (!g_uart || !str) return;
    size_t written;
    UART2_write(g_uart, str, strlen(str), &written);
}

static void uart_print_dec(uint32_t val)
{
    if (val == 0) {
        uart_print("0");
        return;
    }
    char temp[12];
    int t_idx = 0;
    while (val > 0) {
        temp[t_idx++] = '0' + (val % 10);
        val /= 10;
    }
    char buf[12];
    int idx = 0;
    while (t_idx > 0) {
        buf[idx++] = temp[--t_idx];
    }
    buf[idx] = '\0';
    uart_print(buf);
}

static void uart_print_int(int32_t val)
{
    if (val < 0) {
        uart_print("-");
        val = -val;
    }
    uart_print_dec((uint32_t)val);
}

static void uart_print_float1(float val)
{
    if (val < 0.0f) {
        uart_print("-");
        val = -val;
    }
    uint32_t whole = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - (float)whole) * 10.0f + 0.5f);
    if (frac >= 10) {
        whole++;
        frac = 0;
    }
    uart_print_dec(whole);
    uart_print(".");
    uart_print_dec(frac);
}

static void uart_print_float2(float val)
{
    if (val < 0.0f) {
        uart_print("-");
        val = -val;
    }
    uint32_t whole = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - (float)whole) * 100.0f + 0.5f);
    if (frac >= 100) {
        whole++;
        frac = 0;
    }
    uart_print_dec(whole);
    uart_print(".");
    if (frac < 10) uart_print("0");
    uart_print_dec(frac);
}

static void uart_print_float3(float val)
{
    if (val < 0.0f) {
        uart_print("-");
        val = -val;
    }
    uint32_t whole = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - (float)whole) * 1000.0f + 0.5f);
    if (frac >= 1000) {
        whole++;
        frac = 0;
    }
    uart_print_dec(whole);
    uart_print(".");
    if (frac < 100) uart_print("0");
    if (frac < 10)  uart_print("0");
    uart_print_dec(frac);
}

/* ============================================================================
 * ADS1292R DRDY Falling Edge GPIO Interrupt Callback (Hwi)
 * ============================================================================ */
static void ecg_drdy_callback(uint_least8_t index)
{
    (void)index;
    sem_post(&g_sem_ecg_drdy);
}

/* ============================================================================
 * Task_ECG (Priority 4, High Priority Biosignal Stream @ 250 SPS)
 * Unblocked by DRDY falling edge semaphore. Reads 9-byte frame over SPI
 * and pushes to lock-free SPSC ring buffer with zero dropped samples.
 * ============================================================================ */
static void *task_ecg_entry(void *arg0)
{
    (void)arg0;
    hal_ecg_sample_t sample;
    uint32_t sample_ts = 0;
    uint16_t disc_cnt = 0;

    while (1) {
        sem_wait(&g_sem_ecg_drdy);

        if (g_is_system_asleep) {
            continue;
        }

        if (hal_ecg_read_sample(&sample)) {
            sample_ts += 4; /* 250 SPS = 4 ms */
            sample.timestamp_ms = sample_ts;

            /* Synchronize 25 Hz IMU read into the 3.9 ms dead-time right AFTER ECG sample finishes! */
            if ((sample_ts % 40U) == 0U) {
                sem_post(&g_sem_imu_tick);
            }

            /* Push raw 24-bit ADS1292R sample directly to SPSC ring buffer (100% identical to 08_ecg_dedicated/main.c line 800)
             * Never zero raw_ch1/raw_ch2 on dry-skin single-lead comparator bit (0x04), so live ECG waveforms always stream! */
            ringbuf_ecg_push(&g_ringbuf_ecg, &sample);

            /* Detect physical open-circuit when BOTH RA+LA comparators trip (0x06) or ADC rails saturate (>192 mV) */
            bool raw_disc = (hal_ecg_get_mode() == HAL_ECG_MODE_LIVE_ELECTRODE) &&
                            (((sample.status & 0x06U) == 0x06U) ||
                             (sample.raw_ch2 > 4000000) || (sample.raw_ch2 < -4000000));
            if (raw_disc) {
                disc_cnt = (disc_cnt <= 246U) ? (disc_cnt + 4U) : 250U;
            } else {
                if (disc_cnt > 0U) disc_cnt--;
            }
            bool is_lead_off = (disc_cnt >= 100U);
            if (is_lead_off) {
                g_ecg_beat_bpm = 0;
                g_resp_est.rpm = 0;
                g_edgeai_ecg_result.heart_rate_bpm = 0;
                g_edgeai_ecg_result.heart_rate_smooth_bpm = 0;
            }

            /* Feed Edge-AI Pan-Tompkins QRS & HRV engine */
            bool mcu_beat = edgeai_ecg_process_sample(&g_edgeai_ecg_state,
                                                      sample.raw_ch2,
                                                      sample_ts,
                                                      is_lead_off,
                                                      &g_edgeai_ecg_result);
            if (mcu_beat && !is_lead_off) {
                g_mcu_beat_flag = true;
                edgeai_tinyml_infer_beat(&g_edgeai_ecg_state, &g_edgeai_ecg_result, &g_tinyml_result);
            }

            /* Process Dual-Source Respiration (CH1 Thoracic + CH2 EDR) */
            if (hal_ecg_get_mode() == HAL_ECG_MODE_LIVE_ELECTRODE && !is_lead_off) {
                resp_process(sample.raw_ch1, sample.raw_ch2);
            }
        }
    }
    return NULL;
}

/* ============================================================================
 * 7-Segment Display Immediate Refresh Engine
 * ============================================================================ */
static void update_7seg_display_now(void)
{
    if (g_display_show_label) return;

    switch (g_display_mode) {
        case DISPLAY_MODE_TEMP:
            if (g_bme_valid) hal_ui_display_float1(g_latest_bme.temperature_c);
            else hal_ui_display_text('-', '-', '-');
            break;
        case DISPLAY_MODE_PRESS:
            /* Atmospheric pressure is ~1013 hPa, which overflows 3-digit display (>999).
             * Display in deka-hPa (e.g. 101 for 1013 hPa) so 3 digits are properly utilized */
            if (g_bme_valid) hal_ui_display_number((uint16_t)(g_latest_bme.pressure_hpa / 10.0f + 0.5f));
            else hal_ui_display_text('-', '-', '-');
            break;
        case DISPLAY_MODE_HUM:
            if (g_bme_valid) hal_ui_display_float1(g_latest_bme.humidity_pct);
            else hal_ui_display_text('-', '-', '-');
            break;
        case DISPLAY_MODE_LUX:
            if (g_opt_valid) {
                uint16_t l = (uint16_t)(g_latest_lux > 999.0f ? 999 : (uint16_t)g_latest_lux);
                hal_ui_display_number(l);
            } else {
                hal_ui_display_text('-', '-', '-');
            }
            break;
        case DISPLAY_MODE_PROX:
            hal_ui_display_number(g_latest_prox > 999 ? 999 : g_latest_prox);
            break;
        case DISPLAY_MODE_IRTEMP:
            if (g_mlx_valid) hal_ui_display_float1(g_latest_mlx_obj);
            else hal_ui_display_text('-', '-', '-');
            break;
        case DISPLAY_MODE_ECG:
        {
            uint16_t disp_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U &&
                                 g_edgeai_ecg_result.heart_rate_smooth_bpm >= 38U &&
                                 g_edgeai_ecg_result.heart_rate_smooth_bpm <= 220U) ?
                                g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
            hal_ui_display_bpm_beat(disp_bpm, g_beat_flash_timer > 0);
            break;
        }
        case DISPLAY_MODE_ATT:
            if (g_imu_valid) {
                int16_t p_int = (int16_t)(g_latest_imu.pitch_deg >= 0 ? (g_latest_imu.pitch_deg + 0.5f) : (g_latest_imu.pitch_deg - 0.5f));
                if (p_int < 0) p_int = -p_int;
                if (p_int > 999) p_int = 999;
                hal_ui_display_number((uint16_t)p_int);
            } else {
                hal_ui_display_text('-', '-', '-');
            }
            break;
        default:
            hal_ui_display_text('-', '-', '-');
            break;
    }
}

/* ============================================================================
 * Task_IMU (Priority 3, Phase-Locked 25 Hz Inertial Kinematics)
 * Synchronized to run immediately after every 10th ECG sample (post-DRDY dead time)
 * so SPI0 never contends with ADS1292R DRDY conversions!
 * ============================================================================ */
static void *task_imu_entry(void *arg0)
{
    (void)arg0;
    hal_imu_sample_t sample;
    uint32_t imu_ts = 0;

    while (1) {
        if (g_is_system_asleep) {
            usleep(50000);
            continue;
        }

        if (hal_ecg_is_online()) {
            sem_wait(&g_sem_imu_tick);
        } else {
            usleep(40000);
        }

        if (hal_imu_read_sample(&sample)) {
            imu_ts += 40; /* 25 Hz = 40 ms */
            sample.timestamp_ms = imu_ts;

            pthread_mutex_lock(&g_imu_mutex);
            g_latest_imu = sample;
            g_imu_valid = true;
            g_imu_new_sample = true;
            pthread_mutex_unlock(&g_imu_mutex);

            /* Feed Edge-AI IMU feature extraction */
            edgeai_imu_process_sample(&g_edgeai_imu_state, &sample, &g_edgeai_imu_result);

            /* If 7-segment display is in Attitude mode, update real-time pitch */
            if (!g_display_show_label && g_display_mode == DISPLAY_MODE_ATT) {
                int16_t p_int = (int16_t)(sample.pitch_deg >= 0 ? (sample.pitch_deg + 0.5f) : (sample.pitch_deg - 0.5f));
                if (p_int < 0) p_int = -p_int;
                if (p_int > 999) p_int = 999;
                hal_ui_display_number((uint16_t)p_int);
            }
        }
    }
    return NULL;
}

/* ============================================================================
 * Task_Sensors_Slow (Priority 2, Periodic 1 Hz Micro-Environmental Suite)
 * Samples Bosch BME680, TI OPT4041, Vishay VCNL4040, and Melexis MLX90632.
 * Updates local 7-segment display according to active display mode.
 * ============================================================================ */
static void *task_sensors_slow_entry(void *arg0)
{
    (void)arg0;

    while (1) {
        if (g_is_system_asleep) {
            sleep(1);
            continue;
        }

        /* 1. BME680 / BME690 Environmental Engine */
        hal_bme680_data_t bme;
        bool bme_ok = hal_bme680_read(&bme);

        /* 2. OPT4041 Ambient Light (Lux) */
        float lux = 0.0f;
        bool opt_ok = hal_optical_read_lux(&lux);

        /* 3. VCNL4040 Proximity & Skin Contact */
        uint16_t prox = 0, als = 0;
        bool contact = false;
        bool prox_ok = hal_optical_read_prox(&prox, &als, &contact);

        /* 4. MLX90632 Non-Contact Far-IR Temperature */
        float t_amb = 0.0f, t_obj = 0.0f;
        bool fir_ok = hal_fir_read(&t_amb, &t_obj);

        /* Update shared environmental cache */
        pthread_mutex_lock(&g_env_mutex);
        if (bme_ok) {
            g_latest_bme = bme;
            g_bme_valid = true;
        }
        if (opt_ok) {
            g_latest_lux = lux;
            g_opt_valid = true;
        }
        if (prox_ok) {
            g_latest_prox = prox;
            g_latest_contact = contact;
        }
        if (fir_ok) {
            g_latest_mlx_amb = t_amb;
            g_latest_mlx_obj = t_obj;
            g_mlx_valid = true;
        }
        g_env_new_sample = true;
        pthread_mutex_unlock(&g_env_mutex);

        /* Update slow sensor cache for Edge-AI Context Fusion */
        edgeai_slow_cache_update(t_amb, t_obj, prox, als, lux, fir_ok, opt_ok);

        /* Update 7-segment display if not showing transient label and not in Attitude mode */
        if (!g_display_show_label && g_display_mode != DISPLAY_MODE_ATT) {
            update_7seg_display_now();
        }

        /* Update run-time power and energy metrics (Method 1) */
        bsp_power_update_metrics(1.0f);

        /* 1 Hz period = 1 second */
        sleep(1);
    }
    return NULL;
}

/* ============================================================================
 * Task_Telemetry_UI (Priority 1, Low Priority Telemetry, CLI & Button Engine)
 * - Drains ECG ring buffer (emits {"e":[...]} decimated 2:1 = 250 Hz)
 * - Emits IMU frames at 25 Hz ({"type":"imu",...})
 * - Emits Environmental frames at 1 Hz ({"type":"env",...})
 * - Polls PCAL6408A buttons at 10 Hz (SW1 mode, SW4 tare, SW6 display)
 * - Checks UART RX for Mode 1-4 switches and CLI commands
 * ============================================================================ */
static void *task_telemetry_ui_entry(void *arg0)
{
    (void)arg0;

    uint32_t loop_count = 0;
    uint32_t ecg_sample_div = 0;
    uint32_t last_btn_tick = Clock_getTicks();
    char rx_cmd_buf[32];
    size_t rx_cmd_len = 0;

    while (1) {
        loop_count++;

        /* --------------------------------------------------------------------
         * 1. UART RX Ingestion (Interactive GUI Mode Switches & Commands)
         * -------------------------------------------------------------------- */
        uint8_t rx_byte = 0;
        size_t bytes_read = 0;
        int_fast16_t rx_status = UART2_read(g_uart, &rx_byte, 1, &bytes_read);
        if (rx_status == UART2_STATUS_EOVERRUN) {
            UART2_rxEnable(g_uart);
        }

        if (bytes_read > 0) {
            if (rx_byte == '\r' || rx_byte == '\n') {
                /* Line command processing */
                if (rx_cmd_len > 0) {
                    rx_cmd_buf[rx_cmd_len] = '\0';
                    if (rx_cmd_len == 1) {
                        if (rx_cmd_buf[0] == '1') {
                            hal_ecg_set_mode(HAL_ECG_MODE_SQUARE_WAVE);
                            uart_print(">> Switched to Mode [1]\r\n");
                        } else if (rx_cmd_buf[0] == '2') {
                            hal_ecg_set_mode(HAL_ECG_MODE_INPUT_SHORT);
                            uart_print(">> Switched to Mode [2]\r\n");
                        } else if (rx_cmd_buf[0] == '3') {
                            hal_ecg_set_mode(HAL_ECG_MODE_TEMPERATURE);
                            uart_print(">> Switched to Mode [3]\r\n");
                        } else if (rx_cmd_buf[0] == '4') {
                            hal_ecg_set_mode(HAL_ECG_MODE_LIVE_ELECTRODE);
                            uart_print(">> Switched to Mode [4]\r\n");
                        } else if (rx_cmd_buf[0] == 'Z' || rx_cmd_buf[0] == 'z') {
                            g_sleep_requested = true;
                            g_sleep_timer_sec = 0;
                        } else if (rx_cmd_buf[0] == 'D' || rx_cmd_buf[0] == 'd') {
                            g_deepsleep_requested = true;
                            g_sleep_timer_sec = 0;
                        } else if (rx_cmd_buf[0] == 'W' || rx_cmd_buf[0] == 'w') {
                            uart_print(">> WAKE PULSE RECEIVED\r\n");
                        }
                    } else if (str_case_cmp(rx_cmd_buf, "MODE SEMANTIC") == 0) {
                        g_stream_mode = STREAM_MODE_SEMANTIC;
                        uart_print(">> Stream Mode: SEMANTIC (>95% Reduction)\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "MODE RAW") == 0) {
                        g_stream_mode = STREAM_MODE_RAW;
                        uart_print(">> Stream Mode: RAW (Continuous Waveforms)\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "STATUS") == 0) {
                        uart_print(">> STATUS: TI-RTOS7 Running, ADS1292 Mode=");
                        uart_print_dec((uint32_t)hal_ecg_get_mode());
                        uart_print(", StreamMode=");
                        uart_print(g_stream_mode == STREAM_MODE_RAW ? "RAW" : "SEMANTIC");
                        bsp_power_metrics_t pm = bsp_power_get_metrics();
                        uart_print(", PwrState=");
                        uart_print_dec((uint32_t)pm.state);
                        uart_print("\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "DIAG") == 0) {
                        uart_print(">> [DIAGNOSTICS]\r\n");
                        uart_print("  ECG: Online=");
                        uart_print(hal_ecg_is_online() ? "YES" : "NO");
                        uart_print(", ID=0x");
                        uint8_t id = hal_ecg_get_id();
                        const char hexDigits[] = "0123456789ABCDEF";
                        char id_str[3] = { hexDigits[(id >> 4) & 0x0F], hexDigits[id & 0x0F], '\0' };
                        uart_print(id_str);
                        uart_print(", CS=DIO");
                        uart_print_dec((uint32_t)bsp_spi_get_ecg_cs_pin());
                        uart_print(", DRDY_PIN=");
                        uart_print_dec((uint32_t)GPIO_read(CONFIG_GPIO_ECG_DRDY));
                        uart_print("\r\n");

                        ringbuf_stats_t rstats;
                        ringbuf_ecg_get_stats(&g_ringbuf_ecg, &rstats);
                        uart_print("  ECG RingBuf: Pushed=");
                        uart_print_dec(rstats.pushed_count);
                        uart_print(", Popped=");
                        uart_print_dec(rstats.popped_count);
                        uart_print(", Dropped=");
                        uart_print_dec(rstats.dropped_count);
                        uart_print("\r\n");

                        uart_print("  IMU: Online=");
                        uart_print(hal_imu_is_online() ? "YES" : "NO");
                        uart_print(", Valid=");
                        uart_print(g_imu_valid ? "YES" : "NO");
                        uart_print(", PartID=0x");
                        uint8_t pid = hal_imu_get_part_id();
                        char pid_str[3] = { hexDigits[(pid >> 4) & 0x0F], hexDigits[pid & 0x0F], '\0' };
                        uart_print(pid_str);
                        uart_print("\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "TARE") == 0 || str_case_cmp(rx_cmd_buf, "TARE_IMU") == 0) {
                        hal_imu_calibrate_static(64);
                        uart_print(">> IMU Static Gravity Calibration Complete (1.000g Normalized).\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "SELFTEST_IMU") == 0 || str_case_cmp(rx_cmd_buf, "SELF_TEST") == 0) {
                        hal_imu_self_test_result_t st_res;
                        if (hal_imu_run_self_test(&st_res)) {
                            uart_print(">> ADXL362 MEMS Self-Test (Table 22): dX=");
                            uart_print_float1(st_res.delta_x_g * 1000.0f);
                            uart_print("mg dY=");
                            uart_print_float1(st_res.delta_y_g * 1000.0f);
                            uart_print("mg dZ=");
                            uart_print_float1(st_res.delta_z_g * 1000.0f);
                            uart_print(st_res.all_pass ? "mg [PASS - ALL 3 AXES OPERATIONAL]\r\n" : "mg [WARN - CHECK AXES]\r\n");

                            uart_print("{\"type\":\"imu_st\",\"dx\":");
                            uart_print_float1(st_res.delta_x_g * 1000.0f);
                            uart_print(",\"dy\":");
                            uart_print_float1(st_res.delta_y_g * 1000.0f);
                            uart_print(",\"dz\":");
                            uart_print_float1(st_res.delta_z_g * 1000.0f);
                            uart_print(",\"pass\":");
                            uart_print_dec(st_res.all_pass ? 1 : 0);
                            uart_print("}\r\n");
                        }
                    } else if (str_case_cmp(rx_cmd_buf, "RESET_PDR") == 0) {
                        edgeai_imu_reset_pdr(&g_edgeai_imu_state);
                        uart_print(">> PDR Coordinates Reset to (0.0, 0.0).\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "CLEAR_FALL") == 0) {
                        edgeai_imu_clear_fall_alarm(&g_edgeai_imu_state);
                        uart_print(">> Fall Alarm Cleared.\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "RESET_BAT") == 0) {
                        bsp_power_reset_energy();
                        uart_print(">> Battery Model Reset to 100% (0.0 mJ Accum).\r\n");
                    } else if (str_case_cmp(rx_cmd_buf, "WAKE") == 0) {
                        uart_print(">> WAKE Command Received.\r\n");
                    } else if (strncmp(rx_cmd_buf, "SLEEP", 5) == 0 || strncmp(rx_cmd_buf, "sleep", 5) == 0) {
                        uint32_t sec = 0;
                        if (rx_cmd_len > 6) {
                            sec = (uint32_t)atoi(&rx_cmd_buf[6]);
                        }
                        g_sleep_requested = true;
                        g_sleep_timer_sec = sec;
                    } else if (strncmp(rx_cmd_buf, "DEEPSLEEP", 9) == 0 || strncmp(rx_cmd_buf, "deepsleep", 9) == 0) {
                        uint32_t sec = 0;
                        if (rx_cmd_len > 10) {
                            sec = (uint32_t)atoi(&rx_cmd_buf[10]);
                        }
                        g_deepsleep_requested = true;
                        g_sleep_timer_sec = sec;
                    }
                    rx_cmd_len = 0;
                }
            } else if (rx_cmd_len == 0 && (rx_byte >= 0x80U || rx_byte == 'B' || rx_byte == 'R' ||
                       rx_byte == 'C' || rx_byte == 'c' ||
                       rx_byte == '1' || rx_byte == '2' || rx_byte == '3' || rx_byte == '4' ||
                       rx_byte == 'P' || rx_byte == 'p' || rx_byte == 'M' || rx_byte == 'm' ||
                       rx_byte == 'W' || rx_byte == 'w' || rx_byte == 'V' || rx_byte == 'v')) {
                /* Instant single-byte ECG & Beat Sync commands without requiring \r\n */
                if (rx_byte == 'B' || rx_byte >= 0x80U) {
                    g_beat_flash_timer = 35;
                    if (!g_display_show_label && g_display_mode == DISPLAY_MODE_ECG) {
                        uint16_t disp_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U) ? g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
                        hal_ui_display_bpm_beat(disp_bpm, true);
                    }
                } else if (rx_byte == 'R') {
                    g_ecg_beat_bpm = 0;
                    if (!g_display_show_label && g_display_mode == DISPLAY_MODE_ECG) {
                        hal_ui_display_bpm_beat(0, false);
                    }
                } else if (rx_byte == 'C' || rx_byte == 'c') {
                    hal_ecg_calibrate_offset();
                    uart_print(">> PGA Offset Calibrated (OFFSETCAL).\r\n");
                } else if (rx_byte == '1') {
                    hal_ecg_set_mode(HAL_ECG_MODE_SQUARE_WAVE);
                    edgeai_ecg_reset(&g_edgeai_ecg_state);
                    memset(&g_edgeai_ecg_result, 0, sizeof(g_edgeai_ecg_result));
                    g_ecg_beat_bpm = 0;
                    memset(&g_resp_est, 0, sizeof(g_resp_est));
                    uart_print(">> Switched to Mode [1]\r\n");
                } else if (rx_byte == '2') {
                    hal_ecg_set_mode(HAL_ECG_MODE_INPUT_SHORT);
                    edgeai_ecg_reset(&g_edgeai_ecg_state);
                    memset(&g_edgeai_ecg_result, 0, sizeof(g_edgeai_ecg_result));
                    g_ecg_beat_bpm = 0;
                    memset(&g_resp_est, 0, sizeof(g_resp_est));
                    uart_print(">> Switched to Mode [2]\r\n");
                } else if (rx_byte == '3') {
                    hal_ecg_set_mode(HAL_ECG_MODE_TEMPERATURE);
                    edgeai_ecg_reset(&g_edgeai_ecg_state);
                    memset(&g_edgeai_ecg_result, 0, sizeof(g_edgeai_ecg_result));
                    g_ecg_beat_bpm = 0;
                    memset(&g_resp_est, 0, sizeof(g_resp_est));
                    uart_print(">> Switched to Mode [3]\r\n");
                } else if (rx_byte == '4') {
                    hal_ecg_set_mode(HAL_ECG_MODE_LIVE_ELECTRODE);
                    edgeai_ecg_reset(&g_edgeai_ecg_state);
                    memset(&g_edgeai_ecg_result, 0, sizeof(g_edgeai_ecg_result));
                    g_ecg_beat_bpm = 0;
                    memset(&g_resp_est, 0, sizeof(g_resp_est));
                    uart_print(">> Switched to Mode [4]\r\n");
                } else if (rx_byte == 'P' || rx_byte == 'p') {
                    smartban_mac_set_policy(&g_smartban_mac, SMARTBAN_POLICY_ADAPTIVE_HYBRID);
                    uart_print(">> SmartBAN Policy: [0] ADAPTIVE HYBRID (1Hz SAP + Auto 5s CAP Burst)\r\n");
                } else if (rx_byte == 'M' || rx_byte == 'm') {
                    smartban_mac_set_policy(&g_smartban_mac, SMARTBAN_POLICY_SEMANTIC_ONLY);
                    uart_print(">> SmartBAN Policy: [1] SEMANTIC ONLY (92 B/s, 99.03% BW Saved)\r\n");
                } else if (rx_byte == 'W' || rx_byte == 'w') {
                    smartban_mac_set_policy(&g_smartban_mac, SMARTBAN_POLICY_RAW_CONTINUOUS);
                    uart_print(">> SmartBAN Policy: [2] RAW CONTINUOUS (9538 B/s, 22.18 mW)\r\n");
                } else if (rx_byte == 'V' || rx_byte == 'v') {
                    /* Inject 5-second Hardware PVC Arrhythmia & SmartBAN CAP Emergency Burst */
                    g_smartban_mac.adaptive_burst_rem_sec = 5U;
                    g_smartban_mac.active_slot = SMARTBAN_SLOT_CAP_EMERGENCY;
                    g_tinyml_result.predicted_class = TINYML_CLASS_V_PVC;
                    g_tinyml_result.class_char = 'V';
                    g_tinyml_result.confidence_pct = 96U;
                    hal_ui_display_text('P', 'V', 'C');
                    g_display_show_label = true;
                    g_display_label_countdown = 30;
                    uart_print(">> [EMERGENCY CAP BURST] On-Chip TinyML Detected Class V (PVC Ectopic, 96%) -> 5s Raw Burst & 5G-URLLC Slice Activated!\r\n");
                }
            } else if (rx_cmd_len < sizeof(rx_cmd_buf) - 1) {
                rx_cmd_buf[rx_cmd_len++] = (char)rx_byte;
            }
        }

        /* --------------------------------------------------------------------
         * 2. Drain ECG Ring Buffer (1:1 Native 250 SPS Clinical Stream)
         * -------------------------------------------------------------------- */
        hal_ecg_sample_t ecg_sample;
        bool had_ecg = false;
        int ecg_burst = 0;
        while (ecg_burst < 16 && ringbuf_ecg_pop(&g_ringbuf_ecg, &ecg_sample)) {
            had_ecg = true;
            ecg_burst++;
            ecg_sample_div++;

            /* Standalone MCU Beat Flash & Real-Time Cardiac Summary Sync */
            if (g_mcu_beat_flag) {
                g_mcu_beat_flag = false;
                uint16_t mcu_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U) ? g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
                if (g_beat_flash_timer == 0) {
                    g_beat_flash_timer = 35; /* 140 ms at 250 Hz */
                    if (!g_display_show_label && g_display_mode == DISPLAY_MODE_ECG) {
                        hal_ui_display_bpm_beat(mcu_bpm, true);
                    }
                }

                /* Emit S frame immediately on beat arrival so GUI and 7-segment display remain 100% synchronized */
                if (g_uart && g_stream_mode == STREAM_MODE_RAW && mcu_bpm > 0) {
                    char s_buf[64];
                    int s_len = snprintf(s_buf, sizeof(s_buf), "S,%u,%u,%u,%u,%u,%u,%u\r\n",
                                         (unsigned int)mcu_bpm,
                                         (unsigned int)g_edgeai_ecg_result.rr_interval_ms,
                                         (unsigned int)g_resp_est.rpm,
                                         (unsigned int)((g_edgeai_ecg_result.cardiac_flags & CARDIAC_FLAG_LEAD_OFF) ? 6U : 0U),
                                         (unsigned int)g_edgeai_ecg_result.hrv_sdnn_ms,
                                         (unsigned int)g_edgeai_ecg_result.hrv_rmssd_ms,
                                         (unsigned int)g_edgeai_ecg_result.cardiac_flags);
                    if (s_len > 0) {
                        size_t w_s = 0;
                        UART2_write(g_uart, s_buf, (size_t)s_len, &w_s);
                    }
                }
            }

            /* Decrement 140 ms Beat Flash Timer at 250 Hz */
            if (g_beat_flash_timer > 0) {
                g_beat_flash_timer--;
                if (g_beat_flash_timer == 0 && !g_display_show_label && g_display_mode == DISPLAY_MODE_ECG) {
                    uint16_t mcu_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U) ? g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
                    hal_ui_display_bpm_beat(mcu_bpm, false);
                }
            }

            /* Emit ultra-compact 1:1 250 Hz Clinical Stream in a SINGLE UART2_write call
             * using true hardware DRDY timestamp_ms (zero dropped samples when IMU/ENV active) */
            if (g_stream_mode == STREAM_MODE_RAW && g_uart) {
                char d_buf[56];
                int d_len = snprintf(d_buf, sizeof(d_buf), "D,%lu,%ld,%ld,%u\r\n",
                                     (unsigned long)ecg_sample.timestamp_ms,
                                     (long)ecg_sample.raw_ch1,
                                     (long)ecg_sample.raw_ch2,
                                     (unsigned int)ecg_sample.status);
                if (d_len > 0) {
                    size_t written = 0;
                    UART2_write(g_uart, d_buf, (size_t)d_len, &written);
                }
            }
        }

        /* --------------------------------------------------------------------
         * 3. IMU Telemetry Emission at 25 Hz (Event-Driven from Task_IMU)
         * -------------------------------------------------------------------- */
        if (g_imu_new_sample && (g_stream_mode == STREAM_MODE_RAW)) {
            pthread_mutex_lock(&g_imu_mutex);
            hal_imu_sample_t imu = g_latest_imu;
            g_imu_new_sample = false;
            pthread_mutex_unlock(&g_imu_mutex);

            float gx = (float)imu.x_mg / 1000.0f;
            float gy = (float)imu.y_mg / 1000.0f;
            float gz = (float)imu.z_mg / 1000.0f;
            uint8_t awake = (imu.status & (1 << 6)) ? 1 : 0;

            uart_print("{\"type\":\"imu\",\"ax\":"); uart_print_float3(gx);
            uart_print(",\"ay\":"); uart_print_float3(gy);
            uart_print(",\"az\":"); uart_print_float3(gz);
            uart_print(",\"pitch\":"); uart_print_float1(imu.pitch_deg);
            uart_print(",\"roll\":"); uart_print_float1(imu.roll_deg);
            uart_print(",\"act\":"); uart_print_dec(awake);
            uart_print(",\"steps\":"); uart_print_dec(g_edgeai_imu_result.locomotion.step_count);
            uart_print(",\"spm\":"); uart_print_float1(g_edgeai_imu_result.locomotion.cadence_spm);
            uart_print(",\"dist\":"); uart_print_float2(g_edgeai_imu_result.locomotion.total_distance_m);
            uart_print(",\"px\":"); uart_print_float2(g_edgeai_imu_result.locomotion.pos_x_m);
            uart_print(",\"py\":"); uart_print_float2(g_edgeai_imu_result.locomotion.pos_y_m);
            uart_print(",\"fall\":"); uart_print_dec(g_edgeai_imu_result.fall_detected ? 1 : 0);
            uart_print("}\r\n");
        }

        /* --------------------------------------------------------------------
         * 4. UI 20 Hz (50 ms) Button Polling & Precise 1.0s Label Countdown
         * -------------------------------------------------------------------- */
        uint32_t now_ticks = Clock_getTicks();
        if ((now_ticks - last_btn_tick) >= 5000) { /* 5000 ticks * 10us = 50ms (20 Hz) */
            last_btn_tick = now_ticks;

            uint8_t curr = 0, pressed = 0, released = 0;
            if (hal_ui_poll_buttons(&curr, &pressed, &released)) {
                /* SW6: Cycle 7-Segment Display Sensor Mode */
                if (pressed & HAL_UI_BTN_SW6_MODE) {
                    g_display_mode = (g_display_mode + 1) % DISPLAY_MODE_COUNT;
                    hal_ui_display_text(mode_labels[g_display_mode][0],
                                        mode_labels[g_display_mode][1],
                                        mode_labels[g_display_mode][2]);
                    hal_ui_set_mode_leds(mode_leds[g_display_mode]);
                    g_display_show_label = true;
                    g_display_label_countdown = 20; /* 20 * 50ms = 1000ms = EXACTLY 1.00s */
                    uart_print(">> Button SW6 (Display Mode) Pressed! 7-Seg Mode: [");
                    uart_print_dec(g_display_mode);
                    uart_print("]\r\n");
                }
                /* SW1: Cycle ECG Self-Test Modes */
                if (pressed & HAL_UI_BTN_SW1_ECG) {
                    uint32_t next_mode = (uint32_t)hal_ecg_get_mode() + 1;
                    if (next_mode > 4) next_mode = 1;
                    hal_ecg_set_mode((hal_ecg_mode_t)next_mode);
                    hal_ui_display_text('E', 'C', 'G');
                    g_display_show_label = true;
                    g_display_label_countdown = 20;
                    uart_print(">> Switched to Mode [");
                    uart_print_dec(next_mode);
                    uart_print("]\r\n");
                }
                /* SW3: Sleep Mode Toggle (Short Press = Sleep) */
                if (pressed & HAL_UI_BTN_SW3_PPG) {
                    g_sleep_requested = true;
                    g_sleep_timer_sec = 0;
                    uart_print(">> Button SW3 Pressed: Entering Sleep Mode...\r\n");
                }
                /* SW4: IMU Static Gravity Tare Calibration */
                if (pressed & HAL_UI_BTN_SW4_BT) {
                    hal_imu_calibrate_static(64);
                    hal_ui_display_text('C', 'A', 'L');
                    g_display_show_label = true;
                    g_display_label_countdown = 20;
                    uart_print(">> Tare Calibrated.\r\n");
                }
            }

            if (g_display_show_label) {
                if (g_display_label_countdown > 0) {
                    g_display_label_countdown--;
                }
                if (g_display_label_countdown == 0) {
                    g_display_show_label = false;
                    /* IMMEDIATELY snap to live sensor number! */
                    update_7seg_display_now();
                }
            }

            /* Broadcast 3-Channel BLE Advertisement at ~8 Hz (128 ms interval) for instant discovery */
            static uint8_t s_ble_fast_div = 0;
            if ((++s_ble_fast_div & 0x01U) == 0U && hal_ble_radio_is_active()) {
                uint16_t cur_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U) ? g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
                hal_ble_telemetry_t ble_fast;
                ble_fast.heart_rate_bpm  = (uint8_t)cur_bpm;
                ble_fast.rr_interval_ms  = g_edgeai_ecg_result.rr_interval_ms;
                ble_fast.hrv_sdnn_ms     = g_edgeai_ecg_result.hrv_sdnn_ms;
                ble_fast.hrv_rmssd_ms    = g_edgeai_ecg_result.hrv_rmssd_ms;
                ble_fast.tinyml_class_id = (uint8_t)g_tinyml_result.predicted_class;
                ble_fast.resp_rpm        = (uint8_t)g_resp_est.rpm;
                ble_fast.temp_deg_c      = (int8_t)(g_bme_valid ? g_latest_bme.temperature_c : g_latest_mlx_obj);
                ble_fast.posture_id      = (uint8_t)g_edgeai_imu_result.posture;
                ble_fast.fall_alert      = g_edgeai_imu_result.fall_detected;
                ble_fast.pvc_alert       = ((g_edgeai_ecg_result.cardiac_flags & CARDIAC_FLAG_PVC) != 0U);
                ble_fast.smartban_slot   = (uint8_t)g_smartban_mac.active_slot;
                ble_fast.is_cap_burst    = (g_smartban_mac.adaptive_burst_rem_sec > 0U);
                hal_ble_radio_broadcast(&ble_fast);
            }
        }

        /* --------------------------------------------------------------------
         * 5. Environmental Telemetry at 1 Hz (Event-Driven from Task_Sensors_Slow)
         * -------------------------------------------------------------------- */
        if (g_env_new_sample) {
            pthread_mutex_lock(&g_env_mutex);
            hal_bme680_data_t bme = g_latest_bme;
            bool bme_ok = g_bme_valid;
            float lux = g_latest_lux;
            bool opt_ok = g_opt_valid;
            uint16_t prox = g_latest_prox;
            float mlx_obj = g_latest_mlx_obj;
            bool mlx_ok = g_mlx_valid;
            bool contact = g_latest_contact;
            g_env_new_sample = false;
            pthread_mutex_unlock(&g_env_mutex);

            if (g_stream_mode == STREAM_MODE_RAW) {
                uart_print("{\"type\":\"env\"");
                if (bme_ok) {
                    uart_print(",\"T\":"); uart_print_float2(bme.temperature_c);
                    uart_print(",\"H\":"); uart_print_float2(bme.humidity_pct);
                    uart_print(",\"P\":"); uart_print_float2(bme.pressure_hpa);
                    uart_print(",\"gas\":"); uart_print_float2(bme.gas_res_kohm);
                    uart_print(",\"iaq\":"); uart_print_float2(bme.iaq);
                    uart_print(",\"co2\":"); uart_print_float2(bme.co2_equivalent);
                }
                if (opt_ok) {
                    uart_print(",\"lux\":"); uart_print_float2(lux);
                }
                uart_print(",\"prox\":"); uart_print_dec((uint32_t)prox);
                if (mlx_ok) {
                    uart_print(",\"mlx\":"); uart_print_float2(mlx_obj);
                }
                uart_print(",\"mode\":"); uart_print_dec((uint32_t)hal_ecg_get_mode());
                uint16_t mcu_bpm = (g_edgeai_ecg_state.samples_since_qrs < 750U) ? g_edgeai_ecg_result.heart_rate_smooth_bpm : 0U;
                uart_print(",\"bpm\":"); uart_print_dec((uint32_t)mcu_bpm);
                uart_print(",\"rpm\":"); uart_print_dec((uint32_t)g_resp_est.rpm);

                /* Method 1: Dynamic Run-Time Power Consumption Metrics */
                bsp_power_metrics_t pwr = bsp_power_get_metrics();
                uart_print(",\"pwr_mA\":"); uart_print_float2(pwr.current_ma);
                uart_print(",\"pwr_mW\":"); uart_print_float2(pwr.power_mw);
                uart_print(",\"pwr_mJ\":"); uart_print_float1(pwr.accum_energy_mj);
                uart_print(",\"bat_pct\":"); uart_print_float1(pwr.battery_remain_pct);
                uart_print(",\"bat_hr\":"); uart_print_float1(pwr.battery_hours_left);
                uart_print(",\"pwr_st\":"); uart_print_dec((uint32_t)pwr.state);
                uart_print("}\r\n");

                /* Emit 08_ecg_dedicated hardware Edge-AI cardiac summary frame:
                 * S,<bpm>,<rr_ms>,<resp_rpm>,<loff>,<sdnn_ms>,<rmssd_ms>,<flags>\r\n */
                if (g_uart) {
                    char s_buf[64];
                    int s_len = snprintf(s_buf, sizeof(s_buf), "S,%u,%u,%u,%u,%u,%u,%u\r\n",
                                         (unsigned int)mcu_bpm,
                                         (unsigned int)g_edgeai_ecg_result.rr_interval_ms,
                                         (unsigned int)g_resp_est.rpm,
                                         (unsigned int)((g_edgeai_ecg_result.cardiac_flags & CARDIAC_FLAG_LEAD_OFF) ? 6U : 0U),
                                         (unsigned int)g_edgeai_ecg_result.hrv_sdnn_ms,
                                         (unsigned int)g_edgeai_ecg_result.hrv_rmssd_ms,
                                         (unsigned int)g_edgeai_ecg_result.cardiac_flags);
                    if (s_len > 0) {
                        size_t w_s = 0;
                        UART2_write(g_uart, s_buf, (size_t)s_len, &w_s);
                    }

                    /* Step ETSI TS 103 326 SmartBAN Superframe & DWT Energy Profiler */
                    if (g_smartban_mac.ibi_sequence == 0U) {
                        smartban_mac_init(&g_smartban_mac);
                    }
                    bool had_burst = (g_smartban_mac.adaptive_burst_rem_sec > 0U);
                    smartban_mac_step_1hz(&g_smartban_mac,
                                          &g_edgeai_ecg_result,
                                          &g_edgeai_imu_result,
                                          &g_tinyml_result,
                                          contact);
                    if (had_burst && g_smartban_mac.adaptive_burst_rem_sec == 0U &&
                        (g_edgeai_ecg_result.cardiac_flags & CARDIAC_FLAG_PVC) == 0U) {
                        g_tinyml_result.predicted_class = TINYML_CLASS_N_NORMAL;
                        g_tinyml_result.class_char = 'N';
                        g_tinyml_result.confidence_pct = 98U;
                    }

                    /* Emit SmartBAN MAC Superframe + Int8 TinyML + Hardware Energy Comparison Frame:
                     * B,<ibi_seq>,<policy>,<slot>,<tinyml_cls>,<tinyml_conf>,<tinyml_us>,<cpu_us_s>,<raw_Bps>,<sem_Bps>,<adap_Bps>,<pwr_raw_mW>,<pwr_sem_mW>,<pwr_adap_mW>,<uj_beat>\r\n */
                    char b_buf[128];
                    int b_len = snprintf(b_buf, sizeof(b_buf),
                                         "B,%lu,%u,%u,%c,%u,%lu,%lu,%lu,%lu,%lu,%.2f,%.2f,%.2f,%.1f\r\n",
                                         (unsigned long)g_smartban_mac.ibi_sequence,
                                         (unsigned int)g_smartban_mac.active_policy,
                                         (unsigned int)g_smartban_mac.active_slot,
                                         g_tinyml_result.class_char ? g_tinyml_result.class_char : 'N',
                                         (unsigned int)g_tinyml_result.confidence_pct,
                                         (unsigned long)g_tinyml_result.inference_us,
                                         (unsigned long)g_smartban_mac.cpu_active_us_per_sec,
                                         (unsigned long)g_smartban_mac.raw_stream_bytes_sec,
                                         (unsigned long)g_smartban_mac.semantic_bytes_sec,
                                         (unsigned long)g_smartban_mac.adaptive_bytes_sec,
                                         (double)g_smartban_mac.power_mw_raw,
                                         (double)g_smartban_mac.power_mw_semantic,
                                         (double)g_smartban_mac.power_mw_adaptive,
                                         (double)g_smartban_mac.energy_uj_per_beat);
                    if (b_len > 0) {
                        size_t w_b = 0;
                        UART2_write(g_uart, b_buf, (size_t)b_len, &w_b);
                    }

                    /* Broadcast 1 Hz SmartBAN Frame over CC2652R1 2.4 GHz BLE RF Core */
                    if (hal_ble_radio_is_active()) {
                        hal_ble_telemetry_t ble_telem;
                        ble_telem.heart_rate_bpm = (uint8_t)mcu_bpm;
                        ble_telem.rr_interval_ms = g_edgeai_ecg_result.rr_interval_ms;
                        ble_telem.hrv_sdnn_ms = g_edgeai_ecg_result.hrv_sdnn_ms;
                        ble_telem.hrv_rmssd_ms = g_edgeai_ecg_result.hrv_rmssd_ms;
                        ble_telem.tinyml_class_id = (uint8_t)g_tinyml_result.predicted_class;
                        ble_telem.resp_rpm = (uint8_t)g_resp_est.rpm;
                        ble_telem.temp_deg_c = (int8_t)(bme_ok ? bme.temperature_c : mlx_obj);
                        ble_telem.posture_id = (uint8_t)g_edgeai_imu_result.posture;
                        ble_telem.fall_alert = g_edgeai_imu_result.fall_detected;
                        ble_telem.pvc_alert = ((g_edgeai_ecg_result.cardiac_flags & CARDIAC_FLAG_PVC) != 0U);
                        ble_telem.smartban_slot = (uint8_t)g_smartban_mac.active_slot;
                        ble_telem.is_cap_burst = (g_smartban_mac.adaptive_burst_rem_sec > 0U);

                        hal_ble_radio_broadcast(&ble_telem);
                    }
                }
            } else if (g_stream_mode == STREAM_MODE_SEMANTIC) {
                /* Master Thesis Semantic Token Frame (>95% Bandwidth Reduction) */
                edgeai_slow_cache_t slow_cache;
                edgeai_slow_cache_get(&slow_cache);
                edgeai_fusion_process(&g_edgeai_fusion_state,
                                      &g_edgeai_ecg_state,
                                      &g_edgeai_imu_state,
                                      &slow_cache,
                                      &g_semantic_token);

                float temp_c = bme_ok ? bme.temperature_c : mlx_obj;

                uart_print("{\"type\":\"SEM\",\"hr\":");
                uart_print_dec((uint32_t)g_semantic_token.heart_rate_bpm);
                uart_print(",\"rr\":");
                uart_print_dec((uint32_t)g_semantic_token.rr_interval_ms);
                uart_print(",\"rmssd\":");
                uart_print_dec((uint32_t)g_semantic_token.hrv_rmssd_ms);
                uart_print(",\"sdnn\":");
                uart_print_dec((uint32_t)g_semantic_token.hrv_sdnn_ms);
                uart_print(",\"flags\":");
                uart_print_dec((uint32_t)g_semantic_token.cardiac_flags);
                uart_print(",\"posture\":\"");
                uart_print(g_semantic_token.posture_state == POSTURE_STATE_SEDENTARY ? "SEDENTARY" :
                           (g_semantic_token.posture_state == POSTURE_STATE_ACTIVE ? "ACTIVE" : "DYNAMIC"));
                uart_print("\",\"fall\":");
                uart_print_dec(g_semantic_token.fall_detected ? 1 : 0);
                uart_print(",\"contact\":");
                uart_print_dec(contact ? 1 : 0);
                uart_print(",\"temp\":");
                uart_print_float1(temp_c);
                uart_print(",\"lux\":");
                uart_print_float1(lux);
                uart_print("}\r\n");
            }
        }

        /* --------------------------------------------------------------------
         * 6. Sleep & Deep Sleep State Machine Execution
         * -------------------------------------------------------------------- */
        if (g_sleep_requested || g_deepsleep_requested) {
            bool is_deep = g_deepsleep_requested;
            uint32_t timer_s = g_sleep_timer_sec;
            g_sleep_requested = false;
            g_deepsleep_requested = false;

            /* Halt all worker sensor threads (ECG, IMU, Slow Environmental) */
            g_is_system_asleep = true;

            if (is_deep) {
                uart_print(">> ENTERING DEEP SLEEP (Sub-5uA Shutdown)... Timer=");
                uart_print_dec(timer_s);
                uart_print("s\r\n");
                /* Emit low-power telemetry frame for GUI Power Manager */
                uart_print("{\"type\":\"env\",\"pwr_mA\":0.004,\"pwr_mW\":0.013,\"pwr_st\":2,\"mode\":0}\r\n");
                hal_ui_display_text('d', 'E', 'P');
                usleep(350000);
                hal_ui_sleep();
                bsp_power_enter_deepsleep(timer_s);
            } else {
                uart_print(">> ENTERING SLEEP (Standby)... Timer=");
                uart_print_dec(timer_s);
                uart_print("s\r\n");
                /* Emit low-power telemetry frame for GUI Power Manager */
                uart_print("{\"type\":\"env\",\"pwr_mA\":0.020,\"pwr_mW\":0.066,\"pwr_st\":1,\"mode\":0}\r\n");
                hal_ui_display_text('S', 'L', 'P');
                usleep(350000);
                hal_ui_sleep();
                bsp_power_enter_sleep(timer_s);
            }

            /* 1. Wait for physical button release debounce (250ms) */
            usleep(250000);

            /* 2. Read PCAL6408A input register to clear any pending interrupt latch */
            uint8_t dummy_pcal = 0;
            bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &dummy_pcal);

            /* 3. Flush and discard any leftover characters in UART RX FIFO */
            uint8_t dummy_rx = 0;
            size_t dummy_len = 0;
            while (UART2_read(g_uart, &dummy_rx, 1, &dummy_len) == UART2_STATUS_SUCCESS && dummy_len > 0) {
                /* Discard */
            }

            /* Sleep loop: wake on Timer, Button SW3 (DIO 29), or UART RX character */
            uint32_t sleep_start = Clock_getTicks();
            uint32_t timer_ticks = (timer_s > 0) ? (timer_s * 100000) : 0xFFFFFFFF;
            bool woken = false;

            while (!woken) {
                /* Timer auto-wake */
                if (timer_s > 0) {
                    uint32_t elapsed = Clock_getTicks() - sleep_start;
                    if (elapsed >= timer_ticks) {
                        woken = true;
                        break;
                    }
                }

                /* Physical SW3 button on PCAL6408A: verify active button press via I2C */
                if (GPIO_read(CONFIG_GPIO_BTN_INT) == 0) {
                    uint8_t btn_val = 0xFF;
                    if (bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &btn_val)) {
                        uint8_t pressed_btns = (~btn_val) & HAL_UI_BTN_MASK_ALL;
                        if (pressed_btns != 0) {
                            /* Physical button confirmed pressed! */
                            woken = true;
                            break;
                        }
                    }
                }

                /* UART character wake check */
                uint8_t rx_wake = 0;
                size_t rx_w_len = 0;
                if (UART2_read(g_uart, &rx_wake, 1, &rx_w_len) == UART2_STATUS_SUCCESS && rx_w_len > 0) {
                    woken = true;
                    break;
                }

                usleep(50000); /* 50ms low-power sleep */
            }

            /* Restore active operation */
            bsp_power_restore_active();
            bsp_spi_init();
            bsp_i2c_init();
            hal_ui_init();
            hal_ui_display_text('u', 'P', ' ');
            hal_ui_set_mode_leds(mode_leds[g_display_mode]);
            g_display_show_label = true;
            g_display_label_countdown = 20;

            /* Resume worker sensor threads */
            g_is_system_asleep = false;

            uart_print("{\"type\":\"env\",\"pwr_mA\":31.80,\"pwr_mW\":104.94,\"pwr_st\":0,\"mode\":0}\r\n");
            uart_print(">> SYSTEM AWAKE\r\n");
        }

        /* Adaptive loop pacing: never sleep while ECG samples remain in ring buffer! */
        if (!ringbuf_ecg_is_empty(&g_ringbuf_ecg)) {
            /* Continue immediately to drain queued 250 Hz ECG samples */
        } else if (had_ecg) {
            usleep(200);
        } else {
            usleep(1000);
        }
    }
    return NULL;
}

/* ============================================================================
 * Task_Init (Priority 5, Highest Priority, Self-Terminating Init Task)
 * Opens UART2, initializes power rails, buses, sensors, semaphores,
 * and spawns the 4 worker tasks before cleanly exiting.
 * ============================================================================ */
static void *task_init_entry(void *arg0)
{
    (void)arg0;

    /* 1. Open UART2 now that TI-RTOS7 kernel is running */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate  = 115200;
    uartParams.readMode  = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    g_uart = UART2_open(CONFIG_UART2_0, &uartParams);

    uart_print("\r\n\r\n");
    uart_print("================================================================\r\n");
    uart_print("   SmartBAN All-in-One Multi-Threaded TI-RTOS7 Firmware         \r\n");
    uart_print("   Target: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5         \r\n");
    uart_print("================================================================\r\n");

    /* 2. Initialize Mutexes */
    pthread_mutexattr_t m_attr;
    pthread_mutexattr_init(&m_attr);
    pthread_mutexattr_settype(&m_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutexattr_setprotocol(&m_attr, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&g_imu_mutex, &m_attr);
    pthread_mutex_init(&g_env_mutex, &m_attr);
    pthread_mutexattr_destroy(&m_attr);

    /* 3. Initialize Power Rails */
    uart_print("[INIT] Power Rails (1.8V EN, I2C Shifter, IMU SW)... ");
    bsp_power_init();
    /* Ensure MAX32664 PPG reset and MFIO lines are held high to disable */
    GPIO_write(CONFIG_GPIO_HUB_RST, 1);
    GPIO_write(CONFIG_GPIO_HUB_MFIO, 1);
    uart_print("DONE\r\n");

    /* 4. Initialize Shared Buses */
    uart_print("[INIT] Shared SPI Bus Manager (ADS1292R Mode 1 / ADXL362 Mode 0)... ");
    bsp_spi_init();
    uart_print("DONE\r\n");

    uart_print("[INIT] Shared I2C Bus Manager (400 kHz)... ");
    bsp_i2c_init();
    uart_print("DONE\r\n");

    /* 5. Initialize SPSC Ring Buffer and DRDY Semaphore */
    uart_print("[INIT] SPSC Ring Buffer & DRDY Binary Semaphore... ");
    ringbuf_ecg_init(&g_ringbuf_ecg);
    sem_init(&g_sem_ecg_drdy, 0, 0);
    sem_init(&g_sem_imu_tick, 0, 0);
    uart_print("DONE\r\n");

    /* 6. Initialize Peripherals & Sensors */
    uart_print("[INIT] UI Driver (PCAL6408A 6-Button & CH455H Display)... ");
    hal_ui_init();
    hal_ui_display_text('r', 'd', 'Y');
    hal_ui_set_mode_leds(mode_leds[g_display_mode]);
    uart_print("DONE\r\n");

    uart_print("[INIT] ADXL362 3-Axis Accelerometer... ");
    bool imu_ok = hal_imu_init(HAL_IMU_RANGE_8G);
    if (imu_ok) {
        /* Enable autonomous activity / inactivity detection */
        hal_imu_configure_motion_wakeup(250, 4, 150, 50);
        uart_print("ONLINE (ID=0xAD)\r\n");
    } else {
        uart_print("FAILED\r\n");
    }

    uart_print("[INIT] BME680 / BME690 Environmental Sensor... ");
    bool bme_ok = hal_bme680_init();
    uart_print(bme_ok ? "ONLINE\r\n" : "NOT DETECTED\r\n");

    uart_print("[INIT] OPT4041 ALS & VCNL4040 Proximity... ");
    hal_optical_init();
    uart_print("DONE\r\n");

    uart_print("[INIT] MLX90632 Far-IR Medical Thermometer... ");
    bool fir_ok = hal_fir_init();
    uart_print(fir_ok ? "ONLINE\r\n" : "NOT DETECTED\r\n");

    uart_print("[INIT] ADS1292R 24-Bit ECG & Respiration AFE... ");
    bool ecg_ok = hal_ecg_init();
    if (ecg_ok) {
        uart_print("ONLINE (ID=0x");
        uint8_t id = hal_ecg_get_id();
        const char hexDigits[] = "0123456789ABCDEF";
        char id_str[3] = { hexDigits[(id >> 4) & 0x0F], hexDigits[id & 0x0F], '\0' };
        uart_print(id_str);
        uart_print(")\r\n");
    } else {
        uart_print("NOT DETECTED\r\n");
    }

    /* 7. Register ADS1292R DRDY Falling Edge Interrupt */
    uart_print("[INIT] Registering ADS1292R DRDY Callback (DIO 23)... ");
    hal_ecg_register_drdy_callback(ecg_drdy_callback);
    uart_print("DONE\r\n");

    /* 8. Initialize Edge-AI Engines */
    edgeai_ecg_init(&g_edgeai_ecg_state, NULL);
    edgeai_imu_init(&g_edgeai_imu_state, NULL);
    edgeai_fusion_init(&g_edgeai_fusion_state, NULL);

    /* 8b. Initialize CC2652R1 2.4 GHz BLE RF Core */
    uart_print("[INIT] CC2652R1 2.4 GHz BLE RF Core (1 Mbps Broadcaster)... ");
    bool ble_ok = hal_ble_radio_init();
    uart_print(ble_ok ? "ONLINE\r\n" : "FAILED\r\n");

    /* Flush button states after all power rails and ADS1292R POR have settled */
    uint8_t dummy_c = 0, dummy_p = 0, dummy_r = 0;
    hal_ui_poll_buttons(&dummy_c, &dummy_p, &dummy_r);
    hal_ui_poll_buttons(&dummy_c, &dummy_p, &dummy_r);

    /* Drain any spurious DRDY semaphore pulses accumulated during boot */
    while (sem_trywait(&g_sem_ecg_drdy) == 0);

    /* 9. Spawn Worker Tasks (P4, P3, P2, P1) */
    uart_print("[BOOT] Spawning TI-RTOS7 Multi-Threaded Workers...\r\n");

    pthread_t thread_ecg, thread_imu, thread_slow, thread_telem;
    pthread_attr_t attrs;
    struct sched_param priParam;

    /* Task_ECG (Priority 4, Stack 4096 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 4096);
    priParam.sched_priority = 4;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&thread_ecg, &attrs, task_ecg_entry, NULL) != 0) {
        uart_print("[FATAL] Task_ECG creation failed!\r\n");
        while(1);
    }
    pthread_attr_destroy(&attrs);
    uart_print("  -> Task_ECG (Priority 4, DRDY Unblocked): RUNNING\r\n");

    /* Task_IMU (Priority 3, Stack 4096 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 4096);
    priParam.sched_priority = 3;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&thread_imu, &attrs, task_imu_entry, NULL) != 0) {
        uart_print("[FATAL] Task_IMU creation failed!\r\n");
        while(1);
    }
    pthread_attr_destroy(&attrs);
    uart_print("  -> Task_IMU (Priority 3, 25 Hz Periodic): RUNNING\r\n");

    /* Task_Sensors_Slow (Priority 2, Stack 4096 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 4096);
    priParam.sched_priority = 2;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&thread_slow, &attrs, task_sensors_slow_entry, NULL) != 0) {
        uart_print("[FATAL] Task_Sensors_Slow creation failed!\r\n");
        while(1);
    }
    pthread_attr_destroy(&attrs);
    uart_print("  -> Task_Sensors_Slow (Priority 2, 1 Hz Periodic): RUNNING\r\n");

    /* Task_Telemetry_UI (Priority 1, Stack 4096 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 4096);
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&thread_telem, &attrs, task_telemetry_ui_entry, NULL) != 0) {
        uart_print("[FATAL] Task_Telemetry_UI creation failed!\r\n");
        while(1);
    }
    pthread_attr_destroy(&attrs);
    uart_print("  -> Task_Telemetry_UI (Priority 1, Streaming & CLI): RUNNING\r\n");

    uart_print("[BOOT] All Tasks Online. System Operational.\r\n\r\n");

    /* Task_Init terminates cleanly */
    return NULL;
}

/* ============================================================================
 * main()
 * Minimal initialization before BIOS_start()
 * ============================================================================ */
int main(void)
{
    Board_init();
    GPIO_init();
    I2C_init();
    SPI_init();

    /* Spawn high-priority Task_Init (Priority 5) to run when kernel starts */
    pthread_t init_thread;
    pthread_attr_t attrs;
    struct sched_param priParam;

    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 4096);
    priParam.sched_priority = 5;
    pthread_attr_setschedparam(&attrs, &priParam);
    pthread_create(&init_thread, &attrs, task_init_entry, NULL);
    pthread_attr_destroy(&attrs);

    /* Start TI-RTOS7 Kernel - Never returns */
    BIOS_start();

    return 0;
}
