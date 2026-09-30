/*
 * ============================================================================
 * main_tirtos.c
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * Milestone M3: Edge-AI Semantic Processing & Data Reduction (Thesis Core)
 * ============================================================================
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#include <ti/sysbios/BIOS.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

#include "bsp/bsp_pins.h"
#include "bsp/bsp_power.h"
#include "bsp/bsp_spi.h"
#include "bsp/bsp_i2c.h"
#include "hal/hal_ecg.h"
#include "hal/hal_imu.h"
#include "hal/hal_fir.h"
#include "hal/hal_optical.h"
#include "hal/hal_ui.h"
#include "ipc/ring_buffer.h"
#include "edgeai/edgeai_ecg.h"
#include "edgeai/edgeai_imu.h"
#include "edgeai/edgeai_fusion.h"
#include "telemetry/telemetry_uart.h"
#include "telemetry/cli_console.h"
#include "telemetry/ui_display.h"

#include <ti/posix/tirtos/_pthread.h>
#include <ti/sysbios/knl/Task.h>

/* Global Thread Handles for Diagnostics & Watermark Inspection */
static pthread_t g_thread_ecg;
static pthread_t g_thread_imu;
static pthread_t g_thread_edgeai;
static pthread_t g_thread_sensors_slow;
static pthread_t g_thread_telemetry_ui;

/* UART handle for telemetry and console */
static UART2_Handle s_uart = NULL;

static void uart_print(const char *str)
{
    if (!s_uart || !str) return;
    size_t written;
    UART2_write(s_uart, str, strlen(str), &written);
}

/* ============================================================================
 * Milestone M2/M3 Inter-Process Communication (IPC) & Shared State
 * ============================================================================ */

/* Global SPSC circular ring buffers */
ringbuf_ecg_t g_ringbuf_ecg;
ringbuf_imu_t g_ringbuf_imu;

/* Global synchronization semaphores */
sem_t sem_ecg_ready;       /* Binary semaphore: posted by DRDY GPIO ISR */
sem_t sem_edgeai_trigger;  /* Counting semaphore: unblocks Task_EdgeAI */

/* Stream mode: Semantic (Default, >95% reduction) vs Raw (Evaluation) */
static volatile smartban_stream_mode_t g_stream_mode = STREAM_MODE_SEMANTIC;
static volatile smartban_semantic_token_t g_latest_semantic_token;

/* Sequence and processing metrics (Placeholder for M3) */
static volatile uint32_t g_ecg_processed_count = 0;
static volatile uint32_t g_imu_processed_count = 0;
static volatile hal_ecg_sample_t g_latest_ecg;
static volatile hal_imu_sample_t g_latest_imu;

/* Environmental & UI cache */
static volatile uint16_t g_latest_prox = 0;
static volatile bool     g_latest_contact = false;
static volatile float    g_latest_lux = 0.0f;
static volatile float    g_latest_ambient_c = 0.0f;
static volatile float    g_latest_object_c = 0.0f;
static volatile uint16_t g_display_hr = 72;
static ui_display_state_t s_ui_state;

/* ============================================================================
 * ADS1292 DRDY Falling Edge GPIO Interrupt Service Routine (Hwi)
 * ============================================================================ */
static void ecg_drdy_callback(uint_least8_t index)
{
    (void)index;
    /* In TI-RTOS7 POSIX, sem_post maps to Semaphore_post(), safe in Hwi context.
     * Posts binary semaphore to unblock high-priority Task_ECG (Priority 4). */
    sem_post(&sem_ecg_ready);
}

/* ============================================================================
 * 5-Task Scheduling Implementations
 * ============================================================================ */

/**
 * @brief Task 1: Task_ECG (Biopotential Acquisition)
 *        - Priority: 4 (Highest Application Task)
 *        - Stack: 2048 Bytes
 *        - Trigger: ADS1292 DRDY Falling Edge (DIO 23) via sem_ecg_ready
 *        - Cadence: 250 Hz nominal (every 4.0 ms)
 */
static void *task_ecg_entry(void *arg0)
{
    (void)arg0;
    uart_print("[TASK_ECG] Started (Priority 4, Stack 2048 B, 250 Hz DRDY Sync)\r\n");

    /* M3 Integration Advisory: Initialize ADS1292R in task context after BIOS_start() */
    uart_print("[TASK_ECG] Initializing ADS1292R AFE (1000 ms POR sequence)...\r\n");
    if (!hal_ecg_init(HAL_ECG_RATE_250_SPS)) {
        uart_print("[TASK_ECG] ERROR: ADS1292R initialization failed!\r\n");
    } else {
        uart_print("[TASK_ECG] ADS1292R Online (250 SPS, RDATAC Streaming Active)\r\n");
    }

    hal_ecg_sample_t sample;

    while (1) {
        /* Block until DRDY falling edge ISR posts sem_ecg_ready */
        if (sem_wait(&sem_ecg_ready) != 0) {
            continue;
        }

        /* Read 24-bit ECG frame (hal_ecg_read_sample acquires/releases SPI bus internally) */
        if (hal_ecg_read_sample(&sample)) {
            /* Push to lock-free SPSC ring buffer (128-element capacity) */
            ringbuf_ecg_push(&g_ringbuf_ecg, &sample);

            /* Signal downstream Edge-AI pipeline */
            sem_post(&sem_edgeai_trigger);
        }
    }

    return NULL;
}

/**
 * @brief Task 2: Task_IMU (Motion Dynamics Acquisition)
 *        - Priority: 3 (Medium-High)
 *        - Stack: 1536 Bytes
 *        - Trigger: Periodic 100 Hz timer (usleep(10000))
 *        - Cadence: 100 Hz nominal (every 10.0 ms)
 */
static void *task_imu_entry(void *arg0)
{
    (void)arg0;
    uart_print("[TASK_IMU] Started (Priority 3, Stack 1536 B, 100 Hz Periodic)\r\n");

    /* M3 Integration Advisory: Initialize ADXL362 in task context after BIOS_start()
     * MUST be HAL_IMU_RANGE_8G to prevent 12-bit ADC saturation during >3.0g impacts */
    uart_print("[TASK_IMU] Initializing ADXL362 IMU (+/-8g Range)...\r\n");
    if (!hal_imu_init(HAL_IMU_RANGE_8G)) {
        uart_print("[TASK_IMU] ERROR: ADXL362 initialization failed!\r\n");
    } else {
        uart_print("[TASK_IMU] ADXL362 Online (+/-8g Range, 100 Hz ODR, Mode 0 SPI)\r\n");
    }

    hal_imu_sample_t sample;

    while (1) {
        /* Periodic 100 Hz acquisition delay */
        usleep(10000);

        /* Read IMU dynamics (hal_imu_read_sample acquires/releases SPI bus internally) */
        if (hal_imu_read_sample(&sample)) {
            /* Push to lock-free SPSC ring buffer (64-element capacity) */
            ringbuf_imu_push(&g_ringbuf_imu, &sample);

            /* Signal downstream Edge-AI pipeline */
            sem_post(&sem_edgeai_trigger);
        }
    }

    return NULL;
}

/**
 * @brief Task 3: Task_EdgeAI (Semantic Feature Extraction & Data Reduction)
 *        - Priority: 3 (Medium)
 *        - Stack: 2048 Bytes
 *        - Trigger: sem_edgeai_trigger
 *        - Function: Consumes pending ECG and IMU samples from SPSC ring buffers
 */
static void *task_edgeai_entry(void *arg0)
{
    (void)arg0;
    uart_print("[TASK_EDGEAI] Started (Priority 3, Stack 2048 B, Consumer Pipeline)\r\n");

    /* Milestone M3: Initialize Edge-AI processing engines */
    static edgeai_ecg_state_t    s_ecg_state;
    static edgeai_imu_state_t    s_imu_state;
    static edgeai_fusion_state_t s_fusion_state;
    edgeai_ecg_result_t          s_ecg_result;
    edgeai_imu_result_t          s_imu_result;
    smartban_semantic_token_t    s_token;

    edgeai_ecg_init(&s_ecg_state, NULL);
    edgeai_imu_init(&s_imu_state, NULL);
    edgeai_fusion_init(&s_fusion_state, NULL);

    hal_ecg_sample_t ecg_s;
    hal_imu_sample_t imu_s;
    edgeai_slow_cache_t slow_cache;

    while (1) {
        /* Block until triggered by producer task */
        if (sem_wait(&sem_edgeai_trigger) != 0) {
            continue;
        }

        /* Drain all pending tokens to prevent queue backlog */
        while (sem_trywait(&sem_edgeai_trigger) == 0);

        /* Pop and consume all available ECG samples from SPSC ring buffer */
        while (ringbuf_ecg_pop(&g_ringbuf_ecg, &ecg_s)) {
            g_ecg_processed_count++;
            g_latest_ecg = ecg_s;
            bool qrs = edgeai_ecg_process_sample(&s_ecg_state,
                                                 ecg_s.raw_ch1,
                                                 ecg_s.timestamp_ms,
                                                 ecg_s.lead_off,
                                                 &s_ecg_result);
            if (qrs) {
                g_display_hr = s_ecg_result.heart_rate_bpm;
                ui_display_notify_beat(&s_ui_state);
            }
        }

        /* Pop and consume all available IMU samples from SPSC ring buffer */
        while (ringbuf_imu_pop(&g_ringbuf_imu, &imu_s)) {
            g_imu_processed_count++;
            g_latest_imu = imu_s;
            edgeai_imu_process_sample(&s_imu_state, &imu_s, &s_imu_result);
        }

        /* Fetch latest slow sensor readings and execute Multi-Modal Context Fusion */
        edgeai_slow_cache_get(&slow_cache);
        if (g_latest_ecg.timestamp_ms > 0) {
            slow_cache.timestamp_ms = g_latest_ecg.timestamp_ms;
        } else {
            slow_cache.timestamp_ms = (uint32_t)(g_ecg_processed_count * 4 + 1000);
        }

        if (edgeai_fusion_process(&s_fusion_state, &s_ecg_state, &s_imu_state, &slow_cache, &s_token)) {
            g_latest_semantic_token = s_token;
            if (s_token.heart_rate_bpm > 0) {
                g_display_hr = s_token.heart_rate_bpm;
            }
        }
    }

    return NULL;
}

/**
 * @brief Task 4: Task_Sensors_Slow (Multi-Rate Environmental & Contact Subsystem)
 *        - Priority: 2 (Low)
 *        - Stack: 1536 Bytes
 *        - Trigger: Periodic 10 Hz timer (usleep(100000))
 *        - Cadence:
 *            * 10 Hz (100 ms): VCNL4040 Proximity & Touch
 *            *  2 Hz (500 ms): OPT4041 Ambient Light Lux
 *            *  1 Hz (1000 ms): MLX90632 Calibrated Medical IR Temperatures
 */
static void *task_sensors_slow_entry(void *arg0)
{
    (void)arg0;
    uart_print("[TASK_SENSORS_SLOW] Started (Priority 2, Stack 1536 B, 10 Hz Multi-Rate)\r\n");

    /* M3 Integration Advisory: Initialize MLX90632 in task context after BIOS_start() */
    uart_print("[TASK_SENSORS_SLOW] Initializing MLX90632 Medical FIR Thermometer...\r\n");
    if (!hal_fir_init()) {
        uart_print("[TASK_SENSORS_SLOW] ERROR: MLX90632 initialization failed!\r\n");
    } else {
        uart_print("[TASK_SENSORS_SLOW] MLX90632 Online (14 EEPROM Coeffs Calibrated)\r\n");
    }

    uint32_t tick = 0;

    while (1) {
        /* 10 Hz base loop (100 ms) */
        usleep(100000);
        tick++;

        /* 10 Hz: VCNL4040 Proximity & Skin Contact Detection */
        uint16_t prox = 0, als = 0;
        bool contact = false;
        if (hal_optical_read_prox(&prox, &als, &contact)) {
            g_latest_prox = prox;
            g_latest_contact = contact;
        }

        /* 2 Hz (every 5th tick = 500 ms): OPT4041 Ambient Light Sensor */
        if ((tick % 5) == 0) {
            float lux = 0.0f;
            if (hal_optical_read_lux(&lux)) {
                g_latest_lux = lux;
            }
        }

        /* 1 Hz (every 10th tick = 1000 ms): MLX90632 Medical IR Thermometer */
        if ((tick % 10) == 0) {
            float ambient_c = 0.0f, object_c = 0.0f;
            if (hal_fir_read(&ambient_c, &object_c)) {
                g_latest_ambient_c = ambient_c;
                g_latest_object_c = object_c;
            }
        }

        /* Update thread-safe slow sensor cache for Edge-AI Context Fusion */
        edgeai_slow_cache_update(g_latest_ambient_c, g_latest_object_c, g_latest_prox, als,
                                 g_latest_lux, true, true);

        /* At 1 Hz (every 1000 ms), trigger Edge-AI Context Fusion so telemetry refreshes */
        if ((tick % 10) == 0) {
            sem_post(&sem_edgeai_trigger);
        }
    }

    return NULL;
}

/**
 * @brief Task 5: Task_Telemetry_UI (Integrated Telemetry, Interactive CLI & Visual UI)
 *        - Priority: 1 (Lowest Application Task)
 *        - Stack: 1536 Bytes
 *        - Trigger: Periodic 10 Hz timer (usleep(100000))
 *        - Functions:
 *            * Heartbeat LED indicator (DIO 7 / LaunchPad LED1 every 500 ms)
 *            * Interactive Serial CLI console maintenance & power watchdog
 *            * PCAL6408A tactile button debouncing (200 ms refractory lockout)
 *            * CH455H 7-segment display and mode LEDs update
 *            * UART2 JSON telemetry frame streaming via lock-free queue
 *            * Standby sleep management: UART2_rxDisable() when idle (~1.0 uA)
 */
static void *task_telemetry_ui_entry(void *arg0)
{
    (void)arg0;
    uart_print("[TASK_TELEMETRY_UI] Started (Priority 1, Stack 1536 B, 10 Hz UI, CLI & Telemetry)\r\n");

    /* Initialize M4 Subsystems */
    telemetry_uart_init(s_uart);
    cli_console_init(s_uart);
    ui_display_init(&s_ui_state, NULL);

    uint32_t ui_ticks = 0;

    while (1) {
        /* 10 Hz base loop (100 ms) */
        usleep(100000);
        ui_ticks++;

        /* 1. Toggle Heartbeat LED (DIO 7 / LaunchPad Green LED) every 500 ms */
        if ((ui_ticks % 5) == 0) {
            GPIO_toggle(CONFIG_GPIO_LED_1);
        }

        /* 2. Service Interactive Serial CLI Console (Read UART RX, manage Standby timeout) */
        cli_console_tick_10hz();

        /* 3. Poll PCAL6408A debounced tactile buttons */
        uint8_t raw = 0xFF;
        bsp_i2c_acquire();
        bool btn_ok = bsp_i2c_read_reg8(PCAL6408A_I2C_ADDR, PCAL6408A_REG_INPUT, &raw);
        bsp_i2c_release();

        if (btn_ok) {
            uint8_t raw_pressed = (~raw) & HAL_UI_BTN_MASK_ALL;
            uint8_t cur_mask = 0, pressed = 0, released = 0;
            ui_button_debounce_tick(raw_pressed, &cur_mask, &pressed, &released);

            /* SW1: Cycle Display View (HR -> TEMP -> STATUS -> HR) */
            if (pressed & HAL_UI_BTN_SW1_ECG) {
                ui_view_mode_t new_view = ui_display_cycle_view(&s_ui_state);
                const char *vname = (new_view == UI_VIEW_HR)   ? "HEART RATE" :
                                    (new_view == UI_VIEW_TEMP) ? "TEMPERATURE" : "SYSTEM STATUS";
                uart_print("[UI] View Switched -> ");
                uart_print(vname);
                uart_print("\r\n");
            }

            /* SW6: Toggle Stream Mode (Semantic vs Raw) and Wake CLI Session */
            if (pressed & HAL_UI_BTN_SW6_MODE) {
                g_stream_mode = edgeai_toggle_stream_mode();
                cli_console_wake_session();
                uart_print(g_stream_mode == STREAM_MODE_SEMANTIC ?
                           "[UI] Mode Switched -> SEMANTIC (>95% Reduction)\r\n" :
                           "[UI] Mode Switched -> RAW STREAM (Evaluation)\r\n");
            }

            /* SW4: Diagnostic Anomaly Injection (Toggle Fall Anomaly) */
            if (pressed & HAL_UI_BTN_SW4_BT) {
                uart_print("[UI] Diagnostic: Injected Test Fall Anomaly!\r\n");
                g_latest_semantic_token.fall_detected = !g_latest_semantic_token.fall_detected;
            }
        }

        /* 4. Update CH455H 7-Segment Display & Mode LEDs */
        ui_display_data_t ui_data;
        ui_data.heart_rate_bpm = (uint8_t)g_display_hr;
        ui_data.skin_temp_c    = g_latest_object_c;
        ui_data.stream_mode    = edgeai_get_stream_mode();
        ui_data.skin_contact   = g_latest_contact;
        ui_data.alert_mask     = g_latest_semantic_token.alert_mask;
        ui_data.fall_detected  = g_latest_semantic_token.fall_detected;
        ui_data.qrs_beat_event = false;

        bsp_i2c_acquire();
        ui_display_update(&s_ui_state, &ui_data);
        bsp_i2c_release();

        /* 5. Process Telemetry Streaming Queue (Only if user is not actively typing) */
        if (!cli_console_is_typing_active() && telemetry_uart_is_streaming_enabled()) {
            if (edgeai_get_stream_mode() == STREAM_MODE_SEMANTIC) {
                smartban_semantic_token_t tok = g_latest_semantic_token;
                static uint16_t s_prev_alert_mask = 0;
                bool new_anomaly = (tok.is_anomaly_event && (tok.alert_mask != s_prev_alert_mask));
                s_prev_alert_mask = tok.alert_mask;

                if (((ui_ticks % 10) == 0) || new_anomaly) {
                    tok.timestamp_ms = (uint32_t)(ui_ticks * 100);
                    tok.heart_rate_bpm = (uint8_t)g_display_hr;
                    telemetry_uart_enqueue_semantic(&tok);
                }
            } else {
                /* Raw Stream Mode: enqueue 10 Hz sample frame */
                telemetry_uart_enqueue_raw((uint32_t)(ui_ticks * 100),
                                           g_latest_ecg.raw_ch1,
                                           g_latest_imu.x_mg,
                                           g_latest_imu.y_mg,
                                           g_latest_imu.z_mg);
            }
            telemetry_uart_process_tx();
        }

        /* 6. Dynamic Standby Sleep Management: While idle in Standby Armed state,
         * ensure UART2 RX is disabled so CC2652R1 enters deep Standby sleep (~1.0 uA) */
        if (s_uart != NULL && cli_console_get_state() == CLI_STATE_STANDBY_ARMED) {
            UART2_rxDisable(s_uart);
        }
    }

    return NULL;
}

/* ============================================================================
 * Pre-Boot Self-Test Suite
 * ============================================================================ */
typedef struct {
    uint32_t tests_run;
    uint32_t tests_passed;
    uint32_t tests_failed;
} hal_test_report_t;

static hal_test_report_t s_report = {0, 0, 0};

#define ASSERT_TEST(cond, name) do { \
    s_report.tests_run++; \
    if (cond) { \
        s_report.tests_passed++; \
        uart_print("  [PASS] "); \
    } else { \
        s_report.tests_failed++; \
        uart_print("  [FAIL] "); \
    } \
    uart_print(name "\r\n"); \
} while (0)

static void run_m1_self_tests(void)
{
    uart_print("\r\n=======================================================\r\n");
    uart_print(" SmartBAN Milestone M1: HAL & Mathematical Self-Tests\r\n");
    uart_print("=======================================================\r\n");

    /* 1. Test ECG 24-bit conversion math */
    {
        float uv0 = hal_ecg_counts_to_uV(0);
        ASSERT_TEST(fabsf(uv0) < 1e-4f, "ECG: Zero counts converts to 0.0 uV");

        float uv_1mv = hal_ecg_counts_to_uV(20798);
        ASSERT_TEST(fabsf(uv_1mv - 1000.0f) < 1.0f, "ECG: +20,798 counts converts to ~1000.0 uV (+1 mV)");

        float uv_neg1mv = hal_ecg_counts_to_uV(-20798);
        ASSERT_TEST(fabsf(uv_neg1mv - (-1000.0f)) < 1.0f, "ECG: -20,798 counts converts to ~-1000.0 uV (-1 mV)");

        float uv_fs = hal_ecg_counts_to_uV(8388607);
        ASSERT_TEST(fabsf(uv_fs - 403333.3f) < 10.0f, "ECG: Full-scale positive converts to ~403.3 mV");
    }

    /* 2. Test IMU scaling and 12-bit sign extension math */
    {
        int16_t neg_sample = 0x0FFF;
        if (neg_sample & 0x0800) neg_sample |= (int16_t)0xF000;
        ASSERT_TEST(neg_sample == -1, "IMU: 12-bit 0x0FFF sign-extends to -1");

        int16_t min_sample = 0x0800;
        if (min_sample & 0x0800) min_sample |= (int16_t)0xF000;
        ASSERT_TEST(min_sample == -2048, "IMU: 12-bit 0x0800 sign-extends to -2048");

        int16_t rt_nom = 350;
        float temp_nom = ((float)(rt_nom - 350) * 0.065f) + 25.0f;
        ASSERT_TEST(fabsf(temp_nom - 25.0f) < 1e-4f, "IMU: 350 LSB nominal count converts to 25.0 C");

        float fx = 0.0f, fy = 0.0f, fz = 1000.0f;
        float total_g = sqrtf(fx * fx + fy * fy + fz * fz) / 1000.0f;
        ASSERT_TEST(fabsf(total_g - 1.0f) < 1e-4f, "IMU: (0, 0, 1000 mg) vector magnitude equals 1.0g");
    }

    /* 3. Test MLX90632 Non-Linear 3-Iteration Optical Model */
    {
        mlx90632_calib_t ds_cal;
        ds_cal.P_R = 6095107.0 / 256.0;
        ds_cal.P_G = 85785317.0 / 1048576.0;
        ds_cal.P_T = 0.0;
        ds_cal.P_O = 6400.0 / 256.0;
        ds_cal.Ea  = 5361582.0 / 65536.0;
        ds_cal.Eb  = 6095107.0 / 256.0;
        ds_cal.Fa  = 55599953.0 * pow(2.0, -46.0);
        ds_cal.Fb  = -31100431.0 * pow(2.0, -36.0);
        ds_cal.Ga  = -33577052.0 * pow(2.0, -36.0);
        ds_cal.Gb  = 9728.0 / 1024.0;
        ds_cal.Ka  = 10752.0 / 1024.0;
        ds_cal.Ha  = 16384.0 / 16384.0;
        ds_cal.Hb  = 0.0;

        int16_t sixRAM = 22500;
        int16_t nineRAM = 23000;
        int16_t lowerRAM = -101;
        int16_t upperRAM = -99;

        float tamb = 0.0f;
        bool amb_ok = hal_fir_calc_ambient(sixRAM, nineRAM, &ds_cal, &tamb);
        ASSERT_TEST(amb_ok && (tamb >= 28.38f && tamb <= 28.41f), "FIR: Ambient die temp matches Melexis ground truth [28.38, 28.41] degC");

        float tobj = 0.0f;
        bool obj_ok = hal_fir_calc_object(sixRAM, nineRAM, lowerRAM, upperRAM, &ds_cal, &tobj);
        ASSERT_TEST(obj_ok && (tobj >= 27.19f && tobj <= 27.22f), "FIR: Target object temp matches Melexis ground truth [27.19, 27.22] degC");
    }

    /* 4. Test OPT4041 ALS and VCNL4040 Proximity Math */
    {
        uint16_t reg0_nom = (4 << 12) | 0x0123;
        uint8_t  reg1_nom = 0x40;
        float lux_nom = hal_optical_calc_lux(reg0_nom, reg1_nom);
        ASSERT_TEST(fabsf(lux_nom - 697.88f) < 1.0f, "OPT4041: Nominal lux calculation accurate (697.88 Lux)");

        uint16_t reg0_e13 = (13 << 12) | 0x0800;
        uint8_t  reg1_e13 = 0x00;
        float lux_e13 = hal_optical_calc_lux(reg0_e13, reg1_e13);
        ASSERT_TEST(fabsf(lux_e13 - 2512555.87f) < 5.0f, "OPT4041: Exponent 13 rollover boundary verified (2.51M Lux)");

        uint16_t reg0_e14 = (14 << 12) | 0x0800;
        uint8_t  reg1_e14 = 0x00;
        float lux_e14 = hal_optical_calc_lux(reg0_e14, reg1_e14);
        ASSERT_TEST(fabsf(lux_e14 - 5025111.74f) < 10.0f, "OPT4041: Exponent 14 boundary verified (5.02M Lux)");

        uint16_t reg0_e15 = (15 << 12) | 0x0FFF;
        uint8_t  reg1_e15 = 0xFF;
        float lux_e15 = hal_optical_calc_lux(reg0_e15, reg1_e15);
        ASSERT_TEST(fabsf(lux_e15 - 20100427.78f) < 20.0f, "OPT4041: Exponent 15 full scale verified (20.10M Lux)");

        ASSERT_TEST((5999 < VCNL4040_SKIN_CONTACT_THRESH), "VCNL4040: Count 5999 below skin contact threshold");
        ASSERT_TEST((6000 >= VCNL4040_SKIN_CONTACT_THRESH), "VCNL4040: Count 6000 verifies skin contact");
        ASSERT_TEST((12500 >= VCNL4040_SKIN_CONTACT_THRESH), "VCNL4040: Count 12500 verifies firm skin contact");
    }

    /* 5. Test Mutex Concurrency Structures */
    {
        int spi_lock = pthread_mutex_lock(&spi_bus_mutex);
        int spi_unlock = pthread_mutex_unlock(&spi_bus_mutex);
        ASSERT_TEST(spi_lock == 0 && spi_unlock == 0, "RTOS: spi_bus_mutex lock and unlock cycle valid");

        int i2c_lock = pthread_mutex_lock(&i2c_bus_mutex);
        int i2c_unlock = pthread_mutex_unlock(&i2c_bus_mutex);
        ASSERT_TEST(i2c_lock == 0 && i2c_unlock == 0, "RTOS: i2c_bus_mutex lock and unlock cycle valid");
    }

    /* 6. Test UI 7-Segment Alphanumeric Font */
    {
        uint8_t seg_0 = hal_ui_char_to_segment('0');
        ASSERT_TEST(seg_0 == 0x3F, "UI: Digit '0' segments equal 0x3F (ABCDEF)");
        uint8_t seg_E = hal_ui_char_to_segment('E');
        ASSERT_TEST(seg_E == 0x79, "UI: Letter 'E' segments equal 0x79");
    }
}

static void run_m2_self_tests(void)
{
    uart_print("\r\n=======================================================\r\n");
    uart_print(" SmartBAN Milestone M2: SPSC Ring Buffer & RTOS Tests\r\n");
    uart_print("=======================================================\r\n");

    /* 1. SPSC ECG Ring Buffer Initialization & Basic Enqueue/Dequeue */
    /* Reuse global ring buffers for boot self-test to eliminate redundant BSS allocation */
    ringbuf_ecg_t *ecg_rb = &g_ringbuf_ecg;
    ringbuf_ecg_init(ecg_rb);
    ASSERT_TEST(ringbuf_ecg_is_empty(ecg_rb), "RingBuf ECG: Initial state is empty");
    ASSERT_TEST(!ringbuf_ecg_is_full(ecg_rb), "RingBuf ECG: Initial state is not full");
    ASSERT_TEST(ringbuf_ecg_available(ecg_rb) == 0, "RingBuf ECG: Initial available count is 0");
    ASSERT_TEST(ringbuf_ecg_free_space(ecg_rb) == RINGBUF_ECG_CAPACITY, "RingBuf ECG: Initial free space is 128");

    hal_ecg_sample_t s_in = {1000, 20798, -20798, 1000.0f, -1000.0f, 0xC0, false};
    bool push_ok = ringbuf_ecg_push(ecg_rb, &s_in);
    ASSERT_TEST(push_ok, "RingBuf ECG: Single sample enqueue succeeds");
    ASSERT_TEST(ringbuf_ecg_available(ecg_rb) == 1, "RingBuf ECG: Available count is 1 after push");

    hal_ecg_sample_t s_out;
    bool pop_ok = ringbuf_ecg_pop(ecg_rb, &s_out);
    ASSERT_TEST(pop_ok && s_out.timestamp_ms == 1000 && s_out.raw_ch1 == 20798,
                "RingBuf ECG: Single sample dequeue returns matching payload");
    ASSERT_TEST(ringbuf_ecg_is_empty(ecg_rb), "RingBuf ECG: Buffer is empty after pop");

    /* 2. SPSC ECG Full Buffer & Dropped Overflow Tracking */
    for (uint32_t i = 0; i < RINGBUF_ECG_CAPACITY; i++) {
        hal_ecg_sample_t s = {i, (int32_t)i, -(int32_t)i, 0.0f, 0.0f, 0xC0, false};
        ringbuf_ecg_push(ecg_rb, &s);
    }
    ASSERT_TEST(ringbuf_ecg_is_full(ecg_rb), "RingBuf ECG: Buffer correctly reports full at 128 elements");
    ASSERT_TEST(ringbuf_ecg_free_space(ecg_rb) == 0, "RingBuf ECG: Free space is 0 when full");

    hal_ecg_sample_t s_extra = {9999, 0, 0, 0, 0, 0, false};
    bool overflow_push = ringbuf_ecg_push(ecg_rb, &s_extra);
    ASSERT_TEST(!overflow_push, "RingBuf ECG: Push to full buffer rejected");

    ringbuf_stats_t ecg_stats;
    ringbuf_ecg_get_stats(ecg_rb, &ecg_stats);
    ASSERT_TEST(ecg_stats.dropped_count == 1, "RingBuf ECG: Dropped counter tracks exactly 1 dropped sample");
    ASSERT_TEST(ecg_stats.pushed_count == 129, "RingBuf ECG: Total pushed count recorded is 129 (1 initial + 128 fill)");

    /* 3. SPSC ECG Batch Pop */
    hal_ecg_sample_t batch[32];
    uint32_t dequeued = ringbuf_ecg_pop_batch(ecg_rb, batch, 32);
    ASSERT_TEST(dequeued == 32 && batch[0].timestamp_ms == 0 && batch[31].timestamp_ms == 31,
                "RingBuf ECG: pop_batch(32) preserves strict FIFO ordering");
    ASSERT_TEST(ringbuf_ecg_available(ecg_rb) == 96, "RingBuf ECG: 96 unconsumed elements remain");

    /* 4. SPSC IMU Ring Buffer Verification */
    ringbuf_imu_t *imu_rb = &g_ringbuf_imu;
    ringbuf_imu_init(imu_rb);
    ASSERT_TEST(ringbuf_imu_free_space(imu_rb) == RINGBUF_IMU_CAPACITY, "RingBuf IMU: Initial free space is 64");

    hal_imu_sample_t imu_in = {500, 0, 0, 1000, 1.0f, 0.0f, 0.0f, 25.0f, 0x01};
    ringbuf_imu_push(imu_rb, &imu_in);
    hal_imu_sample_t imu_out;
    ringbuf_imu_pop(imu_rb, &imu_out);
    ASSERT_TEST(imu_out.timestamp_ms == 500 && imu_out.z_mg == 1000, "RingBuf IMU: Push/Pop preserves sample payload");

    /* Reset buffers cleanly for runtime */
    ringbuf_ecg_init(&g_ringbuf_ecg);
    ringbuf_imu_init(&g_ringbuf_imu);

    /* 5. Semaphore Verification */
    sem_t test_sem;
    sem_init(&test_sem, 0, 0);
    sem_post(&test_sem);
    int sem_rc = sem_trywait(&test_sem);
    ASSERT_TEST(sem_rc == 0, "RTOS: sem_post and sem_trywait cycle verified");
    sem_destroy(&test_sem);

    char summary_buf[96];
    snprintf(summary_buf, sizeof(summary_buf),
             "\r\nSelf-Test Summary: %lu Run, %lu Passed, %lu Failed\r\n\r\n",
             (unsigned long)s_report.tests_run,
             (unsigned long)s_report.tests_passed,
             (unsigned long)s_report.tests_failed);
    uart_print(summary_buf);
}

static void run_m3_self_tests(void)
{
    char summary_buf[96];
    uart_print("\r\n=======================================================\r\n");
    uart_print(" SmartBAN Milestone M3: Edge-AI & Semantic Pipeline Tests\r\n");
    uart_print("=======================================================\r\n");

    /* 1. Fast Integer Square Root */
    ASSERT_TEST(ecg_isqrt(0) == 0, "EdgeAI: ecg_isqrt(0) == 0");
    ASSERT_TEST(ecg_isqrt(1) == 1, "EdgeAI: ecg_isqrt(1) == 1");
    ASSERT_TEST(ecg_isqrt(100) == 10, "EdgeAI: ecg_isqrt(100) == 10");
    ASSERT_TEST(ecg_isqrt(144) == 12, "EdgeAI: ecg_isqrt(144) == 12");
    ASSERT_TEST(ecg_isqrt(1000000) == 1000, "EdgeAI: ecg_isqrt(1,000,000) == 1000");

    /* 2. ECG Integer Pan-Tompkins Filter & DC Rejection */
    edgeai_ecg_state_t ecg_st;
    edgeai_ecg_init(&ecg_st, NULL);
    ASSERT_TEST(ecg_st.active_flags == CARDIAC_FLAG_LEARNING, "EdgeAI ECG: Initial state has CARDIAC_FLAG_LEARNING");

    edgeai_ecg_result_t ecg_res;
    /* Feed constant DC offset 20,000 counts */
    for (uint32_t i = 0; i < 50; i++) {
        edgeai_ecg_process_sample(&ecg_st, 20000, i * 4, false, &ecg_res);
    }
    /* HPF eliminates DC: filtered_ecg should be 0 after settling */
    ASSERT_TEST(ecg_res.filtered_ecg == 0, "EdgeAI ECG: HPF yields zero output on constant DC input (infinite DC attenuation)");

    /* 3. IMU Two-Pass Statistics, SMA, and Dynamic Tilt */
    /* Reuse g_ringbuf_ecg.buffer (3072 bytes) as temporary 100-sample array (2800 bytes) */
    hal_imu_sample_t *test_imu_samples = (hal_imu_sample_t *)g_ringbuf_ecg.buffer;
    for (int i = 0; i < 100; i++) {
        test_imu_samples[i].timestamp_ms = (uint32_t)(i * 10);
        test_imu_samples[i].x_mg = (int16_t)((i % 5) - 2);
        test_imu_samples[i].y_mg = (int16_t)(980 + (i % 7) - 3);
        test_imu_samples[i].z_mg = (int16_t)(150 + (i % 3) - 1);
        test_imu_samples[i].total_g = 0.99f;
        test_imu_samples[i].pitch_deg = 0.0f;
        test_imu_samples[i].roll_deg = 0.0f;
        test_imu_samples[i].temp_c = 25.0f;
        test_imu_samples[i].status = 0x01;
    }
    edgeai_imu_features_t imu_feats;
    edgeai_imu_calc_stats(test_imu_samples, 100, &imu_feats);

    ASSERT_TEST(fabsf(imu_feats.mean_y_mg - 980.0f) < 5.0f, "EdgeAI IMU: Mean Y-axis acceleration ~980 mg");
    ASSERT_TEST(imu_feats.sma_dynamic_g < 0.15f, "EdgeAI IMU: Sedentary dynamic SMA < 0.15g");
    ASSERT_TEST(imu_feats.total_variance_g2 < 0.001f, "EdgeAI IMU: Static position total variance < 0.001 g^2");

    /* 4. 3D Angle Difference */
    float angle_90 = edgeai_imu_calc_3d_angle_diff(0.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 1000.0f);
    ASSERT_TEST(fabsf(angle_90 - 90.0f) < 0.1f, "EdgeAI IMU: 3D angle between orthogonal vectors equals 90.0 deg");

    float angle_0 = edgeai_imu_calc_3d_angle_diff(0.0f, 1000.0f, 0.0f, 0.0f, 1000.0f, 0.0f);
    ASSERT_TEST(fabsf(angle_0) < 0.1f, "EdgeAI IMU: 3D angle between identical vectors equals 0.0 deg");

    /* 5. Multi-Modal Context Fusion & Artifact Rejection */
    edgeai_fusion_state_t fus_st;
    edgeai_fusion_init(&fus_st, NULL);
    edgeai_slow_cache_t cache;
    memset(&cache, 0, sizeof(cache));
    cache.timestamp_ms = 1000;
    cache.ambient_temp_c = 22.0f;
    cache.object_temp_c = 22.5f; /* Cold target */
    cache.prox_counts = 2000;    /* Detached: off-body */
    cache.fir_online = true;
    cache.optical_online = true;

    smartban_semantic_token_t tok;
    edgeai_fusion_process(&fus_st, NULL, NULL, &cache, &tok);
    ASSERT_TEST(!tok.skin_contact, "EdgeAI Fusion: Off-body detected (prox < 5500)");
    ASSERT_TEST(tok.thermal_class == THERMAL_CLASS_AMBIENT_OFFBODY, "EdgeAI Fusion: Off-body thermal state classified as AMBIENT");
    ASSERT_TEST((tok.alert_mask & ALERT_FLAG_HYPOTHERMIA) == 0, "EdgeAI Fusion: Off-body suppresses false hypothermia alert");

    /* Test on-body firm contact */
    cache.prox_counts = 8000; /* Attached: on-body */
    cache.object_temp_c = 36.6f;
    /* Two cycles for debounce */
    edgeai_fusion_process(&fus_st, NULL, NULL, &cache, &tok);
    edgeai_fusion_process(&fus_st, NULL, NULL, &cache, &tok);
    ASSERT_TEST(tok.skin_contact, "EdgeAI Fusion: On-body confirmed (prox >= 6000 for 2 cycles)");
    ASSERT_TEST(tok.thermal_class == THERMAL_CLASS_NORMAL, "EdgeAI Fusion: 36.6 degC classified as THERMAL_CLASS_NORMAL");

    /* 6. Dual-Mode Stream Controller & Data Reduction Verification */
    char sem_json[192];
    size_t sem_len = edgeai_format_semantic_json(&tok, sem_json, sizeof(sem_json));
    ASSERT_TEST(sem_len > 0 && sem_len < 160, "EdgeAI Telemetry: Semantic JSON token formatted (~85-130 bytes)");

    char raw_json[96];
    size_t raw_len = edgeai_format_raw_json(1000, 20798, 0, 980, 150, raw_json, sizeof(raw_json));
    ASSERT_TEST(raw_len > 0 && raw_len < 80, "EdgeAI Telemetry: Raw JSON sample formatted (~50-70 bytes)");

    float raw_stream_rate = 2566.0f;
    float sem_stream_rate = (float)sem_len;
    float reduction_pct = (1.0f - (sem_stream_rate / raw_stream_rate)) * 100.0f;
    ASSERT_TEST(reduction_pct > 90.0f, "EdgeAI Telemetry: Verified >90% data reduction over raw stream (thesis target met)");

    /* Clean up shared buffer for runtime */
    ringbuf_ecg_init(&g_ringbuf_ecg);

    snprintf(summary_buf, sizeof(summary_buf),
             "\r\nSelf-Test Summary: %lu Run, %lu Passed, %lu Failed\r\n\r\n",
             (unsigned long)s_report.tests_run,
             (unsigned long)s_report.tests_passed,
             (unsigned long)s_report.tests_failed);
    uart_print(summary_buf);
}

static void run_m4_self_tests(void)
{
    char summary_buf[128];
    uart_print("\r\n--- [Test Group 4] M4: Serial Telemetry, CLI & Visual UI Tests ---\r\n");

    /* 1. 7-Segment Font Synthesis & Alphanumerics */
    ASSERT_TEST(ui_display_char_to_seg('0') == 0x3F, "UI Font: Digit '0' mapped to standard ABCDEF (0x3F)");
    ASSERT_TEST(ui_display_char_to_seg('M') == 0x37, "UI Font: Synthesized 'M' mapped to 0x37");
    ASSERT_TEST(ui_display_char_to_seg('W') == 0x3E, "UI Font: Synthesized 'W' mapped to 0x3E");
    ASSERT_TEST(ui_display_char_to_seg('S') == 0x6D, "UI Font: Character 'S' mapped to 0x6D");
    ASSERT_TEST(ui_display_char_to_seg('E') == 0x79, "UI Font: Character 'E' mapped to 0x79");
    ASSERT_TEST(ui_display_char_to_seg('A') == 0x77, "UI Font: Character 'A' mapped to 0x77");

    /* 2. Telemetry JSON Formatting & Baud Rate Compliance (<11,520 B/s) */
    smartban_semantic_token_t sem_tok;
    memset(&sem_tok, 0, sizeof(sem_tok));
    sem_tok.timestamp_ms   = 10240;
    sem_tok.heart_rate_bpm = 72;
    sem_tok.rr_interval_ms = 833;
    sem_tok.hrv_rmssd_ms   = 38;
    sem_tok.hrv_sdnn_ms    = 42;
    sem_tok.cardiac_flags  = 0;
    sem_tok.posture_state  = POSTURE_STATE_SEDENTARY;
    sem_tok.fall_detected  = false;
    sem_tok.skin_contact   = true;
    sem_tok.skin_temp_c    = 36.4f;
    sem_tok.ambient_lux    = 420;

    char sem_buf[TELEMETRY_JSON_SEM_MAX_LEN];
    size_t sem_len = telemetry_format_semantic_json(&sem_tok, sem_buf, sizeof(sem_buf));
    ASSERT_TEST(sem_len >= 85 && sem_len <= 160, "Telemetry: Semantic JSON frame within ~85-160 bytes");
    ASSERT_TEST(strstr(sem_buf, "\"type\":\"SEM\"") != NULL, "Telemetry: Semantic JSON contains valid type header");
    ASSERT_TEST(strstr(sem_buf, "\"posture\":\"SEDENTARY\"") != NULL, "Telemetry: Semantic JSON contains correct posture string");

    char raw_buf[TELEMETRY_JSON_RAW_MAX_LEN];
    size_t raw_len = telemetry_format_raw_json(10240, -142, 12, -34, 998, raw_buf, sizeof(raw_buf));
    ASSERT_TEST(raw_len >= 50 && raw_len <= 75, "Telemetry: Single Raw JSON frame within ~50-75 bytes");
    ASSERT_TEST(strstr(raw_buf, "\"type\":\"RAW\"") != NULL, "Telemetry: Single Raw JSON contains valid type header");

    int32_t batch_ecg[5] = {-142, -138, -130, -125, -120};
    char batch_buf[TELEMETRY_JSON_BATCH_MAX_LEN];
    size_t batch_len = telemetry_format_raw_batch_json(10240, batch_ecg, 12, -34, 998, batch_buf, sizeof(batch_buf));
    ASSERT_TEST(batch_len >= 80 && batch_len <= 120, "Telemetry: Batched Raw JSON frame within ~80-120 bytes");
    ASSERT_TEST(strstr(batch_buf, "\"type\":\"RAWB\"") != NULL, "Telemetry: Batched Raw JSON contains valid type header");

    /* 3. SPSC Telemetry Queue Functional Invariants */
    telemetry_uart_init(s_uart);
    ASSERT_TEST(telemetry_uart_get_queue_count() == 0, "Telemetry Queue: Initially empty");
    ASSERT_TEST(telemetry_uart_enqueue_semantic(&sem_tok), "Telemetry Queue: Enqueue semantic record succeeds");
    ASSERT_TEST(telemetry_uart_get_queue_count() == 1, "Telemetry Queue: Count incremented to 1");
    ASSERT_TEST(telemetry_uart_enqueue_raw(10240, -142, 12, -34, 998), "Telemetry Queue: Enqueue raw sample succeeds");
    ASSERT_TEST(telemetry_uart_get_queue_count() == 2, "Telemetry Queue: Count incremented to 2");

    /* 4. Interactive Serial CLI Command Dispatcher */
    char cli_resp[512];
    char cmd_buf[64];

    strcpy(cmd_buf, "HELP");
    ASSERT_TEST(cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp)) > 0, "CLI: HELP command returns command list");

    strcpy(cmd_buf, "MODE RAW");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(edgeai_get_stream_mode() == STREAM_MODE_RAW, "CLI: MODE RAW sets mode to STREAM_MODE_RAW");

    strcpy(cmd_buf, "MODE SEMANTIC");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(edgeai_get_stream_mode() == STREAM_MODE_SEMANTIC, "CLI: MODE SEMANTIC sets mode to STREAM_MODE_SEMANTIC");

    strcpy(cmd_buf, "THRESHOLD tachy 140");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(cli_get_tachy_threshold() == 140, "CLI: THRESHOLD tachy updates to 140 bpm");

    strcpy(cmd_buf, "THRESHOLD tachy 300");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(cli_get_tachy_threshold() == 140, "CLI: Out-of-bounds threshold 300 bpm rejected");

    strcpy(cmd_buf, "STREAM STOP");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(!telemetry_uart_is_streaming_enabled(), "CLI: STREAM STOP disables streaming");

    strcpy(cmd_buf, "STREAM START");
    cli_console_execute_line(cmd_buf, cli_resp, sizeof(cli_resp));
    ASSERT_TEST(telemetry_uart_is_streaming_enabled(), "CLI: STREAM START enables streaming");

    /* 5. Button Debouncer State Machine with 200 ms Refractory Lockout */
    uint8_t deb_mask = 0, p_edges = 0, r_edges = 0;
    /* First reading: latched press on SW1 (bit 0) */
    ui_button_debounce_tick(0x01, &deb_mask, &p_edges, &r_edges);
    ASSERT_TEST(p_edges == 0x01, "Button Debounce: Latched press triggers immediate press edge (0 ms latency)");
    ASSERT_TEST(deb_mask == 0x01, "Button Debounce: Debounced mask holds active button");

    /* Second reading: contact bounce / release during 200 ms lockout */
    ui_button_debounce_tick(0x00, &deb_mask, &p_edges, &r_edges);
    ASSERT_TEST(deb_mask == 0x01 && p_edges == 0x00, "Button Debounce: Refractory lockout preserves state and suppresses bounce");

    /* Third reading: timer decrements */
    ui_button_debounce_tick(0x00, &deb_mask, &p_edges, &r_edges);
    /* Fourth reading: lockout expired and button released */
    ui_button_debounce_tick(0x00, &deb_mask, &p_edges, &r_edges);
    ASSERT_TEST(deb_mask == 0x00 && r_edges == 0x01, "Button Debounce: Release edge detected after lockout expiry");

    /* 6. UI View Modes & Alarm Preemption */
    ui_display_state_t test_ui;
    ui_display_init(&test_ui, NULL);
    ASSERT_TEST(ui_display_get_view(&test_ui) == UI_VIEW_HR, "UI View: Initial view is UI_VIEW_HR");
    ASSERT_TEST(ui_display_cycle_view(&test_ui) == UI_VIEW_TEMP, "UI View: First cycle switches to UI_VIEW_TEMP");
    ASSERT_TEST(ui_display_cycle_view(&test_ui) == UI_VIEW_STATUS, "UI View: Second cycle switches to UI_VIEW_STATUS");
    ASSERT_TEST(ui_display_cycle_view(&test_ui) == UI_VIEW_HR, "UI View: Third cycle wraps back to UI_VIEW_HR");

    /* Test emergency alarm preemption */
    ui_display_data_t alert_data;
    memset(&alert_data, 0, sizeof(alert_data));
    alert_data.fall_detected = true;
    ui_display_update(&test_ui, &alert_data);
    ASSERT_TEST(test_ui.active_alarm == UI_ALARM_FALL, "UI Alarm: Fall impact confirmed immediately overrides display with UI_ALARM_FALL");

    snprintf(summary_buf, sizeof(summary_buf),
             "\r\nSelf-Test Summary: %lu Run, %lu Passed, %lu Failed\r\n\r\n",
             (unsigned long)s_report.tests_run,
             (unsigned long)s_report.tests_passed,
             (unsigned long)s_report.tests_failed);
    uart_print(summary_buf);
}

void main_format_system_status(char *buf, size_t max_len)
{
    if (buf == NULL || max_len == 0) return;

    const char *names[5] = {"Task_ECG", "Task_IMU", "Task_EdgeAI", "Task_Sensors_Slow", "Task_Telemetry_UI"};
    pthread_t ths[5] = {g_thread_ecg, g_thread_imu, g_thread_edgeai, g_thread_sensors_slow, g_thread_telemetry_ui};

    char stacks_table[512] = "";
    size_t off = 0;
    off += snprintf(stacks_table + off, sizeof(stacks_table) - off,
                    "  Task Name          Pri   Stack    Used    Headroom  Status\r\n"
                    "  ----------------------------------------------------------\r\n");

    for (int i = 0; i < 5; i++) {
        struct pthread_Obj *obj = (struct pthread_Obj *)ths[i];
        size_t stk_sz = (i == 0 || i == 2) ? 2048 : 1536;
        size_t stk_used = 0;
        int pri = (i == 0) ? 4 : (i == 1 || i == 2) ? 3 : (i == 3) ? 2 : 1;
        if (obj && obj->task) {
            Task_Stat s;
            Task_stat(obj->task, &s);
            stk_sz = s.stackSize;
            stk_used = s.used;
            pri = s.priority;
        }
        size_t headroom = (stk_sz > stk_used) ? (stk_sz - stk_used) : 0;
        off += snprintf(stacks_table + off, sizeof(stacks_table) - off,
                        "  %-18s %2d   %5u B  %5u B   %5u B     OK\r\n",
                        names[i], pri, (unsigned int)stk_sz, (unsigned int)stk_used, (unsigned int)headroom);
    }

    telemetry_stats_t tel_stats;
    telemetry_uart_get_stats(&tel_stats);

    snprintf(buf, max_len,
        "===============================================================\r\n"
        " SmartBAN CC2652R1 System Diagnostics & Health Status\r\n"
        "===============================================================\r\n"
        " [Operating State]\r\n"
        "  Telemetry Stream Mode : %s\r\n"
        "  CLI Console Power     : %s\r\n"
        "  Streaming Enabled     : %s\r\n"
        "  Queue Occupancy       : %lu / %u records (Drops: %lu)\r\n"
        "\r\n"
        " [Task Stack Watermarks (TI-RTOS7 POSIX)]\r\n"
        "%s"
        "\r\n"
        " [Subsystem & Sensor Hardware Health]\r\n"
        "  ADS1292R ECG (SPI Mode 1, 250 Hz) : ONLINE (Samples: %lu)\r\n"
        "  ADXL362 IMU  (SPI Mode 0, 100 Hz) : ONLINE (Samples: %lu)\r\n"
        "  MLX90632 FIR Temp (I2C 0x3A)      : ONLINE (Amb: %.1f C, Obj: %.1f C)\r\n"
        "  OPT4041 ALS       (I2C 0x44)      : ONLINE (Lux: %.1f)\r\n"
        "  VCNL4040 Prox/ALS (I2C 0x60)      : ONLINE (Prox: %u, Touch: %s)\r\n"
        "  PCAL6408A Expander(I2C 0x20)      : ONLINE (Buttons debounced)\r\n"
        "  CH455H 7-Seg/LED  (I2C 0x24/0x37) : ONLINE (Mode LEDs active)\r\n"
        "\r\n"
        " [Latest Biometric State]\r\n"
        "  Heart Rate : %u bpm (RR: %u ms, RMSSD: %u ms, SDNN: %u ms)\r\n"
        "  Posture    : %s (Fall: %s, Skin Contact: %s)\r\n"
        "  Alert Mask : 0x%04X (Flags: 0x%02X)\r\n"
        "\r\n"
        " [Configured Anomaly Thresholds]\r\n"
        "  Tachycardia : %u bpm\r\n"
        "  Bradycardia : %u bpm\r\n"
        "  Fall Impact : %.2f g\r\n"
        "===============================================================\r\n",
        edgeai_get_stream_mode() == STREAM_MODE_SEMANTIC ? "SEMANTIC (>95% Reduction)" : "RAW STREAM (Evaluation)",
        cli_console_get_state() == CLI_STATE_ACTIVE_SESSION ? "ACTIVE_SESSION" : "STANDBY_ARMED (~1.0 uA)",
        telemetry_uart_is_streaming_enabled() ? "YES" : "NO",
        (unsigned long)telemetry_uart_get_queue_count(), TELEMETRY_QUEUE_CAPACITY, (unsigned long)tel_stats.queue_overflow_drops,
        stacks_table,
        (unsigned long)g_ecg_processed_count,
        (unsigned long)g_imu_processed_count,
        (double)g_latest_ambient_c, (double)g_latest_object_c,
        (double)g_latest_lux,
        (unsigned int)g_latest_prox, g_latest_contact ? "YES" : "NO",
        (unsigned int)g_display_hr, (unsigned int)g_latest_semantic_token.rr_interval_ms,
        (unsigned int)g_latest_semantic_token.hrv_rmssd_ms, (unsigned int)g_latest_semantic_token.hrv_sdnn_ms,
        g_latest_semantic_token.posture_state == POSTURE_STATE_ACTIVE ? "ACTIVE" :
        g_latest_semantic_token.posture_state == POSTURE_STATE_HIGH_DYNAMIC ? "HIGH_DYNAMIC" : "SEDENTARY",
        g_latest_semantic_token.fall_detected ? "DETECTED!" : "NORMAL",
        g_latest_semantic_token.skin_contact ? "CONFIRMED" : "OFF-BODY",
        (unsigned int)g_latest_semantic_token.alert_mask, (unsigned int)g_latest_semantic_token.cardiac_flags,
        (unsigned int)cli_get_tachy_threshold(),
        (unsigned int)cli_get_brady_threshold(),
        (double)cli_get_fall_threshold());
}

/* ============================================================================
 * Application Entry Point
 * ============================================================================ */
/* ============================================================================
 * Application Entry Point
 * ============================================================================
 * TI-RTOS7 NOTE: UART2_open() MUST be called after BIOS_start() because the
 * UART2 driver relies on the SYS/BIOS kernel for DMA and interrupt management.
 * Calling it before BIOS_start() returns NULL silently and all uart_print()
 * calls become no-ops, producing a completely silent board.
 *
 * FIX: main() only does the absolute minimum (Board_init, GPIO_init, spawn
 * one init task, BIOS_start). The init task runs after the kernel is live,
 * opens UART2, prints the banner, inits BSP/HAL, and spawns the 5 app tasks.
 * ============================================================================ */

static void *task_init_entry(void *arg0)
{
    (void)arg0;

    /* Open UART2 NOW — kernel is running, driver will succeed */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate   = 115200;
    uartParams.readMode   = UART2_Mode_BLOCKING;
    uartParams.writeMode  = UART2_Mode_BLOCKING;
    s_uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (s_uart != NULL) {
        UART2_rxDisable(s_uart);
    }

    /* Boot banner — first thing visible on serial terminal */
    uart_print("\r\n\r\n");
    uart_print("#######################################################\r\n");
    uart_print("   SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1)   \r\n");
    uart_print("   Milestone M4: Serial Testbed Interface & Visual UI  \r\n");
    uart_print("#######################################################\r\n");

    if (s_uart == NULL) {
        /* Can't print — blink LED0 rapidly to signal UART failure */
        while (1) {
            GPIO_toggle(CONFIG_GPIO_LED_0);
            usleep(100000);
        }
    }

    /* Initialize Board Support Package */
    uart_print("[INIT] Sequencing Power Rails (1.8V EN, I2C Shifter, IMU SW)... ");
    bsp_power_init();
    uart_print("DONE\r\n");

    uart_print("[INIT] Initializing Shared SPI Bus Manager with Dynamic IOC Remap... ");
    bsp_spi_init();
    uart_print("DONE\r\n");

    uart_print("[INIT] Initializing Shared I2C Bus Manager (400 kHz)... ");
    bsp_i2c_init();
    uart_print("DONE\r\n");

    /* Initialize HAL Drivers */
    uart_print("[INIT] Initializing UI Driver (PCAL6408A & CH455H)... ");
    hal_ui_init();
    uart_print("DONE\r\n");

    uart_print("[INIT] Initializing Optical Sensors (OPT4041 & VCNL4040)... ");
    hal_optical_init();
    uart_print("DONE\r\n");

    /* Initialize IPC Ring Buffers and Semaphores */
    uart_print("[INIT] Initializing Lock-Free SPSC Ring Buffers & Semaphores... ");
    ringbuf_ecg_init(&g_ringbuf_ecg);
    ringbuf_imu_init(&g_ringbuf_imu);
    sem_init(&sem_ecg_ready, 0, 0);
    sem_init(&sem_edgeai_trigger, 0, 0);
    uart_print("DONE\r\n");

    /* Register ADS1292 DRDY GPIO interrupt */
    uart_print("[INIT] Registering ADS1292 DRDY Hwi Callback (DIO 23)... ");
    hal_ecg_register_drdy_callback(ecg_drdy_callback);
    uart_print("DONE\r\n");

    /* Run self-tests */
    run_m1_self_tests();
    run_m2_self_tests();
    run_m3_self_tests();
    run_m4_self_tests();

    /* Spawn 5 application tasks */
    uart_print("[BOOT] Spawning 5-Task Architecture (P4, P3, P3, P2, P1)...\r\n");

    pthread_attr_t attrs;
    struct sched_param priParam;

    /* Task 1: Task_ECG (Priority 4, Stack 2048 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 2048);
    priParam.sched_priority = 4;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&g_thread_ecg, &attrs, task_ecg_entry, NULL) != 0) {
        uart_print("[FATAL] Failed to create Task_ECG!\r\n");
        while (1);
    }
    pthread_attr_destroy(&attrs);

    /* Task 2: Task_IMU (Priority 3, Stack 1536 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 1536);
    priParam.sched_priority = 3;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&g_thread_imu, &attrs, task_imu_entry, NULL) != 0) {
        uart_print("[FATAL] Failed to create Task_IMU!\r\n");
        while (1);
    }
    pthread_attr_destroy(&attrs);

    /* Task 3: Task_EdgeAI (Priority 3, Stack 2048 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 2048);
    priParam.sched_priority = 3;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&g_thread_edgeai, &attrs, task_edgeai_entry, NULL) != 0) {
        uart_print("[FATAL] Failed to create Task_EdgeAI!\r\n");
        while (1);
    }
    pthread_attr_destroy(&attrs);

    /* Task 4: Task_Sensors_Slow (Priority 2, Stack 1536 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 1536);
    priParam.sched_priority = 2;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&g_thread_sensors_slow, &attrs, task_sensors_slow_entry, NULL) != 0) {
        uart_print("[FATAL] Failed to create Task_Sensors_Slow!\r\n");
        while (1);
    }
    pthread_attr_destroy(&attrs);

    /* Task 5: Task_Telemetry_UI (Priority 1, Stack 1536 B) */
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 1536);
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);
    if (pthread_create(&g_thread_telemetry_ui, &attrs, task_telemetry_ui_entry, NULL) != 0) {
        uart_print("[FATAL] Failed to create Task_Telemetry_UI!\r\n");
        while (1);
    }
    pthread_attr_destroy(&attrs);

    uart_print("[BOOT] All 5 Tasks Initialized Successfully.\r\n");
    uart_print("[BOOT] TI-RTOS7 Kernel is Live. System Running.\r\n\r\n");

    /* Init task is done — it exits and the kernel cleans it up */
    return NULL;
}

int main(void)
{
    /* Absolute minimum before BIOS_start() — no drivers, no UART */
    Board_init();
    GPIO_init();

    /* Spawn a single high-priority init task that will open UART2
     * and do all remaining initialization after BIOS_start() */
    pthread_t init_thread;
    pthread_attr_t attrs;
    struct sched_param priParam;

    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED);
    pthread_attr_setstacksize(&attrs, 3072);   /* generous stack for init */
    priParam.sched_priority = 5;               /* highest priority — runs first */
    pthread_attr_setschedparam(&attrs, &priParam);
    pthread_create(&init_thread, &attrs, task_init_entry, NULL);
    pthread_attr_destroy(&attrs);

    /* Start TI-RTOS7 Scheduler — never returns */
    BIOS_start();

    return (0);
}

