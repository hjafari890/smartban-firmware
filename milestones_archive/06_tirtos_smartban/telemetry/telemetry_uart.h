/*
 * ============================================================================
 * telemetry_uart.h
 * SmartBAN TI-RTOS7 Telemetry Serial Streamer & Serialization Module
 * Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (SDK 8.33, TI-RTOS7)
 * Milestone: M4 (Serial Testbed Interface & Interactive CLI)
 * ============================================================================
 */

#ifndef TELEMETRY_UART_H_
#define TELEMETRY_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <ti/drivers/UART2.h>
#include "edgeai/edgeai_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum buffer sizes for JSON serialization */
#define TELEMETRY_JSON_SEM_MAX_LEN     (192U) /**< Max buffer for Semantic JSON frame */
#define TELEMETRY_JSON_RAW_MAX_LEN     (96U)  /**< Max buffer for Single Raw JSON frame */
#define TELEMETRY_JSON_BATCH_MAX_LEN   (160U) /**< Max buffer for Batched Raw JSON frame */
#define TELEMETRY_QUEUE_CAPACITY       (64U)  /**< Capacity of SPSC telemetry queue (power of 2) */
#define TELEMETRY_QUEUE_MASK           (TELEMETRY_QUEUE_CAPACITY - 1U)

/**
 * @brief Telemetry Record Types queued from Task_EdgeAI to Task_Telemetry_UI.
 */
typedef enum {
    TELEMETRY_TYPE_SEMANTIC = 0, /**< Compact semantic token (~85-115 B) */
    TELEMETRY_TYPE_RAW_SINGLE,   /**< Single raw sample (50 Hz downsampled, ~58 B) */
    TELEMETRY_TYPE_RAW_BATCH     /**< Batched raw frame (5 ECG samples @ 50 Hz = 250 Hz lossless) */
} telemetry_type_t;

/**
 * @brief High-density telemetry sample container for lock-free inter-task transfer.
 */
typedef struct {
    telemetry_type_t type;
    union {
        smartban_semantic_token_t semantic;
        struct {
            uint32_t timestamp_ms;
            int32_t  ecg_raw;
            int16_t  ax_mg;
            int16_t  ay_mg;
            int16_t  az_mg;
        } raw_single;
        struct {
            uint32_t timestamp_ms;
            int32_t  ecg_samples[5];
            int16_t  ax_mg;
            int16_t  ay_mg;
            int16_t  az_mg;
        } raw_batch;
    } payload;
} telemetry_record_t;

/**
 * @brief SPSC lock-free circular queue structure for telemetry records.
 */
typedef struct {
    telemetry_record_t buffer[TELEMETRY_QUEUE_CAPACITY];
    volatile uint32_t  head;
    volatile uint32_t  tail;
    volatile uint32_t  overflow_count;
} telemetry_queue_t;

/**
 * @brief Telemetry Streamer Cumulative Statistics.
 */
typedef struct {
    uint32_t semantic_frames_sent;
    uint32_t raw_frames_sent;
    uint32_t total_bytes_sent;
    uint32_t queue_overflow_drops;
    uint32_t uart_write_errors;
} telemetry_stats_t;

/* ============================================================================
 * Module Initialization & Control APIs
 * ============================================================================ */

/**
 * @brief Initialize the UART2 telemetry streaming subsystem and queues.
 * @param uart_handle Initialized UART2 driver handle.
 * @return true on success; false if handle is NULL.
 */
bool telemetry_uart_init(UART2_Handle uart_handle);

/**
 * @brief Enable or disable automated telemetry frame streaming.
 * @param enable true to start streaming; false to pause (e.g. during CLI sessions).
 */
void telemetry_uart_set_streaming_enabled(bool enable);

/**
 * @brief Query whether automated streaming is currently enabled.
 */
bool telemetry_uart_is_streaming_enabled(void);

/**
 * @brief Enqueue a semantic token for transmission (Called from Task_EdgeAI).
 * @param token Pointer to source semantic token.
 * @return true if enqueued; false if queue is full (drops tracked in stats).
 */
bool telemetry_uart_enqueue_semantic(const smartban_semantic_token_t *token);

/**
 * @brief Enqueue a single raw sample for transmission (Called from Task_EdgeAI).
 * @param ts_ms Acquisition timestamp in milliseconds.
 * @param ecg_raw 24-bit ECG channel 1 raw count.
 * @param ax X-axis acceleration in mg.
 * @param ay Y-axis acceleration in mg.
 * @param az Z-axis acceleration in mg.
 * @return true if enqueued; false if queue is full.
 */
bool telemetry_uart_enqueue_raw(uint32_t ts_ms, int32_t ecg_raw,
                                int16_t ax, int16_t ay, int16_t az);

/**
 * @brief Enqueue a 5-sample batched raw frame (250 Hz lossless over 50 Hz stream).
 * @param ts_ms Acquisition timestamp of first sample.
 * @param ecg_5samples Array of 5 sequential 24-bit ECG channel 1 samples.
 * @param ax X-axis acceleration in mg.
 * @param ay Y-axis acceleration in mg.
 * @param az Z-axis acceleration in mg.
 * @return true if enqueued; false if queue is full.
 */
bool telemetry_uart_enqueue_raw_batch(uint32_t ts_ms, const int32_t ecg_5samples[5],
                                      int16_t ax, int16_t ay, int16_t az);

/**
 * @brief Service pending telemetry transmissions from queue.
 *        Must be called periodically by Task_Telemetry_UI (Priority 1).
 * @return Number of frames transmitted during this invocation.
 */
uint32_t telemetry_uart_process_tx(void);

/**
 * @brief Retrieve current telemetry streaming statistics.
 * @param stats Pointer to destination statistics structure.
 */
void telemetry_uart_get_stats(telemetry_stats_t *stats);

/**
 * @brief Reset telemetry statistics counters.
 */
void telemetry_uart_reset_stats(void);

/**
 * @brief Query number of pending records in the telemetry queue.
 */
uint32_t telemetry_uart_get_queue_count(void);

/**
 * @brief Query whether telemetry queue is currently full.
 */
bool telemetry_uart_is_queue_full(void);

/* ============================================================================
 * High-Speed Formatting APIs
 * ============================================================================ */

/**
 * @brief Fast formatting of Semantic JSON token.
 * @param token Source semantic token pointer.
 * @param buf Destination buffer.
 * @param max_len Size of buffer.
 * @return Formatted length in bytes.
 */
size_t telemetry_format_semantic_json(const smartban_semantic_token_t *token, 
                                      char *buf, size_t max_len);

/**
 * @brief Fast formatting of Single Raw JSON frame (~58 Bytes).
 * @param ts_ms Timestamp in ms.
 * @param ecg_raw 24-bit ECG raw counts.
 * @param ax Acc X in mg.
 * @param ay Acc Y in mg.
 * @param az Acc Z in mg.
 * @param buf Destination buffer.
 * @param max_len Size of buffer.
 * @return Formatted length in bytes.
 */
size_t telemetry_format_raw_json(uint32_t ts_ms, int32_t ecg_raw,
                                 int16_t ax, int16_t ay, int16_t az,
                                 char *buf, size_t max_len);

/**
 * @brief Fast formatting of Batched Raw JSON frame (~98 Bytes).
 * @param ts_ms Timestamp in ms.
 * @param ecg Array of 5 sequential ECG samples.
 * @param ax Acc X in mg.
 * @param ay Acc Y in mg.
 * @param az Acc Z in mg.
 * @param buf Destination buffer.
 * @param max_len Size of buffer.
 * @return Formatted length in bytes.
 */
size_t telemetry_format_raw_batch_json(uint32_t ts_ms, const int32_t ecg[5],
                                       int16_t ax, int16_t ay, int16_t az,
                                       char *buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_UART_H_ */
