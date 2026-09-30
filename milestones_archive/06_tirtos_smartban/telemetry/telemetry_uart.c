/*
 * ============================================================================
 * telemetry_uart.c
 * SmartBAN TI-RTOS7 Telemetry Serial Streamer & Serialization Module
 * Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (SDK 8.33, TI-RTOS7)
 * Milestone: M4 (Serial Testbed Interface & Interactive CLI)
 * ============================================================================
 */

#include "telemetry/telemetry_uart.h"
#include <stdio.h>
#include <string.h>

/* SPSC lock-free circular queue and state */
static telemetry_queue_t s_queue;
static telemetry_stats_t s_stats;
static UART2_Handle      s_uart_handle = NULL;
static volatile bool     s_streaming_enabled = true;

_Static_assert((TELEMETRY_QUEUE_CAPACITY & (TELEMETRY_QUEUE_CAPACITY - 1U)) == 0,
               "TELEMETRY_QUEUE_CAPACITY must be a power of two");

bool telemetry_uart_init(UART2_Handle uart_handle)
{
    if (uart_handle == NULL) {
        return false;
    }

    s_uart_handle = uart_handle;
    s_queue.head = 0;
    s_queue.tail = 0;
    s_queue.overflow_count = 0;
    memset(&s_stats, 0, sizeof(telemetry_stats_t));
    s_streaming_enabled = true;

    return true;
}

void telemetry_uart_set_streaming_enabled(bool enable)
{
    s_streaming_enabled = enable;
}

bool telemetry_uart_is_streaming_enabled(void)
{
    return s_streaming_enabled;
}

bool telemetry_uart_enqueue_semantic(const smartban_semantic_token_t *token)
{
    if (token == NULL) {
        return false;
    }

    uint32_t current_head = s_queue.head;
    uint32_t current_tail = s_queue.tail;

    if ((current_head - current_tail) >= TELEMETRY_QUEUE_CAPACITY) {
        s_queue.overflow_count++;
        s_stats.queue_overflow_drops++;
        return false;
    }

    telemetry_record_t *rec = &s_queue.buffer[current_head & TELEMETRY_QUEUE_MASK];
    rec->type = TELEMETRY_TYPE_SEMANTIC;
    rec->payload.semantic = *token;

    __asm__ __volatile__("dmb" ::: "memory");
    s_queue.head = current_head + 1U;

    return true;
}

bool telemetry_uart_enqueue_raw(uint32_t ts_ms, int32_t ecg_raw,
                                int16_t ax, int16_t ay, int16_t az)
{
    uint32_t current_head = s_queue.head;
    uint32_t current_tail = s_queue.tail;

    if ((current_head - current_tail) >= TELEMETRY_QUEUE_CAPACITY) {
        s_queue.overflow_count++;
        s_stats.queue_overflow_drops++;
        return false;
    }

    telemetry_record_t *rec = &s_queue.buffer[current_head & TELEMETRY_QUEUE_MASK];
    rec->type = TELEMETRY_TYPE_RAW_SINGLE;
    rec->payload.raw_single.timestamp_ms = ts_ms;
    rec->payload.raw_single.ecg_raw      = ecg_raw;
    rec->payload.raw_single.ax_mg        = ax;
    rec->payload.raw_single.ay_mg        = ay;
    rec->payload.raw_single.az_mg        = az;

    __asm__ __volatile__("dmb" ::: "memory");
    s_queue.head = current_head + 1U;

    return true;
}

bool telemetry_uart_enqueue_raw_batch(uint32_t ts_ms, const int32_t ecg_5samples[5],
                                      int16_t ax, int16_t ay, int16_t az)
{
    if (ecg_5samples == NULL) {
        return false;
    }

    uint32_t current_head = s_queue.head;
    uint32_t current_tail = s_queue.tail;

    if ((current_head - current_tail) >= TELEMETRY_QUEUE_CAPACITY) {
        s_queue.overflow_count++;
        s_stats.queue_overflow_drops++;
        return false;
    }

    telemetry_record_t *rec = &s_queue.buffer[current_head & TELEMETRY_QUEUE_MASK];
    rec->type = TELEMETRY_TYPE_RAW_BATCH;
    rec->payload.raw_batch.timestamp_ms = ts_ms;
    for (int i = 0; i < 5; i++) {
        rec->payload.raw_batch.ecg_samples[i] = ecg_5samples[i];
    }
    rec->payload.raw_batch.ax_mg = ax;
    rec->payload.raw_batch.ay_mg = ay;
    rec->payload.raw_batch.az_mg = az;

    __asm__ __volatile__("dmb" ::: "memory");
    s_queue.head = current_head + 1U;

    return true;
}

uint32_t telemetry_uart_process_tx(void)
{
    if (s_uart_handle == NULL || !s_streaming_enabled) {
        return 0;
    }

    char tx_buf[TELEMETRY_JSON_SEM_MAX_LEN];
    uint32_t frames_sent = 0;
    const uint32_t max_burst = 8U; /* Limit processing per invocation to preserve task responsiveness */

    while (frames_sent < max_burst) {
        uint32_t current_head = s_queue.head;
        uint32_t current_tail = s_queue.tail;

        if (current_head == current_tail) {
            break; /* Queue is empty */
        }

        __asm__ __volatile__("dmb" ::: "memory");
        telemetry_record_t rec = s_queue.buffer[current_tail & TELEMETRY_QUEUE_MASK];
        __asm__ __volatile__("dmb" ::: "memory");
        s_queue.tail = current_tail + 1U;

        size_t len = 0;
        if (rec.type == TELEMETRY_TYPE_SEMANTIC) {
            len = telemetry_format_semantic_json(&rec.payload.semantic, tx_buf, sizeof(tx_buf));
        } else if (rec.type == TELEMETRY_TYPE_RAW_SINGLE) {
            len = telemetry_format_raw_json(rec.payload.raw_single.timestamp_ms,
                                            rec.payload.raw_single.ecg_raw,
                                            rec.payload.raw_single.ax_mg,
                                            rec.payload.raw_single.ay_mg,
                                            rec.payload.raw_single.az_mg,
                                            tx_buf, sizeof(tx_buf));
        } else if (rec.type == TELEMETRY_TYPE_RAW_BATCH) {
            len = telemetry_format_raw_batch_json(rec.payload.raw_batch.timestamp_ms,
                                                  rec.payload.raw_batch.ecg_samples,
                                                  rec.payload.raw_batch.ax_mg,
                                                  rec.payload.raw_batch.ay_mg,
                                                  rec.payload.raw_batch.az_mg,
                                                  tx_buf, sizeof(tx_buf));
        }

        if (len > 0) {
            size_t written = 0;
            int_fast16_t status = UART2_write(s_uart_handle, tx_buf, len, &written);
            if (status == UART2_STATUS_SUCCESS && written == len) {
                s_stats.total_bytes_sent += (uint32_t)written;
                if (rec.type == TELEMETRY_TYPE_SEMANTIC) {
                    s_stats.semantic_frames_sent++;
                } else {
                    s_stats.raw_frames_sent++;
                }
                frames_sent++;
            } else {
                s_stats.uart_write_errors++;
            }
        }
    }

    return frames_sent;
}

void telemetry_uart_get_stats(telemetry_stats_t *stats)
{
    if (stats != NULL) {
        *stats = s_stats;
    }
}

void telemetry_uart_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(telemetry_stats_t));
    s_queue.overflow_count = 0;
}

uint32_t telemetry_uart_get_queue_count(void)
{
    uint32_t current_head = s_queue.head;
    uint32_t current_tail = s_queue.tail;
    return (current_head - current_tail);
}

bool telemetry_uart_is_queue_full(void)
{
    return (telemetry_uart_get_queue_count() >= TELEMETRY_QUEUE_CAPACITY);
}

size_t telemetry_format_semantic_json(const smartban_semantic_token_t *token, 
                                      char *buf, size_t max_len)
{
    if (token == NULL || buf == NULL || max_len == 0) {
        return 0;
    }

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

    if (len < 0) {
        return 0;
    }
    if ((size_t)len >= max_len) {
        return max_len - 1;
    }
    return (size_t)len;
}

size_t telemetry_format_raw_json(uint32_t ts_ms, int32_t ecg_raw,
                                 int16_t ax, int16_t ay, int16_t az,
                                 char *buf, size_t max_len)
{
    if (buf == NULL || max_len == 0) {
        return 0;
    }

    int len = snprintf(buf, max_len,
        "{\"type\":\"RAW\",\"ts\":%lu,\"ecg\":%ld,\"ax\":%d,\"ay\":%d,\"az\":%d}\r\n",
        (unsigned long)ts_ms,
        (long)ecg_raw,
        (int)ax,
        (int)ay,
        (int)az);

    if (len < 0) {
        return 0;
    }
    if ((size_t)len >= max_len) {
        return max_len - 1;
    }
    return (size_t)len;
}

size_t telemetry_format_raw_batch_json(uint32_t ts_ms, const int32_t ecg[5],
                                       int16_t ax, int16_t ay, int16_t az,
                                       char *buf, size_t max_len)
{
    if (buf == NULL || ecg == NULL || max_len == 0) {
        return 0;
    }

    int len = snprintf(buf, max_len,
        "{\"type\":\"RAWB\",\"ts\":%lu,\"ecg\":[%ld,%ld,%ld,%ld,%ld],\"ax\":%d,\"ay\":%d,\"az\":%d}\r\n",
        (unsigned long)ts_ms,
        (long)ecg[0], (long)ecg[1], (long)ecg[2], (long)ecg[3], (long)ecg[4],
        (int)ax,
        (int)ay,
        (int)az);

    if (len < 0) {
        return 0;
    }
    if ((size_t)len >= max_len) {
        return max_len - 1;
    }
    return (size_t)len;
}
