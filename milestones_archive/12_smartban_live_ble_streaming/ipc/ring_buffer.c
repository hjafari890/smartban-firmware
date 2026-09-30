/*
 * ============================================================================
 * ring_buffer.c
 * Lock-Free Single-Producer Single-Consumer (SPSC) Circular Ring Buffer
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * ============================================================================
 */

#include "ipc/ring_buffer.h"
#include <string.h>

/* Architectural Data Memory Barrier & Compiler Memory Fence for ARM Cortex-M4 */
#if defined(__clang__) || defined(__GNUC__)
    #define DMB()  __asm__ __volatile__("dmb" ::: "memory")
#else
    #define DMB()
#endif

/* ============================================================================
 * ECG Ring Buffer Implementation
 * ============================================================================ */

void ringbuf_ecg_init(ringbuf_ecg_t *rb)
{
    if (!rb) return;
    memset(rb, 0, sizeof(ringbuf_ecg_t));
    rb->capacity = RINGBUF_ECG_CAPACITY;
    rb->mask = RINGBUF_ECG_MASK;
}

void ringbuf_ecg_reset(ringbuf_ecg_t *rb)
{
    if (!rb) return;
    rb->head = 0;
    rb->tail = 0;
    memset(&rb->stats, 0, sizeof(ringbuf_stats_t));
}

bool ringbuf_ecg_push(ringbuf_ecg_t *rb, const ecg_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_head = rb->head;
    uint32_t current_tail = rb->tail;

    /* Check if buffer is full */
    if ((current_head - current_tail) >= rb->capacity) {
        rb->stats.dropped_count++;
        return false;
    }

    /* Store sample into circular slot */
    uint32_t index = current_head & rb->mask;
    rb->buffer[index] = *sample;

    /* Data Memory Barrier: commit buffer payload write before updating head */
    DMB();

    /* Advance monotonic head pointer (atomic 32-bit store) */
    rb->head = current_head + 1;

    /* Update metrics */
    rb->stats.pushed_count++;
    uint32_t util = (current_head + 1) - current_tail;
    if (util > rb->stats.peak_utilization) {
        rb->stats.peak_utilization = util;
    }

    return true;
}

bool ringbuf_ecg_pop(ringbuf_ecg_t *rb, ecg_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;

    /* Check if buffer is empty */
    if (current_head == current_tail) {
        return false;
    }

    /* Memory Barrier: ensure head observation precedes payload read */
    DMB();

    /* Read sample from buffer slot */
    uint32_t index = current_tail & rb->mask;
    *sample = rb->buffer[index];

    /* Data Memory Barrier: ensure sample read finishes before advancing tail */
    DMB();

    /* Advance monotonic tail pointer (atomic 32-bit store) */
    rb->tail = current_tail + 1;

    rb->stats.popped_count++;
    return true;
}

bool ringbuf_ecg_peek(const ringbuf_ecg_t *rb, ecg_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;

    if (current_head == current_tail) {
        return false;
    }

    DMB();
    uint32_t index = current_tail & rb->mask;
    *sample = rb->buffer[index];
    return true;
}

uint32_t ringbuf_ecg_pop_batch(ringbuf_ecg_t *rb, ecg_sample_t *dest, uint32_t max_count)
{
    if (!rb || !dest || max_count == 0) return 0;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;
    uint32_t available = current_head - current_tail;

    if (available == 0) return 0;

    uint32_t count = (available < max_count) ? available : max_count;

    DMB();
    for (uint32_t i = 0; i < count; i++) {
        dest[i] = rb->buffer[(current_tail + i) & rb->mask];
    }

    DMB();
    rb->tail = current_tail + count;
    rb->stats.popped_count += count;

    return count;
}

uint32_t ringbuf_ecg_available(const ringbuf_ecg_t *rb)
{
    if (!rb) return 0;
    return (rb->head - rb->tail);
}

uint32_t ringbuf_ecg_count(const ringbuf_ecg_t *rb)
{
    return ringbuf_ecg_available(rb);
}

uint32_t ringbuf_ecg_free_space(const ringbuf_ecg_t *rb)
{
    if (!rb) return 0;
    uint32_t count = rb->head - rb->tail;
    return (count < rb->capacity) ? (rb->capacity - count) : 0;
}

bool ringbuf_ecg_is_empty(const ringbuf_ecg_t *rb)
{
    if (!rb) return true;
    return (rb->head == rb->tail);
}

bool ringbuf_ecg_is_full(const ringbuf_ecg_t *rb)
{
    if (!rb) return false;
    return ((rb->head - rb->tail) >= rb->capacity);
}

void ringbuf_ecg_get_stats(const ringbuf_ecg_t *rb, ringbuf_stats_t *stats_out)
{
    if (!rb || !stats_out) return;
    stats_out->pushed_count = rb->stats.pushed_count;
    stats_out->popped_count = rb->stats.popped_count;
    stats_out->dropped_count = rb->stats.dropped_count;
    stats_out->peak_utilization = rb->stats.peak_utilization;
}

/* ============================================================================
 * IMU Ring Buffer Implementation
 * ============================================================================ */

void ringbuf_imu_init(ringbuf_imu_t *rb)
{
    if (!rb) return;
    memset(rb, 0, sizeof(ringbuf_imu_t));
    rb->capacity = RINGBUF_IMU_CAPACITY;
    rb->mask = RINGBUF_IMU_MASK;
}

void ringbuf_imu_reset(ringbuf_imu_t *rb)
{
    if (!rb) return;
    rb->head = 0;
    rb->tail = 0;
    memset(&rb->stats, 0, sizeof(ringbuf_stats_t));
}

bool ringbuf_imu_push(ringbuf_imu_t *rb, const imu_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_head = rb->head;
    uint32_t current_tail = rb->tail;

    /* Check if buffer is full */
    if ((current_head - current_tail) >= rb->capacity) {
        rb->stats.dropped_count++;
        return false;
    }

    /* Store sample into circular slot */
    uint32_t index = current_head & rb->mask;
    rb->buffer[index] = *sample;

    /* Data Memory Barrier: commit buffer payload write before updating head */
    DMB();

    /* Advance monotonic head pointer (atomic 32-bit store) */
    rb->head = current_head + 1;

    /* Update metrics */
    rb->stats.pushed_count++;
    uint32_t util = (current_head + 1) - current_tail;
    if (util > rb->stats.peak_utilization) {
        rb->stats.peak_utilization = util;
    }

    return true;
}

bool ringbuf_imu_pop(ringbuf_imu_t *rb, imu_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;

    /* Check if buffer is empty */
    if (current_head == current_tail) {
        return false;
    }

    /* Memory Barrier: ensure head observation precedes payload read */
    DMB();

    /* Read sample from buffer slot */
    uint32_t index = current_tail & rb->mask;
    *sample = rb->buffer[index];

    /* Data Memory Barrier: ensure sample read finishes before advancing tail */
    DMB();

    /* Advance monotonic tail pointer (atomic 32-bit store) */
    rb->tail = current_tail + 1;

    rb->stats.popped_count++;
    return true;
}

bool ringbuf_imu_peek(const ringbuf_imu_t *rb, imu_sample_t *sample)
{
    if (!rb || !sample) return false;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;

    if (current_head == current_tail) {
        return false;
    }

    DMB();
    uint32_t index = current_tail & rb->mask;
    *sample = rb->buffer[index];
    return true;
}

uint32_t ringbuf_imu_pop_batch(ringbuf_imu_t *rb, imu_sample_t *dest, uint32_t max_count)
{
    if (!rb || !dest || max_count == 0) return 0;

    uint32_t current_tail = rb->tail;
    uint32_t current_head = rb->head;
    uint32_t available = current_head - current_tail;

    if (available == 0) return 0;

    uint32_t count = (available < max_count) ? available : max_count;

    DMB();
    for (uint32_t i = 0; i < count; i++) {
        dest[i] = rb->buffer[(current_tail + i) & rb->mask];
    }

    DMB();
    rb->tail = current_tail + count;
    rb->stats.popped_count += count;

    return count;
}

uint32_t ringbuf_imu_available(const ringbuf_imu_t *rb)
{
    if (!rb) return 0;
    return (rb->head - rb->tail);
}

uint32_t ringbuf_imu_count(const ringbuf_imu_t *rb)
{
    return ringbuf_imu_available(rb);
}

uint32_t ringbuf_imu_free_space(const ringbuf_imu_t *rb)
{
    if (!rb) return 0;
    uint32_t count = rb->head - rb->tail;
    return (count < rb->capacity) ? (rb->capacity - count) : 0;
}

bool ringbuf_imu_is_empty(const ringbuf_imu_t *rb)
{
    if (!rb) return true;
    return (rb->head == rb->tail);
}

bool ringbuf_imu_is_full(const ringbuf_imu_t *rb)
{
    if (!rb) return false;
    return ((rb->head - rb->tail) >= rb->capacity);
}

void ringbuf_imu_get_stats(const ringbuf_imu_t *rb, ringbuf_stats_t *stats_out)
{
    if (!rb || !stats_out) return;
    stats_out->pushed_count = rb->stats.pushed_count;
    stats_out->popped_count = rb->stats.popped_count;
    stats_out->dropped_count = rb->stats.dropped_count;
    stats_out->peak_utilization = rb->stats.peak_utilization;
}
