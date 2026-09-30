/*
 * ============================================================================
 * ring_buffer.h
 * Lock-Free Single-Producer Single-Consumer (SPSC) Circular Ring Buffer
 * SmartBAN TI-RTOS7 Sensor Node Firmware (CC2652R1 Cortex-M4F)
 * ============================================================================
 */

#ifndef IPC_RING_BUFFER_H_
#define IPC_RING_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hal/hal_ecg.h"
#include "hal/hal_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Type aliases ensuring PROJECT.md interface contract compatibility */
typedef hal_ecg_sample_t ecg_sample_t;
typedef hal_imu_sample_t imu_sample_t;

/* Buffer Capacities (Strictly Power of Two) */
#define RINGBUF_ECG_CAPACITY    128U
#define RINGBUF_ECG_MASK        (RINGBUF_ECG_CAPACITY - 1U)

#define RINGBUF_IMU_CAPACITY    64U
#define RINGBUF_IMU_MASK        (RINGBUF_IMU_CAPACITY - 1U)

/* Compile-time verification of power-of-two capacity */
_Static_assert((RINGBUF_ECG_CAPACITY & (RINGBUF_ECG_CAPACITY - 1U)) == 0,
               "RINGBUF_ECG_CAPACITY must be a power of two");
_Static_assert((RINGBUF_IMU_CAPACITY & (RINGBUF_IMU_CAPACITY - 1U)) == 0,
               "RINGBUF_IMU_CAPACITY must be a power of two");

/* Statistics Tracking Structure with transparent union aliases */
typedef struct {
    union {
        uint32_t pushed_count;
        uint32_t pushed;
    };
    union {
        uint32_t popped_count;
        uint32_t popped;
    };
    union {
        uint32_t dropped_count;
        uint32_t dropped;
    };
    uint32_t peak_utilization;
} ringbuf_stats_t;

/* ECG Ring Buffer Control Block (Footprint: 128 * 24 + 32 = 3,104 Bytes) */
typedef struct {
    ecg_sample_t buffer[RINGBUF_ECG_CAPACITY];
    volatile uint32_t head;     /* Mutated ONLY by Producer (Task_ECG) */
    volatile uint32_t tail;     /* Mutated ONLY by Consumer (Task_EdgeAI) */
    uint32_t capacity;          /* Constant 128 */
    uint32_t mask;              /* Constant 127 (0x7F) */
    ringbuf_stats_t stats;      /* Monotonic performance & overflow metrics */
} ringbuf_ecg_t;

/* IMU Ring Buffer Control Block (Footprint: 64 * 28 + 32 = 1,824 Bytes) */
typedef struct {
    imu_sample_t buffer[RINGBUF_IMU_CAPACITY];
    volatile uint32_t head;     /* Mutated ONLY by Producer (Task_IMU) */
    volatile uint32_t tail;     /* Mutated ONLY by Consumer (Task_EdgeAI) */
    uint32_t capacity;          /* Constant 64 */
    uint32_t mask;              /* Constant 63 (0x3F) */
    ringbuf_stats_t stats;      /* Monotonic performance & overflow metrics */
} ringbuf_imu_t;

/* Compile-time verification of exact struct memory footprint */
_Static_assert(sizeof(ecg_sample_t) == 24, "ecg_sample_t must be 24 bytes");
_Static_assert(sizeof(ringbuf_ecg_t) == 3104, "ringbuf_ecg_t must be 3104 bytes");
_Static_assert(sizeof(imu_sample_t) == sizeof(hal_imu_sample_t), "imu_sample_t must match hal_imu_sample_t");
_Static_assert(sizeof(ringbuf_imu_t) == (sizeof(imu_sample_t) * RINGBUF_IMU_CAPACITY + sizeof(uint32_t) * 4 + sizeof(ringbuf_stats_t)), "ringbuf_imu_t size check");

/* ============================================================================
 * ECG Ring Buffer API
 * ============================================================================ */

/**
 * @brief Initialize ECG ring buffer structure, zeroing indices and stats.
 */
void ringbuf_ecg_init(ringbuf_ecg_t *rb);

/**
 * @brief Reset ECG ring buffer indices and stats.
 *        WARNING: Only invoke when producer and consumer tasks are stopped.
 */
void ringbuf_ecg_reset(ringbuf_ecg_t *rb);

/**
 * @brief Lock-free enqueue of single ECG sample (Producer: Task_ECG).
 * @param rb Pointer to ringbuf_ecg_t.
 * @param sample Pointer to sample to enqueue.
 * @return true if pushed, false if buffer full (sample dropped, stats.dropped incremented).
 */
bool ringbuf_ecg_push(ringbuf_ecg_t *rb, const ecg_sample_t *sample);

/**
 * @brief Lock-free dequeue of single ECG sample (Consumer: Task_EdgeAI).
 * @param rb Pointer to ringbuf_ecg_t.
 * @param sample Pointer to destination sample buffer.
 * @return true if sample popped, false if buffer empty.
 */
bool ringbuf_ecg_pop(ringbuf_ecg_t *rb, ecg_sample_t *sample);

/**
 * @brief Peek at oldest sample without advancing tail pointer.
 * @param rb Pointer to ringbuf_ecg_t.
 * @param sample Destination sample buffer.
 * @return true if sample available, false if buffer empty.
 */
bool ringbuf_ecg_peek(const ringbuf_ecg_t *rb, ecg_sample_t *sample);

/**
 * @brief Batch dequeue of up to max_count ECG samples for block processing.
 * @param rb Pointer to ringbuf_ecg_t.
 * @param dest Destination array.
 * @param max_count Maximum number of samples to dequeue.
 * @return Actual number of samples dequeued.
 */
uint32_t ringbuf_ecg_pop_batch(ringbuf_ecg_t *rb, ecg_sample_t *dest, uint32_t max_count);

/**
 * @brief Query current number of unconsumed ECG samples available.
 */
uint32_t ringbuf_ecg_available(const ringbuf_ecg_t *rb);

/**
 * @brief Query current number of unconsumed ECG samples (alias of available).
 */
uint32_t ringbuf_ecg_count(const ringbuf_ecg_t *rb);

/**
 * @brief Query free space available in ECG buffer.
 */
uint32_t ringbuf_ecg_free_space(const ringbuf_ecg_t *rb);

/**
 * @brief Check if ECG buffer is empty.
 */
bool ringbuf_ecg_is_empty(const ringbuf_ecg_t *rb);

/**
 * @brief Check if ECG buffer is full.
 */
bool ringbuf_ecg_is_full(const ringbuf_ecg_t *rb);

/**
 * @brief Copy out telemetry statistics safely.
 */
void ringbuf_ecg_get_stats(const ringbuf_ecg_t *rb, ringbuf_stats_t *stats_out);

/* ============================================================================
 * IMU Ring Buffer API
 * ============================================================================ */

/**
 * @brief Initialize IMU ring buffer structure, zeroing indices and stats.
 */
void ringbuf_imu_init(ringbuf_imu_t *rb);

/**
 * @brief Reset IMU ring buffer indices and stats.
 */
void ringbuf_imu_reset(ringbuf_imu_t *rb);

/**
 * @brief Lock-free enqueue of single IMU sample (Producer: Task_IMU).
 * @param rb Pointer to ringbuf_imu_t.
 * @param sample Pointer to sample to enqueue.
 * @return true if pushed, false if buffer full (sample dropped, stats.dropped incremented).
 */
bool ringbuf_imu_push(ringbuf_imu_t *rb, const imu_sample_t *sample);

/**
 * @brief Lock-free dequeue of single IMU sample (Consumer: Task_EdgeAI).
 * @param rb Pointer to ringbuf_imu_t.
 * @param sample Pointer to destination sample buffer.
 * @return true if popped, false if empty.
 */
bool ringbuf_imu_pop(ringbuf_imu_t *rb, imu_sample_t *sample);

/**
 * @brief Peek at oldest IMU sample without advancing tail.
 */
bool ringbuf_imu_peek(const ringbuf_imu_t *rb, imu_sample_t *sample);

/**
 * @brief Batch dequeue of up to max_count IMU samples.
 */
uint32_t ringbuf_imu_pop_batch(ringbuf_imu_t *rb, imu_sample_t *dest, uint32_t max_count);

/**
 * @brief Query current number of unconsumed IMU samples available.
 */
uint32_t ringbuf_imu_available(const ringbuf_imu_t *rb);

/**
 * @brief Query current number of unconsumed IMU samples (alias of available).
 */
uint32_t ringbuf_imu_count(const ringbuf_imu_t *rb);

/**
 * @brief Query free space available in IMU buffer.
 */
uint32_t ringbuf_imu_free_space(const ringbuf_imu_t *rb);

/**
 * @brief Check if IMU buffer is empty.
 */
bool ringbuf_imu_is_empty(const ringbuf_imu_t *rb);

/**
 * @brief Check if IMU buffer is full.
 */
bool ringbuf_imu_is_full(const ringbuf_imu_t *rb);

/**
 * @brief Copy out telemetry statistics safely.
 */
void ringbuf_imu_get_stats(const ringbuf_imu_t *rb, ringbuf_stats_t *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* IPC_RING_BUFFER_H_ */
