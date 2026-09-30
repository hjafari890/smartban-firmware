/*
 * ============================================================================
 * smartban_mac.h
 * ETSI TS 103 326 SmartBAN Superframe MAC, Adaptive Semantic Controller &
 * Hardware DWT Cycle-Accurate Energy Profiler
 * Target: TI CC2652R1 (ARM Cortex-M4F @ 48 MHz)
 * ============================================================================
 */

#ifndef SMARTBAN_MAC_H_
#define SMARTBAN_MAC_H_

#include <stdint.h>
#include <stdbool.h>
#include "../edgeai/edgeai_ecg.h"
#include "../edgeai/edgeai_imu.h"
#include "../edgeai/edgeai_fusion.h"
#include "../edgeai/edgeai_tinyml.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SMARTBAN_POLICY_ADAPTIVE_HYBRID = 0, /* Default ISMICT Mode: 1Hz Semantic + Auto 5s Raw Burst on Anomaly */
    SMARTBAN_POLICY_SEMANTIC_ONLY   = 1, /* Strict 1Hz Semantic Token Only (92 B/s) */
    SMARTBAN_POLICY_RAW_CONTINUOUS  = 2  /* Continuous 250Hz ECG + 25Hz IMU + 1Hz ENV (9538 B/s) */
} smartban_policy_t;

typedef enum {
    SMARTBAN_SLOT_SAP_SCHEDULED = 0, /* Normal TDMA Scheduled Access Period (1 Hz token) */
    SMARTBAN_SLOT_CAP_EMERGENCY = 1  /* High-Priority Contention Access Period + Raw Waveform Burst */
} smartban_slot_type_t;

typedef struct {
    uint32_t             ibi_sequence;             /* Monotonic SmartBAN Inter-Beacon Interval counter */
    smartban_policy_t    active_policy;            /* Selected policy (Adaptive, Semantic, Raw) */
    smartban_slot_type_t active_slot;              /* SAP (0) or CAP Emergency (1) */
    uint8_t              adaptive_burst_rem_sec;   /* Countdown seconds remaining in anomaly raw burst */
    uint32_t             cpu_active_us_per_sec;    /* Measured Cortex-M4F DSP + TinyML active time (us/s) */
    uint32_t             raw_stream_bytes_sec;     /* Measured Raw stream throughput (Bytes/s) */
    uint32_t             semantic_bytes_sec;       /* Measured Semantic token throughput (Bytes/s) */
    uint32_t             adaptive_bytes_sec;       /* Measured Adaptive Hybrid throughput (Bytes/s) */
    float                power_mw_raw;             /* Empirical RF+MCU power in Raw mode (mW) */
    float                power_mw_semantic;        /* Empirical RF+MCU power in Semantic mode (mW) */
    float                power_mw_adaptive;        /* Empirical RF+MCU power in Adaptive mode (mW) */
    float                energy_uj_per_beat;       /* Energy consumed per cardiac cycle (uJ/beat) */
} smartban_mac_telemetry_t;

void smartban_mac_init(smartban_mac_telemetry_t *mac);
void smartban_mac_set_policy(smartban_mac_telemetry_t *mac, smartban_policy_t policy);
void smartban_mac_step_1hz(smartban_mac_telemetry_t *mac,
                           const edgeai_ecg_result_t *ecg_res,
                           const edgeai_imu_result_t *imu_res,
                           const edgeai_tinyml_result_t *tinyml_res,
                           bool skin_contact);

#ifdef __cplusplus
}
#endif

#endif /* SMARTBAN_MAC_H_ */
