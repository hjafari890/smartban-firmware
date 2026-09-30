/*
 * ============================================================================
 * hal_ble_radio.h
 * SmartBAN CC2652R1 2.4 GHz RF Low Energy Direct Telemetry Broadcaster
 * Target: CC2652R1 LaunchPad (ARM Cortex-M4F + Cortex-M0 RF Core)
 * ============================================================================
 */

#ifndef HAL_BLE_RADIO_H_
#define HAL_BLE_RADIO_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  heart_rate_bpm;
    uint16_t rr_interval_ms;
    uint16_t hrv_sdnn_ms;
    uint16_t hrv_rmssd_ms;
    uint8_t  tinyml_class_id; /* 0=N, 1=S, 2=V, 3=F, 4=Q */
    uint8_t  resp_rpm;
    int8_t   temp_deg_c;
    uint8_t  posture_id;      /* 0=Stand, 1=Sit, 2=Supine, 3=Walk */
    bool     fall_alert;
    bool     pvc_alert;
    uint8_t  smartban_slot;   /* 0..7 */
    bool     is_cap_burst;    /* true during Contention Access Period PVC burst */
} hal_ble_telemetry_t;

/**
 * @brief Initialize CC2652R1 2.4 GHz RF Core in BLE 1 Mbps Broadcaster Mode.
 * @return true if RF_open succeeded, false otherwise.
 */
bool hal_ble_radio_init(void);

/**
 * @brief Check if BLE radio driver is active and ready.
 */
bool hal_ble_radio_is_active(void);

/**
 * @brief Update advertisement payload with latest SmartBAN health metrics and transmit
 *        non-connectable advertising packet on BLE Advertising Channels (37, 38, 39).
 *        Non-blocking radio command execution (< 1.5 ms on Cortex-M0).
 * @param telem Pointer to telemetry metrics.
 * @return true on successful RF command submission, false otherwise.
 */
bool hal_ble_radio_broadcast(const hal_ble_telemetry_t *telem);

/**
 * @brief Get total number of BLE advertising packets transmitted over the air.
 */
uint32_t hal_ble_radio_get_tx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_BLE_RADIO_H_ */
