/*
 * ============================================================================
 * hal_ble_radio.h
 * SmartBAN CC2652R1 2.4 GHz Direct RF BLE Real-Time Multi-Modal Broadcaster
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

#define BLE_FRAME_TYPE_VITALS   0x01
#define BLE_FRAME_TYPE_ECG_RAW  0x02
#define BLE_FRAME_TYPE_IMU_ENV  0x03

typedef struct {
    uint8_t  heart_rate_bpm;
    uint16_t rr_interval_ms;
    uint16_t hrv_sdnn_ms;
    uint16_t hrv_rmssd_ms;
    uint8_t  tinyml_class_id; /* 0=N, 1=S, 2=V, 3=F, 4=Q */
    uint8_t  tinyml_conf_pct;
    uint8_t  tinyml_us;
    uint8_t  resp_rpm;
    int8_t   skin_temp_c;
    int8_t   amb_temp_c;
    uint8_t  posture_id;      /* 0=Stand, 1=Sit, 2=Supine, 3=Walk */
    bool     fall_alert;
    bool     pvc_alert;
    uint8_t  smartban_slot;   /* 0..7 */
    bool     is_cap_burst;    /* true during Contention Access Period PVC burst */
    uint8_t  bandwidth_saved_pct;
    uint8_t  active_mode;     /* 1..4 */
    uint8_t  vpp_div10_uv;
    uint8_t  snr_db;
    bool     ra_connected;
    bool     la_connected;
} hal_ble_telemetry_t;

typedef struct {
    int16_t  ax_mg;
    int16_t  ay_mg;
    int16_t  az_mg;
    int16_t  pitch_tenth_deg;
    int16_t  roll_tenth_deg;
    uint16_t steps;
    uint8_t  motion_state;    /* 0=Sedentary, 1=Active, 2=Dynamic */
    bool     fall_alert;
    uint16_t press_tenth_hpa;
    uint8_t  hum_pct;
    uint16_t lux;
    uint8_t  iaq;
    uint16_t co2_ppm;
    uint16_t prox;            /* VCNL4040 optical proximity counts (0..4096) */
} hal_ble_imu_env_t;

/**
 * @brief Broadcast standard BLE advertisement with Complete Local Name "SmartBAN-Node".
 */
bool hal_ble_radio_broadcast_name(void);

/**
 * @brief Initialize CC2652R1 2.4 GHz RF Core in BLE 1 Mbps Broadcaster Mode.
 */
bool hal_ble_radio_init(void);

/**
 * @brief Check if BLE radio driver is active and ready.
 */
bool hal_ble_radio_is_active(void);

/**
 * @brief Broadcast SmartBAN Vitals, TinyML, and MAC Superframe Frame (Type 0x01).
 */
bool hal_ble_radio_broadcast_vitals(const hal_ble_telemetry_t *telem);

/**
 * @brief Broadcast real-time 250 Hz ADS1292R biopotential samples (Type 0x02).
 * @param samples_uv Pointer to 10 consecutive int16_t biopotential samples (in microvolts).
 * @param count Number of samples (up to 10).
 * @param rpeak_mask Bitmask indicating which sample index contains an R-peak (bits 0..9).
 * @param lead_status Contact status (bit 0: RA ok, bit 1: LA ok).
 */
bool hal_ble_radio_broadcast_ecg(const int16_t *samples_uv, uint8_t count, uint16_t rpeak_mask, uint8_t lead_status);

/**
 * @brief Broadcast 3-Axis IMU dynamics and Environmental Sensor metrics (Type 0x03).
 */
bool hal_ble_radio_broadcast_imu_env(const hal_ble_imu_env_t *imu_env);

/**
 * @brief Legacy broadcast wrapper for backwards compatibility.
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
