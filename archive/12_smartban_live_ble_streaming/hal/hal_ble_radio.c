/*
 * ============================================================================
 * hal_ble_radio.c
 * SmartBAN CC2652R1 2.4 GHz Direct RF BLE Real-Time Multi-Modal Broadcaster
 * Target: CC2652R1 LaunchPad (ARM Cortex-M4F + Cortex-M0 RF Core)
 * ============================================================================
 */

#include "hal_ble_radio.h"
#include <string.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/rf_mailbox.h)
#include DeviceFamily_constructPath(driverlib/rf_common_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_cmd.h)
#include DeviceFamily_constructPath(driverlib/rf_ble_mailbox.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_cpe_bt5.h)
#include DeviceFamily_constructPath(rf_patches/rf_patch_mce_bt5.h)
#include <ti/drivers/rf/RF.h>

/* RF Overrides for CC2652R1 2.4 GHz BLE (+5 dBm TX Power, Internal Bias) */
static uint32_t s_bleOverrides[] = {
    0x00F388D3,
    HW_REG_OVERRIDE(0x6024, 0x2E20),
    0x01280263,
    HW_REG_OVERRIDE(0x5328, 0x0000),
    0x00058683,
    HW32_ARRAY_OVERRIDE(0x4004, 1),
    0x38183C30,
    HW_REG_OVERRIDE(0x4064, 0x3C),
    0x00E787E3,
    0x00950803,
    HW32_ARRAY_OVERRIDE(0x4020, 1),
    0x41005F00,
    0xC0040141,
    0x0007DD44,
    0x000D8C73,
    0x001F8B73,
    0xC0040361,
    0x00000000,
    HW_REG_OVERRIDE(0x5320, 0x03C0),
    0x015302A3,
    HW_REG_OVERRIDE(0x50D4, 0x00F9),
    HW_REG_OVERRIDE(0x50E0, 0x0087),
    HW_REG_OVERRIDE(0x50F8, 0x0014),
    (uint32_t)0xFFFFFFFF
};

static RF_Mode s_bleRfMode = {
    .rfMode      = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_bt5,
    .mcePatchFxn = &rf_patch_mce_bt5,
    .rfePatchFxn = 0
};

static rfc_CMD_BLE5_RADIO_SETUP_t s_bleRadioSetup = {
    .commandNo                = CMD_BLE5_RADIO_SETUP,
    .status                   = IDLE,
    .pNextOp                  = NULL,
    .startTime                = 0,
    .startTrigger.triggerType = TRIG_NOW,
    .startTrigger.bEnaCmd     = 0,
    .startTrigger.triggerNo   = 0,
    .startTrigger.pastTrig    = 0,
    .condition.rule           = COND_NEVER,
    .condition.nSkip          = 0,
    .defaultPhy.mainMode      = 0, /* 1 Mbps */
    .defaultPhy.coding        = 0,
    .loDivider                = 0,
    .config.frontEndMode      = 0, /* Differential mode on LAUNCHXL-CC26X2R1 */
    .config.biasMode          = 0, /* Internal bias */
    .config.analogCfgMode     = 0,
    .config.bNoFsPowerUp      = 0,
    .config.bSynthNarrowBand  = 0,
    .txPower                  = 0x7217, /* +5 dBm */
    .pRegOverrideCommon       = s_bleOverrides,
    .pRegOverride1Mbps        = NULL,
    .pRegOverride2Mbps        = NULL,
    .pRegOverrideCoded        = NULL
};

/* Static BLE Device Random Address (EE:4E:01:CC:26:52) in SRAM 4-byte aligned */
static uint16_t s_bleMacAddress[3] __attribute__((aligned(4))) = {
    0x2652, 0x01CC, 0xEE4E
};

/* Scan Response Payload ("SmartBAN-Node") so both active & passive scanners identify the node */
static uint8_t s_scanRspPayload[15] __attribute__((aligned(4))) = {
    0x0E, 0x09, 'S','m','a','r','t','B','A','N','-','N','o','d','e'
};

#define ADV_PAYLOAD_LEN 31
static uint8_t s_advPayload[ADV_PAYLOAD_LEN] __attribute__((aligned(4)));
static uint8_t s_packet_seq = 0;
static uint32_t s_tx_count = 0;
static bool s_ble_active = false;

static RF_Object s_rfObject;
static RF_Handle s_rfHandle = NULL;
static rfc_bleAdvOutput_t s_advOutput __attribute__((aligned(4)));

/* BLE Advertising Parameters */
static rfc_bleAdvPar_t s_bleAdvParams __attribute__((aligned(4))) = {
    .pRxQ                       = NULL,
    .rxConfig.bAutoFlushIgnored = 0,
    .rxConfig.bAutoFlushCrcErr  = 0,
    .rxConfig.bAutoFlushEmpty   = 0,
    .rxConfig.bIncludeLenByte   = 0,
    .rxConfig.bIncludeCrc       = 0,
    .rxConfig.bAppendRssi       = 0,
    .rxConfig.bAppendStatus     = 0,
    .rxConfig.bAppendTimestamp  = 0,
    .advConfig.advFilterPolicy  = 0, /* Allow any scanner */
    .advConfig.deviceAddrType   = 1, /* Random static address (MSB 2 bits = 11 in 0xEE) */
    .advConfig.peerAddrType     = 0,
    .advConfig.bStrictLenFilter = 0,
    .advConfig.chSel            = 0,
    .advConfig.privIgnMode      = 0,
    .advConfig.rpaMode          = 0,
    .advLen                     = ADV_PAYLOAD_LEN,
    .scanRspLen                 = sizeof(s_scanRspPayload),
    .pAdvData                   = s_advPayload,
    .pScanRspData               = s_scanRspPayload,
    .pDeviceAddress             = s_bleMacAddress,
    .pWhiteList                 = NULL,
    .behConfig.scanRspEndType   = 0,
    .__dummy0                   = 0,
    .__dummy1                   = 0,
    .endTrigger.triggerType     = TRIG_NEVER,
    .endTrigger.bEnaCmd         = 0,
    .endTrigger.triggerNo       = 0,
    .endTrigger.pastTrig        = 1,
    .endTime                    = 0
};

/* 3-Channel Chained Scannable Advertising Commands (Ch 37 -> Ch 38 -> Ch 39) */
static rfc_CMD_BLE_ADV_SCAN_t s_cmdBleAdvChain[3] __attribute__((aligned(4)));

bool hal_ble_radio_init(void)
{
    if (s_ble_active) return true;

    for (int i = 0; i < 3; i++) {
        memset(&s_cmdBleAdvChain[i], 0, sizeof(rfc_CMD_BLE_ADV_SCAN_t));
        s_cmdBleAdvChain[i].commandNo                = CMD_BLE_ADV_SCAN;
        s_cmdBleAdvChain[i].status                   = IDLE;
        s_cmdBleAdvChain[i].startTime                = 0;
        s_cmdBleAdvChain[i].startTrigger.triggerType = TRIG_NOW;
        s_cmdBleAdvChain[i].startTrigger.bEnaCmd     = 0;
        s_cmdBleAdvChain[i].startTrigger.triggerNo   = 0;
        s_cmdBleAdvChain[i].startTrigger.pastTrig    = 1;
        s_cmdBleAdvChain[i].channel                  = 37 + i;
        s_cmdBleAdvChain[i].whitening.init           = 0;
        s_cmdBleAdvChain[i].whitening.bOverride      = 0;
        s_cmdBleAdvChain[i].pParams                  = &s_bleAdvParams;
        s_cmdBleAdvChain[i].pOutput                  = &s_advOutput;

        if (i < 2) {
            s_cmdBleAdvChain[i].condition.rule = COND_ALWAYS;
            s_cmdBleAdvChain[i].pNextOp        = (rfc_radioOp_t*)&s_cmdBleAdvChain[i + 1];
        } else {
            s_cmdBleAdvChain[i].condition.rule = COND_NEVER;
            s_cmdBleAdvChain[i].pNextOp        = NULL;
        }
    }

    RF_Params rfParams;
    RF_Params_init(&rfParams);
    rfParams.nInactivityTimeout = 200;

    s_rfHandle = RF_open(&s_rfObject, &s_bleRfMode, (RF_RadioSetup*)&s_bleRadioSetup, &rfParams);
    if (!s_rfHandle) {
        s_ble_active = false;
        return false;
    }

    s_ble_active = true;
    s_tx_count = 0;
    return true;
}

bool hal_ble_radio_is_active(void)
{
    return s_ble_active;
}

uint32_t hal_ble_radio_get_tx_count(void)
{
    return s_tx_count;
}

static bool run_radio_chain(void)
{
    for (int i = 0; i < 3; i++) {
        s_cmdBleAdvChain[i].status = IDLE;
    }
    RF_EventMask result = RF_runCmd(s_rfHandle, (RF_Op*)&s_cmdBleAdvChain[0], RF_PriorityNormal, NULL, 0);
    if (result & RF_EventLastCmdDone) {
        s_tx_count++;
        return true;
    }
    return false;
}

bool hal_ble_radio_broadcast_vitals(const hal_ble_telemetry_t *telem)
{
    if (!s_ble_active || !s_rfHandle || !telem) return false;

    memset(s_advPayload, 0, ADV_PAYLOAD_LEN);
    /* Flags */
    s_advPayload[0] = 0x02;
    s_advPayload[1] = 0x01;
    s_advPayload[2] = 0x06;

    /* Manufacturer Specific Data Header (27 bytes: 1 type + 26 payload) */
    s_advPayload[3] = 0x1B;
    s_advPayload[4] = 0xFF;
    s_advPayload[5] = 0x53; /* 'S' */
    s_advPayload[6] = 0x42; /* 'B' */
    s_advPayload[7] = BLE_FRAME_TYPE_VITALS; /* 0x01 */
    s_advPayload[8] = s_packet_seq++;

    s_advPayload[9]  = telem->heart_rate_bpm;
    s_advPayload[10] = (uint8_t)(telem->rr_interval_ms >> 8);
    s_advPayload[11] = (uint8_t)(telem->rr_interval_ms & 0xFF);
    s_advPayload[12] = telem->tinyml_class_id;
    s_advPayload[13] = telem->tinyml_conf_pct;
    s_advPayload[14] = telem->tinyml_us;
    s_advPayload[15] = telem->resp_rpm;
    s_advPayload[16] = (int8_t)telem->skin_temp_c;
    s_advPayload[17] = (int8_t)telem->amb_temp_c;

    uint8_t status_byte = 0;
    if (telem->fall_alert)   status_byte |= 0x01;
    if (telem->pvc_alert)    status_byte |= 0x02;
    if (telem->is_cap_burst) status_byte |= 0x04;
    status_byte |= ((telem->posture_id & 0x03) << 3);
    status_byte |= ((telem->smartban_slot & 0x07) << 5);
    s_advPayload[18] = status_byte;

    s_advPayload[19] = (uint8_t)((telem->hrv_sdnn_ms > 255) ? 255 : telem->hrv_sdnn_ms);
    s_advPayload[20] = (uint8_t)((telem->hrv_rmssd_ms > 255) ? 255 : telem->hrv_rmssd_ms);
    s_advPayload[21] = telem->bandwidth_saved_pct;
    s_advPayload[22] = telem->active_mode;
    s_advPayload[23] = telem->vpp_div10_uv;
    s_advPayload[24] = telem->snr_db;

    uint8_t leads = 0;
    if (telem->ra_connected) leads |= 0x01;
    if (telem->la_connected) leads |= 0x02;
    s_advPayload[25] = leads;

    return run_radio_chain();
}

bool hal_ble_radio_broadcast_ecg(const int16_t *samples_uv, uint8_t count, uint16_t rpeak_mask, uint8_t lead_status)
{
    if (!s_ble_active || !s_rfHandle || !samples_uv || count == 0) return false;
    if (count > 10) count = 10;

    memset(s_advPayload, 0, ADV_PAYLOAD_LEN);
    /* Flags */
    s_advPayload[0] = 0x02;
    s_advPayload[1] = 0x01;
    s_advPayload[2] = 0x06;

    /* Manufacturer Specific Data Header (27 bytes: 1 type + 26 payload) */
    s_advPayload[3] = 0x1B;
    s_advPayload[4] = 0xFF;
    s_advPayload[5] = 0x53; /* 'S' */
    s_advPayload[6] = 0x42; /* 'B' */
    s_advPayload[7] = BLE_FRAME_TYPE_ECG_RAW; /* 0x02 */
    s_advPayload[8] = s_packet_seq++;

    /* 10 samples * 2 bytes = 20 bytes (offset 9..28) */
    for (uint8_t i = 0; i < count; i++) {
        int16_t val = samples_uv[i];
        s_advPayload[9 + (i * 2)]     = (uint8_t)(val >> 8);
        s_advPayload[9 + (i * 2) + 1] = (uint8_t)(val & 0xFF);
    }

    /* Trailing metadata: lead status and rpeak mask */
    s_advPayload[29] = (lead_status & 0x03) | ((uint8_t)(rpeak_mask & 0x3F) << 2);
    s_advPayload[30] = count;

    return run_radio_chain();
}

bool hal_ble_radio_broadcast_imu_env(const hal_ble_imu_env_t *ie)
{
    if (!s_ble_active || !s_rfHandle || !ie) return false;

    memset(s_advPayload, 0, ADV_PAYLOAD_LEN);
    /* Flags */
    s_advPayload[0] = 0x02;
    s_advPayload[1] = 0x01;
    s_advPayload[2] = 0x06;

    /* Manufacturer Specific Data Header (27 bytes: 1 type + 26 payload) */
    s_advPayload[3] = 0x1B;
    s_advPayload[4] = 0xFF;
    s_advPayload[5] = 0x53; /* 'S' */
    s_advPayload[6] = 0x42; /* 'B' */
    s_advPayload[7] = BLE_FRAME_TYPE_IMU_ENV; /* 0x03 */
    s_advPayload[8] = s_packet_seq++;

    /* 3-Axis Accel */
    s_advPayload[9]  = (uint8_t)(ie->ax_mg >> 8);
    s_advPayload[10] = (uint8_t)(ie->ax_mg & 0xFF);
    s_advPayload[11] = (uint8_t)(ie->ay_mg >> 8);
    s_advPayload[12] = (uint8_t)(ie->ay_mg & 0xFF);
    s_advPayload[13] = (uint8_t)(ie->az_mg >> 8);
    s_advPayload[14] = (uint8_t)(ie->az_mg & 0xFF);

    /* Attitude */
    s_advPayload[15] = (uint8_t)(ie->pitch_tenth_deg >> 8);
    s_advPayload[16] = (uint8_t)(ie->pitch_tenth_deg & 0xFF);
    s_advPayload[17] = (uint8_t)(ie->roll_tenth_deg >> 8);
    s_advPayload[18] = (uint8_t)(ie->roll_tenth_deg & 0xFF);

    /* Steps & Motion */
    s_advPayload[19] = (uint8_t)(ie->steps >> 8);
    s_advPayload[20] = (uint8_t)(ie->steps & 0xFF);
    s_advPayload[21] = (ie->motion_state & 0x03) | (ie->fall_alert ? 0x80 : 0);

    /* Environment */
    s_advPayload[22] = (uint8_t)(ie->press_tenth_hpa >> 8);
    s_advPayload[23] = (uint8_t)(ie->press_tenth_hpa & 0xFF);
    s_advPayload[24] = ie->hum_pct;
    s_advPayload[25] = (uint8_t)(ie->lux >> 8);
    s_advPayload[26] = (uint8_t)(ie->lux & 0xFF);
    s_advPayload[27] = ie->iaq;
    s_advPayload[28] = (uint8_t)((ie->co2_ppm > 2550) ? 255 : (ie->co2_ppm / 10));
    s_advPayload[29] = (uint8_t)(ie->prox >> 8);
    s_advPayload[30] = (uint8_t)(ie->prox & 0xFF);

    return run_radio_chain();
}

bool hal_ble_radio_broadcast_name(void)
{
    if (!s_ble_active || !s_rfHandle) return false;

    memset(s_advPayload, 0, ADV_PAYLOAD_LEN);
    /* 1. Flags AD Structure (3 bytes) */
    s_advPayload[0] = 0x02;
    s_advPayload[1] = 0x01;
    s_advPayload[2] = 0x06;

    /* 2. Complete Local Name AD Structure (14 bytes): "SmartBAN-Node" */
    s_advPayload[3]  = 0x0E;
    s_advPayload[4]  = 0x09; /* GAP_ADTYPE_LOCAL_NAME_COMPLETE */
    s_advPayload[5]  = 'S';
    s_advPayload[6]  = 'm';
    s_advPayload[7]  = 'a';
    s_advPayload[8]  = 'r';
    s_advPayload[9]  = 't';
    s_advPayload[10] = 'B';
    s_advPayload[11] = 'A';
    s_advPayload[12] = 'N';
    s_advPayload[13] = '-';
    s_advPayload[14] = 'N';
    s_advPayload[15] = 'o';
    s_advPayload[16] = 'd';
    s_advPayload[17] = 'e';

    /* 3. Manufacturer Specific Signature AD Structure (13 bytes) */
    s_advPayload[18] = 0x0C;
    s_advPayload[19] = 0xFF;
    s_advPayload[20] = 0x53; /* 'S' */
    s_advPayload[21] = 0x42; /* 'B' */
    s_advPayload[22] = s_packet_seq++;
    s_advPayload[23] = 0x00; /* Discovery Beacon */

    return run_radio_chain();
}

bool hal_ble_radio_broadcast(const hal_ble_telemetry_t *telem)
{
    return hal_ble_radio_broadcast_vitals(telem);
}
