/*
 * ============================================================================
 * hal_ble_radio.c
 * SmartBAN CC2652R1 2.4 GHz RF Low Energy Direct Telemetry Broadcaster
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

/* Common RF Overrides for CC2652R1 2.4 GHz BLE (+5 dBm TX Power, Internal Bias) */
static uint32_t s_bleOverridesCommon[] __attribute__((aligned(4))) = {
    /* DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0x3 (DITHER_EN=0 and IPEAK=3) */
    (uint32_t)0x00F388D3,
    /* Bluetooth 5: Set pilot tone length to 20 us Common */
    HW_REG_OVERRIDE(0x6024, 0x2E20),
    /* Bluetooth 5: Compensate for reduced pilot tone length */
    (uint32_t)0x01280263,
    /* Bluetooth 5: Default to no CTE */
    HW_REG_OVERRIDE(0x5328, 0x0000),
    /* Synth: Increase mid code calibration time to 5 us */
    (uint32_t)0x00058683,
    HW32_ARRAY_OVERRIDE(0x4004, 1),
    (uint32_t)0x38183C30,
    /* Bluetooth 5: Move synth start code */
    HW_REG_OVERRIDE(0x4064, 0x3C),
    /* Bluetooth 5: Set DTX gain -5% for 1 Mbps */
    (uint32_t)0x00E787E3,
    /* Bluetooth 5: Set DTX threshold 1 Mbps */
    (uint32_t)0x00950803,
    /* Bluetooth 5: Set DTX gain -2.5% for 2 Mbps */
    (uint32_t)0x00F487F3,
    /* Bluetooth 5: Set DTX threshold 2 Mbps */
    (uint32_t)0x012A0823,
    /* Bluetooth 5: Set synth fine code calibration interval */
    HW32_ARRAY_OVERRIDE(0x4020, 1),
    (uint32_t)0x41005F00,
    (uint32_t)0xC0040141,
    (uint32_t)0x0007DD44,
    /* Bluetooth 5: Set enhanced TX shape */
    (uint32_t)0x000D8C73,
    /* BLE Stack Overrides */
    (uint32_t)0x001F8B73,
    (uint32_t)0xC0040361,
    (uint32_t)0x00000000,
    (uint32_t)0xFFFFFFFF
};

/* 1 Mbps Specific Overrides for CC2652R1 */
static uint32_t s_bleOverrides1Mbps[] __attribute__((aligned(4))) = {
    HW_REG_OVERRIDE(0x5320, 0x03C0),
    (uint32_t)0x015302A3,
    HW_REG_OVERRIDE(0x50D4, 0x00F9),
    HW_REG_OVERRIDE(0x50E0, 0x0087),
    HW_REG_OVERRIDE(0x50F8, 0x0014),
    (uint32_t)0xFFFFFFFF
};

/* RF Mode for Bluetooth Low Energy */
static RF_Mode s_bleRfMode = {
    .rfMode      = RF_MODE_AUTO,
    .cpePatchFxn = &rf_patch_cpe_bt5,
    .mcePatchFxn = &rf_patch_mce_bt5,
    .rfePatchFxn = 0
};

/* CMD_BLE5_RADIO_SETUP for CC2652R1 */
static rfc_CMD_BLE5_RADIO_SETUP_t s_bleRadioSetup __attribute__((aligned(4))) = {
    .commandNo                = CMD_BLE5_RADIO_SETUP,
    .status                   = IDLE,
    .pNextOp                  = NULL,
    .startTime                = 0,
    .startTrigger.triggerType = TRIG_NOW,
    .startTrigger.bEnaCmd     = 0,
    .startTrigger.triggerNo   = 0,
    .startTrigger.pastTrig    = 1,
    .condition.rule           = COND_NEVER,
    .condition.nSkip          = 0,
    .defaultPhy.mainMode      = 0, /* 1 Mbps */
    .defaultPhy.coding        = 0,
    .loDivider                = 0,
    .config.frontEndMode      = 0, /* Differential mode on LAUNCHXL-CC26X2R1 */
    .config.biasMode          = 0, /* Internal bias */
    .config.analogCfgMode     = 0, /* Initial boot configuration */
    .config.bNoFsPowerUp      = 0,
    .config.bSynthNarrowBand  = 0,
    .txPower                  = 0x7217, /* +5 dBm */
    .pRegOverrideCommon       = s_bleOverridesCommon,
    .pRegOverride1Mbps        = s_bleOverrides1Mbps,
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

    /* Initialize 3-channel chained BLE scannable advertiser (2402, 2426, 2480 MHz) */
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
    rfParams.nInactivityTimeout = 200; /* Keep synth warm for 200us between bursts */

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

bool hal_ble_radio_broadcast(const hal_ble_telemetry_t *telem)
{
    if (!s_ble_active || !s_rfHandle || !telem) return false;

    /* Build 31-byte standard BLE Advertising Packet */
    /* 1. Flags AD Structure (3 bytes) */
    s_advPayload[0] = 0x02; /* Length */
    s_advPayload[1] = 0x01; /* GAP_ADTYPE_FLAGS */
    s_advPayload[2] = 0x06; /* LE General Discoverable Mode | BR/EDR Not Supported */

    /* 2. Complete Local Name AD Structure (14 bytes): "SmartBAN-Node" */
    s_advPayload[3]  = 0x0E; /* Length = 13 chars + 1 type byte = 14 (0x0E) */
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

    /* 3. Manufacturer Specific Data AD Structure (13 bytes): Total 3 + 15 + 13 = 31 bytes */
    s_advPayload[18] = 0x0C; /* Length = 12 bytes (1 type + 2 company + 9 payload) */
    s_advPayload[19] = 0xFF; /* GAP_ADTYPE_MANUFACTURER_SPECIFIC */
    s_advPayload[20] = 0x53; /* Company ID LSB: 'S' */
    s_advPayload[21] = 0x42; /* Company ID MSB: 'B' */
    s_advPayload[22] = s_packet_seq++;
    s_advPayload[23] = telem->heart_rate_bpm;
    s_advPayload[24] = (uint8_t)(telem->rr_interval_ms >> 8);
    s_advPayload[25] = (uint8_t)(telem->rr_interval_ms & 0xFF);
    s_advPayload[26] = telem->tinyml_class_id; /* 0=N, 1=S, 2=V, 3=F, 4=Q */
    s_advPayload[27] = telem->resp_rpm;
    s_advPayload[28] = (uint8_t)telem->temp_deg_c;

    uint8_t status_byte = 0;
    if (telem->fall_alert)   status_byte |= 0x01;
    if (telem->pvc_alert)    status_byte |= 0x02;
    if (telem->is_cap_burst) status_byte |= 0x04;
    status_byte |= ((telem->posture_id & 0x03) << 3);
    status_byte |= ((telem->smartban_slot & 0x07) << 5);
    s_advPayload[29] = status_byte;

    s_advPayload[30] = (uint8_t)((telem->hrv_sdnn_ms > 255) ? 255 : telem->hrv_sdnn_ms);

    for (int i = 0; i < 3; i++) {
        s_cmdBleAdvChain[i].status = IDLE;
    }

    /* Execute 3-channel chained broadcast (Ch 37 + Ch 38 + Ch 39) on Cortex-M0 */
    RF_EventMask result = RF_runCmd(s_rfHandle, (RF_Op*)&s_cmdBleAdvChain[0], RF_PriorityNormal, NULL, 0);

    if (result & RF_EventLastCmdDone) {
        s_tx_count += 3;
        return true;
    }

    return false;
}
