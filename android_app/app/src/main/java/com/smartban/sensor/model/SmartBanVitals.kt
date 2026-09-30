package com.smartban.sensor.model

enum class TelemetrySource(val label: String) {
    DISCONNECTED("OFFLINE"),
    BLE_BROADCAST("BLE CONNECTED"),
    WIFI_HUB("WI-FI CONNECTED")
}

data class SmartBanVitals(
    // Connectivity & Source
    val isConnected: Boolean = false,
    val source: TelemetrySource = TelemetrySource.DISCONNECTED,
    val deviceName: String = "SmartBAN-Node",
    val deviceAddress: String = "--:--:--:--:--:--",
    val hubUrl: String = "http://192.168.1.100:8080",
    val rssiDbm: Int = -100,
    val sequenceNumber: Int = 0,
    val activeMode: Int = 4,

    // 1. Live ECG & Biopotentials
    val ecgSamples: List<Float> = emptyList(),
    val ecgRpeaks: List<Int> = emptyList(),
    val vppUv: Float = 0.0f,
    val snrDb: Float = 24.5f,
    val raConnected: Boolean = true,
    val laConnected: Boolean = true,

    // 2. Live Biometrics
    val heartRateBpm: Float = 0.0f,
    val rrIntervalMs: Float = 0.0f,
    val hrvSdnnMs: Float = 0.0f,
    val hrvRmssdMs: Float = 0.0f,
    val respRpm: Float = 0.0f,
    val skinTempC: Float = 0.0f,
    val ambientTempC: Float = 0.0f,

    // 3. 3-Axis IMU Dynamics & Locomotion
    val accelX: Float = 0.0f,
    val accelY: Float = 0.0f,
    val accelZ: Float = 1.0f,
    val pitchDeg: Float = 0.0f,
    val rollDeg: Float = 0.0f,
    val aoaDeg: Float = 0.0f,
    val gTotal: Float = 1.0f,
    val motionState: String = "Sedentary",
    val stepCount: Int = 0,
    val spm: Float = 0.0f,
    val fallAlert: Boolean = false,

    // 4. Optical & Environmental
    val lux: Float = 0.0f,
    val proximity: Int = 0,
    val pressureHpa: Float = 1013.25f,
    val humidityPct: Float = 45.0f,
    val iaqIndex: Float = 25.0f,
    val co2Ppm: Float = 420.0f,
    val altitudeM: Float = 0.0f,

    // 5. On-Chip Int8 TinyML Classifier
    val tinyMlClass: Char = 'N',
    val tinyMlClassName: String = "Normal Sinus (N)",
    val tinyMlConfidencePct: Int = 98,
    val tinyMlLatencyUs: Int = 30,
    val tinyMlCycles: Int = 1440,

    // 6. SmartBAN MAC Superframe (ETSI TS 103 326)
    val ibiNumber: Int = 1,
    val policyId: Int = 0,
    val policyName: String = "Adaptive Hybrid",
    val slotIndex: Int = 0,
    val slotName: String = "SAP Scheduled (Slot 0)",
    val isCapBurst: Boolean = false,
    val pvcAlert: Boolean = false,
    val slice5G: String = "5G-mMTC (SST=3, 5QI=9, 1Hz)",
    val bandwidthSavedPct: Float = 99.03f,
    val powerSavedPct: Float = 92.8f,
    val powerActiveMw: Float = 1.60f,
    val powerBaselineMw: Float = 22.18f,
    val cumulativeSbKb: Float = 0.0f,
    val cumulativeRawKb: Float = 0.0f,

    // Timing
    val lastReceivedMs: Long = 0L
)
