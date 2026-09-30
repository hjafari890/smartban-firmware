package com.smartban.sensor.repository

import com.smartban.sensor.ble.BleDeviceInfo
import com.smartban.sensor.ble.BleScanner
import com.smartban.sensor.ble.DecodedPacket
import com.smartban.sensor.ble.EcgFilterPipeline
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.model.TelemetrySource
import com.smartban.sensor.net.WifiHubClient
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.sqrt

class SmartBanRepository(
    private val bleScanner: BleScanner,
    val wifiClient: WifiHubClient
) {
    private val scope = CoroutineScope(Dispatchers.Default)

    private val _vitals = MutableStateFlow(SmartBanVitals())
    val vitals: StateFlow<SmartBanVitals> = _vitals.asStateFlow()

    private val _statusMessage = MutableStateFlow("Listening for CC2652R1 SmartBAN Node...")
    val statusMessage: StateFlow<String> = _statusMessage.asStateFlow()

    // 250 Hz Rolling Waveform Buffer (750 samples = 3.0s clinical display)
    private val bufferCapacity = 750
    private val _ecgWaveform = MutableStateFlow(FloatArray(bufferCapacity) { 0f })
    val ecgWaveform: StateFlow<FloatArray> = _ecgWaveform.asStateFlow()

    private val _rpeakIndices = MutableStateFlow<List<Int>>(emptyList())
    val rpeakIndices: StateFlow<List<Int>> = _rpeakIndices.asStateFlow()

    // 5-Stage Clinical Morphology-Preserving Filter matching Firmware 12 & GUI 12
    private val ecgFilter = EcgFilterPipeline(250.0f)

    // Discovered Devices from Scanner
    val discoveredDevices: StateFlow<List<BleDeviceInfo>> = bleScanner.discoveredDevices

    private var wifiPollingJob: Job? = null
    private var lastWifiSuccessMs = 0L

    // IMU EMA Filter States for Zero-Jitter Resting
    private var prevFiltGx = 0f
    private var prevFiltGy = 0f
    private var prevFiltGz = 1f
    private var prevPitch = 0f
    private var prevRoll = 0f

    init {
        // Collect real BLE packets from the CC2652R1 sensor node
        scope.launch {
            bleScanner.packetFlow.collect { packet ->
                val now = System.currentTimeMillis()
                // Wi-Fi takes precedence only if it recently responded; otherwise 100% BLE hardware stream
                val isWifiActive = (now - lastWifiSuccessMs < 2000)

                when (packet) {
                    is DecodedPacket.EcgBatch -> {
                        if (!isWifiActive) {
                            // REAL 250 Hz Hardware ADS1292R biopotential samples directly from node
                            pushEcgSamples(packet.samplesMv, packet.rpeaks)
                            _vitals.value = _vitals.value.copy(
                                isConnected = true,
                                source = TelemetrySource.BLE_BROADCAST,
                                raConnected = packet.raConnected,
                                laConnected = packet.laConnected,
                                lastReceivedMs = now
                            )
                            _statusMessage.value = "Live BLE 250 Hz ADS1292R Stream (Seq #${packet.seq})"
                        }
                    }

                    is DecodedPacket.ImuEnvUpdate -> {
                        if (!isWifiActive) {
                            // REAL 3-Axis ADXL362 IMU & Environmental readings directly from node
                            val rawGx = packet.accelX
                            val rawGy = packet.accelY
                            val rawGz = packet.accelZ
                            val rawPitch = packet.pitchDeg
                            val rawRoll = packet.rollDeg
                            val rawGtot = sqrt(rawGx * rawGx + rawGy * rawGy + rawGz * rawGz)

                            // Table rest detection: near 1g total and small horizontal accel
                            val isTableRest = abs(rawGtot - 1.0f) < 0.08f && abs(rawGx) < 0.12f && abs(rawGy) < 0.12f

                            val filtGx: Float
                            val filtGy: Float
                            val filtGz: Float
                            val filtPitch: Float
                            val filtRoll: Float
                            val finalMotionState: String

                            if (isTableRest) {
                                // Strong smoothing at rest to eliminate MEMS LSB noise & status jitter
                                prevFiltGx = 0.88f * prevFiltGx + 0.12f * rawGx
                                prevFiltGy = 0.88f * prevFiltGy + 0.12f * rawGy
                                prevFiltGz = 0.88f * prevFiltGz + 0.12f * rawGz
                                prevPitch  = if (abs(rawPitch) < 1.5f) 0f else (0.88f * prevPitch + 0.12f * rawPitch)
                                prevRoll   = if (abs(rawRoll) < 1.5f) 0f else (0.88f * prevRoll + 0.12f * rawRoll)
                                filtGx = prevFiltGx
                                filtGy = prevFiltGy
                                filtGz = prevFiltGz
                                filtPitch = prevPitch
                                filtRoll = prevRoll
                                finalMotionState = "Sedentary"
                            } else {
                                // Dynamic motion tracking
                                prevFiltGx = 0.35f * prevFiltGx + 0.65f * rawGx
                                prevFiltGy = 0.35f * prevFiltGy + 0.65f * rawGy
                                prevFiltGz = 0.35f * prevFiltGz + 0.65f * rawGz
                                prevPitch  = 0.35f * prevPitch + 0.65f * rawPitch
                                prevRoll   = 0.35f * prevRoll + 0.65f * rawRoll
                                filtGx = prevFiltGx
                                filtGy = prevFiltGy
                                filtGz = prevFiltGz
                                filtPitch = prevPitch
                                filtRoll = prevRoll
                                finalMotionState = packet.motionState
                            }

                            val calcGtot = sqrt(filtGx * filtGx + filtGy * filtGy + filtGz * filtGz)
                            val calcAoa = acos((filtGz / calcGtot.coerceAtLeast(0.01f)).coerceIn(-1f, 1f)) * (180f / Math.PI.toFloat())

                            _vitals.value = _vitals.value.copy(
                                isConnected = true,
                                source = TelemetrySource.BLE_BROADCAST,
                                accelX = filtGx,
                                accelY = filtGy,
                                accelZ = filtGz,
                                pitchDeg = filtPitch,
                                rollDeg = filtRoll,
                                aoaDeg = calcAoa,
                                gTotal = calcGtot,
                                motionState = finalMotionState,
                                stepCount = packet.steps,
                                fallAlert = packet.fallAlert,
                                pressureHpa = packet.pressureHpa,
                                humidityPct = packet.humidityPct,
                                lux = packet.lux,
                                proximity = packet.proximity,
                                iaqIndex = packet.iaqIndex,
                                co2Ppm = packet.co2Ppm,
                                lastReceivedMs = now
                            )
                        }
                    }

                    is DecodedPacket.VitalsUpdate -> {
                        if (!isWifiActive) {
                            // Merge vitals while preserving real waveform buffer, smoothed IMU, and latest proximity
                            val current = _vitals.value
                            _vitals.value = packet.vitals.copy(
                                accelX = if (current.accelX != 0f) current.accelX else packet.vitals.accelX,
                                accelY = if (current.accelY != 0f) current.accelY else packet.vitals.accelY,
                                accelZ = if (current.accelZ != 1f) current.accelZ else packet.vitals.accelZ,
                                pitchDeg = current.pitchDeg,
                                rollDeg = current.rollDeg,
                                aoaDeg = current.aoaDeg,
                                gTotal = current.gTotal,
                                motionState = if (current.motionState.isNotEmpty()) current.motionState else packet.vitals.motionState,
                                stepCount = if (current.stepCount != 0) current.stepCount else packet.vitals.stepCount,
                                pressureHpa = if (current.pressureHpa != 1013.25f) current.pressureHpa else packet.vitals.pressureHpa,
                                humidityPct = if (current.humidityPct != 45f) current.humidityPct else packet.vitals.humidityPct,
                                lux = if (current.lux != 0f) current.lux else packet.vitals.lux,
                                proximity = if (current.proximity != 0) current.proximity else packet.vitals.proximity,
                                iaqIndex = if (current.iaqIndex != 25f) current.iaqIndex else packet.vitals.iaqIndex,
                                co2Ppm = if (current.co2Ppm != 420f) current.co2Ppm else packet.vitals.co2Ppm,
                                lastReceivedMs = now
                            )
                        }
                    }
                }
            }
        }

    }

    fun connectDevice(address: String) {
        bleScanner.connectDevice(address)
        _vitals.value = _vitals.value.copy(
            isConnected = true,
            deviceAddress = address,
            deviceName = "SmartBAN-Node",
            source = TelemetrySource.BLE_BROADCAST,
            lastReceivedMs = System.currentTimeMillis()
        )
        _statusMessage.value = "Connected to SmartBAN-Node ($address)"
    }

    fun disconnectDevice() {
        bleScanner.disconnectDevice()
        wifiPollingJob?.cancel()
        // Reset rolling waveform buffer to flatline
        _ecgWaveform.value = FloatArray(bufferCapacity) { 0f }
        _rpeakIndices.value = emptyList()
        ecgFilter.reset()
        // Reset telemetry vitals to clean disconnected standby state
        _vitals.value = SmartBanVitals(
            isConnected = false,
            source = TelemetrySource.DISCONNECTED,
            lastReceivedMs = 0L
        )
        _statusMessage.value = "Disconnected. Tap 'SEARCH NODE' to connect."
    }

    fun startWifiPolling() {
        wifiPollingJob?.cancel()
        wifiPollingJob = scope.launch {
            while (isActive) {
                val data = wifiClient.fetchData()
                val now = System.currentTimeMillis()
                if (data != null) {
                    lastWifiSuccessMs = now
                    _vitals.value = data
                    _statusMessage.value = "Connected to Wi-Fi Hub (${wifiClient.getBaseUrl()}) - 250 Hz Stream"

                    if (data.ecgSamples.isNotEmpty()) {
                        pushEcgSamples(data.ecgSamples, data.ecgRpeaks)
                    }
                    delay(40)
                } else {
                    if (now - lastWifiSuccessMs > 3000) {
                        if (!_vitals.value.isConnected || _vitals.value.source == TelemetrySource.WIFI_HUB) {
                            _statusMessage.value = "Wi-Fi Hub Offline. Streaming live via Direct BLE 5.2..."
                        }
                    }
                    delay(400)
                }
            }
        }
    }

    private fun pushEcgSamples(newSamples: List<Float>, rpeaks: List<Int>) {
        val current = _ecgWaveform.value
        val n = newSamples.size
        if (n == 0) return

        // 5-Stage Clinical Morphology-Preserving Filter matching Firmware 12 & GUI 12
        val filtered = FloatArray(n)
        for (i in 0 until n) {
            filtered[i] = ecgFilter.process(newSamples[i])
        }

        val updated = FloatArray(bufferCapacity)

        if (n >= bufferCapacity) {
            for (i in 0 until bufferCapacity) {
                updated[i] = filtered[n - bufferCapacity + i]
            }
        } else {
            System.arraycopy(current, n, updated, 0, bufferCapacity - n)
            for (i in 0 until n) {
                updated[bufferCapacity - n + i] = filtered[i]
            }
        }
        _ecgWaveform.value = updated

        if (rpeaks.isNotEmpty()) {
            val valid = rpeaks.map { it + (bufferCapacity - n) }.filter { it in 0 until bufferCapacity }
            _rpeakIndices.value = valid
        }
    }

    fun triggerPvcBurst() {
        // Immediate local state update for instant UI feedback
        _vitals.value = _vitals.value.copy(
            isCapBurst = true,
            pvcAlert = true,
            slotIndex = 1,
            slotName = "CAP Emergency Burst",
            slice5G = "5G-URLLC (Emergency Low-Latency <1ms)",
            tinyMlClass = 'V',
            tinyMlClassName = "Premature Ventricular Contraction"
        )
        scope.launch {
            wifiClient.sendPvcBurst()
        }
        // Auto-recover after 4 seconds
        scope.launch {
            delay(4000)
            _vitals.value = _vitals.value.copy(
                isCapBurst = false,
                pvcAlert = false,
                slotIndex = 0,
                slotName = "SAP Scheduled (Slot 0)",
                slice5G = "5G-mMTC (High-Efficiency Semantic 1Hz)",
                tinyMlClass = 'N',
                tinyMlClassName = "Normal Sinus Beat"
            )
        }
    }

    fun setPolicy(policyId: Int) {
        val polNames = listOf("Adaptive Hybrid", "Semantic Only", "Raw Continuous")
        val name = polNames.getOrElse(policyId) { "Adaptive Hybrid" }
        _vitals.value = _vitals.value.copy(
            policyId = policyId,
            policyName = name,
            bandwidthSavedPct = if (policyId == 2) 0f else 99.03f,
            powerSavedPct = if (policyId == 2) 0f else 92.8f
        )
        scope.launch {
            wifiClient.setPolicy(policyId)
        }
    }

    fun setNodeMode(modeId: Int) {
        _vitals.value = _vitals.value.copy(
            activeMode = modeId
        )
        scope.launch {
            wifiClient.setNodeMode(modeId)
        }
    }

    fun updateHubIp(ip: String) {
        wifiClient.updateBaseUrl(ip)
        startWifiPolling()
    }
}
