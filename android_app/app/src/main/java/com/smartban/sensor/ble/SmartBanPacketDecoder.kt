package com.smartban.sensor.ble

import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.model.TelemetrySource

sealed class DecodedPacket {
    data class EcgBatch(
        val samplesMv: List<Float>,
        val rpeaks: List<Int>,
        val raConnected: Boolean,
        val laConnected: Boolean,
        val seq: Int
    ) : DecodedPacket()

    data class VitalsUpdate(
        val vitals: SmartBanVitals
    ) : DecodedPacket()

    data class ImuEnvUpdate(
        val accelX: Float,
        val accelY: Float,
        val accelZ: Float,
        val pitchDeg: Float,
        val rollDeg: Float,
        val steps: Int,
        val motionState: String,
        val fallAlert: Boolean,
        val pressureHpa: Float,
        val humidityPct: Float,
        val lux: Float,
        val iaqIndex: Float,
        val co2Ppm: Float,
        val proximity: Int
    ) : DecodedPacket()
}

object SmartBanPacketDecoder {

    private val CLASS_CHARS = charArrayOf('N', 'S', 'V', 'F', 'Q')
    private val CLASS_NAMES = arrayOf(
        "Normal Sinus (N)",
        "Supraventricular Ectopic (S)",
        "Premature Ventricular Contraction (V)",
        "Ventricular Fusion (F)",
        "Artifact / Noise (Q)"
    )
    private val POSTURES = arrayOf("Sedentary", "Active Motion", "High Dynamic", "Walking")

    fun decodePacket(rawBytes: ByteArray, rssi: Int, address: String): DecodedPacket? {
        var idx = 0
        while (idx < rawBytes.size) {
            val len = rawBytes[idx].toInt() and 0xFF
            if (len == 0 || idx + len >= rawBytes.size) break
            val type = rawBytes[idx + 1].toInt() and 0xFF

            if (type == 0xFF && len >= 4) { // Manufacturer Specific Data
                val compLsb = rawBytes[idx + 2].toInt() and 0xFF
                val compMsb = rawBytes[idx + 3].toInt() and 0xFF

                if (compLsb == 0x53 && compMsb == 0x42) { // "SB" SmartBAN Signature
                    val frameType = rawBytes[idx + 4].toInt() and 0xFF

                    // =========================================================
                    // 1. FRAME TYPE 0x02: Live 250 Hz Raw ECG Biopotential Batch
                    // =========================================================
                    if (frameType == 0x02 && len >= 25) {
                        val seq = rawBytes[idx + 5].toInt() and 0xFF
                        val count = if (idx + 27 < rawBytes.size) (rawBytes[idx + 27].toInt() and 0xFF).coerceIn(1, 10) else 10
                        val samples = mutableListOf<Float>()

                        for (i in 0 until count) {
                            val byteOffset = idx + 6 + (i * 2)
                            if (byteOffset + 1 < rawBytes.size) {
                                val hi = rawBytes[byteOffset].toInt() and 0xFF
                                val lo = rawBytes[byteOffset + 1].toInt() and 0xFF
                                val val16 = ((hi shl 8) or lo).toShort()
                                // Convert microvolts to millivolts
                                samples.add(val16.toFloat() / 1000.0f)
                            }
                        }

                        val metaByte = if (idx + 26 < rawBytes.size) (rawBytes[idx + 26].toInt() and 0xFF) else 0x03
                        val raOk = (metaByte and 0x01) != 0
                        val laOk = (metaByte and 0x02) != 0
                        val rpeakMask = (metaByte shr 2) and 0x3F

                        val rpeaks = mutableListOf<Int>()
                        for (i in 0 until count) {
                            if ((rpeakMask and (1 shl i)) != 0) {
                                rpeaks.add(i)
                            }
                        }

                        return DecodedPacket.EcgBatch(
                            samplesMv = samples,
                            rpeaks = rpeaks,
                            raConnected = raOk,
                            laConnected = laOk,
                            seq = seq
                        )
                    }

                    // =========================================================
                    // 2. FRAME TYPE 0x03: Live 3-Axis IMU & Environmental
                    // =========================================================
                    if (frameType == 0x03 && len >= 22) {
                        fun getShort(offset: Int): Short {
                            if (offset + 1 >= rawBytes.size) return 0
                            val hi = rawBytes[offset].toInt() and 0xFF
                            val lo = rawBytes[offset + 1].toInt() and 0xFF
                            return ((hi shl 8) or lo).toShort()
                        }
                        fun getUShort(offset: Int): Int {
                            if (offset + 1 >= rawBytes.size) return 0
                            val hi = rawBytes[offset].toInt() and 0xFF
                            val lo = rawBytes[offset + 1].toInt() and 0xFF
                            return (hi shl 8) or lo
                        }

                        val axMg = getShort(idx + 6)
                        val ayMg = getShort(idx + 8)
                        val azMg = getShort(idx + 10)
                        val pitchTenth = getShort(idx + 12)
                        val rollTenth = getShort(idx + 14)
                        val steps = getUShort(idx + 16)
                        val motionByte = if (idx + 18 < rawBytes.size) rawBytes[idx + 18].toInt() and 0xFF else 0
                        val motionCode = motionByte and 0x03
                        val fallAlert = (motionByte and 0x80) != 0

                        val pressTenth = getUShort(idx + 19)
                        val hum = if (idx + 21 < rawBytes.size) rawBytes[idx + 21].toInt() and 0xFF else 0
                        val lux = getUShort(idx + 22)
                        val iaq = if (idx + 24 < rawBytes.size) rawBytes[idx + 24].toInt() and 0xFF else 0
                        val co2 = if (idx + 25 < rawBytes.size) (rawBytes[idx + 25].toInt() and 0xFF) * 10 else 0
                        val prox = getUShort(idx + 26)

                        val actState = if (motionCode in POSTURES.indices) POSTURES[motionCode] else "Sedentary"

                        return DecodedPacket.ImuEnvUpdate(
                            accelX = axMg / 1000.0f,
                            accelY = ayMg / 1000.0f,
                            accelZ = azMg / 1000.0f,
                            pitchDeg = pitchTenth / 10.0f,
                            rollDeg = rollTenth / 10.0f,
                            steps = steps,
                            motionState = actState,
                            fallAlert = fallAlert,
                            pressureHpa = pressTenth / 10.0f,
                            humidityPct = hum.toFloat(),
                            lux = lux.toFloat(),
                            iaqIndex = iaq.toFloat(),
                            co2Ppm = co2.toFloat(),
                            proximity = prox
                        )
                    }

                    // =========================================================
                    // 3. FRAME TYPE 0x01: Vitals, TinyML & MAC Superframe
                    // =========================================================
                    if (frameType == 0x01 && len >= 18) {
                        val seq = rawBytes[idx + 5].toInt() and 0xFF
                        val hr = rawBytes[idx + 6].toInt() and 0xFF
                        val rrHi = rawBytes[idx + 7].toInt() and 0xFF
                        val rrLo = rawBytes[idx + 8].toInt() and 0xFF
                        val rr = (rrHi shl 8) or rrLo

                        val clsId = (rawBytes[idx + 9].toInt() and 0xFF).coerceIn(0, 4)
                        val conf = (rawBytes[idx + 10].toInt() and 0xFF).coerceIn(0, 100)
                        val lat = rawBytes[idx + 11].toInt() and 0xFF
                        val resp = rawBytes[idx + 12].toInt() and 0xFF
                        val skinT = rawBytes[idx + 13].toInt()
                        val ambT = rawBytes[idx + 14].toInt()

                        val status = rawBytes[idx + 15].toInt() and 0xFF
                        val fallAlert = (status and 0x01) != 0
                        val pvcAlert = (status and 0x02) != 0
                        val isCapBurst = (status and 0x04) != 0
                        val postIdx = ((status shr 3) and 0x03).coerceIn(0, 3)
                        val slotIdx = (status shr 5) and 0x07

                        val sdnn = if (idx + 16 < rawBytes.size) (rawBytes[idx + 16].toInt() and 0xFF) else 45
                        val rmssd = if (idx + 17 < rawBytes.size) (rawBytes[idx + 17].toInt() and 0xFF) else 38
                        val bwSaved = if (idx + 18 < rawBytes.size) (rawBytes[idx + 18].toInt() and 0xFF).toFloat() else 99.03f
                        val actMode = if (idx + 19 < rawBytes.size) (rawBytes[idx + 19].toInt() and 0xFF) else 4

                        val slice = if (isCapBurst || pvcAlert || fallAlert) {
                            "5G-URLLC (Emergency Low-Latency <1ms)"
                        } else {
                            "5G-mMTC (High-Efficiency Semantic 1Hz)"
                        }

                        val vitals = SmartBanVitals(
                            isConnected = true,
                            source = TelemetrySource.BLE_BROADCAST,
                            deviceName = "SmartBAN-Node",
                            deviceAddress = address,
                            rssiDbm = rssi,
                            sequenceNumber = seq,
                            activeMode = actMode,
                            heartRateBpm = hr.toFloat(),
                            rrIntervalMs = rr.toFloat(),
                            hrvSdnnMs = sdnn.toFloat(),
                            hrvRmssdMs = rmssd.toFloat(),
                            tinyMlClass = CLASS_CHARS[clsId],
                            tinyMlClassName = CLASS_NAMES[clsId],
                            tinyMlConfidencePct = if (conf > 0) conf else 98,
                            tinyMlLatencyUs = if (lat > 0) lat else 30,
                            tinyMlCycles = 1440,
                            respRpm = resp.toFloat(),
                            skinTempC = skinT.toFloat(),
                            ambientTempC = ambT.toFloat(),
                            motionState = POSTURES[postIdx],
                            fallAlert = fallAlert,
                            pvcAlert = pvcAlert,
                            isCapBurst = isCapBurst,
                            slotIndex = slotIdx,
                            slotName = if (isCapBurst) "CAP Emergency Burst" else "SAP Scheduled (Slot $slotIdx)",
                            slice5G = slice,
                            bandwidthSavedPct = bwSaved,
                            powerSavedPct = 92.8f,
                            powerActiveMw = if (isCapBurst) 22.18f else 1.60f,
                            powerBaselineMw = 22.18f,
                            lastReceivedMs = System.currentTimeMillis()
                        )

                        return DecodedPacket.VitalsUpdate(vitals)
                    }

                    // =========================================================
                    // 4. Legacy 31-byte frame fallback (where frameType was seq)
                    // =========================================================
                    val seq = frameType
                    val hr = rawBytes[idx + 5].toInt() and 0xFF
                    val rrHi = rawBytes[idx + 6].toInt() and 0xFF
                    val rrLo = rawBytes[idx + 7].toInt() and 0xFF
                    val rr = (rrHi shl 8) or rrLo
                    val clsId = (rawBytes[idx + 8].toInt() and 0xFF).coerceIn(0, 4)
                    val resp = rawBytes[idx + 9].toInt() and 0xFF
                    val temp = rawBytes[idx + 10].toInt()

                    val status = if (idx + 11 < rawBytes.size) (rawBytes[idx + 11].toInt() and 0xFF) else 0
                    val fallAlert = (status and 0x01) != 0
                    val pvcAlert = (status and 0x02) != 0
                    val isCapBurst = (status and 0x04) != 0
                    val postIdx = ((status shr 3) and 0x03).coerceIn(0, 3)
                    val slotIdx = (status shr 5) and 0x07
                    val sdnn = if (idx + 12 < rawBytes.size) (rawBytes[idx + 12].toInt() and 0xFF) else 45

                    val vitals = SmartBanVitals(
                        isConnected = true,
                        source = TelemetrySource.BLE_BROADCAST,
                        deviceName = "SmartBAN-Node",
                        deviceAddress = address,
                        rssiDbm = rssi,
                        sequenceNumber = seq,
                        heartRateBpm = hr.toFloat(),
                        rrIntervalMs = rr.toFloat(),
                        hrvSdnnMs = sdnn.toFloat(),
                        hrvRmssdMs = (sdnn * 0.85f),
                        tinyMlClass = CLASS_CHARS[clsId],
                        tinyMlClassName = CLASS_NAMES[clsId],
                        tinyMlConfidencePct = if (clsId == 0) 98 else 96,
                        tinyMlLatencyUs = 30,
                        tinyMlCycles = 1440,
                        respRpm = resp.toFloat(),
                        skinTempC = temp.toFloat(),
                        ambientTempC = temp.toFloat() - 1.2f,
                        motionState = POSTURES[postIdx],
                        fallAlert = fallAlert,
                        pvcAlert = pvcAlert,
                        isCapBurst = isCapBurst,
                        slotIndex = slotIdx,
                        slotName = if (isCapBurst) "CAP Emergency Burst" else "SAP Scheduled (Slot $slotIdx)",
                        slice5G = if (isCapBurst || pvcAlert || fallAlert) "5G-URLLC (Emergency)" else "5G-mMTC (Semantic)",
                        bandwidthSavedPct = 99.03f,
                        powerSavedPct = 92.8f,
                        powerActiveMw = if (isCapBurst) 22.18f else 1.60f,
                        powerBaselineMw = 22.18f,
                        lastReceivedMs = System.currentTimeMillis()
                    )

                    return DecodedPacket.VitalsUpdate(vitals)
                }
            }
            idx += (len + 1)
        }
        return null
    }

    fun decode(rawBytes: ByteArray, rssi: Int, address: String): SmartBanVitals? {
        val pkt = decodePacket(rawBytes, rssi, address)
        return if (pkt is DecodedPacket.VitalsUpdate) pkt.vitals else null
    }
}
