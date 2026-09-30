package com.smartban.sensor.net

import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.model.TelemetrySource
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONObject
import java.util.concurrent.TimeUnit

class WifiHubClient(private var baseUrl: String = "http://192.168.1.100:8080") {

    private val client = OkHttpClient.Builder()
        .connectTimeout(800, TimeUnit.MILLISECONDS)
        .readTimeout(800, TimeUnit.MILLISECONDS)
        .build()

    fun updateBaseUrl(newUrl: String) {
        baseUrl = if (newUrl.startsWith("http://") || newUrl.startsWith("https://")) {
            newUrl.trimEnd('/')
        } else {
            "http://${newUrl.trimEnd('/')}"
        }
    }

    fun getBaseUrl(): String = baseUrl

    suspend fun fetchData(): SmartBanVitals? = withContext(Dispatchers.IO) {
        try {
            val request = Request.Builder()
                .url("$baseUrl/data")
                .build()

            client.newCall(request).execute().use { response ->
                if (!response.isSuccessful) return@withContext null
                val bodyStr = response.body?.string() ?: return@withContext null
                val json = JSONObject(bodyStr)

                // ECG biopotentials & peaks
                val ecgArr = json.optJSONArray("ecg")
                val ecgList = mutableListOf<Float>()
                if (ecgArr != null) {
                    for (i in 0 until ecgArr.length()) {
                        ecgList.add(ecgArr.getDouble(i).toFloat())
                    }
                }

                val peaksArr = json.optJSONArray("ecg_rpeaks")
                val peaksList = mutableListOf<Int>()
                if (peaksArr != null) {
                    for (i in 0 until peaksArr.length()) {
                        peaksList.add(peaksArr.getInt(i))
                    }
                }

                // Vitals
                val vObj = json.optJSONObject("vitals") ?: JSONObject()
                val hr = vObj.optDouble("bpm", 0.0).toFloat()
                val rr = vObj.optDouble("rr_ms", 0.0).toFloat()
                val sdnn = vObj.optDouble("sdnn_ms", 0.0).toFloat()
                val rmssd = vObj.optDouble("rmssd_ms", 0.0).toFloat()
                val rpm = vObj.optDouble("rpm", 0.0).toFloat()
                val skinT = vObj.optDouble("temp_skin", 0.0).toFloat()
                val ambT = vObj.optDouble("temp_amb", 0.0).toFloat()

                // IMU
                val imuObj = json.optJSONObject("imu") ?: JSONObject()
                val ax = imuObj.optDouble("ax", 0.0).toFloat()
                val ay = imuObj.optDouble("ay", 0.0).toFloat()
                val az = imuObj.optDouble("az", 1.0).toFloat()
                val pitch = imuObj.optDouble("pitch", 0.0).toFloat()
                val roll = imuObj.optDouble("roll", 0.0).toFloat()
                val aoa = imuObj.optDouble("aoa", 0.0).toFloat()
                val gTot = imuObj.optDouble("g_total", 1.0).toFloat()
                val actCode = imuObj.optInt("act_state", 0)
                val actState = when (actCode) {
                    1 -> "Active"
                    2 -> "Dynamic Motion"
                    else -> "Sedentary"
                }
                val steps = imuObj.optInt("steps", 0)
                val spm = imuObj.optDouble("spm", 0.0).toFloat()
                val fall = imuObj.optBoolean("fall_alarm", false)

                // Environment
                val envObj = json.optJSONObject("env") ?: JSONObject()
                val lux = envObj.optDouble("lux", 0.0).toFloat()
                val prox = envObj.optInt("prox", 0)
                val press = envObj.optDouble("pressure", 1013.25).toFloat()
                val hum = envObj.optDouble("humidity", 40.0).toFloat()
                val iaq = envObj.optDouble("iaq", 25.0).toFloat()
                val co2 = envObj.optDouble("co2", 420.0).toFloat()
                val alt = envObj.optDouble("alt", 0.0).toFloat()

                // TinyML
                val mlObj = json.optJSONObject("tinyml") ?: JSONObject()
                val clsChar = mlObj.optString("cls", "N").firstOrNull() ?: 'N'
                val clsName = mlObj.optString("cls_name", "Normal Sinus")
                val conf = mlObj.optInt("conf", 98)
                val lat = mlObj.optInt("latency_us", 30)
                val cycles = mlObj.optInt("cycles", 1440)

                // MAC
                val macObj = json.optJSONObject("mac") ?: JSONObject()
                val ibi = macObj.optInt("ibi", 1)
                val polId = macObj.optInt("policy_id", 0)
                val polName = macObj.optString("policy_name", "Adaptive Hybrid")
                val slotId = macObj.optInt("slot_id", 0)
                val slotName = macObj.optString("slot_name", "SAP Slot 0")
                val isBurst = macObj.optBoolean("is_cap_burst", false)
                val slice = macObj.optString("slice_5g", "5G-mMTC")
                val bwSaved = macObj.optDouble("bw_saved_pct", 99.03).toFloat()
                val pwrSaved = macObj.optDouble("pwr_saved_pct", 92.8).toFloat()
                val pwrAct = macObj.optDouble("pwr_active_mw", 1.60).toFloat()
                val pwrBase = macObj.optDouble("pwr_baseline_mw", 22.18).toFloat()
                val sbKb = macObj.optDouble("sb_kb", 0.0).toFloat()
                val rawKb = macObj.optDouble("raw_kb", 0.0).toFloat()

                SmartBanVitals(
                    isConnected = true,
                    source = TelemetrySource.WIFI_HUB,
                    deviceName = "SmartBAN Coordinator Hub",
                    deviceAddress = baseUrl,
                    hubUrl = baseUrl,
                    rssiDbm = -45, // Wi-Fi LAN direct
                    sequenceNumber = ibi,
                    activeMode = json.optInt("active_mode", 4),

                    ecgSamples = ecgList,
                    ecgRpeaks = peaksList,
                    vppUv = json.optDouble("vpp_uv", 0.0).toFloat(),
                    snrDb = json.optDouble("snr_db", 24.5).toFloat(),
                    raConnected = json.optBoolean("ra_connected", true),
                    laConnected = json.optBoolean("la_connected", true),

                    heartRateBpm = hr,
                    rrIntervalMs = rr,
                    hrvSdnnMs = sdnn,
                    hrvRmssdMs = rmssd,
                    respRpm = rpm,
                    skinTempC = skinT,
                    ambientTempC = ambT,

                    accelX = ax,
                    accelY = ay,
                    accelZ = az,
                    pitchDeg = pitch,
                    rollDeg = roll,
                    aoaDeg = aoa,
                    gTotal = gTot,
                    motionState = actState,
                    stepCount = steps,
                    spm = spm,
                    fallAlert = fall,

                    lux = lux,
                    proximity = prox,
                    pressureHpa = press,
                    humidityPct = hum,
                    iaqIndex = iaq,
                    co2Ppm = co2,
                    altitudeM = alt,

                    tinyMlClass = clsChar,
                    tinyMlClassName = clsName,
                    tinyMlConfidencePct = conf,
                    tinyMlLatencyUs = lat,
                    tinyMlCycles = cycles,

                    ibiNumber = ibi,
                    policyId = polId,
                    policyName = polName,
                    slotIndex = slotId,
                    slotName = slotName,
                    isCapBurst = isBurst,
                    pvcAlert = (clsChar == 'V'),
                    slice5G = slice,
                    bandwidthSavedPct = bwSaved,
                    powerSavedPct = pwrSaved,
                    powerActiveMw = pwrAct,
                    powerBaselineMw = pwrBase,
                    cumulativeSbKb = sbKb,
                    cumulativeRawKb = rawKb,

                    lastReceivedMs = System.currentTimeMillis()
                )
            }
        } catch (_: Exception) {
            null
        }
    }

    suspend fun sendPvcBurst(): Boolean = withContext(Dispatchers.IO) {
        try {
            val req = Request.Builder().url("$baseUrl/cmd/pvc").build()
            client.newCall(req).execute().use { it.isSuccessful }
        } catch (_: Exception) {
            false
        }
    }

    suspend fun setPolicy(policyId: Int): Boolean = withContext(Dispatchers.IO) {
        try {
            val req = Request.Builder().url("$baseUrl/cmd/policy?id=$policyId").build()
            client.newCall(req).execute().use { it.isSuccessful }
        } catch (_: Exception) {
            false
        }
    }

    suspend fun setNodeMode(modeId: Int): Boolean = withContext(Dispatchers.IO) {
        try {
            val req = Request.Builder().url("$baseUrl/cmd/mode?id=$modeId").build()
            client.newCall(req).execute().use { it.isSuccessful }
        } catch (_: Exception) {
            false
        }
    }
}
