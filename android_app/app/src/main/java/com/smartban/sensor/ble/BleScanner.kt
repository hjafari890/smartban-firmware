package com.smartban.sensor.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import com.smartban.sensor.model.SmartBanVitals
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

data class BleDeviceInfo(
    val name: String,
    val address: String,
    val rssi: Int,
    val lastSeenMs: Long = System.currentTimeMillis()
)

class BleScanner(private val context: Context) {

    private val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager?.adapter

    private val _vitals = MutableStateFlow(SmartBanVitals())
    val vitals: StateFlow<SmartBanVitals> = _vitals.asStateFlow()

    private val _packetFlow = MutableSharedFlow<DecodedPacket>(extraBufferCapacity = 128)
    val packetFlow: SharedFlow<DecodedPacket> = _packetFlow.asSharedFlow()

    private val _isScanning = MutableStateFlow(false)
    val isScanning: StateFlow<Boolean> = _isScanning.asStateFlow()

    private val _statusText = MutableStateFlow("Ready to Scan")
    val statusText: StateFlow<String> = _statusText.asStateFlow()

    private val _discoveredDevices = MutableStateFlow<List<BleDeviceInfo>>(emptyList())
    val discoveredDevices: StateFlow<List<BleDeviceInfo>> = _discoveredDevices.asStateFlow()

    private val _connectedDeviceAddress = MutableStateFlow<String?>(null)
    val connectedDeviceAddress: StateFlow<String?> = _connectedDeviceAddress.asStateFlow()

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            result?.let { handleScanResult(it) }
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>?) {
            results?.forEach { handleScanResult(it) }
        }

        override fun onScanFailed(errorCode: Int) {
            _statusText.value = "Scan Failed (Error $errorCode)"
            _isScanning.value = false
        }
    }

    private fun handleScanResult(result: ScanResult) {
        val record = result.scanRecord ?: return
        val rawBytes = record.bytes
        val rawName = result.device?.name ?: record.deviceName ?: ""
        val address = result.device?.address ?: ""

        val packet = SmartBanPacketDecoder.decodePacket(rawBytes, result.rssi, address)
        val isSmartBan = (packet != null) || rawName.contains("SmartBAN", ignoreCase = true) || rawName.contains("SB", ignoreCase = true)

        if (isSmartBan) {
            val name = if (rawName.isNotBlank()) rawName else "SmartBAN-Node"
            val currentList = _discoveredDevices.value.toMutableList()
            val existingIdx = currentList.indexOfFirst { it.address == address }
            val info = BleDeviceInfo(name, address, result.rssi, System.currentTimeMillis())
            if (existingIdx >= 0) {
                currentList[existingIdx] = info
            } else {
                currentList.add(info)
            }
            _discoveredDevices.value = currentList
        }

        // Strict Target Address Filter: ONLY ingest telemetry if explicitly CONNECTED to this node!
        val targetAddr = _connectedDeviceAddress.value
        if (targetAddr == null || targetAddr != address) {
            // In Standby / Disconnected mode: discover device info for the scan list, but DO NOT ingest telemetry!
            return
        }

        if (packet != null) {
            _packetFlow.tryEmit(packet)
            when (packet) {
                is DecodedPacket.VitalsUpdate -> {
                    _vitals.value = packet.vitals
                    _statusText.value = "Connected to ${packet.vitals.deviceName} ($address) RSSI: ${result.rssi} dBm"
                }
                is DecodedPacket.EcgBatch -> {
                    _statusText.value = "Streaming 250 Hz Raw ECG ($address) Seq #${packet.seq}"
                }
                is DecodedPacket.ImuEnvUpdate -> {
                    // Handled in Repository
                }
            }
        }
    }

    fun connectDevice(address: String) {
        _connectedDeviceAddress.value = address
        _statusText.value = "Connected to $address"
    }

    fun disconnectDevice() {
        _connectedDeviceAddress.value = null
        _statusText.value = "Disconnected - Scanning for nodes..."
    }

    @SuppressLint("MissingPermission")
    fun startScan() {
        val scanner = bluetoothAdapter?.bluetoothLeScanner
        if (scanner == null) {
            _statusText.value = "Bluetooth is disabled or unavailable"
            return
        }

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .setReportDelay(0)
            .setCallbackType(ScanSettings.CALLBACK_TYPE_ALL_MATCHES)
            .setMatchMode(ScanSettings.MATCH_MODE_AGGRESSIVE)
            .setNumOfMatches(ScanSettings.MATCH_NUM_MAX_ADVERTISEMENT)
            .build()

        try {
            scanner.startScan(null, settings, scanCallback)
            _isScanning.value = true
            _statusText.value = "Scanning for SmartBAN-Node..."
        } catch (e: Exception) {
            _statusText.value = "Scan error: ${e.localizedMessage}"
            _isScanning.value = false
        }
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        try {
            bluetoothAdapter?.bluetoothLeScanner?.stopScan(scanCallback)
        } catch (_: Exception) {}
        _isScanning.value = false
        _statusText.value = "Scan Stopped"
    }
}
