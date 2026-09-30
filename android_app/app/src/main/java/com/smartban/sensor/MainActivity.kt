package com.smartban.sensor

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.core.content.ContextCompat
import com.smartban.sensor.ble.BleScanner
import com.smartban.sensor.net.WifiHubClient
import com.smartban.sensor.repository.SmartBanRepository
import com.smartban.sensor.ui.DashboardScreen
import com.smartban.sensor.ui.theme.SmartBANTheme

class MainActivity : ComponentActivity() {

    private lateinit var bleScanner: BleScanner
    private lateinit var wifiHubClient: WifiHubClient
    private lateinit var repository: SmartBanRepository

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val allGranted = permissions.entries.all { it.value }
        if (allGranted) {
            bleScanner.startScan()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        bleScanner = BleScanner(this)
        wifiHubClient = WifiHubClient("http://192.168.1.100:8080")
        repository = SmartBanRepository(bleScanner, wifiHubClient)

        checkAndRequestPermissions()

        setContent {
            SmartBANTheme {
                val vitals by repository.vitals.collectAsState()
                val ecgWaveform by repository.ecgWaveform.collectAsState()
                val rpeaks by repository.rpeakIndices.collectAsState()
                val isScanning by bleScanner.isScanning.collectAsState()
                val statusText by repository.statusMessage.collectAsState()
                val devices by repository.discoveredDevices.collectAsState()
                val connectedAddress by bleScanner.connectedDeviceAddress.collectAsState()

                DashboardScreen(
                    vitals = vitals,
                    isScanning = isScanning,
                    statusText = statusText,
                    devices = devices,
                    connectedAddress = connectedAddress,
                    onToggleScan = {
                        if (isScanning) {
                            bleScanner.stopScan()
                        } else {
                            checkAndRequestPermissions()
                        }
                    },
                    onConnectDevice = { addr ->
                        repository.connectDevice(addr)
                    },
                    onDisconnectDevice = {
                        repository.disconnectDevice()
                    },
                    onInjectPvc = {
                        repository.triggerPvcBurst()
                    },
                    onSetPolicy = { policyId ->
                        repository.setPolicy(policyId)
                    },
                    onSetMode = { modeId ->
                        repository.setNodeMode(modeId)
                    },
                    onUpdateHubIp = { newIp ->
                        repository.updateHubIp(newIp)
                    }
                )
            }
        }
    }

    private fun checkAndRequestPermissions() {
        val permissions = mutableListOf<String>()

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)
            permissions.add(Manifest.permission.ACCESS_COARSE_LOCATION)
        }

        val missing = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (missing.isNotEmpty()) {
            permissionLauncher.launch(missing.toTypedArray())
        } else {
            bleScanner.startScan()
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        bleScanner.stopScan()
    }
}
