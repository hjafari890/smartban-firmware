package com.smartban.sensor.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Explore
import androidx.compose.material.icons.filled.MonitorHeart
import androidx.compose.material.icons.filled.Tune
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.smartban.sensor.ble.BleDeviceInfo
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.ui.components.*
import com.smartban.sensor.ui.theme.*

enum class DashboardTab(val title: String) {
    CARDIAC_AI("CARDIAC & AI"),
    IMU_ENV("IMU & ENV"),
    CONTROLS("MAC & CTRL")
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun DashboardScreen(
    vitals: SmartBanVitals,
    isScanning: Boolean,
    statusText: String,
    devices: List<BleDeviceInfo> = emptyList(),
    connectedAddress: String? = null,
    onToggleScan: () -> Unit,
    onConnectDevice: (String) -> Unit = {},
    onDisconnectDevice: () -> Unit = {},
    onInjectPvc: () -> Unit,
    onSetPolicy: (Int) -> Unit,
    onSetMode: (Int) -> Unit,
    onUpdateHubIp: (String) -> Unit
) {
    var selectedTab by remember { mutableStateOf(DashboardTab.CARDIAC_AI) }
    var showScanDialog by remember { mutableStateOf(false) }

    Scaffold(
        topBar = {
            TopAppBar(
                title = {
                    Column {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text(
                                text = "SMARTBAN TELEMETRY",
                                fontFamily = FontFamily.Monospace,
                                fontWeight = FontWeight.Bold,
                                fontSize = 15.sp,
                                color = AccentGreen
                            )
                        }
                        Text(
                            text = "CC2652R1 SENSOR NODE | BLE 5.2 | ETSI TS 103 326",
                            fontFamily = FontFamily.Monospace,
                            fontSize = 9.sp,
                            color = TextSecondary
                        )
                    }
                },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = BgDark,
                    titleContentColor = TextPrimary
                ),
                actions = {
                    val isLive = vitals.isConnected && (System.currentTimeMillis() - vitals.lastReceivedMs < 3500)
                    Box(
                        modifier = Modifier
                            .padding(end = 14.dp)
                            .width(116.dp)
                            .height(26.dp)
                            .clip(RoundedCornerShape(4.dp))
                            .background(if (isLive) Color(0xFF064E3B) else Color(0xFF7F1D1D))
                            .border(1.dp, if (isLive) AccentGreen else AccentRed, RoundedCornerShape(4.dp))
                            .clickable { showScanDialog = true },
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = if (isLive) vitals.source.label else "SEARCH NODE",
                            fontFamily = FontFamily.Monospace,
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold,
                            color = if (isLive) AccentGreen else AccentRed,
                            textAlign = androidx.compose.ui.text.style.TextAlign.Center
                        )
                    }
                }
            )
        },
        bottomBar = {
            NavigationBar(
                containerColor = Color(0xFF0F172A),
                contentColor = TextPrimary,
                tonalElevation = 0.dp
            ) {
                DashboardTab.values().forEach { tab ->
                    val selected = (selectedTab == tab)
                    val icon = when (tab) {
                        DashboardTab.CARDIAC_AI -> Icons.Default.MonitorHeart
                        DashboardTab.IMU_ENV -> Icons.Default.Explore
                        DashboardTab.CONTROLS -> Icons.Default.Tune
                    }

                    NavigationBarItem(
                        selected = selected,
                        onClick = { selectedTab = tab },
                        icon = {
                            Icon(
                                imageVector = icon,
                                contentDescription = tab.title,
                                tint = if (selected) AccentGreen else TextSecondary
                            )
                        },
                        label = {
                            Text(
                                text = tab.title,
                                fontFamily = FontFamily.Monospace,
                                fontSize = 9.sp,
                                fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
                                color = if (selected) AccentGreen else TextSecondary
                            )
                        },
                        colors = NavigationBarItemDefaults.colors(
                            selectedIconColor = AccentGreen,
                            selectedTextColor = AccentGreen,
                            indicatorColor = Color(0xFF1E293B),
                            unselectedIconColor = TextSecondary,
                            unselectedTextColor = TextSecondary
                        )
                    )
                }
            }
        },
        containerColor = BgDark
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(innerPadding)
                .padding(horizontal = 14.dp)
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Spacer(modifier = Modifier.height(4.dp))

            when (selectedTab) {
                DashboardTab.CARDIAC_AI -> {
                    // TAB 1: Detailed Clinical Vitals & On-Chip Int8 TinyML AAMI EC57 Classifier
                    CardiacVitalsCard(vitals = vitals)
                    TinyMlClassifierCard(vitals = vitals)
                }

                DashboardTab.IMU_ENV -> {
                    // TAB 2: 3-Axis IMU Flight Dynamics & Environmental Telemetry
                    ImuDynamicsCard(vitals = vitals)
                    OpticalEnvCard(vitals = vitals)
                }

                DashboardTab.CONTROLS -> {
                    // TAB 3: ETSI TS 103 326 MAC Superframe, 5G Slice, and Remote Action Controls
                    SmartBanSuperframeCard(vitals = vitals)
                    SystemControlCard(
                        vitals = vitals,
                        isScanning = isScanning,
                        statusText = statusText,
                        onToggleScan = onToggleScan,
                        onInjectPvc = onInjectPvc,
                        onSetPolicy = onSetPolicy,
                        onSetMode = onSetMode,
                        onUpdateHubIp = onUpdateHubIp,
                        onOpenDeviceScan = { showScanDialog = true }
                    )
                }
            }

            Spacer(modifier = Modifier.height(16.dp))
        }

        if (showScanDialog) {
            DeviceScanDialog(
                isScanning = isScanning,
                devices = devices,
                connectedAddress = connectedAddress,
                onConnect = { addr ->
                    onConnectDevice(addr)
                    showScanDialog = false
                },
                onDisconnect = {
                    onDisconnectDevice()
                },
                onToggleScan = onToggleScan,
                onDismiss = { showScanDialog = false }
            )
        }
    }
}
