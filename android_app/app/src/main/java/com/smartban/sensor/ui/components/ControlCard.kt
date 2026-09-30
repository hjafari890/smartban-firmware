package com.smartban.sensor.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
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
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.ui.theme.*

@Composable
fun SystemControlCard(
    vitals: SmartBanVitals,
    isScanning: Boolean,
    statusText: String,
    onToggleScan: () -> Unit,
    onInjectPvc: () -> Unit,
    onSetPolicy: (Int) -> Unit,
    onSetMode: (Int) -> Unit,
    onUpdateHubIp: (String) -> Unit,
    onOpenDeviceScan: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    var showIpDialog by remember { mutableStateOf(false) }
    var ipInputText by remember { mutableStateOf("192.168.1.100") }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, CardBorder, RoundedCornerShape(8.dp))
            .padding(14.dp)
    ) {
        // Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "BIDIRECTIONAL REMOTE CONTROLS",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 12.sp,
                color = AccentCyan
            )
            Text(
                text = vitals.source.label,
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = if (vitals.isConnected) AccentGreen else AccentRed
            )
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Link Status Block
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(4.dp))
                .background(Color(0xFF0F172A))
                .padding(horizontal = 10.dp, vertical = 6.dp)
        ) {
            Text(
                text = statusText,
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "HUB: ${vitals.hubUrl}",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 10.sp,
                    color = AccentCyan,
                    modifier = Modifier.clickable { showIpDialog = true }
                )
                Text(
                    text = "RSSI: ${vitals.rssiDbm} dBm",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 10.sp,
                    color = AccentGreen
                )
            }
        }

        Spacer(modifier = Modifier.height(12.dp))

        // 1. Emergency Arrhythmia / CAP Trigger Button
        Button(
            onClick = onInjectPvc,
            modifier = Modifier.fillMaxWidth(),
            colors = ButtonDefaults.buttonColors(
                containerColor = AccentRed,
                contentColor = Color.White
            ),
            shape = RoundedCornerShape(6.dp)
        ) {
            Text(
                text = "INJECT 5s PVC ARRHYTHMIA & CAP EMERGENCY BURST",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp
            )
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 2. MAC Policy Switcher
        Text(
            text = "ETSI TS 103 326 MAC STREAMING POLICY:",
            fontFamily = FontFamily.Monospace,
            fontSize = 9.sp,
            color = TextSecondary
        )
        Spacer(modifier = Modifier.height(4.dp))
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            PolicyButton(
                label = "ADAPTIVE",
                policyId = 0,
                activePolicyId = vitals.policyId,
                onClick = { onSetPolicy(0) },
                modifier = Modifier.weight(1f)
            )
            PolicyButton(
                label = "SEMANTIC",
                policyId = 1,
                activePolicyId = vitals.policyId,
                onClick = { onSetPolicy(1) },
                modifier = Modifier.weight(1f)
            )
            PolicyButton(
                label = "RAW CONT",
                policyId = 2,
                activePolicyId = vitals.policyId,
                onClick = { onSetPolicy(2) },
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 3. Node Hardware Mode Switcher
        Text(
            text = "ADS1292R HARDWARE ACQUISITION MODE:",
            fontFamily = FontFamily.Monospace,
            fontSize = 9.sp,
            color = TextSecondary
        )
        Spacer(modifier = Modifier.height(4.dp))
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            ModeButton(label = "M1:TEST", modeId = 1, activeMode = vitals.activeMode, onClick = { onSetMode(1) }, modifier = Modifier.weight(1f))
            ModeButton(label = "M2:SHORT", modeId = 2, activeMode = vitals.activeMode, onClick = { onSetMode(2) }, modifier = Modifier.weight(1f))
            ModeButton(label = "M3:TEMP", modeId = 3, activeMode = vitals.activeMode, onClick = { onSetMode(3) }, modifier = Modifier.weight(1f))
            ModeButton(label = "M4:LIVE", modeId = 4, activeMode = vitals.activeMode, onClick = { onSetMode(4) }, modifier = Modifier.weight(1f))
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 4. Bluetooth Search & Connect Primary Button
        Button(
            onClick = onOpenDeviceScan,
            modifier = Modifier.fillMaxWidth().height(38.dp),
            shape = RoundedCornerShape(6.dp),
            colors = ButtonDefaults.buttonColors(containerColor = AccentGreen, contentColor = BgDark)
        ) {
            Text(
                text = "SEARCH & CONNECT BLUETOOTH NODE",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp
            )
        }

        Spacer(modifier = Modifier.height(8.dp))

        // 5. BLE Scan & Hub IP Actions
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            OutlinedButton(
                onClick = onToggleScan,
                modifier = Modifier.weight(1f),
                shape = RoundedCornerShape(6.dp),
                colors = ButtonDefaults.outlinedButtonColors(
                    contentColor = if (isScanning) AccentOrange else AccentGreen
                ),
                border = androidx.compose.foundation.BorderStroke(1.dp, if (isScanning) AccentOrange else AccentGreen)
            ) {
                Text(
                    text = if (isScanning) "STOP BLE SCAN" else "START BLE SCAN",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp
                )
            }

            OutlinedButton(
                onClick = { showIpDialog = true },
                modifier = Modifier.weight(1f),
                shape = RoundedCornerShape(6.dp),
                colors = ButtonDefaults.outlinedButtonColors(contentColor = AccentCyan),
                border = androidx.compose.foundation.BorderStroke(1.dp, AccentCyan)
            ) {
                Text(
                    text = "CONFIG HUB IP",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp
                )
            }
        }
    }

    if (showIpDialog) {
        AlertDialog(
            onDismissRequest = { showIpDialog = false },
            title = {
                Text(
                    text = "CONFIGURE WI-FI HUB IP",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 14.sp,
                    color = AccentCyan
                )
            },
            text = {
                Column {
                    Text(
                        text = "Enter desktop PC IP address running sensor_gui.py:",
                        fontFamily = FontFamily.Monospace,
                        fontSize = 11.sp,
                        color = TextSecondary
                    )
                    Spacer(modifier = Modifier.height(8.dp))
                    OutlinedTextField(
                        value = ipInputText,
                        onValueChange = { ipInputText = it },
                        singleLine = true,
                        placeholder = { Text("e.g. 192.168.1.100") }
                    )
                }
            },
            confirmButton = {
                Button(
                    onClick = {
                        val formatted = if (ipInputText.contains(":")) ipInputText else "$ipInputText:8080"
                        onUpdateHubIp(formatted)
                        showIpDialog = false
                    },
                    colors = ButtonDefaults.buttonColors(containerColor = AccentCyan)
                ) {
                    Text("CONNECT", fontFamily = FontFamily.Monospace, color = BgDark, fontWeight = FontWeight.Bold)
                }
            },
            dismissButton = {
                TextButton(onClick = { showIpDialog = false }) {
                    Text("CANCEL", fontFamily = FontFamily.Monospace, color = TextSecondary)
                }
            },
            containerColor = CardSurface
        )
    }
}

@Composable
private fun PolicyButton(
    label: String,
    policyId: Int,
    activePolicyId: Int,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    val isActive = (policyId == activePolicyId)
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(4.dp))
            .background(if (isActive) AccentCyan else Color(0xFF0F172A))
            .border(1.dp, if (isActive) AccentCyan else Color(0xFF1E293B), RoundedCornerShape(4.dp))
            .clickable { onClick() }
            .padding(vertical = 6.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 10.sp,
            color = if (isActive) BgDark else TextPrimary
        )
    }
}

@Composable
private fun ModeButton(
    label: String,
    modeId: Int,
    activeMode: Int,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    val isActive = (modeId == activeMode)
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(4.dp))
            .background(if (isActive) AccentGreen else Color(0xFF0F172A))
            .border(1.dp, if (isActive) AccentGreen else Color(0xFF1E293B), RoundedCornerShape(4.dp))
            .clickable { onClick() }
            .padding(vertical = 6.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 9.sp,
            color = if (isActive) BgDark else TextPrimary
        )
    }
}
