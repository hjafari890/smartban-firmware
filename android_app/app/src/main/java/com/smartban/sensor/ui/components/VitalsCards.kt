package com.smartban.sensor.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.ui.theme.*

@Composable
fun CardiacVitalsCard(vitals: SmartBanVitals, modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, if (vitals.pvcAlert) AccentRed else CardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
    ) {
        // Header with rigid-height status badge to prevent any vertical jitter
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(24.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "CLINICAL CARDIAC VITALS & HRV",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 12.sp,
                color = AccentGreen
            )

            // Rigid Badge Slot: identical dimensions whether normal or PVC alert
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(3.dp))
                    .background(if (vitals.pvcAlert) AccentRed else Color(0xFF1E293B))
                    .border(1.dp, if (vitals.pvcAlert) AccentRed else Color(0xFF334155), RoundedCornerShape(3.dp))
                    .padding(horizontal = 6.dp, vertical = 2.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = if (vitals.pvcAlert) "PVC ARRHYTHMIA ALERT" else "PAN-TOMPKINS QRS",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 9.sp,
                    fontWeight = FontWeight.Bold,
                    color = if (vitals.pvcAlert) Color.White else TextSecondary
                )
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Row 1: Heart Rate, RR Interval, Dual-Source Respiration
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            VitalMetricItem(
                label = "HEART RATE",
                value = if (vitals.heartRateBpm > 0) String.format("%.0f", vitals.heartRateBpm) else "--",
                unit = "BPM",
                valueColor = if (vitals.pvcAlert) AccentRed else AccentGreen,
                modifier = Modifier.weight(1f)
            )
            VitalMetricItem(
                label = "RR INTERVAL",
                value = if (vitals.rrIntervalMs > 0) String.format("%.0f", vitals.rrIntervalMs) else "--",
                unit = "ms",
                valueColor = TextPrimary,
                modifier = Modifier.weight(1f)
            )
            VitalMetricItem(
                label = "RESPIRATION",
                value = if (vitals.respRpm > 0) String.format("%.1f", vitals.respRpm) else "--",
                unit = "RPM",
                valueColor = AccentCyan,
                modifier = Modifier.weight(1f)
            )
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Row 2: Autonomic Tone HRV SDNN & RMSSD (Fixed height container)
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(34.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(Color(0xFF0F172A))
                .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
                .padding(horizontal = 10.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("AUTONOMIC HRV: ", fontFamily = FontFamily.Monospace, fontSize = 9.sp, color = TextSecondary)
                Text(
                    text = "SDNN ${if (vitals.hrvSdnnMs > 0) String.format("%.1f", vitals.hrvSdnnMs) else "--"} ms",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp,
                    color = AccentCyan
                )
            }
            Text(
                text = "RMSSD ${if (vitals.hrvRmssdMs > 0) String.format("%.1f", vitals.hrvRmssdMs) else "--"} ms",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 10.sp,
                color = AccentCyan
            )
        }
    }
}

@Composable
fun VitalMetricItem(
    label: String,
    value: String,
    unit: String,
    valueColor: Color,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .height(64.dp)
            .clip(RoundedCornerShape(4.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
            .padding(horizontal = 8.dp, vertical = 6.dp)
    ) {
        Column(
            modifier = Modifier.fillMaxSize(),
            verticalArrangement = Arrangement.SpaceBetween
        ) {
            Text(
                text = label,
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = TextSecondary,
                maxLines = 1
            )
            Row(
                verticalAlignment = Alignment.Bottom,
                modifier = Modifier.fillMaxWidth()
            ) {
                Text(
                    text = value,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 19.sp,
                    color = valueColor,
                    maxLines = 1
                )
                if (unit.isNotEmpty()) {
                    Spacer(modifier = Modifier.width(3.dp))
                    Text(
                        text = unit,
                        fontFamily = FontFamily.Monospace,
                        fontSize = 9.sp,
                        color = TextSecondary,
                        modifier = Modifier.padding(bottom = 2.dp)
                    )
                }
            }
        }
    }
}
