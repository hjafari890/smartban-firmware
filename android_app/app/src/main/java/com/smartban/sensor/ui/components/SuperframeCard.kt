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
fun SmartBanSuperframeCard(vitals: SmartBanVitals, modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, CardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
    ) {
        // Header with rigid policy badge
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(24.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "SMARTBAN MAC SUPERFRAME (ETSI TS 103 326)",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp,
                color = AccentCyan
            )
            // Policy Badge
            Box(
                modifier = Modifier
                    .clip(RoundedCornerShape(3.dp))
                    .background(Color(0xFF0F172A))
                    .border(1.dp, AccentCyan.copy(alpha = 0.5f), RoundedCornerShape(3.dp))
                    .padding(horizontal = 6.dp, vertical = 2.dp)
            ) {
                Text(
                    text = vitals.policyName.uppercase(),
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 9.sp,
                    color = AccentCyan
                )
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 8-Slot TDMA Superframe visualizer (Strictly fixed height)
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(26.dp),
            horizontalArrangement = Arrangement.spacedBy(4.dp)
        ) {
            for (i in 0..7) {
                val isCurrentSlot = (vitals.slotIndex == i)
                val isCapSlot = (i == 7 || vitals.isCapBurst)
                val slotBg = when {
                    isCurrentSlot && isCapSlot -> AccentRed
                    isCurrentSlot -> AccentGreen
                    isCapSlot -> AccentOrange.copy(alpha = 0.25f)
                    else -> Color(0xFF0F172A)
                }

                Box(
                    modifier = Modifier
                        .weight(1f)
                        .fillMaxHeight()
                        .clip(RoundedCornerShape(3.dp))
                        .background(slotBg)
                        .border(1.dp, if (isCurrentSlot) Color.White.copy(alpha = 0.8f) else Color(0xFF1E293B), RoundedCornerShape(3.dp)),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = if (i == 7) "CAP" else "S$i",
                        fontFamily = FontFamily.Monospace,
                        fontWeight = FontWeight.Bold,
                        fontSize = 10.sp,
                        color = if (isCurrentSlot) BgDark else TextPrimary
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Metrics Table (Stable row count and fixed item layout)
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            SuperframeRow(
                label = "ACTIVE MAC SLOT:",
                value = vitals.slotName,
                valueColor = if (vitals.isCapBurst) AccentRed else AccentGreen
            )

            SuperframeRow(
                label = "5G CORE UPF SLICE:",
                value = vitals.slice5G,
                valueColor = if (vitals.isCapBurst) AccentRed else AccentGreen
            )

            SuperframeRow(
                label = "BANDWIDTH REDUCTION:",
                value = "${vitals.bandwidthSavedPct}% (9,538 -> 92 B/s)",
                valueColor = AccentGreen
            )

            SuperframeRow(
                label = "POWER REDUCTION:",
                value = "${vitals.powerSavedPct}% (${vitals.powerActiveMw} mW vs ${vitals.powerBaselineMw} mW)",
                valueColor = AccentCyan
            )

            SuperframeRow(
                label = "CUMULATIVE SENT:",
                value = "${String.format("%.1f", vitals.cumulativeSbKb)} KB (SmartBAN) vs ${String.format("%.1f", vitals.cumulativeRawKb)} KB (Raw)",
                valueColor = TextPrimary
            )
        }
    }
}

@Composable
private fun SuperframeRow(label: String, value: String, valueColor: Color) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .height(20.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontSize = 10.sp,
            color = TextSecondary
        )
        Text(
            text = value,
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 10.sp,
            color = valueColor,
            textAlign = TextAlign.End
        )
    }
}
