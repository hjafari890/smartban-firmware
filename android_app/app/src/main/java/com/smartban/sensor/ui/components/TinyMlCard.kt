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
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.ui.theme.*

@Composable
fun TinyMlClassifierCard(vitals: SmartBanVitals, modifier: Modifier = Modifier) {
    val isV = vitals.tinyMlClass == 'V'
    val badgeColor = when (vitals.tinyMlClass) {
        'N' -> AccentGreen
        'V' -> AccentRed
        'S', 'F' -> AccentOrange
        else -> TextSecondary
    }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, if (isV) AccentRed else CardBorder, RoundedCornerShape(8.dp))
            .padding(14.dp)
    ) {
        // Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "ON-CHIP INT8 TINYML CLASSIFIER",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 13.sp,
                color = AccentGreen
            )
            Text(
                text = "AAMI EC57 STANDARD",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
        }

        Spacer(modifier = Modifier.height(12.dp))

        // Main Class Badge & Confidence
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.CenterVertically
        ) {
            // Large Beat Class Avatar
            Box(
                modifier = Modifier
                    .size(52.dp)
                    .clip(RoundedCornerShape(6.dp))
                    .background(Color(0xFF0F172A))
                    .border(2.dp, badgeColor, RoundedCornerShape(6.dp)),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "[${vitals.tinyMlClass}]",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 24.sp,
                    color = badgeColor
                )
            }

            Spacer(modifier = Modifier.width(14.dp))

            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = vitals.tinyMlClassName,
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 14.sp,
                    color = TextPrimary
                )
                Spacer(modifier = Modifier.height(4.dp))
                // Confidence bar
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Box(
                        modifier = Modifier
                            .weight(1f)
                            .height(6.dp)
                            .clip(RoundedCornerShape(3.dp))
                            .background(Color(0xFF0F172A))
                    ) {
                        Box(
                            modifier = Modifier
                                .fillMaxHeight()
                                .fillMaxWidth((vitals.tinyMlConfidencePct / 100f).coerceIn(0f, 1f))
                                .background(badgeColor)
                        )
                    }
                    Spacer(modifier = Modifier.width(8.dp))
                    Text(
                        text = "${vitals.tinyMlConfidencePct}% CONF",
                        fontFamily = FontFamily.Monospace,
                        fontWeight = FontWeight.Bold,
                        fontSize = 10.sp,
                        color = badgeColor,
                        textAlign = androidx.compose.ui.text.style.TextAlign.End,
                        modifier = Modifier.width(64.dp)
                    )
                }
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Execution & Neural Engine Telemetry
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(4.dp))
                .background(Color(0xFF0F172A))
                .padding(horizontal = 10.dp, vertical = 6.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "LATENCY: ${vitals.tinyMlLatencyUs} us (1,440 CYCLES)",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = AccentCyan
            )
            Text(
                text = "ENERGY: 0.05 uJ",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
            Text(
                text = "48 MHz M4F",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
        }
    }
}
