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
fun ImuDynamicsCard(vitals: SmartBanVitals, modifier: Modifier = Modifier) {
    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, CardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
    ) {
        // Card Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "3D IMU & BIOMECHANICAL DYNAMICS",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 12.sp,
                color = AccentCyan
            )
            // Motion Activity State Badge (Fixed size to avoid width jitter)
            Box(
                modifier = Modifier
                    .width(100.dp)
                    .height(24.dp)
                    .clip(RoundedCornerShape(4.dp))
                    .background(Color(0xFF1E293B))
                    .border(1.dp, AccentCyan.copy(alpha = 0.5f), RoundedCornerShape(4.dp)),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = vitals.motionState.uppercase(),
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp,
                    color = AccentCyan,
                    textAlign = TextAlign.Center
                )
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 3D Artificial Horizon & Attitude Sphere (60-120 FPS sub-frame filter)
        AttitudeHorizonCanvas(
            targetRoll = vitals.rollDeg,
            targetPitch = vitals.pitchDeg,
            targetAoa = vitals.aoaDeg,
            targetG = vitals.gTotal
        )

        Spacer(modifier = Modifier.height(10.dp))

        // Fixed-Height Impact / Status Banner (Rigid height to eliminate ALL layout shifts / jitter!)
        val isShock = vitals.fallAlert || vitals.gTotal >= 2.0f
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(28.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(if (isShock) AccentRed else Color(0xFF0F172A))
                .border(1.dp, if (isShock) AccentRed else Color(0xFF1E293B), RoundedCornerShape(4.dp))
                .padding(horizontal = 8.dp),
            contentAlignment = Alignment.CenterStart
        ) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = if (isShock) "CRITICAL: IMPACT / FALL EVENT" else "STATUS: BIOMECHANICAL NOMINAL",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp,
                    color = if (isShock) Color.White else TextSecondary
                )
                Text(
                    text = "G: ${String.format("%.2f", vitals.gTotal)}g",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 10.sp,
                    color = if (isShock) Color.White else AccentCyan
                )
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // 3-Axis Normalized Acceleration Bars (Fixed width tabular numbers)
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            AccelAxisBar(axis = "X", value = vitals.accelX, color = AccentCyan)
            AccelAxisBar(axis = "Y", value = vitals.accelY, color = AccentGreen)
            AccelAxisBar(axis = "Z", value = vitals.accelZ, color = AccentOrange)
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Locomotion & PDR Telemetry
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .height(30.dp)
                .clip(RoundedCornerShape(4.dp))
                .background(Color(0xFF0F172A))
                .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
                .padding(horizontal = 10.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "STEPS: ${vitals.stepCount}",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                color = TextPrimary
            )
            Text(
                text = "CADENCE: ${String.format("%.1f", vitals.spm)} SPM",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
            Text(
                text = "ADXL362 25Hz",
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = TextSecondary
            )
        }
    }
}

@Composable
private fun AccelAxisBar(axis: String, value: Float, color: Color) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = "$axis:",
            fontFamily = FontFamily.Monospace,
            fontWeight = FontWeight.Bold,
            fontSize = 11.sp,
            color = color,
            modifier = Modifier.width(22.dp)
        )

        // Bar container
        Box(
            modifier = Modifier
                .weight(1f)
                .height(8.dp)
                .clip(RoundedCornerShape(2.dp))
                .background(Color(0xFF0F172A))
        ) {
            val fraction = ((value + 2.0f) / 4.0f).coerceIn(0f, 1f)
            val isPositive = value >= 0f

            Box(
                modifier = Modifier
                    .fillMaxHeight()
                    .fillMaxWidth(if (isPositive) (fraction - 0.5f).coerceAtLeast(0.02f) else (0.5f - fraction).coerceAtLeast(0.02f))
                    .background(color)
            )
        }

        Spacer(modifier = Modifier.width(8.dp))

        Text(
            text = "${String.format("%+.2f", value)} g",
            fontFamily = FontFamily.Monospace,
            fontSize = 11.sp,
            color = TextPrimary,
            textAlign = TextAlign.End,
            modifier = Modifier.width(60.dp)
        )
    }
}
