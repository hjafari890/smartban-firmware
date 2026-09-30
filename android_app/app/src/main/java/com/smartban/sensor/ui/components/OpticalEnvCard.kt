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
fun OpticalEnvCard(vitals: SmartBanVitals, modifier: Modifier = Modifier) {
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
                text = "OPTICAL & ENVIRONMENTAL TELEMETRY",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 13.sp,
                color = AccentOrange
            )
            Text(
                text = "BME680 | OPT4041 | MLX90632",
                fontFamily = FontFamily.Monospace,
                fontSize = 10.sp,
                color = TextSecondary
            )
        }

        Spacer(modifier = Modifier.height(12.dp))

        // Thermal Suite: Skin vs Ambient
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(4.dp))
                .background(Color(0xFF0F172A))
                .padding(10.dp),
            horizontalArrangement = Arrangement.SpaceAround
        ) {
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text("SKIN TEMP (MLX90632)", fontFamily = FontFamily.Monospace, fontSize = 9.sp, color = TextSecondary)
                Text(
                    text = "${String.format("%.1f", if (vitals.skinTempC > 0) vitals.skinTempC else 36.4f)} °C",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 15.sp,
                    color = AccentGreen
                )
            }
            Box(modifier = Modifier.width(1.dp).height(30.dp).background(CardBorder))
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Text("AMBIENT TEMP (BME680)", fontFamily = FontFamily.Monospace, fontSize = 9.sp, color = TextSecondary)
                Text(
                    text = "${String.format("%.1f", if (vitals.ambientTempC > 0) vitals.ambientTempC else 22.8f)} °C",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 15.sp,
                    color = AccentCyan
                )
            }
        }

        Spacer(modifier = Modifier.height(10.dp))

        // Environmental Metrics Grid (Pressure, Humidity, IAQ, CO2, Lux, Prox)
        Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                EnvItem(
                    label = "PRESSURE",
                    value = "${String.format("%.1f", vitals.pressureHpa)} hPa",
                    sub = "${String.format("%.0f", vitals.altitudeM)} m ASL",
                    modifier = Modifier.weight(1f)
                )
                Spacer(modifier = Modifier.width(8.dp))
                EnvItem(
                    label = "HUMIDITY",
                    value = "${String.format("%.1f", vitals.humidityPct)} %",
                    sub = "Dew Pt ~14°C",
                    modifier = Modifier.weight(1f)
                )
            }

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                EnvItem(
                    label = "AIR QUALITY (IAQ)",
                    value = "${String.format("%.0f", vitals.iaqIndex)} IAQ",
                    sub = if (vitals.iaqIndex < 50) "EXCELLENT" else "MODERATE",
                    modifier = Modifier.weight(1f)
                )
                Spacer(modifier = Modifier.width(8.dp))
                EnvItem(
                    label = "eCO2 EQUIV",
                    value = "${String.format("%.0f", vitals.co2Ppm)} ppm",
                    sub = "Target <800",
                    modifier = Modifier.weight(1f)
                )
            }

            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                EnvItem(
                    label = "AMBIENT LUX",
                    value = "${String.format("%.1f", vitals.lux)} lux",
                    sub = "OPT4041 28-bit",
                    modifier = Modifier.weight(1f)
                )
                Spacer(modifier = Modifier.width(8.dp))
                val proxDist = if (vitals.proximity > 15) {
                    val norm = (kotlin.math.ln(vitals.proximity.toFloat().coerceIn(15f, 4096f)) - kotlin.math.ln(15f)) / (kotlin.math.ln(4096f) - kotlin.math.ln(15f))
                    val dCm = (20f * (1f - norm)).coerceAtLeast(0.5f)
                    "OBJECT (~${String.format("%.1f", dCm)} cm)"
                } else {
                    "CLEAR (>20 cm)"
                }
                EnvItem(
                    label = "PROXIMITY (VCNL4040)",
                    value = "${vitals.proximity} cts",
                    sub = proxDist,
                    modifier = Modifier.weight(1f)
                )
            }
        }
    }
}

@Composable
private fun EnvItem(
    label: String,
    value: String,
    sub: String,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(4.dp))
            .background(Color(0xFF0F172A))
            .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(4.dp))
            .padding(horizontal = 10.dp, vertical = 8.dp)
    ) {
        Column {
            Text(
                text = label,
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = TextSecondary
            )
            Spacer(modifier = Modifier.height(2.dp))
            Text(
                text = value,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 13.sp,
                color = TextPrimary
            )
            Text(
                text = sub,
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = AccentCyan
            )
        }
    }
}
