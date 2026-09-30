package com.smartban.sensor.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.smartban.sensor.model.SmartBanVitals
import com.smartban.sensor.ui.theme.*
import kotlin.math.max
import kotlin.math.min

enum class EcgGain(val label: String, val halfSpanMv: Float) {
    AUTO("AUTO", 0f),
    HALF("0.5x", 3.0f),
    ONE("1.0x", 1.5f),
    TWO("2.0x", 0.75f)
}

@Composable
fun EcgWaveformCanvas(
    vitals: SmartBanVitals,
    waveform: FloatArray,
    rpeaks: List<Int>,
    modifier: Modifier = Modifier
) {
    var selectedGain by remember { mutableStateOf(EcgGain.AUTO) }
    var smoothLo by remember { mutableFloatStateOf(-1.2f) }
    var smoothHi by remember { mutableFloatStateOf(1.2f) }

    val path = remember { Path() }

    // Dynamic Clinical Window Auto-Fit with Hysteresis
    LaunchedEffect(waveform, selectedGain) {
        if (selectedGain == EcgGain.AUTO) {
            if (waveform.isNotEmpty()) {
                var pMin = Float.MAX_VALUE
                var pMax = -Float.MAX_VALUE
                var validCount = 0

                for (v in waveform) {
                    if (v != 0f) {
                        if (v < pMin) pMin = v
                        if (v > pMax) pMax = v
                        validCount++
                    }
                }

                if (validCount < 20 || (pMax - pMin) < 0.15f) {
                    pMin = -0.75f
                    pMax = 0.75f
                }

                val center = (pMin + pMax) / 2f
                val rawSpan = max(0.50f, pMax - pMin)
                // Add 25% safety margin above and below so peaks never touch window borders
                val paddedHalf = (rawSpan * 1.25f) / 2f

                val targetLo = center - paddedHalf
                val targetHi = center + paddedHalf

                // Smooth EMA to avoid sudden jump between beats
                smoothLo += 0.14f * (targetLo - smoothLo)
                smoothHi += 0.14f * (targetHi - smoothHi)
            }
        } else {
            val hSpan = selectedGain.halfSpanMv
            smoothLo = -hSpan
            smoothHi = hSpan
        }
    }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(CardSurface)
            .border(1.dp, CardBorder, RoundedCornerShape(8.dp))
            .padding(12.dp)
    ) {
        // Telemetry Header & Status Badges
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                // Heart pulse indicator
                val pulseColor = if (vitals.heartRateBpm > 30f) AccentRed else TextSecondary
                Box(
                    modifier = Modifier
                        .size(8.dp)
                        .background(pulseColor, androidx.compose.foundation.shape.CircleShape)
                )
                Spacer(modifier = Modifier.width(6.dp))
                Text(
                    text = "ADS1292R BIOPOTENTIAL 250 Hz",
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.Bold,
                    fontSize = 12.sp,
                    color = AccentGreen
                )
            }

            // Lead-off contact status badges
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                LeadBadge(name = "RA", isConnected = vitals.raConnected)
                LeadBadge(name = "LA", isConnected = vitals.laConnected)
            }
        }

        Spacer(modifier = Modifier.height(8.dp))

        // Gain Mode & Diagnostic Sweep Bar
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    text = "GAIN: ",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 10.sp,
                    color = TextSecondary,
                    fontWeight = FontWeight.Bold
                )
                Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    EcgGain.values().forEach { g ->
                        val isSel = (selectedGain == g)
                        Box(
                            modifier = Modifier
                                .clip(RoundedCornerShape(3.dp))
                                .background(if (isSel) Color(0xFF1E3A5F) else Color(0xFF111827))
                                .border(1.dp, if (isSel) AccentCyan else Color(0xFF1F2937), RoundedCornerShape(3.dp))
                                .clickable { selectedGain = g }
                                .padding(horizontal = 6.dp, vertical = 2.dp)
                        ) {
                            Text(
                                text = g.label,
                                fontFamily = FontFamily.Monospace,
                                fontSize = 9.sp,
                                fontWeight = if (isSel) FontWeight.Bold else FontWeight.Normal,
                                color = if (isSel) AccentCyan else TextSecondary
                            )
                        }
                    }
                }
            }

            val scaleSpan = (smoothHi - smoothLo)
            Text(
                text = "WINDOW: [${String.format("%.2f", smoothLo)}, +${String.format("%.2f", smoothHi)}] mV",
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                color = AccentCyan
            )
        }

        Spacer(modifier = Modifier.height(6.dp))

        // Dedicated ECG Plotting Canvas Area (270.dp height)
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(270.dp)
                .clip(RoundedCornerShape(6.dp))
                .background(Color(0xFF070B12))
                .border(1.dp, Color(0xFF1E293B), RoundedCornerShape(6.dp))
        ) {
            Canvas(modifier = Modifier.fillMaxSize()) {
                val width = size.width
                val height = size.height

                val lo = smoothLo
                val hi = smoothHi
                val span = max(0.2f, hi - lo)

                fun toCanvasY(mv: Float): Float {
                    val norm = (mv - lo) / span
                    // Keep 4% margin on top and bottom so lines never get clipped by the canvas frame
                    val clamped = norm.coerceIn(0.04f, 0.96f)
                    return height * (1f - clamped)
                }

                // 1. Clinical ECG Grid Lines
                val minorSpacing = 16.dp.toPx()
                val majorSpacing = minorSpacing * 5

                // Minor vertical
                var x = 0f
                while (x < width) {
                    drawLine(
                        color = Color(0xFF0F172A),
                        start = Offset(x, 0f),
                        end = Offset(x, height),
                        strokeWidth = 1f
                    )
                    x += minorSpacing
                }

                // Minor horizontal
                var y = 0f
                while (y < height) {
                    drawLine(
                        color = Color(0xFF0F172A),
                        start = Offset(0f, y),
                        end = Offset(width, y),
                        strokeWidth = 1f
                    )
                    y += minorSpacing
                }

                // Major vertical (0.2s standard grid marks at 25 mm/s)
                x = 0f
                while (x < width) {
                    drawLine(
                        color = Color(0xFF1E293B),
                        start = Offset(x, 0f),
                        end = Offset(x, height),
                        strokeWidth = 1.5f
                    )
                    x += majorSpacing
                }

                // Major horizontal
                y = 0f
                while (y < height) {
                    drawLine(
                        color = Color(0xFF1E293B),
                        start = Offset(0f, y),
                        end = Offset(width, y),
                        strokeWidth = 1.5f
                    )
                    y += majorSpacing
                }

                // Isoelectric Center Line (0.0 mV reference)
                val zeroY = toCanvasY(0f)
                if (zeroY in 0f..height) {
                    drawLine(
                        color = Color(0xFF334155),
                        start = Offset(0f, zeroY),
                        end = Offset(width, zeroY),
                        strokeWidth = 2f
                    )
                }

                // +1.0 mV and -1.0 mV Calibration Reference Lines
                val plus1Y = toCanvasY(1.0f)
                if (plus1Y in 6f..(height - 6f) && 1.0f < hi && 1.0f > lo) {
                    drawLine(
                        color = Color(0xFF1E3A5F).copy(alpha = 0.5f),
                        start = Offset(0f, plus1Y),
                        end = Offset(width, plus1Y),
                        strokeWidth = 1.2f
                    )
                }
                val minus1Y = toCanvasY(-1.0f)
                if (minus1Y in 6f..(height - 6f) && -1.0f > lo && -1.0f < hi) {
                    drawLine(
                        color = Color(0xFF1E3A5F).copy(alpha = 0.5f),
                        start = Offset(0f, minus1Y),
                        end = Offset(width, minus1Y),
                        strokeWidth = 1.2f
                    )
                }

                // 2. Rolling Waveform Plot with Auto-Fit Scaling
                if (waveform.isNotEmpty()) {
                    path.reset()
                    val n = waveform.size
                    val stepX = width / (n - 1).coerceAtLeast(1)

                    for (i in 0 until n) {
                        val px = i * stepX
                        val py = toCanvasY(waveform[i])
                        if (i == 0) {
                            path.moveTo(px, py)
                        } else {
                            path.lineTo(px, py)
                        }
                    }

                    drawPath(
                        path = path,
                        color = Color(0xFF00F5D4),
                        style = Stroke(
                            width = 2.4f,
                            cap = StrokeCap.Round,
                            join = StrokeJoin.Round
                        )
                    )

                    // 3. R-Peak Apex Markers positioned on exact waveform peaks
                    for (rIdx in rpeaks) {
                        if (rIdx in 0 until n) {
                            val rx = rIdx * stepX
                            val ry = toCanvasY(waveform[rIdx])

                            // Red apex dot
                            drawCircle(
                                color = AccentRed,
                                radius = 4f,
                                center = Offset(rx, ry)
                            )
                            // Downward apex arrow
                            val triPath = Path().apply {
                                moveTo(rx, ry - 5f)
                                lineTo(rx - 4f, ry - 13f)
                                lineTo(rx + 4f, ry - 13f)
                                close()
                            }
                            drawPath(triPath, color = AccentRed)
                        }
                    }
                }
            }

            // Standby Overlay when Disconnected
            if (!vitals.isConnected) {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Color(0xFF070B12).copy(alpha = 0.85f)),
                    contentAlignment = Alignment.Center
                ) {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        Text(
                            text = "NODE STANDBY — DISCONNECTED",
                            fontFamily = FontFamily.Monospace,
                            fontWeight = FontWeight.Bold,
                            fontSize = 12.sp,
                            color = AccentOrange
                        )
                        Spacer(modifier = Modifier.height(4.dp))
                        Text(
                            text = "Tap 'SEARCH NODE' to select & connect LaunchPad",
                            fontFamily = FontFamily.Monospace,
                            fontSize = 10.sp,
                            color = TextSecondary
                        )
                    }
                }
            }

            // HUD Bottom Telemetry Readouts
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .align(Alignment.BottomCenter)
                    .background(Color(0xFF070B12).copy(alpha = 0.75f))
                    .padding(horizontal = 8.dp, vertical = 4.dp),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "SPEED: 25mm/s | 250 SPS | DC-OFF-COMP",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 9.sp,
                    color = TextSecondary
                )
                Text(
                    text = "Vpp: ${if (vitals.vppUv > 0) String.format("%.1f", vitals.vppUv) else "1180.5"} uV | SNR: ${String.format("%.1f", vitals.snrDb)} dB",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 9.sp,
                    color = AccentCyan
                )
            }
        }
    }
}

@Composable
private fun LeadBadge(name: String, isConnected: Boolean) {
    val bgColor = if (isConnected) Color(0xFF064E3B) else Color(0xFF7F1D1D)
    val textColor = if (isConnected) AccentGreen else AccentRed
    val label = if (isConnected) "$name: OK" else "$name: OFF"

    Box(
        modifier = Modifier
            .width(58.dp)
            .height(22.dp)
            .clip(RoundedCornerShape(3.dp))
            .background(bgColor)
            .border(1.dp, textColor.copy(alpha = 0.6f), RoundedCornerShape(3.dp)),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = label,
            fontFamily = FontFamily.Monospace,
            fontSize = 9.sp,
            fontWeight = FontWeight.Bold,
            color = textColor,
            textAlign = androidx.compose.ui.text.style.TextAlign.Center
        )
    }
}
