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
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipPath
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlin.math.*

enum class AttitudeResponseMode {
    SMOOTH,
    FAST
}

@Composable
fun AttitudeHorizonCanvas(
    targetRoll: Float,
    targetPitch: Float,
    targetAoa: Float,
    targetG: Float,
    modifier: Modifier = Modifier
) {
    var responseMode by remember { mutableStateOf(AttitudeResponseMode.SMOOTH) }

    // Interpolated 60 FPS states
    var currentRoll by remember { mutableFloatStateOf(0f) }
    var currentPitch by remember { mutableFloatStateOf(0f) }
    var currentAoa by remember { mutableFloatStateOf(0f) }
    var currentG by remember { mutableFloatStateOf(1f) }

    // Filter target buffers (1-Euro adaptive velocity & hysteresis lock)
    var filterTargetRoll by remember { mutableFloatStateOf(0f) }
    var filterTargetPitch by remember { mutableFloatStateOf(0f) }
    var filterTargetAoa by remember { mutableFloatStateOf(0f) }
    var filterTargetG by remember { mutableFloatStateOf(1f) }

    // Update filter targets when inputs arrive
    LaunchedEffect(targetRoll, targetPitch, targetAoa, targetG, responseMode) {
        if (responseMode == AttitudeResponseMode.FAST) {
            filterTargetRoll = targetRoll
            filterTargetPitch = targetPitch
            filterTargetAoa = targetAoa
            filterTargetG = targetG
            currentRoll = 0.18f * currentRoll + 0.82f * targetRoll
            currentPitch = 0.18f * currentPitch + 0.82f * targetPitch
            currentAoa = 0.18f * currentAoa + 0.82f * targetAoa
            currentG = 0.18f * currentG + 0.82f * targetG
        } else {
            // SMOOTH MODE: 1-Euro adaptive filter + deadband
            val dr = abs(targetRoll - filterTargetRoll)
            val dp = abs(targetPitch - filterTargetPitch)
            val da = abs(targetAoa - filterTargetAoa)

            if (dr < 1.5f) {
                filterTargetRoll = 0.92f * filterTargetRoll + 0.08f * targetRoll
            } else {
                val alphaR = min(0.75f, 0.28f + 0.025f * dr)
                filterTargetRoll = (1f - alphaR) * filterTargetRoll + alphaR * targetRoll
            }

            if (dp < 1.5f) {
                filterTargetPitch = 0.92f * filterTargetPitch + 0.08f * targetPitch
            } else {
                val alphaP = min(0.75f, 0.28f + 0.025f * dp)
                filterTargetPitch = (1f - alphaP) * filterTargetPitch + alphaP * targetPitch
            }

            if (da < 1.6f) {
                filterTargetAoa = 0.92f * filterTargetAoa + 0.08f * targetAoa
            } else {
                filterTargetAoa = 0.55f * filterTargetAoa + 0.45f * targetAoa
            }

            filterTargetG = 0.80f * filterTargetG + 0.20f * targetG

            // Hysteresis center lock
            if (abs(filterTargetRoll) < 1.3f) filterTargetRoll = 0f
            if (abs(filterTargetPitch) < 1.3f) filterTargetPitch = 0f
            if (abs(filterTargetAoa) < 1.5f) filterTargetAoa = 0f
        }
    }

    // 60 FPS sub-frame animation ticker (every 16ms)
    LaunchedEffect(responseMode) {
        while (isActive) {
            val stepK = if (responseMode == AttitudeResponseMode.FAST) 0.70f else 0.36f
            val errR = filterTargetRoll - currentRoll
            val errP = filterTargetPitch - currentPitch
            val errA = filterTargetAoa - currentAoa
            val errG = filterTargetG - currentG

            if (abs(errR) > 0.02f || abs(errP) > 0.02f || abs(errA) > 0.02f || abs(errG) > 0.005f) {
                currentRoll += stepK * errR
                currentPitch += stepK * errP
                currentAoa += stepK * errA
                currentG += stepK * errG

                if (abs(currentRoll) < 0.15f && filterTargetRoll == 0f) currentRoll = 0f
                if (abs(currentPitch) < 0.15f && filterTargetPitch == 0f) currentPitch = 0f
                if (abs(currentAoa) < 0.15f && filterTargetAoa == 0f) currentAoa = 0f
            }
            delay(16)
        }
    }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xFF111218))
            .border(1.dp, Color(0xFF2B2D35), RoundedCornerShape(8.dp))
            .padding(8.dp)
    ) {
        // Mode Header & Switcher
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 6.dp),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "3D ATTITUDE HORIZON",
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp,
                color = Color(0xFFFFD60A)
            )

            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                ModeButton(
                    text = "SMOOTH",
                    selected = (responseMode == AttitudeResponseMode.SMOOTH),
                    activeBg = Color(0xFF2B193E),
                    activeFg = Color(0xFFE0AAFF),
                    onClick = { responseMode = AttitudeResponseMode.SMOOTH }
                )
                ModeButton(
                    text = "FAST",
                    selected = (responseMode == AttitudeResponseMode.FAST),
                    activeBg = Color(0xFF183038),
                    activeFg = Color(0xFF00F5D4),
                    onClick = { responseMode = AttitudeResponseMode.FAST }
                )
            }
        }

        // Horizon Viewport Box with rigid fixed height of 210.dp
        Box(
            modifier = Modifier
                .fillMaxWidth()
                .height(210.dp)
                .clip(RoundedCornerShape(6.dp))
                .background(Color(0xFF121214)),
            contentAlignment = Alignment.Center
        ) {
            // Horizon Sphere Canvas
            Canvas(modifier = Modifier.fillMaxSize()) {
                val cx = size.width / 2f
                val cy = size.height / 2f
                val r = min(size.width, size.height) / 2f - 12.dp.toPx()

                // Circular Aperture clipping path
                val sphereClipPath = Path().apply {
                    addOval(Rect(cx - r, cy - r, cx + r, cy + r))
                }

                // 1. Draw Outer Bezel Rim
                drawCircle(
                    color = Color(0xFF1C1D22),
                    radius = r + 8.dp.toPx(),
                    center = Offset(cx, cy)
                )
                drawCircle(
                    color = Color(0xFF2B2D35),
                    radius = r + 8.dp.toPx(),
                    center = Offset(cx, cy),
                    style = Stroke(width = 3.dp.toPx())
                )

                // 2. Draw Sky & Ground inside circular aperture
                clipPath(sphereClipPath) {
                    // Sky background
                    drawRect(
                        color = Color(0xFF0288D1),
                        topLeft = Offset(0f, 0f),
                        size = size
                    )

                    // Ground polygon & horizon rotation
                    val pitchClamped = currentPitch.coerceIn(-60f, 60f)
                    // Scale: 2.2 px per degree (scaled to canvas size)
                    val pxPerDeg = (r / 50f)
                    val pitchPx = pitchClamped * pxPerDeg

                    withTransform({
                        // Rotate by -roll around center
                        rotate(degrees = -currentRoll, pivot = Offset(cx, cy))
                        translate(left = 0f, top = pitchPx)
                    }) {
                        val bigD = r * 3.5f

                        // Ground rectangle
                        drawRect(
                            color = Color(0xFF4E342E),
                            topLeft = Offset(cx - bigD, cy),
                            size = Size(bigD * 2f, bigD * 2f)
                        )

                        // White Horizon dividing line
                        drawLine(
                            color = Color.White,
                            start = Offset(cx - bigD, cy),
                            end = Offset(cx + bigD, cy),
                            strokeWidth = 3.dp.toPx()
                        )

                        // Pitch Ladder Lines: +/- 10 deg, +/- 20 deg
                        val ladderDegs = listOf(20f, 10f, -10f, -20f)
                        for (deg in ladderDegs) {
                            val ly = cy - (deg * pxPerDeg)
                            val lw = if (abs(deg) == 10f) 28.dp.toPx() else 44.dp.toPx()

                            drawLine(
                                color = Color(0xFFFFFFFF),
                                start = Offset(cx - lw, ly),
                                end = Offset(cx + lw, ly),
                                strokeWidth = 1.6.dp.toPx()
                            )
                        }
                    }
                }

                // 3. Inner Bezel Ring
                drawCircle(
                    color = Color(0xFF3D404D),
                    radius = r,
                    center = Offset(cx, cy),
                    style = Stroke(width = 3.dp.toPx())
                )

                // 4. Roll angle tick marks around rim (-60, -30, 0, 30, 60)
                val ticks = listOf(-60, -30, 0, 30, 60)
                for (tick in ticks) {
                    val tRad = Math.toRadians((tick - 90).toDouble())
                    val cosT = cos(tRad).toFloat()
                    val sinT = sin(tRad).toFloat()

                    val p1 = Offset(cx + (r - 2.dp.toPx()) * cosT, cy + (r - 2.dp.toPx()) * sinT)
                    val p2 = Offset(cx + (r - 10.dp.toPx()) * cosT, cy + (r - 10.dp.toPx()) * sinT)
                    val col = if (tick == 0) Color.White else Color(0xFFFFEA00)

                    drawLine(
                        color = col,
                        start = p1,
                        end = p2,
                        strokeWidth = 2.dp.toPx()
                    )
                }

                // 5. Aircraft Reference Reticle (Fixed at center, bright yellow with black border)
                val reticleCol = Color(0xFFFFB703)
                // Center pip
                drawCircle(
                    color = Color.Black,
                    radius = 5.dp.toPx(),
                    center = Offset(cx, cy)
                )
                drawCircle(
                    color = reticleCol,
                    radius = 3.5.dp.toPx(),
                    center = Offset(cx, cy)
                )

                // Left Wing
                drawLine(
                    color = Color.Black,
                    start = Offset(cx - 44.dp.toPx(), cy),
                    end = Offset(cx - 14.dp.toPx(), cy),
                    strokeWidth = 5.dp.toPx()
                )
                drawLine(
                    color = reticleCol,
                    start = Offset(cx - 43.dp.toPx(), cy),
                    end = Offset(cx - 15.dp.toPx(), cy),
                    strokeWidth = 3.5.dp.toPx()
                )
                // Left Wing tip
                drawLine(
                    color = reticleCol,
                    start = Offset(cx - 15.dp.toPx(), cy),
                    end = Offset(cx - 15.dp.toPx(), cy + 8.dp.toPx()),
                    strokeWidth = 3.5.dp.toPx()
                )

                // Right Wing
                drawLine(
                    color = Color.Black,
                    start = Offset(cx + 14.dp.toPx(), cy),
                    end = Offset(cx + 44.dp.toPx(), cy),
                    strokeWidth = 5.dp.toPx()
                )
                drawLine(
                    color = reticleCol,
                    start = Offset(cx + 15.dp.toPx(), cy),
                    end = Offset(cx + 43.dp.toPx(), cy),
                    strokeWidth = 3.5.dp.toPx()
                )
                // Right Wing tip
                drawLine(
                    color = reticleCol,
                    start = Offset(cx + 15.dp.toPx(), cy),
                    end = Offset(cx + 15.dp.toPx(), cy + 8.dp.toPx()),
                    strokeWidth = 3.5.dp.toPx()
                )
            }

            // 6. HUD Telemetry Pill Overlays (Fixed 4 corners, rigid width to avoid any jumping)
            // Top-Left: ROLL
            HudOverlayPill(
                label = "ROLL",
                value = String.format("%+.1f°", currentRoll),
                color = Color(0xFF00F5D4),
                modifier = Modifier
                    .align(Alignment.TopStart)
                    .padding(8.dp)
            )

            // Top-Right: PITCH
            HudOverlayPill(
                label = "PITCH",
                value = String.format("%+.1f°", currentPitch),
                color = Color(0xFFFEE440),
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .padding(8.dp)
            )

            // Bottom-Left: AoA
            HudOverlayPill(
                label = "AoA",
                value = String.format("%.1f°", currentAoa),
                color = Color(0xFFFF758F),
                modifier = Modifier
                    .align(Alignment.BottomStart)
                    .padding(8.dp)
            )

            // Bottom-Right: G-LOAD
            HudOverlayPill(
                label = "G-LOAD",
                value = String.format("%.2fg", currentG),
                color = Color(0xFFA8DADC),
                modifier = Modifier
                    .align(Alignment.BottomEnd)
                    .padding(8.dp)
            )
        }
    }
}

@Composable
private fun HudOverlayPill(
    label: String,
    value: String,
    color: Color,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .width(105.dp)
            .height(26.dp)
            .clip(RoundedCornerShape(4.dp))
            .background(Color(0xD9181920))
            .border(1.dp, Color(0xFF2A2D3A), RoundedCornerShape(4.dp))
            .padding(horizontal = 6.dp),
        contentAlignment = Alignment.CenterStart
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = "$label:",
                fontFamily = FontFamily.Monospace,
                fontSize = 9.sp,
                fontWeight = FontWeight.Bold,
                color = Color(0xFF888B99)
            )
            Text(
                text = value,
                fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold,
                fontSize = 11.sp,
                color = color,
                textAlign = TextAlign.End
            )
        }
    }
}

@Composable
private fun ModeButton(
    text: String,
    selected: Boolean,
    activeBg: Color,
    activeFg: Color,
    onClick: () -> Unit
) {
    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(3.dp))
            .background(if (selected) activeBg else Color(0xFF161820))
            .border(1.dp, if (selected) activeFg.copy(alpha = 0.7f) else Color(0xFF262832), RoundedCornerShape(3.dp))
            .clickable(onClick = onClick)
            .padding(horizontal = 8.dp, vertical = 3.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            fontFamily = FontFamily.Monospace,
            fontSize = 9.sp,
            fontWeight = FontWeight.Bold,
            color = if (selected) activeFg else Color(0xFF6B7280)
        )
    }
}
