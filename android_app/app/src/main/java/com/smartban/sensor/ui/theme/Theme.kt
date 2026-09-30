package com.smartban.sensor.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable

private val DarkColorScheme = darkColorScheme(
    primary = AccentGreen,
    secondary = AccentCyan,
    tertiary = AccentOrange,
    background = BgDark,
    surface = CardSurface,
    onPrimary = BgDark,
    onSecondary = BgDark,
    onTertiary = BgDark,
    onBackground = TextPrimary,
    onSurface = TextPrimary
)

@Composable
fun SmartBANTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = DarkColorScheme,
        typography = Typography,
        content = content
    )
}
