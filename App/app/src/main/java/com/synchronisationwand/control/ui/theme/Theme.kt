package com.synchronisationwand.control.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

private val WandPrimary = Color(0xFF3D6BF2)
private val WandSecondary = Color(0xFF1B2A4A)

private val DarkColors = darkColorScheme(
    primary = Color(0xFF9AB6FF),
    secondary = Color(0xFFB8C6E8),
    background = Color(0xFF10131A),
    surface = Color(0xFF181C24),
)

private val LightColors = lightColorScheme(
    primary = WandPrimary,
    secondary = WandSecondary,
    background = Color(0xFFF6F7FB),
    surface = Color(0xFFFFFFFF),
)

@Composable
fun SynchronisationWandTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit,
) {
    val colors = if (darkTheme) DarkColors else LightColors
    MaterialTheme(
        colorScheme = colors,
        content = content,
    )
}
