package com.synchronisationwand.control.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.BatteryStd
import androidx.compose.material.icons.filled.BluetoothConnected
import androidx.compose.material.icons.filled.LinkOff
import androidx.compose.material.icons.filled.NotificationsActive
import androidx.compose.material.icons.filled.NotificationsOff
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material3.Card
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.synchronisationwand.control.data.ConnectionState
import com.synchronisationwand.control.data.WandState
import com.synchronisationwand.control.ui.components.EditableFloatRow
import com.synchronisationwand.control.ui.components.EditableIntRow
import com.synchronisationwand.control.ui.components.EditableRow
import com.synchronisationwand.control.ui.components.ReadOnlyRow
import com.synchronisationwand.control.viewmodel.WandViewModel

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ControlScreen(viewModel: WandViewModel, modifier: Modifier = Modifier) {
    val connectionState by viewModel.connectionState.collectAsState()
    val state by viewModel.wandState.collectAsState()
    val deviceName by viewModel.connectedDeviceName.collectAsState()

    Scaffold(
        modifier = modifier,
        topBar = {
            TopAppBar(
                title = { Text(deviceName ?: "Synchronisation Wand") },
                actions = {
                    IconButton(onClick = { viewModel.refreshAll() }) {
                        Icon(Icons.Default.Refresh, contentDescription = "Refresh values")
                    }
                    IconButton(onClick = { viewModel.disconnect() }) {
                        Icon(Icons.Default.LinkOff, contentDescription = "Disconnect")
                    }
                },
            )
        },
    ) { padding ->
        when (connectionState) {
            ConnectionState.CONNECTING, ConnectionState.DISCOVERING_SERVICES, ConnectionState.DISCONNECTING ->
                Column(modifier = Modifier.fillMaxSize().padding(padding)) { ConnectingOverlay() }
            else ->
                WandDetails(
                    state = state,
                    syncingValues = connectionState == ConnectionState.SYNCING_VALUES,
                    viewModel = viewModel,
                    modifier = Modifier.fillMaxSize().padding(padding),
                )
        }
    }
}

@Composable
private fun WandDetails(
    state: WandState,
    syncingValues: Boolean,
    viewModel: WandViewModel,
    modifier: Modifier = Modifier,
) {
    Column(modifier = modifier) {
        if (syncingValues) {
            LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
        }
        LazyColumn(contentPadding = PaddingValues(16.dp)) {
            item {
                SectionCard(title = "Device Information", icon = Icons.Default.BluetoothConnected) {
                    ReadOnlyRow("Manufacturer", state.manufacturerName ?: "—")
                }
            }

            item {
                SectionCard(title = "Battery", icon = Icons.Default.BatteryStd) {
                    BatteryRow(
                        percent = state.batteryLevelPercent,
                        notificationsEnabled = state.batteryNotificationsEnabled,
                        onToggleNotifications = viewModel::setBatteryNotifications,
                    )
                }
            }

            item {
                SectionCard(title = "Wi-Fi Configuration") {
                    EditableRow(
                        label = "Wi-Fi SSID",
                        currentValue = state.wifiSsid ?: "",
                        supportingText = "Up to 31 characters",
                        validate = { it.toByteArray(Charsets.UTF_8).size <= 31 },
                        onSave = viewModel::saveWifiSsid,
                    )
                    EditableRow(
                        label = "Wi-Fi Password",
                        currentValue = state.wifiPassword ?: "",
                        isPassword = true,
                        supportingText = "Up to 63 characters",
                        validate = { it.toByteArray(Charsets.UTF_8).size <= 63 },
                        onSave = viewModel::saveWifiPassword,
                    )
                }
            }

            item {
                SectionCard(title = "Synchronisation Parameters") {
                    EditableIntRow(
                        label = "Point of Interest",
                        unit = "",
                        currentValue = state.pointOfInterest,
                        range = 0..65535,
                        onSave = viewModel::savePointOfInterest,
                    )
                    EditableFloatRow(
                        label = "Maximum Synchronisation Error",
                        unit = "ms",
                        currentValue = state.maxSyncErrorMs,
                        supportingText = "Minimum 0.001 ms",
                        onSave = viewModel::saveMaxSyncError,
                    )
                    EditableFloatRow(
                        label = "Target IMU Sample Period",
                        unit = "ms",
                        currentValue = state.targetImuSamplePeriodMs,
                        supportingText = "Should be an integer multiple of the maximum synchronisation error",
                        onSave = viewModel::saveTargetImuSamplePeriod,
                    )
                    EditableFloatRow(
                        label = "Synchronisation Signal Pulse Width",
                        unit = "ms",
                        currentValue = state.syncSignalPulseWidthMs,
                        supportingText = "Minimum 0.001 ms",
                        onSave = viewModel::saveSyncSignalPulseWidth,
                    )
                    SyncSignalDurationRow(
                        valueMs = state.syncSignalDurationMs,
                        notificationsEnabled = state.syncDurationNotificationsEnabled,
                        onToggleNotifications = viewModel::setSyncDurationNotifications,
                    )
                }
            }

            item {
                SectionCard(title = "On-board IMU Settings") {
                    ReadOnlyRow(
                        "Status",
                        when (state.onboardImuStatus) {
                            true -> "Enabled"
                            false -> "Disabled"
                            null -> "—"
                        },
                    )
                    EditableIntRow(
                        label = "Sample Period",
                        unit = "ms",
                        currentValue = state.onboardImuSamplePeriodMs,
                        range = 0..65535,
                        onSave = viewModel::saveOnboardImuSamplePeriod,
                    )
                }
            }
        }
    }
}

@Composable
private fun BatteryRow(
    percent: Int?,
    notificationsEnabled: Boolean,
    onToggleNotifications: (Boolean) -> Unit,
) {
    Column(modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp)) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(percent?.let { "$it%" } ?: "—", style = MaterialTheme.typography.headlineSmall)
            NotifyToggle(enabled = notificationsEnabled, onToggle = onToggleNotifications)
        }
        LinearProgressIndicator(
            progress = { (percent ?: 0) / 100f },
            modifier = Modifier.fillMaxWidth().padding(top = 8.dp),
        )
    }
}

@Composable
private fun SyncSignalDurationRow(
    valueMs: Float?,
    notificationsEnabled: Boolean,
    onToggleNotifications: (Boolean) -> Unit,
) {
    Column(modifier = Modifier.fillMaxWidth().padding(vertical = 6.dp)) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Column {
                Text("Synchronisation Signal Duration", style = MaterialTheme.typography.bodyMedium)
                Text(
                    valueMs?.let { "%.3f ms".format(it) } ?: "—",
                    style = MaterialTheme.typography.bodyMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            NotifyToggle(enabled = notificationsEnabled, onToggle = onToggleNotifications)
        }
        Text(
            "Recalculated automatically by the wand; read-only here.",
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }
}

@Composable
private fun NotifyToggle(enabled: Boolean, onToggle: (Boolean) -> Unit) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Icon(
            if (enabled) Icons.Default.NotificationsActive else Icons.Default.NotificationsOff,
            contentDescription = null,
            tint = MaterialTheme.colorScheme.onSurfaceVariant,
            modifier = Modifier.padding(end = 4.dp),
        )
        Switch(checked = enabled, onCheckedChange = onToggle)
    }
}

@Composable
private fun SectionCard(
    title: String,
    icon: androidx.compose.ui.graphics.vector.ImageVector? = null,
    content: @Composable () -> Unit,
) {
    Card(modifier = Modifier.fillMaxWidth().padding(bottom = 16.dp)) {
        Column(modifier = Modifier.padding(16.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                if (icon != null) {
                    Icon(icon, contentDescription = null, modifier = Modifier.padding(end = 8.dp))
                }
                Text(title, style = MaterialTheme.typography.titleMedium)
            }
            Column(modifier = Modifier.padding(top = 8.dp)) {
                content()
            }
        }
    }
}
