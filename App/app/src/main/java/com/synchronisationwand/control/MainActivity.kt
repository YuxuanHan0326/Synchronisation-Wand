package com.synchronisationwand.control

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.BluetoothSearching
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.synchronisationwand.control.data.ConnectionState
import com.synchronisationwand.control.ui.ControlScreen
import com.synchronisationwand.control.ui.ScanScreen
import com.synchronisationwand.control.ui.theme.SynchronisationWandTheme
import com.synchronisationwand.control.viewmodel.WandViewModel

private fun requiredBlePermissions(): Array<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    } else {
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }

private fun hasAllPermissions(context: android.content.Context): Boolean =
    requiredBlePermissions().all {
        context.checkSelfPermission(it) == PackageManager.PERMISSION_GRANTED
    }

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            SynchronisationWandTheme {
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
                    AppRoot()
                }
            }
        }
    }
}

@Composable
private fun AppRoot() {
    val context = LocalContext.current
    val viewModel: WandViewModel = viewModel()

    var permissionsGranted by remember { mutableStateOf(hasAllPermissions(context)) }
    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { results -> permissionsGranted = results.values.all { it } }

    var bluetoothEnabledTick by remember { mutableStateOf(0) }
    val enableBluetoothLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.StartActivityForResult(),
    ) { bluetoothEnabledTick++ }

    LaunchedEffect(Unit) {
        viewModel.events.collect { message ->
            Toast.makeText(context, message, Toast.LENGTH_SHORT).show()
        }
    }

    if (!permissionsGranted) {
        PermissionRequestScreen(onRequest = { permissionLauncher.launch(requiredBlePermissions()) })
        return
    }

    val bluetoothEnabled = remember(bluetoothEnabledTick) { viewModel.isBluetoothEnabled() }
    if (!bluetoothEnabled) {
        BluetoothDisabledScreen(
            onEnable = { enableBluetoothLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)) },
        )
        return
    }

    val connectionState by viewModel.connectionState.collectAsState()
    when (connectionState) {
        ConnectionState.DISCONNECTED, ConnectionState.SCANNING ->
            ScanScreen(viewModel = viewModel)
        else ->
            ControlScreen(viewModel = viewModel)
    }
}

@Composable
private fun PermissionRequestScreen(onRequest: () -> Unit) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp),
            modifier = Modifier.padding(32.dp),
        ) {
            Icon(Icons.Default.BluetoothSearching, contentDescription = null)
            Text(
                text = "Bluetooth permission is needed to find and control your Synchronisation Wand.",
                textAlign = androidx.compose.ui.text.style.TextAlign.Center,
            )
            Button(onClick = onRequest) { Text("Grant permission") }
        }
    }
}

@Composable
private fun BluetoothDisabledScreen(onEnable: () -> Unit) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp),
            modifier = Modifier.padding(32.dp),
        ) {
            Icon(Icons.Default.BluetoothSearching, contentDescription = null)
            Text(
                text = "Bluetooth is turned off. Turn it on to connect to your Synchronisation Wand.",
                textAlign = androidx.compose.ui.text.style.TextAlign.Center,
            )
            Button(onClick = onEnable) { Text("Turn on Bluetooth") }
        }
    }
}
