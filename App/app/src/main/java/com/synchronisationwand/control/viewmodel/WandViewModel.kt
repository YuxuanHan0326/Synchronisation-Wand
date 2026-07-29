package com.synchronisationwand.control.viewmodel

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import com.synchronisationwand.control.ble.WandBleManager
import com.synchronisationwand.control.data.ConnectionState
import com.synchronisationwand.control.data.ScannedDevice
import com.synchronisationwand.control.data.WandState
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow

class WandViewModel(application: Application) : AndroidViewModel(application) {

    private val bleManager = WandBleManager(application)

    val connectionState: StateFlow<ConnectionState> = bleManager.connectionState
    val wandState: StateFlow<WandState> = bleManager.wandState
    val scannedDevices: StateFlow<List<ScannedDevice>> = bleManager.scannedDevices
    val connectedDeviceName: StateFlow<String?> = bleManager.connectedDeviceName
    val events: SharedFlow<String> = bleManager.events

    fun hasScanPermission() = bleManager.hasScanPermission()
    fun hasConnectPermission() = bleManager.hasConnectPermission()
    fun isBluetoothEnabled() = bleManager.isBluetoothEnabled()

    fun startScan() = bleManager.startScan()
    fun stopScan() = bleManager.stopScan()

    fun connect(device: ScannedDevice) = bleManager.connect(device.address)
    fun disconnect() = bleManager.disconnect()

    fun refreshAll() = bleManager.refreshAll()

    fun saveWifiSsid(value: String) = bleManager.writeWifiSsid(value)
    fun saveWifiPassword(value: String) = bleManager.writeWifiPassword(value)
    fun savePointOfInterest(value: Int) = bleManager.writePointOfInterest(value)
    fun saveMaxSyncError(value: Float) = bleManager.writeMaxSyncError(value)
    fun saveTargetImuSamplePeriod(value: Float) = bleManager.writeTargetImuSamplePeriod(value)
    fun saveSyncSignalPulseWidth(value: Float) = bleManager.writeSyncSignalPulseWidth(value)
    fun saveOnboardImuSamplePeriod(value: Int) = bleManager.writeOnboardImuSamplePeriod(value)

    fun setBatteryNotifications(enabled: Boolean) = bleManager.setBatteryNotificationsEnabled(enabled)
    fun setSyncDurationNotifications(enabled: Boolean) = bleManager.setSyncDurationNotificationsEnabled(enabled)

    override fun onCleared() {
        bleManager.disconnect()
        super.onCleared()
    }
}
