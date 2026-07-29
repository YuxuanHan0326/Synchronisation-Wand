package com.synchronisationwand.control.ble

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import com.synchronisationwand.control.data.ConnectionState
import com.synchronisationwand.control.data.ScannedDevice
import com.synchronisationwand.control.data.WandState
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.UUID

/**
 * Owns the BLE scan + GATT connection lifecycle for a single Synchronisation Wand.
 *
 * GATT operations on Android must be executed one at a time (a second call made
 * before the previous one's callback fires silently fails), so every read/write/
 * descriptor-write is funnelled through a small FIFO queue.
 */
class WandBleManager(context: Context) {

    private val appContext = context.applicationContext
    private val bluetoothManager =
        appContext.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val adapter: BluetoothAdapter? get() = bluetoothManager.adapter

    private var bluetoothGatt: BluetoothGatt? = null
    private var targetMtuRequested = false
    private val disconnectHandler = Handler(Looper.getMainLooper())
    private var disconnectFallback: Runnable? = null
    private val heartbeatRunnable = object : Runnable {
        override fun run() {
            if (_connectionState.value == ConnectionState.READY) {
                enqueueHeartbeat()
                disconnectHandler.postDelayed(this, HEARTBEAT_INTERVAL_MS)
            }
        }
    }

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _wandState = MutableStateFlow(WandState())
    val wandState: StateFlow<WandState> = _wandState.asStateFlow()

    private val _scannedDevices = MutableStateFlow<List<ScannedDevice>>(emptyList())
    val scannedDevices: StateFlow<List<ScannedDevice>> = _scannedDevices.asStateFlow()

    private val _connectedDeviceName = MutableStateFlow<String?>(null)
    val connectedDeviceName: StateFlow<String?> = _connectedDeviceName.asStateFlow()

    private val _events = MutableSharedFlow<String>(extraBufferCapacity = 8)
    val events: SharedFlow<String> = _events.asSharedFlow()

    private val discoveredDeviceMap = LinkedHashMap<String, ScannedDevice>()

    // ---- GATT operation queue -------------------------------------------------

    private val operationQueue = ArrayDeque<() -> Boolean>()
    private var queueBusy = false
    private var pendingPeripheralDisconnect: BluetoothGatt? = null
    private var controlWriteInFlight: ControlCommand? = null

    private enum class ControlCommand(val value: Byte) {
        DISCONNECT(0x01),
        HEARTBEAT(0x02),
    }

    private fun enqueue(op: () -> Boolean) {
        operationQueue.addLast(op)
        pumpQueue()
    }

    private fun pumpQueue() {
        if (queueBusy) return
        val next = operationQueue.removeFirstOrNull() ?: return
        queueBusy = true
        val started = runCatching { next() }.getOrDefault(false)
        if (!started) {
            queueBusy = false
            pumpQueue()
        }
    }

    private fun completeOperation() {
        queueBusy = false
        pendingPeripheralDisconnect?.let { gatt ->
            pendingPeripheralDisconnect = null
            sendPeripheralDisconnect(gatt)
            return
        }
        pumpQueue()
    }

    private fun clearQueue() {
        operationQueue.clear()
        queueBusy = false
        pendingPeripheralDisconnect = null
        controlWriteInFlight = null
    }

    // ---- Permission helpers -----------------------------------------------

    fun hasScanPermission(): Boolean = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        appContext.checkSelfPermission(android.Manifest.permission.BLUETOOTH_SCAN) ==
            android.content.pm.PackageManager.PERMISSION_GRANTED
    } else {
        appContext.checkSelfPermission(android.Manifest.permission.ACCESS_FINE_LOCATION) ==
            android.content.pm.PackageManager.PERMISSION_GRANTED
    }

    fun hasConnectPermission(): Boolean = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        appContext.checkSelfPermission(android.Manifest.permission.BLUETOOTH_CONNECT) ==
            android.content.pm.PackageManager.PERMISSION_GRANTED
    } else {
        true
    }

    fun isBluetoothEnabled(): Boolean = adapter?.isEnabled == true

    // ---- Scanning ---------------------------------------------------------

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.scanRecord?.deviceName ?: result.device.address
            discoveredDeviceMap[result.device.address] = ScannedDevice(
                address = result.device.address,
                name = name,
                rssi = result.rssi,
            )
            _scannedDevices.value = discoveredDeviceMap.values.sortedByDescending { it.rssi }
        }

        override fun onScanFailed(errorCode: Int) {
            _events.tryEmit("Scan failed (error $errorCode)")
            if (_connectionState.value == ConnectionState.SCANNING) {
                _connectionState.value = ConnectionState.DISCONNECTED
            }
        }
    }

    fun startScan() {
        if (!hasScanPermission() || !isBluetoothEnabled()) return
        val scanner = adapter?.bluetoothLeScanner ?: return

        discoveredDeviceMap.clear()
        _scannedDevices.value = emptyList()

        val filters = listOf(
            ScanFilter.Builder().setDeviceName(BleUuids.DEVICE_NAME).build(),
        )
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        runCatching {
            scanner.startScan(filters, settings, scanCallback)
            _connectionState.value = ConnectionState.SCANNING
        }.onFailure {
            _events.tryEmit("Unable to start scan: ${it.message}")
        }
    }

    fun stopScan() {
        if (!hasScanPermission()) return
        val scanner = adapter?.bluetoothLeScanner ?: return
        runCatching { scanner.stopScan(scanCallback) }
        if (_connectionState.value == ConnectionState.SCANNING) {
            _connectionState.value = ConnectionState.DISCONNECTED
        }
    }

    // ---- Connection ---------------------------------------------------------

    fun connect(address: String) {
        if (!hasConnectPermission()) return
        val device = runCatching { adapter?.getRemoteDevice(address) }.getOrNull() ?: return
        stopScan()
        clearQueue()
        targetMtuRequested = false
        _wandState.value = WandState()
        _connectionState.value = ConnectionState.CONNECTING
        _connectedDeviceName.value = discoveredDeviceMap[address]?.name

        runCatching {
            bluetoothGatt = device.connectGatt(appContext, false, gattCallback)
        }.onFailure {
            _events.tryEmit("Unable to connect: ${it.message}")
            _connectionState.value = ConnectionState.DISCONNECTED
        }
    }

    fun disconnect() {
        val gatt = bluetoothGatt
        if (gatt == null) {
            teardownGatt()
            return
        }
        if (!hasConnectPermission()) {
            teardownGatt(gatt)
            return
        }

        stopHeartbeat()

        // Drop queued reads and writes, but let an operation already on the
        // controller finish before sending the disconnect command.
        operationQueue.clear()
        _connectionState.value = ConnectionState.DISCONNECTING

        // Android occasionally reports a local disconnect without promptly
        // releasing the controller link.  If its callback never arrives,
        // close this exact GATT client so the peripheral can observe the link
        // loss and return to advertising.
        cancelDisconnectFallback()
        disconnectFallback = Runnable {
            if (bluetoothGatt === gatt &&
                _connectionState.value == ConnectionState.DISCONNECTING
            ) {
                _events.tryEmit("Disconnect timed out; forcing Bluetooth link closed")
                teardownGatt(gatt)
            }
        }.also { disconnectHandler.postDelayed(it, DISCONNECT_TIMEOUT_MS) }

        pendingPeripheralDisconnect = gatt
        if (!queueBusy) {
            pendingPeripheralDisconnect = null
            sendPeripheralDisconnect(gatt)
        }
    }

    @Suppress("DEPRECATION")
    private fun sendPeripheralDisconnect(gatt: BluetoothGatt) {
        val target = controlTarget(gatt, ControlCommand.DISCONNECT)
        if (target == null) {
            _events.tryEmit("Wand disconnect command unavailable; using Android disconnect")
            runCatching { gatt.disconnect() }
            return
        }

        val (characteristic, command) = target
        characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        characteristic.value = command
        queueBusy = true
        controlWriteInFlight = ControlCommand.DISCONNECT
        val started = runCatching { gatt.writeCharacteristic(characteristic) == true }
            .getOrDefault(false)
        if (!started) {
            queueBusy = false
            controlWriteInFlight = null
            _events.tryEmit("Wand disconnect command failed; using Android disconnect")
            runCatching { gatt.disconnect() }
        }
    }

    private fun startHeartbeat() {
        disconnectHandler.removeCallbacks(heartbeatRunnable)
        disconnectHandler.postDelayed(heartbeatRunnable, HEARTBEAT_INTERVAL_MS)
    }

    private fun stopHeartbeat() {
        disconnectHandler.removeCallbacks(heartbeatRunnable)
    }

    @Suppress("DEPRECATION")
    private fun enqueueHeartbeat() {
        enqueue {
            val gatt = bluetoothGatt ?: return@enqueue false
            val target = controlTarget(gatt, ControlCommand.HEARTBEAT)
                ?: return@enqueue false
            val (characteristic, command) = target
            characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            characteristic.value = command
            controlWriteInFlight = ControlCommand.HEARTBEAT
            val started = runCatching { gatt.writeCharacteristic(characteristic) == true }
                .getOrDefault(false)
            if (!started) {
                controlWriteInFlight = null
            }
            started
        }
    }

    private fun controlTarget(
        gatt: BluetoothGatt,
        command: ControlCommand,
    ): Pair<BluetoothGattCharacteristic, ByteArray>? {
        val controlCharacteristic = gatt.getService(BleUuids.SVC_CONNECTION_CONTROL)
            ?.getCharacteristic(BleUuids.CHR_DISCONNECT_COMMAND)
        val characteristic = controlCharacteristic
            ?: gatt.getService(BleUuids.SVC_WIFI)
                ?.getCharacteristic(BleUuids.CHR_WIFI_SSID)
            ?: return null
        val value = if (controlCharacteristic != null) {
            byteArrayOf(command.value)
        } else {
            when (command) {
                ControlCommand.DISCONNECT -> CACHED_GATT_DISCONNECT_COMMAND
                ControlCommand.HEARTBEAT -> CACHED_GATT_HEARTBEAT_COMMAND
            }
        }
        return characteristic to value
    }

    private fun cancelDisconnectFallback() {
        disconnectFallback?.let(disconnectHandler::removeCallbacks)
        disconnectFallback = null
    }

    private fun teardownGatt(gatt: BluetoothGatt? = bluetoothGatt) {
        stopHeartbeat()
        cancelDisconnectFallback()
        clearQueue()
        runCatching { gatt?.close() }
        if (bluetoothGatt === gatt) {
            bluetoothGatt = null
        }
        _connectionState.value = ConnectionState.DISCONNECTED
        _connectedDeviceName.value = null
    }

    // ---- Public read/write API ------------------------------------------------

    /** Re-reads every characteristic exposed by the wand. */
    fun refreshAll() {
        readCharacteristic(BleUuids.SVC_DEVICE_INFO, BleUuids.CHR_MANUFACTURER_NAME)
        readCharacteristic(BleUuids.SVC_BATTERY, BleUuids.CHR_BATTERY_LEVEL)
        readCharacteristic(BleUuids.SVC_WIFI, BleUuids.CHR_WIFI_SSID)
        readCharacteristic(BleUuids.SVC_WIFI, BleUuids.CHR_WIFI_PASSWORD)
        readCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_POI)
        readCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_MAX_SYNC_ERROR)
        readCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_TARGET_IMU_SAMPLE_PERIOD)
        readCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_SYNC_SIGNAL_PULSE_WIDTH)
        readCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_SYNC_SIGNAL_DURATION)
        readCharacteristic(BleUuids.SVC_ONBOARD_IMU, BleUuids.CHR_ONBOARD_IMU_STATUS)
        readCharacteristic(BleUuids.SVC_ONBOARD_IMU, BleUuids.CHR_ONBOARD_IMU_SAMPLE_PERIOD)
    }

    fun writeWifiSsid(value: String) =
        writeCharacteristic(BleUuids.SVC_WIFI, BleUuids.CHR_WIFI_SSID, GattCodec.encodeUtf8(value))

    fun writeWifiPassword(value: String) =
        writeCharacteristic(BleUuids.SVC_WIFI, BleUuids.CHR_WIFI_PASSWORD, GattCodec.encodeUtf8(value))

    fun writePointOfInterest(value: Int) =
        writeCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_POI, GattCodec.encodeUInt16LE(value))

    fun writeMaxSyncError(value: Float) =
        writeCharacteristic(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_MAX_SYNC_ERROR, GattCodec.encodeFloatText(value))

    fun writeTargetImuSamplePeriod(value: Float) = writeCharacteristic(
        BleUuids.SVC_SYNC_PARAMS,
        BleUuids.CHR_TARGET_IMU_SAMPLE_PERIOD,
        GattCodec.encodeFloatText(value),
    )

    fun writeSyncSignalPulseWidth(value: Float) = writeCharacteristic(
        BleUuids.SVC_SYNC_PARAMS,
        BleUuids.CHR_SYNC_SIGNAL_PULSE_WIDTH,
        GattCodec.encodeFloatText(value),
    )

    fun writeOnboardImuSamplePeriod(value: Int) = writeCharacteristic(
        BleUuids.SVC_ONBOARD_IMU,
        BleUuids.CHR_ONBOARD_IMU_SAMPLE_PERIOD,
        GattCodec.encodeUInt16LE(value),
    )

    fun setBatteryNotificationsEnabled(enabled: Boolean) =
        setNotifications(BleUuids.SVC_BATTERY, BleUuids.CHR_BATTERY_LEVEL, enabled)

    fun setSyncDurationNotificationsEnabled(enabled: Boolean) =
        setNotifications(BleUuids.SVC_SYNC_PARAMS, BleUuids.CHR_SYNC_SIGNAL_DURATION, enabled)

    // ---- Queue-backed primitives -----------------------------------------------

    private fun readCharacteristic(serviceUuid: UUID, charUuid: UUID) {
        if (!hasConnectPermission()) return
        enqueue {
            val characteristic = bluetoothGatt?.getService(serviceUuid)?.getCharacteristic(charUuid)
                ?: return@enqueue false
            runCatching { bluetoothGatt?.readCharacteristic(characteristic) == true }.getOrDefault(false)
        }
    }

    @Suppress("DEPRECATION")
    private fun writeCharacteristic(serviceUuid: UUID, charUuid: UUID, value: ByteArray) {
        if (!hasConnectPermission()) return
        enqueue {
            val characteristic = bluetoothGatt?.getService(serviceUuid)?.getCharacteristic(charUuid)
                ?: return@enqueue false
            characteristic.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            characteristic.value = value
            runCatching { bluetoothGatt?.writeCharacteristic(characteristic) == true }.getOrDefault(false)
        }
    }

    @Suppress("DEPRECATION")
    private fun setNotifications(serviceUuid: UUID, charUuid: UUID, enable: Boolean) {
        if (!hasConnectPermission()) return
        enqueue {
            val gatt = bluetoothGatt ?: return@enqueue false
            val characteristic = gatt.getService(serviceUuid)?.getCharacteristic(charUuid)
                ?: return@enqueue false
            val localOk = runCatching { gatt.setCharacteristicNotification(characteristic, enable) }
                .getOrDefault(false)
            if (!localOk) return@enqueue false
            val cccd = characteristic.getDescriptor(BleUuids.CCCD) ?: return@enqueue false
            cccd.value = if (enable) {
                BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            } else {
                BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE
            }
            runCatching { gatt.writeDescriptor(cccd) == true }.getOrDefault(false)
        }
    }

    // ---- GATT callback ---------------------------------------------------------

    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    _connectionState.value = ConnectionState.DISCOVERING_SERVICES
                    runCatching { gatt.discoverServices() }
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    teardownGatt(gatt)
                }
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            _connectionState.value = ConnectionState.SYNCING_VALUES
            // Establish the firmware-side connection lease before the longer
            // initial read queue, so even a wedged GATT operation expires.
            enqueueHeartbeat()
            refreshAll()
            setBatteryNotificationsEnabled(true)
            setSyncDurationNotificationsEnabled(true)
            _connectionState.value = ConnectionState.READY
            startHeartbeat()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                _events.tryEmit("Service discovery failed (status $status)")
                disconnect()
                return
            }
            if (!targetMtuRequested) {
                targetMtuRequested = true
                val requested = runCatching { gatt.requestMtu(247) }.getOrDefault(false)
                if (!requested) {
                    // Fall back to default MTU if the request couldn't be issued.
                    onMtuChanged(gatt, 23, BluetoothGatt.GATT_SUCCESS)
                }
            }
        }

        @Suppress("DEPRECATION")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                applyCharacteristicValue(characteristic.uuid, characteristic.value)
            }
            completeOperation()
        }

        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
        ) {
            applyCharacteristicValue(characteristic.uuid, characteristic.value)
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            val controlCommand = controlWriteInFlight
            if (controlCommand != null) {
                controlWriteInFlight = null
                if (controlCommand == ControlCommand.DISCONNECT &&
                    status != BluetoothGatt.GATT_SUCCESS
                ) {
                    _events.tryEmit("Wand rejected disconnect command (status $status)")
                    runCatching { gatt.disconnect() }
                }
                completeOperation()
                return
            }

            if (status == BluetoothGatt.GATT_SUCCESS) {
                readCharacteristic(characteristic.service.uuid, characteristic.uuid)
            } else {
                _events.tryEmit("Write failed (status $status)")
            }
            completeOperation()
        }

        @Suppress("DEPRECATION")
        override fun onDescriptorWrite(
            gatt: BluetoothGatt,
            descriptor: BluetoothGattDescriptor,
            status: Int,
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val enabled = descriptor.value?.contentEquals(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE) == true
                when (descriptor.characteristic.uuid) {
                    BleUuids.CHR_BATTERY_LEVEL ->
                        _wandState.value = _wandState.value.copy(batteryNotificationsEnabled = enabled)
                    BleUuids.CHR_SYNC_SIGNAL_DURATION ->
                        _wandState.value = _wandState.value.copy(syncDurationNotificationsEnabled = enabled)
                }
            }
            completeOperation()
        }
    }

    private fun applyCharacteristicValue(uuid: UUID, value: ByteArray?) {
        val current = _wandState.value
        _wandState.value = when (uuid) {
            BleUuids.CHR_MANUFACTURER_NAME -> current.copy(manufacturerName = GattCodec.decodeUtf8(value))
            BleUuids.CHR_BATTERY_LEVEL -> current.copy(batteryLevelPercent = GattCodec.decodeUInt8(value))
            BleUuids.CHR_WIFI_SSID -> current.copy(wifiSsid = GattCodec.decodeUtf8(value))
            BleUuids.CHR_WIFI_PASSWORD -> current.copy(wifiPassword = GattCodec.decodeUtf8(value))
            BleUuids.CHR_POI -> current.copy(pointOfInterest = GattCodec.decodeUInt16LE(value))
            BleUuids.CHR_MAX_SYNC_ERROR -> current.copy(maxSyncErrorMs = GattCodec.decodeFloatText(value))
            BleUuids.CHR_TARGET_IMU_SAMPLE_PERIOD ->
                current.copy(targetImuSamplePeriodMs = GattCodec.decodeFloatText(value))
            BleUuids.CHR_SYNC_SIGNAL_PULSE_WIDTH ->
                current.copy(syncSignalPulseWidthMs = GattCodec.decodeFloatText(value))
            BleUuids.CHR_SYNC_SIGNAL_DURATION ->
                current.copy(syncSignalDurationMs = GattCodec.decodeFloatText(value))
            BleUuids.CHR_ONBOARD_IMU_STATUS -> current.copy(onboardImuStatus = GattCodec.decodeBool(value))
            BleUuids.CHR_ONBOARD_IMU_SAMPLE_PERIOD ->
                current.copy(onboardImuSamplePeriodMs = GattCodec.decodeUInt16LE(value))
            else -> current
        }
    }

    private companion object {
        const val DISCONNECT_TIMEOUT_MS = 2_000L
        const val HEARTBEAT_INTERVAL_MS = 2_000L
        val CACHED_GATT_DISCONNECT_COMMAND =
            byteArrayOf(0x00, 0xD1.toByte(), 0x5C, 0x0A, 0x4E, 0x44)
        val CACHED_GATT_HEARTBEAT_COMMAND =
            byteArrayOf(0x00, 0x48, 0x42, 0x0A, 0x4E, 0x44)
    }
}
