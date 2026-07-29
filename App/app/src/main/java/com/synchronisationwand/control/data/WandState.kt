package com.synchronisationwand.control.data

enum class ConnectionState {
    DISCONNECTED,
    SCANNING,
    CONNECTING,
    DISCOVERING_SERVICES,
    SYNCING_VALUES,
    READY,
    DISCONNECTING,
}

/** Snapshot of every value read from (or pushed via notification by) the wand. */
data class WandState(
    val manufacturerName: String? = null,

    val batteryLevelPercent: Int? = null,
    val batteryNotificationsEnabled: Boolean = false,

    val wifiSsid: String? = null,
    val wifiPassword: String? = null,

    val pointOfInterest: Int? = null,
    val maxSyncErrorMs: Float? = null,
    val targetImuSamplePeriodMs: Float? = null,
    val syncSignalPulseWidthMs: Float? = null,
    val syncSignalDurationMs: Float? = null,
    val syncDurationNotificationsEnabled: Boolean = false,

    val onboardImuStatus: Boolean? = null,
    val onboardImuSamplePeriodMs: Int? = null,
)

/** A minimal, permission-safe representation of a scanned BLE device. */
data class ScannedDevice(
    val address: String,
    val name: String?,
    val rssi: Int,
)
