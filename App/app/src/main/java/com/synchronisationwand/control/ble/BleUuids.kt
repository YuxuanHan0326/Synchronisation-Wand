package com.synchronisationwand.control.ble

import java.util.UUID

/**
 * GATT service/characteristic UUIDs, mirrored from the firmware's
 * Firmware/main/ble_gatt.h so the two stay in lock-step.
 */
object BleUuids {

    const val DEVICE_NAME = "SYNCHRONISATION WAND"

    val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    val SVC_DEVICE_INFO: UUID = UUID.fromString("0000180a-0000-1000-8000-00805f9b34fb")
    val CHR_MANUFACTURER_NAME: UUID = UUID.fromString("00002a29-0000-1000-8000-00805f9b34fb")

    val SVC_BATTERY: UUID = UUID.fromString("0000180f-0000-1000-8000-00805f9b34fb")
    val CHR_BATTERY_LEVEL: UUID = UUID.fromString("00002a19-0000-1000-8000-00805f9b34fb")

    val SVC_WIFI: UUID = UUID.fromString("cae6f78a-c2cf-4dea-b1cf-4db4208191a4")
    val CHR_WIFI_SSID: UUID = UUID.fromString("645be00e-ce39-4537-b6cc-050fe1978a02")
    val CHR_WIFI_PASSWORD: UUID = UUID.fromString("c877d27c-1049-4314-8681-9ae1603f1693")

    val SVC_SYNC_PARAMS: UUID = UUID.fromString("ccd806bf-e77a-437a-99f6-d0293c791288")
    val CHR_POI: UUID = UUID.fromString("2a5b8a41-d52f-472c-97ff-0c6f25c2b5c1")
    val CHR_MAX_SYNC_ERROR: UUID = UUID.fromString("4b43f4ca-a1a1-4b85-a4d1-78ba0f4248e9")
    val CHR_TARGET_IMU_SAMPLE_PERIOD: UUID = UUID.fromString("90636119-33bd-42de-b02d-704a197ed724")
    val CHR_SYNC_SIGNAL_PULSE_WIDTH: UUID = UUID.fromString("a92adf1d-50a8-47ce-8f92-df6489e66de2")
    val CHR_SYNC_SIGNAL_DURATION: UUID = UUID.fromString("fbff0f62-e4b3-440f-8a9f-549bc0523c58")

    val SVC_ONBOARD_IMU: UUID = UUID.fromString("61d91904-e6ce-4ffe-8000-8072f1f881cf")
    val CHR_ONBOARD_IMU_STATUS: UUID = UUID.fromString("0842e3d0-a040-4d66-80c8-95d2d3878d7d")
    val CHR_ONBOARD_IMU_SAMPLE_PERIOD: UUID = UUID.fromString("e22dc86f-f46c-40b6-9327-cff9d3ba0857")

    val SVC_CONNECTION_CONTROL: UUID = UUID.fromString("9f6d7c20-9e47-4e2b-8a61-6f5e1d2c3b40")
    val CHR_DISCONNECT_COMMAND: UUID = UUID.fromString("9f6d7c21-9e47-4e2b-8a61-6f5e1d2c3b40")
}
