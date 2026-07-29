package com.synchronisationwand.control.ble

import java.util.Locale

/**
 * Encode/decode helpers matching the exact wire format each characteristic
 * uses in Firmware/main/ble_gatt.c:
 *  - Free-text and float characteristics are transferred as UTF-8 strings
 *    (floats formatted with 3 decimal places, C/US locale "." separator).
 *  - Small numeric/boolean status characteristics (battery level, POI,
 *    on-board IMU status/sample period) are transferred as raw binary
 *    (little-endian for multi-byte values), matching the firmware's
 *    os_mbuf_append(&value, sizeof(value)) calls.
 */
object GattCodec {

    fun encodeUtf8(value: String): ByteArray = value.toByteArray(Charsets.UTF_8)

    fun decodeUtf8(bytes: ByteArray?): String =
        bytes?.toString(Charsets.UTF_8) ?: ""

    fun encodeFloatText(value: Float): ByteArray =
        String.format(Locale.US, "%.3f", value).toByteArray(Charsets.UTF_8)

    fun decodeFloatText(bytes: ByteArray?): Float? {
        if (bytes == null || bytes.isEmpty()) return null
        return bytes.toString(Charsets.UTF_8).trim().toFloatOrNull()
    }

    fun encodeUInt16LE(value: Int): ByteArray {
        val v = value.coerceIn(0, 0xFFFF)
        return byteArrayOf((v and 0xFF).toByte(), ((v shr 8) and 0xFF).toByte())
    }

    fun decodeUInt16LE(bytes: ByteArray?): Int? {
        if (bytes == null || bytes.size < 2) return null
        return (bytes[0].toInt() and 0xFF) or ((bytes[1].toInt() and 0xFF) shl 8)
    }

    fun decodeUInt8(bytes: ByteArray?): Int? {
        if (bytes == null || bytes.isEmpty()) return null
        return bytes[0].toInt() and 0xFF
    }

    fun decodeBool(bytes: ByteArray?): Boolean? {
        if (bytes == null || bytes.isEmpty()) return null
        return (bytes[0].toInt() and 0xFF) != 0
    }
}
