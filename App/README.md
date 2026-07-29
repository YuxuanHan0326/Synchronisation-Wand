# Synchronisation Wand — Android Control App

A small Kotlin + Jetpack Compose app that connects to the Synchronisation
Wand over Bluetooth Low Energy, shows every value exposed by its GATT
profile, and lets you edit the writable ones. Built and tested against
a Pixel 7 (minSdk 26, targetSdk/compileSdk 34).

## Opening the project

1. Open the `App/` folder in Android Studio (Koala or newer).
2. Android Studio will notice there's no Gradle wrapper checked in and
   offer to create one — accept that (or run `gradle wrapper --gradle-version 8.7`
   yourself from `App/` if you have a system Gradle installed). The wrapper
   jar is a binary file, so it isn't included in this patch.
3. Let Gradle sync, then run the `app` configuration on a device or the
   Pixel 7 emulator image (BLE requires a physical device — the emulator
   has no real Bluetooth radio).

No other setup is required; there are no third-party BLE libraries, just
the platform `android.bluetooth` APIs.

## What it does

- Scans for a device advertising the name `SYNCHRONISATION WAND` and lists
  it (with RSSI) so you can tap to connect.
- On connect, discovers services, requests a larger MTU (needed for the
  Wi-Fi password field), and reads every characteristic in the profile.
- Shows values grouped exactly like the GATT tree: Device Information,
  Battery, Wi-Fi Configuration, Synchronisation Parameters, On-board IMU
  Settings, and Connection Control.
- Lets you edit every writable characteristic (Wi-Fi SSID/password, Point
  of Interest, Maximum Synchronisation Error, Target IMU Sample Period,
  Synchronisation Signal Pulse Width, On-board IMU Sample Period) with
  inline validation before the save button becomes active. After a
  successful write the app reads the characteristic back, so what's shown
  reflects what the firmware actually stored (it clamps/truncates some
  inputs — see `Firmware/main/ble_gatt.c`).
- Subscribes to notifications for Battery Level and Synchronisation Signal
  Duration, with a toggle to turn them on/off.
- Serialises every Android GATT operation through a FIFO queue. Android only
  supports one outstanding GATT operation at a time, so this avoids silent
  failures caused by overlapping reads, writes, and descriptor updates.
- Sends a connection heartbeat every two seconds. If Android leaves a stale
  controller link behind, the firmware's eight-second connection lease
  terminates it and resumes advertising automatically.
- Requests a peripheral-initiated disconnect before closing the Android GATT
  client, allowing the wand to clear its OLED Bluetooth indicator and become
  discoverable again immediately.

## Wire format — matched to the firmware, not just the report

The Figure 19 GATT diagram / report text say "all writes are UTF-8 text",
but the current firmware (`Firmware/main/ble_gatt.c`) actually mixes two
encodings, and the app follows the firmware since that's what will
actually be on the wire:

| Characteristic | Wire format |
|---|---|
| Manufacturer Name | UTF-8 text |
| Battery Level | raw `uint8` (0–100), **read + notify only, no write** |
| Wi-Fi SSID / Password | UTF-8 text |
| Point of Interest | raw `uint16`, little-endian |
| Maximum Synchronisation Error | UTF-8 text, float formatted to 3 decimals |
| Target IMU Sample Period | UTF-8 text, float formatted to 3 decimals |
| Synchronisation Signal Pulse Width | UTF-8 text, float formatted to 3 decimals |
| Synchronisation Signal Duration | UTF-8 text, float; **read + notify only, no write** (firmware recalculates it automatically) |
| On-board IMU Status | raw `uint8`/bool, read-only |
| On-board IMU Sample Period | raw `uint16`, little-endian |
| Disconnect Command | raw `uint8`: `0x01` disconnect, `0x02` heartbeat |

If the firmware is later changed to make Synchronisation Signal Duration
writable, or to send everything as UTF-8, update `GattCodec.kt` and the
corresponding `WandBleManager` read/write calls to match — the app is
intentionally centralised there so that's a one-file change.

## Known limitations / things to revisit

- BLE scanning matches on the advertised device name only (the firmware's
  advertisement packet doesn't include service UUIDs, so filtering by
  service isn't possible without connecting first).
- No pairing/bonding or encryption is configured — this mirrors the
  firmware, which doesn't request it either.
- The connection is only kept alive while the app is in the foreground
  (no foreground service), matching the "hand-held configuration tool"
  use case described in the report. If the app process disappears without a
  clean disconnect, the firmware heartbeat lease releases the link.
