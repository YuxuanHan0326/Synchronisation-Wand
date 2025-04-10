import serial
import struct
import csv


def main():
    # Configure serial port
    ser = serial.Serial('COM5', baudrate=115200, timeout=1)  # Replace 'COMx' with your port
    log_file_main = "imu_data_log.csv"
    log_file_EM = "imu_EM_log.csv"

    sync_event_count = 1

    # Open the CSV files for logging
    with open(log_file_main, mode='w', newline='') as csv_file_main, open(log_file_EM, mode='w', newline='') as csv_file_0xFF:
        # Define headers for standard IMUs (0x01 & 0x02)
        fieldnames_main = [
            'timestamp_IMU1', 'accel_X_IMU1', 'accel_Y_IMU1', 'accel_Z_IMU1',
            'gyro_X_IMU1', 'gyro_Y_IMU1', 'gyro_Z_IMU1',
            'mag_X_IMU1', 'mag_Y_IMU1', 'mag_Z_IMU1',
            'timestamp_IMU2', 'accel_X_IMU2', 'accel_Y_IMU2', 'accel_Z_IMU2',
            'gyro_X_IMU2', 'gyro_Y_IMU2', 'gyro_Z_IMU2',
            'mag_X_IMU2', 'mag_Y_IMU2', 'mag_Z_IMU2'
        ]
        writer_main = csv.DictWriter(csv_file_main, fieldnames=fieldnames_main)
        writer_main.writeheader()

        # Define headers for IMU 0xFF
        fieldnames_0xFF = [
            'timestamp_separation', 'timestamp_sync_event', 'offset', 'mag_sampling_leading_time'
        ]
        writer_0xFF = csv.DictWriter(csv_file_0xFF, fieldnames=fieldnames_0xFF)
        writer_0xFF.writeheader()

        buffer = bytearray()

        try:
            while True:
                # Read data from the serial port
                data = ser.read(100)  # Adjust based on throughput
                if data:
                    buffer.extend(data)

                # Process complete samples in the buffer
                while len(buffer) >= 25:
                    if buffer[0] != 0x2C:
                        # Realign to the next header
                        print("Desynchronization detected! Realigning...")
                        buffer.pop(0)  # Remove the first byte and search for the next header
                        continue

                    # Extract a single 25-byte sample
                    sample = buffer[:25]
                    buffer = buffer[25:]

                    # Validate sample length
                    if len(sample) != 25:
                        print("Incomplete sample detected! Skipping...")
                        continue

                    # Parse the sample
                    try:
                        imu_id = sample[1]  # Get the IMU identifier

                        # **Parsing for IMU 0x01 & 0x02 (Standard)**
                        if imu_id in [0x01, 0x02]:
                            timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = struct.unpack(
                                '>5s3h3h3h', sample[2:]
                            )
                            timestamp = int.from_bytes(timestamp, byteorder='big', signed=False)

                            imu_data = {
                                f'timestamp_IMU{imu_id}': timestamp,
                                f'accel_X_IMU{imu_id}': accel_x,
                                f'accel_Y_IMU{imu_id}': accel_y,
                                f'accel_Z_IMU{imu_id}': accel_z,
                                f'gyro_X_IMU{imu_id}': gyro_x,
                                f'gyro_Y_IMU{imu_id}': gyro_y,
                                f'gyro_Z_IMU{imu_id}': gyro_z,
                                f'mag_X_IMU{imu_id}': mag_x,
                                f'mag_Y_IMU{imu_id}': mag_y,
                                f'mag_Z_IMU{imu_id}': mag_z
                            }

                            # Combine IMU1 and IMU2 data into one row (if both available)
                            if imu_id == 0x01:
                                imu1_data = imu_data
                            elif imu_id == 0x02:
                                imu2_data = imu_data
                                if 'imu1_data' in locals():
                                    writer_main.writerow({**imu1_data, **imu2_data})
                                    del imu1_data, imu2_data

                        # **Parsing for EM 0xFF (Different Structure)**
                        elif imu_id == 0xFF:

                            # Structure: 1-byte header + 1-byte ID + 5-byte timestamp_1 + 5-byte timestamp_2 + 4-byte offset (ignore rest)
                            timestamp_separation = int.from_bytes(sample[2:7], byteorder='big', signed=False)
                            timestamp_sync_event = int.from_bytes(sample[7:12], byteorder='big', signed=False)
                            offset = int.from_bytes(sample[12:16], byteorder='big', signed=False)
                            mag_sampling_leading_time = int.from_bytes(sample[17:21], byteorder='big', signed=False)

                            # Write to the separate 0xFF log file
                            writer_0xFF.writerow({
                                'timestamp_separation': timestamp_separation,
                                'timestamp_sync_event': timestamp_sync_event,
                                'offset': offset,
                                'mag_sampling_leading_time': mag_sampling_leading_time
                            })

                            print(f"Sync event {sync_event_count} finished! Time offset = {offset / 28000} ms")
                            sync_event_count += 1

                    except struct.error:
                        print("Invalid sample format! Skipping...")
                        continue

        except KeyboardInterrupt:
            print("Logging stopped.")


if __name__ == "__main__":
    main()
