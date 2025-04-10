%%
clear all
clc

%%
% Read and plot IMU data from CSV

% Define the CSV file name (update this if needed)
csvFile = '.\imu_data_log.csv';

% Read the CSV data
data = readtable(csvFile);

% Extract columns (update names based on your CSV structure)
timestamp_IMU1 = data.timestamp_IMU1;
accel_X_IMU1 = data.accel_X_IMU1;
accel_Y_IMU1 = data.accel_Y_IMU1;
accel_Z_IMU1 = data.accel_Z_IMU1;
gyro_X_IMU1 = data.gyro_X_IMU1;
gyro_Y_IMU1 = data.gyro_Y_IMU1;
gyro_Z_IMU1 = data.gyro_Z_IMU1;
mag_X_IMU1 = data.mag_X_IMU1;
mag_Y_IMU1 = data.mag_Y_IMU1;
mag_Z_IMU1 = data.mag_Z_IMU1;

timestamp_IMU2 = data.timestamp_IMU2;
accel_X_IMU2 = data.accel_X_IMU2;
accel_Y_IMU2 = data.accel_Y_IMU2;
accel_Z_IMU2 = data.accel_Z_IMU2;
gyro_X_IMU2 = data.gyro_X_IMU2;
gyro_Y_IMU2 = data.gyro_Y_IMU2;
gyro_Z_IMU2 = data.gyro_Z_IMU2;
mag_X_IMU2 = data.mag_X_IMU2;
mag_Y_IMU2 = data.mag_Y_IMU2;
mag_Z_IMU2 = data.mag_Z_IMU2;

% Calculate vector moduli for IMU1
accel_mod_IMU1 = sqrt(accel_X_IMU1.^2 + accel_Y_IMU1.^2 + accel_Z_IMU1.^2);
gyro_mod_IMU1 = sqrt(gyro_X_IMU1.^2 + gyro_Y_IMU1.^2 + gyro_Z_IMU1.^2);
mag_mod_IMU1 = mag_X_IMU1;

% Calculate vector moduli for IMU2
accel_mod_IMU2 = sqrt(accel_X_IMU2.^2 + accel_Y_IMU2.^2 + accel_Z_IMU2.^2);
gyro_mod_IMU2 = sqrt(gyro_X_IMU2.^2 + gyro_Y_IMU2.^2 + gyro_Z_IMU2.^2);
mag_mod_IMU2 = mag_X_IMU2;

% Plot results
figure;
sgtitle('IMU Data Comparison'); % Title for the figure

% Accelerometer
subplot(3, 1, 1);
plot(timestamp_IMU1, accel_mod_IMU1, 'r', 'DisplayName', 'IMU1 Accel'); hold on;
plot(timestamp_IMU2, accel_mod_IMU2, 'b', 'DisplayName', 'IMU2 Accel');
title('Accelerometer (Modulus)');
xlabel('Time Elapsed (μs)');
ylabel('Acceleration (arb. u.)');
legend;
grid on;

% Gyroscope
subplot(3, 1, 2);
plot(timestamp_IMU1, gyro_mod_IMU1, 'r', 'DisplayName', 'IMU1 Gyro'); hold on;
plot(timestamp_IMU2, gyro_mod_IMU2, 'b', 'DisplayName', 'IMU2 Gyro');
title('Gyroscope (Modulus)');
xlabel('Time Elapsed (μs)');
ylabel('Angular Velocity (arb. u.)');
legend;
grid on;

% Magnetometer
subplot(3, 1, 3);
plot(timestamp_IMU1, mag_X_IMU1, 'r-x', 'DisplayName', 'IMU1 Mag'); hold on;
plot(timestamp_IMU2, mag_X_IMU2, 'b-x', 'DisplayName', 'IMU2 Mag');
title('Magnetometer (Modulus)');
xlabel('Time Elapsed (μs)');
ylabel('Readings in x (arb. u.)');
legend;
grid on;
