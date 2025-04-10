%% Analyze Time Intervals in IMU Data Log (Including IMU2 - IMU1 Time Difference)

% Clear workspace and command window
clear;
clc;

% Define the CSV file name
csvFile = 'imu_data_log.csv';

% User-defined clock frequency (Hz)
clockFrequency_Hz = 28e6; % Example: 28 MHz, change as needed

% Read the CSV file
data = readtable(csvFile);

% Compute time intervals (differences between successive timestamps in clock ticks)
timeIntervals_IMU1_ticks = diff(data.timestamp_IMU1);
timeIntervals_IMU2_ticks = diff(data.timestamp_IMU2);

% Compute IMU2 - IMU1 timestamp differences (in clock ticks)
delta_t_IMU2_IMU1_ticks = data.timestamp_IMU2 - data.timestamp_IMU1;

% Convert clock ticks to milliseconds (ms)
tickPeriod_ms = (1 / clockFrequency_Hz) * 1e3; % Convert clock period to milliseconds
timeIntervals_IMU1_ms = timeIntervals_IMU1_ticks * tickPeriod_ms;
timeIntervals_IMU2_ms = timeIntervals_IMU2_ticks * tickPeriod_ms;
delta_t_IMU2_IMU1_ms = delta_t_IMU2_IMU1_ticks * tickPeriod_ms;

% Compute statistical parameters for IMU1, IMU2, and IMU2-IMU1 difference
stats_IMU1.mean = mean(timeIntervals_IMU1_ms);
stats_IMU1.std = std(timeIntervals_IMU1_ms);
stats_IMU1.min = min(timeIntervals_IMU1_ms);
stats_IMU1.max = max(timeIntervals_IMU1_ms);

stats_IMU2.mean = mean(timeIntervals_IMU2_ms);
stats_IMU2.std = std(timeIntervals_IMU2_ms);
stats_IMU2.min = min(timeIntervals_IMU2_ms);
stats_IMU2.max = max(timeIntervals_IMU2_ms);

stats_IMU2_IMU1.mean = mean(delta_t_IMU2_IMU1_ms);
stats_IMU2_IMU1.std = std(delta_t_IMU2_IMU1_ms);
stats_IMU2_IMU1.min = min(delta_t_IMU2_IMU1_ms);
stats_IMU2_IMU1.max = max(delta_t_IMU2_IMU1_ms);

% Display statistical parameters
fprintf('IMU1 Time Interval Stats (in milliseconds):\n');
fprintf('Mean: %.6f ms, Std: %.6f ms, Min: %.6f ms, Max: %.6f ms\n', ...
    stats_IMU1.mean, stats_IMU1.std, stats_IMU1.min, stats_IMU1.max);

fprintf('\nIMU2 Time Interval Stats (in milliseconds):\n');
fprintf('Mean: %.6f ms, Std: %.6f ms, Min: %.6f ms, Max: %.6f ms\n', ...
    stats_IMU2.mean, stats_IMU2.std, stats_IMU2.min, stats_IMU2.max);

fprintf('\nIMU2 - IMU1 Time Difference Stats (in milliseconds):\n');
fprintf('Mean: %.6f ms, Std: %.6f ms, Min: %.6f ms, Max: %.6f ms\n', ...
    stats_IMU2_IMU1.mean, stats_IMU2_IMU1.std, stats_IMU2_IMU1.min, stats_IMU2_IMU1.max);

% Plot the distribution of time intervals
figure;
subplot(3,1,1);
histogram(timeIntervals_IMU1_ms, 50, 'FaceColor', 'r');
title('Distribution of Time Intervals - IMU1');
xlabel('Time Interval (ms)');
ylabel('Frequency');
grid on;

subplot(3,1,2);
histogram(timeIntervals_IMU2_ms, 50, 'FaceColor', 'b');
title('Distribution of Time Intervals - IMU2');
xlabel('Time Interval (ms)');
ylabel('Frequency');
grid on;

subplot(3,1,3);
histogram(delta_t_IMU2_IMU1_ms, 50, 'FaceColor', 'g');
title('Distribution of IMU2 - IMU1 Time Differences');
xlabel('Time Difference (ms)');
ylabel('Frequency');
grid on;
