%% Split and Plot Magnetometer Data Based on timestamp_separation from imu_EM_log.csv

clear all;
clc;

% Define input CSV files
script_dir = fileparts(mfilename('fullpath'));
runtime_dir = fullfile(script_dir, '..', 'runtime');
csvFileSeparation = fullfile(runtime_dir, 'imu_EM_log.csv');
csvFileData = fullfile(runtime_dir, 'imu_data_log.csv');
targetPath = fullfile(runtime_dir, 'test_1'); % Directory where output files are saved
plotting = false;
if ~exist(targetPath, 'dir')
    mkdir(targetPath);
end

% Read the CSV files
timestamps_data = readtable(csvFileData);
timestamps_separation = readtable(csvFileSeparation);

% Extract separation timestamps and offsets
timestamps = timestamps_separation.timestamp_separation;
offsets = timestamps_separation.offset ./ 28000;

% Keep only samples before the last separation timestamp
valid_data = timestamps_data(timestamps_data.timestamp_IMU1 < max(timestamps), :);
valid_data = valid_data(2:end, :);  % pop first element

% Auto-detect the next available index
existingFiles = dir(fullfile(targetPath, 'sync_event_*.csv'));
existingIndices = regexp({existingFiles.name}, 'sync_event_(\d+)_.*ms.csv', 'tokens');
existingIndices = cellfun(@(x) str2double(x{1}{1}), existingIndices(~cellfun('isempty', existingIndices)), 'UniformOutput', true);
nextIndex = max([existingIndices, 0]) + 1;

% Add start time but exclude the last timestamp
timestamps = [min(valid_data.timestamp_IMU1); timestamps(:)];

% Create a new figure for plotting
figure;
sgtitle('Magnetometer Data for Each Synchronization Event');

% Loop through each synchronization event
for i = 1:length(timestamps)-1
    % Filter rows between the timestamps
    eventData = valid_data(valid_data.timestamp_IMU1 >= timestamps(i) & valid_data.timestamp_IMU1 < timestamps(i+1), :);
    
    % Determine the offset corresponding to this segment
    if i > 0 && i <= length(offsets)
        offset_ms = offsets(i);
    else
        offset_ms = 0;
    end
    
    % Write the filtered data to a new CSV file
    outputFile = fullfile(targetPath, sprintf('sync_event_%d_%.5fms.csv', nextIndex + i - 1, offset_ms));
    writetable(eventData, outputFile);
    
    % Extract magnetometer data for plotting
    timestamp_IMU1 = eventData.timestamp_IMU1;
    mag_X_IMU1 = eventData.mag_X_IMU1;
    mag_Y_IMU1 = eventData.mag_Y_IMU1;
    mag_Z_IMU1 = eventData.mag_Z_IMU1;

    timestamp_IMU2 = eventData.timestamp_IMU2;
    mag_X_IMU2 = eventData.mag_X_IMU2;
    mag_Y_IMU2 = eventData.mag_Y_IMU2;
    mag_Z_IMU2 = eventData.mag_Z_IMU2;

    mag_mod_IMU1 = mag_X_IMU1;
    mag_mod_IMU2 = mag_X_IMU2;

    if plotting == true
        % Subplot for each synchronization event
        subplot(length(timestamps)-1, 1, i);
        plot(timestamp_IMU1, mag_mod_IMU1, 'r-x', 'DisplayName', 'IMU1 Mag X'); hold on;
        plot(timestamp_IMU2, mag_mod_IMU2, 'b-x', 'DisplayName', 'IMU2 Mag X');
        title(sprintf('Synchronization Event %d', nextIndex + i - 1));
        xlabel('Time Elapsed (μs)');
        ylabel('Magnetic Field in X (arb. u.)');
        legend;
        grid on;
    end
end

fprintf('Magnetometer plots and CSV files generated for all synchronization events, starting from index %d.\n', nextIndex);
