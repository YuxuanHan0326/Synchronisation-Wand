% Define parameters
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
max_sync_error_ms = 0.5;
T_inter_mag_offset = 2.75;
T_sample = 40;

% Get list of CSV files in the folder
folder_path = fullfile(data_dir, '0.5ms', 'Labeled Events');
file_list = dir(fullfile(folder_path, '*.csv'));

% Initialize storage for results
num_files = length(file_list);
test1_errors = zeros(num_files, 1);
test2_errors = zeros(num_files, 1);
test1_results = zeros(num_files, 1);
test2_results = zeros(num_files, 1);

% Lists to store filenames of failed tests
failed_test1_files = {};
failed_test2_files = {};

% Loop through each file
for i = 1:num_files
    filename = fullfile(folder_path, file_list(i).name);
    
    % Extract time parameter from filename
    time_str = regexp(file_list(i).name, '_(\d+\.\d+)ms\.csv', 'tokens', 'once'); % Extracts time
    T_desync_imu1 = str2double(time_str{1});
    
    if T_desync_imu1 > T_sample
        T_desync_imu1 = T_desync_imu1 - T_sample;
    end

    % Read CSV file
    data = readtable(filename);

    % Extract pulses
    imu1_plus_pulse = find_first_plus_pulse(data, 'IMU1');
    imu2_plus_pulse = find_first_plus_pulse(data, 'IMU2');

    % Compute Test 1 Sync Error
    if imu2_plus_pulse >= imu1_plus_pulse
        test1_sync_error = abs((imu2_plus_pulse - imu1_plus_pulse) * max_sync_error_ms - T_inter_mag_offset);
    else
        test1_sync_error = abs((imu2_plus_pulse - imu1_plus_pulse) * max_sync_error_ms - T_inter_mag_offset + T_sample);
    end
    
    % Compute Test 2 Sync Error
    if imu1_plus_pulse <= T_sample / max_sync_error_ms
        test2_sync_error = abs(imu1_plus_pulse * max_sync_error_ms - T_desync_imu1);
    else
        test2_sync_error = abs((imu1_plus_pulse - T_sample / max_sync_error_ms) * max_sync_error_ms - T_desync_imu1);
    end

    % Store errors
    test1_errors(i) = test1_sync_error;
    test2_errors(i) = test2_sync_error;

    % Determine pass/fail
    test1_results(i) = test1_sync_error <= max_sync_error_ms;
    test2_results(i) = test2_sync_error <= max_sync_error_ms;

    % Store filenames of failed tests
    if ~test1_results(i)
        failed_test1_files{end+1} = file_list(i).name;
    end
    if ~test2_results(i)
        failed_test2_files{end+1} = file_list(i).name;
    end
end

% Calculate pass rates
test1_pass_rate = mean(test1_results) * 100;
test2_pass_rate = mean(test2_results) * 100;

% Display results
fprintf('Test1 Pass Rate: %.2f%%\n', test1_pass_rate);
fprintf('Test2 Pass Rate: %.2f%%\n', test2_pass_rate);

% Print failed test filenames
if ~isempty(failed_test1_files)
    fprintf('\nFiles that failed Test 1:\n');
    fprintf('%s\n', failed_test1_files{:});
else
    fprintf('\nAll files passed Test 1.\n');
end

if ~isempty(failed_test2_files)
    fprintf('\nFiles that failed Test 2:\n');
    fprintf('%s\n', failed_test2_files{:});
else
    fprintf('\nAll files passed Test 2.\n');
end

% Plot distributions
figure;
histogram(test1_errors, 'BinWidth', 0.1);
xlabel('Test1 Sync Error (ms)');
ylabel('Frequency');
title('Distribution of Test1 Sync Error');

figure;
histogram(test2_errors, 'BinWidth', 0.1);
xlabel('Test2 Sync Error (ms)');
ylabel('Frequency');
title('Distribution of Test2 Sync Error');

% Function to find first occurrence of '+'
function pulse_no = find_first_plus_pulse(table_data, imu_suffix)
    marker_col = table_data.(['Marker_' imu_suffix]);
    pulse_col = table_data.(['Pulse_No_' imu_suffix]);
    
    valid_idx = find(strcmp(marker_col, '+') & pulse_col ~= 0, 1, 'first');
    if isempty(valid_idx)
        pulse_no = NaN; % Handle missing values
    else
        pulse_no = pulse_col(valid_idx);
    end
end

% Function to find extra sample pulses
function pulse_no = find_extra_sample_pulse(table_data, imu_suffix)
    extra_col = table_data.(['Extra_Sample_Pulse_' imu_suffix]);
    pulse_col = table_data.(['Pulse_No_' imu_suffix']);
    pulse_no = pulse_col(extra_col == 1);
end
