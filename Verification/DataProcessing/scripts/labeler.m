% Clear workspace and command window
clear;
clc;

% Configure Path (e.g. the 2ms folder) and sync event
script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
folder_name = '0.5ms';
max_sync_error_ms = 0.5;
imu1_invert_axis = false;
imu2_invert_axis = true;

% Define reference sample number
reference_sample_number = 8;

% Define time_difference between two IMUs in ms
Inter_IMU_time_offset_ms = 2.75;

offset_compensation_ms = 0.705;  % add to theoretical offset. 0.705

% Define the target folder
targetPath = fullfile(data_dir, folder_name); % Change folder_name above if needed

% Display Info?
display_plot = false;
display_message = false;
force_step_through = false;

% Get a list of all files matching the pattern
filePattern = fullfile(targetPath, 'sync_event_*_*ms.csv');
csvFiles = dir(filePattern);

for k = 1:length(csvFiles)
    % Get the file name
    fileName = csvFiles(k).name;
    
    % Extract index and offset using regex
    tokens = regexp(fileName, 'sync_event_(\d+)_([\d\.]+)ms.csv', 'tokens');

    index = str2double(tokens{1}{1});  % Extract index
    offset_IMU1 = str2double(tokens{1}{2}) + offset_compensation_ms; % Extract offset
    offset_IMU2 = offset_IMU1 + Inter_IMU_time_offset_ms;
    extra_sample_pulse_IMU1 = ceil(offset_IMU1 / max_sync_error_ms);
    extra_sample_pulse_IMU2 = ceil(offset_IMU2/ max_sync_error_ms);

    % Get the full file name
    fileNameFull = fullfile(targetPath, fileName);
    
    % Read the CSV file
    eventData = readtable(fileNameFull);
    
    % Display file processing message
    fprintf('Processing file: %s\n', csvFiles(k).name);

    % Extract relevant data
    timestamp_IMU1 = eventData.timestamp_IMU1;
    mag_X_IMU1 = eventData.mag_X_IMU1;
    mag_Y_IMU1 = eventData.mag_Y_IMU1;
    mag_Z_IMU1 = eventData.mag_Z_IMU1;

    timestamp_IMU2 = eventData.timestamp_IMU2;
    mag_X_IMU2 = eventData.mag_X_IMU2;
    mag_Y_IMU2 = eventData.mag_Y_IMU2;
    mag_Z_IMU2 = eventData.mag_Z_IMU2;

    % Change to correct direction
    if imu1_invert_axis == true
        mag_mod_IMU1 = - mag_X_IMU1;
    else
        mag_mod_IMU1 = mag_X_IMU1;
    end

    if imu2_invert_axis == true
        mag_mod_IMU2 = - mag_X_IMU2;
    else
        mag_mod_IMU2 = mag_X_IMU2;
    end

    % Determine threshold
    maxValue_IMU1 = max(mag_mod_IMU1);
    minValue_IMU1 = min(mag_mod_IMU1);
    threshold_IMU1 = (maxValue_IMU1 + minValue_IMU1) / 2;
    maxValue_IMU2 = max(mag_mod_IMU2);
    minValue_IMU2 = min(mag_mod_IMU2);
    threshold_IMU2 = (maxValue_IMU2 + minValue_IMU2) / 2;

    % Initialize variables
    peakColors = {'r', 'g', 'b', 'm', 'c', 'y', 'k'};

    peakIndices_IMU1 = {};
    criticalValues_IMU1 = []; % To store critical values for each test
    criticalTime_IMU1 = [];   % To store the time of the critical point
    shiftedPeaks_IMU1 = [];   % To mark peaks with shifted points
    rejectedPeaks_IMU1 = [];  % To mark peaks with rejected first samples
    anomalousPeaks_IMU1 = [];  % To mark peaks with anomalous last samples

    peakIndices_IMU2 = {};
    criticalValues_IMU2 = []; % To store critical values for each test
    criticalTime_IMU2 = [];   % To store the time of the critical point
    shiftedPeaks_IMU2 = [];   % To mark peaks with shifted points
    rejectedPeaks_IMU2 = [];  % To mark peaks with rejected first samples
    anomalousPeaks_IMU2 = [];  % To mark peaks with anomalous last samples

    % User-defined parameters
    significanceLevel = 1e-6; % Confidence level for hypothesis test
    weight = [10, 1, 10]; % Weights for first, current, and last peaks


    % Processing
    [peakIndices_IMU1, criticalValues_IMU1, criticalTime_IMU1, shiftedPeaks_IMU1, rejectedPeaks_IMU1, anomalousPeaks_IMU1] = process_IMU_data(mag_mod_IMU1, timestamp_IMU1, threshold_IMU1, significanceLevel, weight);
    [peakIndices_IMU2, criticalValues_IMU2, criticalTime_IMU2, shiftedPeaks_IMU2, rejectedPeaks_IMU2, anomalousPeaks_IMU2] = process_IMU_data(mag_mod_IMU2, timestamp_IMU2, threshold_IMU2, significanceLevel, weight);


    % Plotting
    if display_plot == true
        % ----------------------------- Plot IMU 1 -------------------------------
        % Plot the square wave and highlight peaks
        figure;
        subplot(2, 1, 1);
        plot(timestamp_IMU1, mag_mod_IMU1, 'k', 'DisplayName', 'Square Wave');
        hold on;
        plot(timestamp_IMU1, threshold_IMU1 * ones(size(timestamp_IMU1)), '--b', 'LineWidth', 1.5, 'DisplayName', 'Threshold');

        % Plot critical values and peaks
        for i = 1:length(peakIndices_IMU1)
            peakTime = timestamp_IMU1(peakIndices_IMU1{i});
            peakWave = mag_mod_IMU1(peakIndices_IMU1{i});
            plot(peakTime, peakWave, 'Color', peakColors{mod(i-1, length(peakColors))+1}, ...
                'LineWidth', 1.5, 'DisplayName', sprintf('Peak %d', i));
            plot(peakTime, peakWave, 'x', 'Color', peakColors{mod(i-1, length(peakColors))+1});
        end

        % Annotate critical values on the graph
        for k = 1:length(criticalValues_IMU1)
            plot(criticalTime_IMU1(k), criticalValues_IMU1(k), 'r+', 'MarkerSize', 8, 'DisplayName', sprintf('Critical Value (Peak %d)', k));
        end

        title('Square Wave with Peaks, Critical Values Highlighted');
        xlabel('Time');
        ylabel('Amplitude');
        % legend show;
        grid on;
        hold off;

        % ---------------------------- Plot IMU 2 --------------------------------
        subplot(2, 1, 2);
        plot(timestamp_IMU2, mag_mod_IMU2, 'k', 'DisplayName', 'Square Wave');
        hold on;
        plot(timestamp_IMU2, threshold_IMU2 * ones(size(timestamp_IMU2)), '--b', 'LineWidth', 1.5, 'DisplayName', 'Threshold');

        % Plot critical values and peaks
        for i = 1:length(peakIndices_IMU2)
            peakTime = timestamp_IMU2(peakIndices_IMU2{i});
            peakWave = mag_mod_IMU2(peakIndices_IMU2{i});
            plot(peakTime, peakWave, 'Color', peakColors{mod(i-1, length(peakColors))+1}, ...
                'LineWidth', 1.5, 'DisplayName', sprintf('Peak %d', i));
            plot(peakTime, peakWave, 'x', 'Color', peakColors{mod(i-1, length(peakColors))+1});
        end

        % Annotate critical values on the graph
        for k = 1:length(criticalValues_IMU2)
            plot(criticalTime_IMU2(k), criticalValues_IMU2(k), 'r+', 'MarkerSize', 8, 'DisplayName', sprintf('Critical Value (Peak %d)', k));
        end

        title('Square Wave with Peaks, Critical Values Highlighted');
        xlabel('Time');
        ylabel('Amplitude');
        % legend show;
        grid on;
        hold off;
    end

    % Calculate Sample Numbers

    % Exclude the first and last peaks
    usablePeaks_IMU1 = peakIndices_IMU1(2:end-1);
    usablePeaks_IMU2 = peakIndices_IMU2(2:end-1);
    
    if display_message == true
        % Print IMU1 sample number for each pulse
        fprintf('\nIMU1 sample number for each pulse (Reference = %d):\n', reference_sample_number);
        fprintf('--------------------------------------\n');
        fprintf('Initial Peak: %d samples%s%s\n', length(usablePeaks_IMU1{1}), mark_if_shifted(2, shiftedPeaks_IMU1, rejectedPeaks_IMU1), mark_if_anomalous(2, anomalousPeaks_IMU1));

        for i = 2:length(usablePeaks_IMU1)
            fprintf('%d%s Peak: %d samples%s%s\n', i-1, ordinal_suffix(i-1), length(usablePeaks_IMU1{i}), mark_if_shifted(i+1, shiftedPeaks_IMU1, rejectedPeaks_IMU1), mark_if_anomalous(i+1, anomalousPeaks_IMU1));
        end

        % Print IMU2 sample number for each pulse
        fprintf('\nIMU2 sample number for each pulse (Reference = %d):\n', reference_sample_number);
        fprintf('--------------------------------------\n');
        fprintf('Initial Peak: %d samples%s%s\n', length(usablePeaks_IMU2{1}), mark_if_shifted(2, shiftedPeaks_IMU2, rejectedPeaks_IMU2), mark_if_anomalous(2, anomalousPeaks_IMU2));

        for i = 2:length(usablePeaks_IMU2)
            fprintf('%d%s Peak: %d samples%s%s\n', i-1, ordinal_suffix(i-1), length(usablePeaks_IMU2{i}), mark_if_shifted(i+1, shiftedPeaks_IMU2, rejectedPeaks_IMU2), mark_if_anomalous(i+1, anomalousPeaks_IMU2));
        end
    end

    % Initialize arrays (use appropriate types)
    numPeaks_IMU1 = length(usablePeaks_IMU1);
    pulseNumbers_IMU1 = (0:numPeaks_IMU1-1)';  % Ensure row count matches

    markers_IMU1 = strings(numPeaks_IMU1, 1);  % String array for markers
    leading_edge_sample_IMU1 = zeros(numPeaks_IMU1, 1); % Numeric array for 0 or 1
    anomalous_IMU1 = zeros(numPeaks_IMU1, 1); % Numeric array for 0 or 1
    sample_number_IMU1 = zeros(numPeaks_IMU1, 1); % Numeric array for sample count
    is_extra_IMU1 = zeros(numPeaks_IMU1, 1); % For the extra sample point

    for i = 1:numPeaks_IMU1
        % Determine marker
        if length(usablePeaks_IMU1{i}) == reference_sample_number
            markers_IMU1(i) = "=";
        elseif length(usablePeaks_IMU1{i}) < reference_sample_number
            markers_IMU1(i) = "-";
        else
            markers_IMU1(i) = "+";
        end

        % Determine leading edge sample
        if any(shiftedPeaks_IMU1 == i+1) || any(rejectedPeaks_IMU1 == i+1)
            leading_edge_sample_IMU1(i) = 1;
        end

        % Determine anomalous sample
        if any(anomalousPeaks_IMU1 == i+1)
            anomalous_IMU1(i) = 1;
        end

        % Record number of samples for each peak
        sample_number_IMU1(i) = length(usablePeaks_IMU1{i});

        % Record if this is extra sample pulse
        if extra_sample_pulse_IMU1 == i - 1
            is_extra_IMU1(i) = 1;
        else
            is_extra_IMU1(i) = 0;
        end
    end

    % Initialize arrays (use appropriate types)
    numPeaks_IMU2 = length(usablePeaks_IMU2);
    pulseNumbers_IMU2 = (0:numPeaks_IMU2-1)';  % Ensure row count matches

    markers_IMU2 = strings(numPeaks_IMU2, 1);  % String array for markers
    leading_edge_sample_IMU2 = zeros(numPeaks_IMU2, 1); % Numeric array for 0 or 1
    anomalous_IMU2 = zeros(numPeaks_IMU2, 1); % Numeric array for 0 or 1
    sample_number_IMU2 = zeros(numPeaks_IMU2, 1); % Numeric array for sample count
    is_extra_IMU2 = zeros(numPeaks_IMU2, 1); % For the extra sample point


    for i = 1:numPeaks_IMU2
        % Determine marker
        if length(usablePeaks_IMU2{i}) == reference_sample_number
            markers_IMU2(i) = "=";
        elseif length(usablePeaks_IMU2{i}) < reference_sample_number
            markers_IMU2(i) = "-";
        else
            markers_IMU2(i) = "+";
        end

        % Determine leading edge sample
        if any(shiftedPeaks_IMU2 == i+1) || any(rejectedPeaks_IMU2 == i+1)
            leading_edge_sample_IMU2(i) = 1;
        end

        % Determine anomalous sample
        if any(anomalousPeaks_IMU2 == i+1)
            anomalous_IMU2(i) = 1;
        end

        % Record number of samples for each peak
        sample_number_IMU2(i) = length(usablePeaks_IMU2{i});

        % Record if this is extra sample pulse
        if extra_sample_pulse_IMU2 == i - 1
            is_extra_IMU2(i) = 1;
        else
            is_extra_IMU2(i) = 0;
        end
    end


    % Create table for CSV
    to_csv_data = table(pulseNumbers_IMU1, sample_number_IMU1, markers_IMU1, leading_edge_sample_IMU1, anomalous_IMU1, is_extra_IMU1, pulseNumbers_IMU2, sample_number_IMU2, markers_IMU2, leading_edge_sample_IMU2, anomalous_IMU2, is_extra_IMU2, ...
        'VariableNames', {'Pulse_No_IMU1', 'Number_of_Samples_IMU1', 'Marker_IMU1', 'Leading_Edge_Sample_IMU1', 'Anomalous_IMU1', 'Extra_Sample_Pulse_IMU1', 'Pulse_No_IMU2', 'Number_of_Samples_IMU2', 'Marker_IMU2', 'Leading_Edge_Sample_IMU2', 'Anomalous_IMU2', 'Extra_Sample_Pulse_IMU2'});

    % Write table to CSV file
    writetable(to_csv_data, fullfile(targetPath, 'Labeled Events', sprintf('labeled_event_%d_%5fms.csv', index, offset_IMU1)));

    % Print confirmation
    fprintf(' *** CSV file has been saved successfully! ***\n');
    
    if display_message || display_plot || force_step_through == true
        input('Enter anything to continue > ', 's');
        close all;
    end
end

%% Functions

function [peakIndices, criticalValues, criticalTime, shiftedPeaks, rejectedPeaks, anomalousPeaks] = process_IMU_data(mag_mod, timestamp, threshold, significanceLevel, weight)

% Initialize variables
startIndex = 1;
peakIndices = {};
criticalValues = [];
criticalTime = [];
shiftedPeaks = [];
rejectedPeaks = [];
anomalousPeaks = [];

if mag_mod(1) < threshold
    currentState = 'low';
else
    currentState = 'high';
end

% Classify peaks
for i = 2:length(mag_mod)
    if strcmp(currentState, 'low') && mag_mod(i) > threshold
        peakIndices{end+1} = startIndex:i-1;
        startIndex = i;
        currentState = 'high';
    elseif strcmp(currentState, 'high') && mag_mod(i) < threshold
        peakIndices{end+1} = startIndex:i-1;
        startIndex = i;
        currentState = 'low';
    end
end
peakIndices{end+1} = startIndex:length(mag_mod);

% Handle the first and last peaks
firstPeak = mag_mod(peakIndices{1});
firstVar = var(firstPeak(2:end-1)); % Exclude first and last samples

lastPeak = mag_mod(peakIndices{end});
lastVar = var(lastPeak(2:end-1)); % Exclude first and last samples

% Store reclassification information
reclassify = zeros(1, length(peakIndices));

% Apply hypothesis testing for the first and last sample of each peak
for k = 1:length(peakIndices)-1
    % Extract current peak samples (excluding first and last)
    currentPeak = mag_mod(peakIndices{k});
    innerPeak = currentPeak(2:end-1);

    meanCurrent = mean(innerPeak);

    varCurrent = var(innerPeak);

    % Weighted variance
    combinedVariance = (weight(1)*firstVar + weight(2)*varCurrent + weight(3)*lastVar) / sum(weight);

    % First Sample Test
    firstSample = currentPeak(1);
    if mag_mod(peakIndices{k}(1)) > threshold
        % High peak: Alternative hypothesis is that the first sample is lower
        criticalValue = meanCurrent + tinv(significanceLevel, length(innerPeak) - 1) * sqrt(combinedVariance / length(innerPeak));
        if firstSample < criticalValue
            rejectedPeaks = [rejectedPeaks, k]; % Mark this peak as rejected
        end
    else
        % Low peak: Alternative hypothesis is that the first sample is higher
        criticalValue = meanCurrent - tinv(significanceLevel, length(innerPeak) - 1) * sqrt(combinedVariance / length(innerPeak));
        if firstSample > criticalValue
            rejectedPeaks = [rejectedPeaks, k]; % Mark this peak as rejected
        end
    end

    % Last Sample Test
    lastSample = currentPeak(end);
    criticalValues = [criticalValues; criticalValue];
    criticalTime = [criticalTime; timestamp(peakIndices{k}(end))];

    if (mag_mod(peakIndices{k}(1)) > threshold && lastSample < criticalValue) || ...
            (mag_mod(peakIndices{k}(1)) <= threshold && lastSample > criticalValue)
        % Mark this peak for reclassification
        reclassify(k) = 1;
    end
end

% Apply reclassification after all tests
for k = 1:length(peakIndices)-1
    if reclassify(k) == 1
        % Reclassify the last sample to the next peak
        lastSample = peakIndices{k}(end);
        peakIndices{k} = peakIndices{k}(1:end-1);
        peakIndices{k+1} = [lastSample; peakIndices{k+1}(:)];

        % Mark the next peak as containing a shifted point
        shiftedPeaks = [shiftedPeaks, k+1];
    end
end

% Re-test last sample with opposite hypothesis to identify anomalies
for k = 1:length(peakIndices)-1
    currentPeak = mag_mod(peakIndices{k});
    innerPeak = currentPeak(2:end-1);
    meanCurrent = mean(innerPeak);
    varCurrent = var(innerPeak);

    combinedVariance = (weight(1)*firstVar + weight(2)*varCurrent + weight(3)*lastVar) / sum(weight);

    % Opposite hypothesis test for the last sample
    lastSample = currentPeak(end);
    if k == 1
        fprintf('test');
    end
    if mag_mod(peakIndices{k}(2)) > threshold
        % High peak: Alternative hypothesis is that the last sample is higher
        criticalValue = meanCurrent - tinv(significanceLevel, length(innerPeak) - 1) * sqrt(combinedVariance / length(innerPeak));
        if lastSample > criticalValue
            anomalousPeaks = [anomalousPeaks, k]; % Mark this peak as anomalous
        end
    else
        % Low peak: Alternative hypothesis is that the last sample is lower
        criticalValue = meanCurrent + tinv(significanceLevel, length(innerPeak) - 1) * sqrt(combinedVariance / length(innerPeak));
        if lastSample < criticalValue
            anomalousPeaks = [anomalousPeaks, k]; % Mark this peak as anomalous
        end
    end
end
end


% Helper function to mark shifted or rejected peaks
function mark = mark_if_shifted(peakIndex, shiftedPeaks, rejectedPeaks)
if any(shiftedPeaks == peakIndex) || any(rejectedPeaks == peakIndex)
    mark = '*';
else
    mark = '';
end
end

% Helper function to mark anomalous peaks
function mark = mark_if_anomalous(peakIndex, anomalousPeaks)
if any(anomalousPeaks == peakIndex)
    mark = '!';
else
    mark = '';
end
end

% Helper function to determine ordinal suffix
function suffix = ordinal_suffix(n)
if mod(n, 10) == 1 && mod(n, 100) ~= 11
    suffix = 'st';
elseif mod(n, 10) == 2 && mod(n, 100) ~= 12
    suffix = 'nd';
elseif mod(n, 10) == 3 && mod(n, 100) ~= 13
    suffix = 'rd';
else
    suffix = 'th';
end
end
