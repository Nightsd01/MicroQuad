mex mex_gateway.cpp Shared.cpp QuadcopterController.cpp PIDController.cpp Fusion/FusionAhrs.cpp Fusion/FusionCompass.cpp Fusion/FusionOffset.cpp Filters/KalmanFilter.cpp Filters/MedianFilter.cpp
output = mex_gateway(data);

% requirements: Assumes output1 and output2 are defined

figure; 
    
function toggleVisibility(src, event, lineHandle)
    % This callback function toggles the visibility of a line
    if src.Value
        lineHandle.Visible = 'on';
    else
        lineHandle.Visible = 'off';
    end
end

function addSeriesLabels(plt, label1, label2, label3)
  % Adding legend
    legend(label1, label2, label3);
    
    % Adding data tips (available by default, no need for additional code here)
    
    % Adding UI control to toggle visibility of series
    uicontrol('Style', 'checkbox', 'String', 'Hide/Show Gyro X', ...
                       'Value',1,'Position', [20 20 140 20], ...
                       'Callback', {@toggleVisibility, plt(1)});
end

% Raw gyro plot
subplot(2, 3, 1); 
h1 = plot(data(:, 1), data(:, 8:10), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('Gyro Raw');
title('Raw Gyro Data');
addSeriesLabels(h1, 'X', 'Y', 'Z')

% Raw accel plot
subplot(2, 3, 2); 
h2 = plot(data(:, 1), data(:, 5:7), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('Accel Raw');
title('Raw Accel Data');
addSeriesLabels(h2, 'X', 'Y', 'Z')

% YPR plot
subplot(2, 3, 3); 
h3 = plot(output(:, 1), output(:, 5:7), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('YPR');
title('Yaw Pitch Roll Data');
addSeriesLabels(h3, 'Yaw', 'Pitch', 'Roll')

% Filtered gyro plot
subplot(2, 3, 4); 
h4 = plot(output(:, 1), output(:, 2:4), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('Gyro Filtered');
title('Filtered Gyro Data');
addSeriesLabels(h4, 'X', 'Y', 'Z')

% Filtered accel plot
subplot(2, 3, 5); 
h5 = plot(output(:, 1), output(:, 8:10), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('Accel Filtered');
title('Filtered Accel Data');
addSeriesLabels(h5, 'X', 'Y', 'Z')

% Gyro Y plot
subplot(2, 3, 6); 
h6 = plot(data(:, 1), data(:, 9), 'LineWidth', 1.5); 
xlabel('Time (ms)');
ylabel('Gyro Raw - Y');
title('Raw Gyro Data Y');
addSeriesLabels(h6, 'Y', 'Y', 'Y')