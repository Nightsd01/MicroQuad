% timestamp 1
% yaw 2
% pitch 3
% roll 4
% raw accel x 5
% raw accel y 6
% raw accel z 7
% raw gyro x 8
% raw gyro y 9
% raw gyro z 10
% filtered accel x 11
% filtered accel y 12
% filtered accel z 13
% filtered gyro x 14
% filtered gyro y 15
% filtered gyro z 16
% angle out yaw 17
% angle out pitch 18
% angle out roll 19
% rate out yaw 20
% rate out pitch 21
% rate out roll 22
% motor 1 23
% motor 2 24
% motor 3 25
% motor 4 26
% mag heading 27
% throttle 28
% voltage 29

function analyze_flight(directory)
    videoFilePath = fullfile(directory, 'flight.mov');
    if ~isfile(videoFilePath)
        error('The flight.mov file does not exist in the specified directory: %s', directory);
    end

    dataFilePath = fullfile(directory, 'debug.csv');
    if ~isfile(dataFilePath)
        error('The debug.csv file does not exist in the specified directory: %s', directory);
    end

    % Read the data
    opts = detectImportOptions(dataFilePath);
    data = readmatrix(dataFilePath, opts);

    % Define start and end data times (in microseconds, as given)
    data_start = data(1, 1);
    data_end = data(end, 1);

    % Open the video
    videoReader = VideoReader(videoFilePath);

    figure;

    % Show the first frame
    subplot(2, 3, 1);
    hImage = imshow(readFrame(videoReader));
    title('Video Playback');

    hLines = [];

    % Post-Filter Gyro Data
    subplot(2, 3, 2);
    plot(data(:, 1), data(:, 14:16), 'LineWidth', 1.5);
    hold on;
    hLines(end + 1) = xline(data_start, 'r', 'LineWidth', 2);
    xlabel('Time (seconds)');
    ylabel('Degrees per Second');
    title('Post-Filter Gyro Data');
    legend('X', 'Y', 'Z');

    % Post-Filter Accelerometer Data
    subplot(2, 3, 3);
    plot(data(:, 1), data(:, 11:13), 'LineWidth', 1.5);
    hold on;
    hLines(end + 1) = xline(data_start, 'r', 'LineWidth', 2);
    xlabel('Time (seconds)');
    ylabel('Gs per Second');
    title('Post-Filter Accelerometer Data');
    legend('X', 'Y', 'Z');

    % Yaw Pitch Roll
    subplot(2, 3, 4);
    plot(data(:, 1), data(:, 2:4), 'LineWidth', 1.5);
    hold on;
    hLines(end + 1) = xline(data_start, 'r', 'LineWidth', 2);
    xlabel('Time (seconds)');
    ylabel('Degrees');
    title('Yaw Pitch Roll');
    legend('Yaw', 'Pitch', 'Roll');

    % Raw Gyro Data
    subplot(2, 3, 5);
    plot(data(:, 1), data(:, 8:10), 'LineWidth', 1.5);
    hold on;
    hLines(end + 1) = xline(data_start, 'r', 'LineWidth', 2);
    xlabel('Time (seconds)');
    ylabel('Degrees per Second');
    title('Raw Gyro Data');
    legend('X', 'Y', 'Z');

    % Raw Accelerometer Data
    subplot(2, 3, 6);
    plot(data(:, 1), data(:, 5:7), 'LineWidth', 1.5);
    hold on;
    hLines(end + 1) = xline(data_start, 'r', 'LineWidth', 2);
    xlabel('Time (seconds)');
    ylabel('Gs per Second');
    title('Raw Accelerometer Data');
    legend('X', 'Y', 'Z');

    % Create slider for manual scrubbing
    % Slider in terms of video time [0, videoReader.Duration]
    hSlider = uicontrol('Style', 'slider', ...
        'Min', 0, 'Max', videoReader.Duration, 'Value', 0, ...
        'Units', 'normalized', 'Position', [0.1 0.01 0.8 0.05], ...
        'Callback', @(src, ~) scrubSliderCallback(src, videoReader, hImage, hLines, data, data_start, data_end), ...
        'SliderStep', [0.01, 0.1]);

    addlistener(hSlider, 'ContinuousValueChange', @(src, ~) scrubSliderCallback(src, videoReader, hImage, hLines, data, data_start, data_end));

    % Add Play/Pause button
    isPlaying = true; % Initial state - playing
    uicontrol('Style', 'togglebutton', 'String', 'Pause', ...
        'Units', 'normalized', 'Position', [0.92 0.01 0.07 0.05], ...
        'Callback', @(src, ~) playPauseCallback(src));

    % Playback and update loop
    while hasFrame(videoReader)
        if isPlaying
            frame = readFrame(videoReader);
            set(hImage, 'CData', frame);
            currentTime = videoReader.CurrentTime;

            % Map video time to data time
            actualTime = data_start + (currentTime / videoReader.Duration) * (data_end - data_start);

            % Find the closest time in the dataset
            [~, closestIdx] = min(abs(data(:, 1) - actualTime));
            closestTime = data(closestIdx, 1);

            % Update vertical lines
            for i = 1:length(hLines)
                set(hLines(i), 'Value', closestTime);
            end

            % Update the slider position
            set(hSlider, 'Value', currentTime);

            % Stop if we reach the end of the data
            if closestIdx == size(data, 1)
                isPlaying = false;
                break;
            end

            % Pause to match video frame rate
            pause(1/videoReader.FrameRate);
        else
            % If paused, just pause for a short time before checking again
            pause(0.1);
        end
    end

    function scrubSliderCallback(src, videoReader, hImage, hLines, data, data_start, data_end)
        % Seek to the specified time in the video
        videoReader.CurrentTime = min(src.Value, videoReader.Duration);
        frame = readFrame(videoReader);
        set(hImage, 'CData', frame);

        % Map video time to data time
        t_video = videoReader.CurrentTime;
        actualTime = data_start + (t_video / videoReader.Duration) * (data_end - data_start);

        [~, closestIdx] = min(abs(data(:, 1) - actualTime));
        closestTime = data(closestIdx, 1);

        for i = 1:length(hLines)
            set(hLines(i), 'Value', closestTime);
        end
    end

    function playPauseCallback(src)
        if src.Value == 1
            isPlaying = false;
            src.String = 'Play';
        else
            isPlaying = true;
            src.String = 'Pause';
        end
    end
end