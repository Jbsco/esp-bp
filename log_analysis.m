clear all; close all; clc;
% Enhanced pressure log parser and analyzer

%% Choose between reading a log file from serial output or serial direct
read_type = "log";
% read_type = "serial";

% NOTE: Some Linux MATLAB installations may encounter lock file errors, try:
% sudo touch /run/lock/LCK..ttyACM0
% sudo chgrp $USER /run/lock/LCK..ttyACM0
% sudo chown $USER /run/lock/LCK..ttyACM0

if read_type == "serial"
    % Set up serial port
    %port = "COM3"; % <-- Change to your serial port
    %port = "/dev/ttyUSB0"
    port = "/dev/ttyACM0"
    baud = 115200; % <-- Change to your device's baud rate
    s = serialport(port, baud);
    configureTerminator(s, "LF");
    flush(s); % clear buffer
    
    % Send start command
    writeline(s, "start");  % <-- Adjust if a different command is used
    
    % Initialize storage
    loop_time = [];
    pressure = [];
    
    disp("Reading from serial...");

    % Read until pressure >= 200
    tic;
    while true
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));
            time_match = regexp(line, 'Loop Time: ([\d\.]+)', 'tokens');
            pressure_match = regexp(line, 'Pressure: ([\d\.]+)', 'tokens');
    
            if ~isempty(time_match) && ~isempty(pressure_match)
                p = str2double(pressure_match{1}{1});
                t = str2double(time_match{1}{1});
    
                if p >= 200
                    disp("Stopping: pressure threshold reached.");
                    break;
                end
    
                loop_time(end+1) = t;
                pressure(end+1) = p;
            end
        end
    end
    
    
    % Close serial port
    clear s;
else
    filename = 'run logs/log_9_hs.txt';
    lines = readlines(filename);
    
    loop_time = [];
    pressure = [];
    
    % Parse the log file
    for i = 1:length(lines)
        line = strtrim(lines(i));
        if line == ""
            continue;
        end
        time_match = regexp(line, 'Loop Time: ([\d\.]+)', 'tokens');
        pressure_match = regexp(line, 'Pressure: ([\d\.]+)', 'tokens');
        if ~isempty(time_match) && ~isempty(pressure_match)
            if str2double(pressure_match{1}{1}) >= 200
                break;
            end
            loop_time(end+1) = str2double(time_match{1}{1});
            pressure(end+1) = str2double(pressure_match{1}{1});
        end
    end
end

% Interpolate to uniform timebase
dt = mean(diff(loop_time));
fs = 1 / dt; % sampling frequency
t_uniform = loop_time(1):dt:loop_time(end);
p_uniform = interp1(loop_time, pressure, t_uniform, 'linear');

% FFT of raw signal
n = length(p_uniform);
f = (0:n-1)*(fs/n);
Y = fft(p_uniform);
mag = abs(Y)/n;
half_n = floor(n/2);
f = f(1:half_n);
mag = mag(1:half_n);

% Plot 1: Pressure vs Time (Raw)
figure;
subplot(3,2,1);
plot(t_uniform, p_uniform);
xlabel('Time [s]');
ylabel('Pressure');
title('Raw Pressure Signal');
grid on;

% Plot 2: Zoomed FFT (0–5 Hz)
subplot(3,2,2);
plot(f, mag);
xlim([0 5]);
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Zoomed FFT: Heart Rate Range (0–5 Hz)');
grid on;

% Bandpass filter: 0.5–3 Hz
bpFilt = designfilt('bandpassiir', ...
    'FilterOrder', 6, ...
    'HalfPowerFrequency1', 0.5, ...
    'HalfPowerFrequency2', 3, ...
    'SampleRate', fs);

p_filtered = filtfilt(bpFilt, p_uniform);

% Plot 3: Filtered signal (Heart Rate Only)
subplot(3,2,3);
plot(t_uniform, p_filtered, 'r');
xlabel('Time [s]');
ylabel('Filtered Pressure');
title('Bandpass Filtered Pressure (0.5–3 Hz)');
grid on;

% Plot 4: Filtered FFT (0–5 Hz)
Y = fft(p_filtered);
mag = abs(Y)/n;
half_n = floor(n/2);
f = f(1:half_n);
mag = mag(1:half_n);

subplot(3,2,4);
plot(f, mag);
xlim([0 5]);
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Filtered FFT: Heart Rate Range (0–5 Hz)');
grid on;

%% Iterate on peak frequency
[peak_mag, idx_peak] = max(mag);
peak_freq = f(idx_peak);
% Also find the Q factor to use in threshold value relax factor
% Find -3dB bandwidth (half power)
half_power = peak_mag / sqrt(2);  % -3 dB point
% Search for left and right -3dB crossing
left_idx = find(mag(1:idx_peak) < half_power, 1, 'last');
right_idx = find(mag(idx_peak:end) < half_power, 1, 'first') + idx_peak - 1;
% Interpolate for better precision
f_left = interp1(mag(left_idx:left_idx+1), f(left_idx:left_idx+1), half_power);
f_right = interp1(mag(right_idx-1:right_idx), f(right_idx-1:right_idx), half_power);
bandwidth = f_right - f_left;
Q = peak_freq / bandwidth
thresh_relax = Q / 18

% Bandpass filter: 0.5–3 Hz
bpFilt = designfilt('bandpassiir', ...
    'FilterOrder', 6, ...
    'HalfPowerFrequency1', peak_freq-0.25, ...
    'HalfPowerFrequency2', peak_freq+0.25, ...
    'SampleRate', fs);

p_filtered2 = filtfilt(bpFilt, p_filtered);

% Get the envelope from the Hilbert transform
pressure_envelope = abs(hilbert(p_filtered2.^2));
fc = 0.5; % cutoff frequency for smoothing (Hz)
[b, a] = butter(4, fc/(fs/2));  % 4th-order Butterworth
env_smooth = filtfilt(b, a, pressure_envelope);  % zero-phase filtering

% Set thresholds and find crossings from transformed pressure
thresh1 = 0.35*max(env_smooth)/thresh_relax;
thresh2 = 0.55*max(env_smooth)*thresh_relax;
function ind = crossing(x, thresh)
    % Returns indices where signal x crosses threshold going upwards or downwards
    ind = find((x(1:end-1) < thresh & x(2:end) >= thresh) | ...
               (x(1:end-1) > thresh & x(2:end) <= thresh));
end
diastolic = crossing(env_smooth, thresh1);
systolic = crossing(env_smooth, thresh2);
diastolic = diastolic(1);
systolic = systolic(end);

% Plot 3: Filtered signal (Heart Rate Only)
subplot(3,2,5);
plot(t_uniform, env_smooth, 'r');
xlabel('Time [s]');
ylabel('Filtered Pressure');
title('Pressure Transform (Filtered Hilbert^2)');
grid on; hold on;
yline(thresh1,'--','Color',"#0072BD");
yline(thresh2,'--','Color',"#D95319");
dia_print = sprintf('Diastolic: %.3f\n',p_uniform(diastolic))
sys_print = sprintf('Systolic: %.3f\n',p_uniform(systolic))
plot(t_uniform(diastolic),env_smooth(diastolic),'o','Color',"#0072BD");
plot(t_uniform(systolic),env_smooth(systolic),'o','Color',"#D95319");
xline(t_uniform(diastolic),'--','Color',"#0072BD");
xline(t_uniform(systolic),'--','Color',"#D95319");
subplot(3,2,3); hold on;
xline(t_uniform(diastolic),'--','Color',"#0072BD");
xline(t_uniform(systolic),'--','Color',"#D95319");
subplot(3,2,1); hold on;
xline(t_uniform(diastolic),'--','Color',"#0072BD");
xline(t_uniform(systolic),'--','Color',"#D95319");

% Plot 4: Filtered FFT (0–5 Hz)
Y = fft(p_filtered2);
mag = abs(Y)/n;
half_n = floor(n/2);
f = f(1:half_n);
mag = mag(1:half_n);

subplot(3,2,6);
plot(f, mag);
xlim([0 5]);
xlabel('Frequency [Hz]');
ylabel('Magnitude');
title('Filtered x2 FFT: Heart Rate Range (0–5 Hz)');
grid on;