import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft
from scipy.signal import butter, filtfilt, hilbert, iirfilter
import re
import sys
import os

def crossing(x, thresh):
    return np.where((x[:-1] < thresh) & (x[1:] >= thresh) |
                    (x[:-1] > thresh) & (x[1:] <= thresh))[0]

# Parse command-line arguments
if len(sys.argv) > 1 and os.path.isfile(sys.argv[1]):
    read_type = "log"
    log_path = sys.argv[1]
else:
    read_type = "serial"

loop_time = []
pressure = []

if read_type == "serial":
    import serial
    import time

    port = "/dev/ttyACM0"
    baud = 115200
    s = serial.Serial(port, baud, timeout=1)
    s.flush()

    s.write(b"start\n")
    print("Reading from serial...")

    while True:
        if s.in_waiting > 0:
            line = s.readline().decode("utf-8").strip()
            time_match = re.search(r"Loop Time: ([\d\.]+)", line)
            pressure_match = re.search(r"Pressure: ([\d\.]+)", line)

            if time_match and pressure_match:
                p = float(pressure_match.group(1))
                t = float(time_match.group(1))

                if p >= 200:
                    print("Stopping: pressure threshold reached.")
                    break

                loop_time.append(t)
                pressure.append(p)

    s.close()
else:
    print(f"Reading from log file: {log_path}")
    with open(log_path, "r") as f:
        for line in f:
            line = line.strip()
            if line == "":
                continue
            time_match = re.search(r"Loop Time: ([\d\.]+)", line)
            pressure_match = re.search(r"Pressure: ([\d\.]+)", line)
            if time_match and pressure_match:
                p = float(pressure_match.group(1))
                t = float(time_match.group(1))
                if p >= 200:
                    break
                loop_time.append(t)
                pressure.append(p)

loop_time = np.array(loop_time)
pressure = np.array(pressure)

# Interpolation
dt = np.mean(np.diff(loop_time))
fs = 1 / dt
t_uniform = np.arange(loop_time[0], loop_time[-1], dt)
p_uniform = np.interp(t_uniform, loop_time, pressure)

# FFT
n = len(p_uniform)
f = np.arange(n) * fs / n
Y = fft(p_uniform)
mag = np.abs(Y) / n
half_n = n // 2
f = f[:half_n]
mag = mag[:half_n]

# Plotting
plt.figure(figsize=(12, 10))
plt.subplot(3, 2, 1)
plt.plot(t_uniform, p_uniform)
plt.title("Raw Pressure Signal")
plt.xlabel("Time [s]")
plt.ylabel("Pressure")
plt.grid(True)

plt.subplot(3, 2, 2)
plt.plot(f, mag)
plt.xlim([0, 5])
plt.title("Zoomed FFT: Heart Rate Range (0–5 Hz)")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid(True)

# Bandpass filter
b, a = iirfilter(6, [0.5 / (fs / 2), 3 / (fs / 2)], btype='band', ftype='butter')
p_filtered = filtfilt(b, a, p_uniform)

plt.subplot(3, 2, 3)
plt.plot(t_uniform, p_filtered, 'r')
plt.title("Bandpass Filtered Pressure (0.5–3 Hz)")
plt.xlabel("Time [s]")
plt.ylabel("Filtered Pressure")
plt.grid(True)

Y = fft(p_filtered)
mag = np.abs(Y) / n
mag = mag[:half_n]

plt.subplot(3, 2, 4)
plt.plot(f, mag)
plt.xlim([0, 5])
plt.title("Filtered FFT: Heart Rate Range (0–5 Hz)")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid(True)

# Peak and Q
idx_peak = np.argmax(mag)
peak_freq = f[idx_peak]
peak_mag = mag[idx_peak]
half_power = peak_mag / np.sqrt(2)

left_idx = np.where(mag[:idx_peak] < half_power)[0][-1]
right_idx = idx_peak + np.where(mag[idx_peak:] < half_power)[0][0]

f_left = np.interp(half_power, mag[left_idx:left_idx+2], f[left_idx:left_idx+2])
f_right = np.interp(half_power, mag[right_idx-1:right_idx+1], f[right_idx-1:right_idx+1])
bandwidth = f_right - f_left
Q = peak_freq / bandwidth
thresh_relax = Q / 18

# Refined bandpass
b, a = iirfilter(6, [(peak_freq - 0.25) / (fs / 2), (peak_freq + 0.25) / (fs / 2)], btype='band', ftype='butter')
p_filtered2 = filtfilt(b, a, p_filtered)

# Hilbert envelope + smoothing
envelope = np.abs(hilbert(p_filtered2 ** 2))
fc = 0.5
b, a = butter(4, fc / (fs / 2))
env_smooth = filtfilt(b, a, envelope)

# Systolic/diastolic detection
thresh1 = 0.35 * np.max(env_smooth) / thresh_relax
thresh2 = 0.55 * np.max(env_smooth) * thresh_relax
systolic = crossing(env_smooth, thresh1)[0]
diastolic = crossing(env_smooth, thresh2)[-1]

plt.subplot(3, 2, 5)
plt.plot(t_uniform, env_smooth, 'r')
plt.title("Pressure Transform (Filtered Hilbert²)")
plt.xlabel("Time [s]")
plt.ylabel("Filtered Pressure")
plt.grid(True)
plt.axhline(thresh1, linestyle='--', color="#0072BD")
plt.axhline(thresh2, linestyle='--', color="#D95319")
plt.plot(t_uniform[systolic], env_smooth[systolic], 'o', color="#0072BD")
plt.plot(t_uniform[diastolic], env_smooth[diastolic], 'o', color="#D95319")
plt.axvline(t_uniform[systolic], linestyle='--', color="#0072BD")
plt.axvline(t_uniform[diastolic], linestyle='--', color="#D95319")

print(f"Systolic: {p_uniform[systolic]:.3f}")
print(f"Diastolic: {p_uniform[diastolic]:.3f}")

# Final FFT
Y = fft(p_filtered2)
mag = np.abs(Y) / n
mag = mag[:half_n]

plt.subplot(3, 2, 6)
plt.plot(f, mag)
plt.xlim([0, 5])
plt.title("Filtered x2 FFT: Heart Rate Range (0–5 Hz)")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude")
plt.grid(True)

plt.tight_layout()
plt.show()
