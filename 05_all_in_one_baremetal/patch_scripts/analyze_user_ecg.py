import serial
import json
import time
import numpy as np

s = serial.Serial('COM3', 115200, timeout=0.5)
time.sleep(0.2)
s.reset_input_buffer()

print("Capturing 8 seconds of live telemetry...")
t0 = time.time()
ecg_samples = []
imu_samples = []

while time.time() - t0 < 8.0:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if not (line.startswith('{') and line.endswith('}')):
        continue
    try:
        data = json.loads(line)
        if "e" in data:
            ecg_samples.append((time.time() - t0, data["e"][0], data["e"][1], data["e"][2]))
        elif data.get("type") == "imu":
            imu_samples.append(data)
    except:
        pass

s.close()

print(f"Captured {len(ecg_samples)} ECG samples and {len(imu_samples)} IMU samples in 8.0s")
if not ecg_samples:
    print("No ECG samples received!")
    exit(0)

times = np.array([x[0] for x in ecg_samples])
dt = np.diff(times)
fs_empirical = 1.0 / np.median(dt)
print(f"Empirical ECG sample rate: {fs_empirical:.2f} Hz (mean dt: {np.mean(dt)*1000:.2f} ms)")

stats = [x[1] for x in ecg_samples]
ch1_raw = np.array([x[2] for x in ecg_samples])
ch2_raw = np.array([x[3] for x in ecg_samples])
ch2_mv = ch2_raw * 0.000048077

print(f"Status byte distribution: {dict((x, stats.count(x)) for x in set(stats))}")
print(f"CH2 mV: min={np.min(ch2_mv):.3f}, max={np.max(ch2_mv):.3f}, mean={np.mean(ch2_mv):.3f}, pk-pk={np.ptp(ch2_mv):.3f}")

# Now let's test a simple high-pass and band-pass filter at the actual empirical sample rate
# 2nd order Butterworth highpass at 0.5Hz, lowpass at 40Hz
from scipy.signal import butter, filtfilt, iirnotch

try:
    nyq = 0.5 * fs_empirical
    b_hp, a_hp = butter(2, 0.5 / nyq, btype='high')
    b_lp, a_lp = butter(2, min(35.0, nyq - 2.0) / nyq, btype='low')
    
    filtered = filtfilt(b_hp, a_hp, ch2_mv)
    filtered = filtfilt(b_lp, a_lp, filtered)
    
    # 50 Hz notch if nyquist allows
    if nyq > 52.0:
        b_n, a_n = iirnotch(50.0 / nyq, 30.0)
        filtered = filtfilt(b_n, a_n, filtered)
        print("Applied 50 Hz notch filter")
    else:
        print(f"Nyquist frequency ({nyq:.1f} Hz) is too low for 50 Hz notch! (Data rate {fs_empirical:.1f} Hz)")

    print(f"\nFiltered ECG mV: min={np.min(filtered):.4f}, max={np.max(filtered):.4f}, ptp={np.ptp(filtered):.4f}, std={np.std(filtered):.4f}")
    
    # Find peaks with prominence
    from scipy.signal import find_peaks
    peaks_pos, props_pos = find_peaks(filtered, height=0.08, distance=int(0.35 * fs_empirical), prominence=0.05)
    peaks_neg, props_neg = find_peaks(-filtered, height=0.08, distance=int(0.35 * fs_empirical), prominence=0.05)
    
    print(f"\nPositive peaks found: {len(peaks_pos)} at times: {np.round(times[peaks_pos], 2)}")
    print(f"Negative peaks found: {len(peaks_neg)} at times: {np.round(times[peaks_neg], 2)}")
    
    if len(peaks_pos) >= 2:
        rr = np.diff(times[peaks_pos])
        bpm = 60.0 / np.median(rr)
        print(f"Heart Rate from POSITIVE peaks: {bpm:.1f} BPM (RR: {np.round(rr, 3)}s)")
    if len(peaks_neg) >= 2:
        rr = np.diff(times[peaks_neg])
        bpm = 60.0 / np.median(rr)
        print(f"Heart Rate from NEGATIVE peaks (inverted leads): {bpm:.1f} BPM (RR: {np.round(rr, 3)}s)")

except Exception as e:
    print("DSP error:", e)
