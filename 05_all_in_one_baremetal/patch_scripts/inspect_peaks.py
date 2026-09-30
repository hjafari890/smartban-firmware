import serial
import json
import time
import numpy as np
from test_detector import design_highpass_biquad, design_lowpass_biquad, design_notch_biquad

s = serial.Serial('COM3', 115200, timeout=0.5)
s.reset_input_buffer()
t0 = time.time()
fs = 178.0
hp = design_highpass_biquad(0.5, fs)
lp = design_lowpass_biquad(30.0, fs)
notch = design_notch_biquad(50.0, fs, Q=30.0)

samples = []
times = []
while time.time() - t0 < 5.0:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":'):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            v = notch.process(lp.process(hp.process(e2 * 0.000048077)))
            samples.append(v)
            times.append(time.time() - t0)
        except:
            pass
s.close()

from scipy.signal import find_peaks
arr = np.array(samples[80:])
t_arr = np.array(times[80:])

p_pos, _ = find_peaks(arr, distance=int(0.38*fs), prominence=0.03)
p_neg, _ = find_peaks(-arr, distance=int(0.38*fs), prominence=0.03)

print(f"Positive peaks: {len(p_pos)}")
if len(p_pos) >= 2:
    rr_pos = np.diff(t_arr[p_pos])
    print("  RR (pos):", np.round(rr_pos, 3))
    print(f"  BPM (pos): {60.0 / np.median(rr_pos):.1f}")

print(f"Negative peaks: {len(p_neg)}")
if len(p_neg) >= 2:
    rr_neg = np.diff(t_arr[p_neg])
    print("  RR (neg):", np.round(rr_neg, 3))
    print(f"  BPM (neg): {60.0 / np.median(rr_neg):.1f}")
