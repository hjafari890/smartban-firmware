import serial
import json
import time
import numpy as np
import sys
sys.path.insert(0, r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650")
from sensor_gui import design_highpass_biquad, design_lowpass_biquad, design_notch_biquad

s = serial.Serial('COM3', 115200, timeout=0.5)
s.write(b'4') # Live Lead I mode
time.sleep(0.5)
s.reset_input_buffer()

t0 = time.time()
fs = 179.0
hp = design_highpass_biquad(0.5, fs)
lp = design_lowpass_biquad(35.0, fs)
notch = design_notch_biquad(50.0, fs, Q=30.0)

samples = []
raw_mv = []
prev_v = 0.0

print("Capturing 5 seconds of live filtered ECG with slew-limiting...")
while time.time() - t0 < 5.0:
    raw = s.readline()
    if not raw: continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":['):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            v = e2 * 0.000048077
            # Slew limiter: reject SPI bus-switch glitches (> 5 mV jump)
            if len(raw_mv) > 0 and abs(v - prev_v) > 6.0:
                v = prev_v
            prev_v = v
            raw_mv.append(v)
            
            # Cascade: HP (0.5Hz) -> LP (35Hz) -> Notch (50Hz)
            filt = notch.process(lp.process(hp.process(v)))
            samples.append(filt)
        except:
            pass

s.close()

arr = np.array(samples[60:])
print(f"Captured {len(arr)} clean filtered samples")
print(f"Clean filtered ECG mV: min={np.min(arr):.4f}, max={np.max(arr):.4f}, ptp={np.ptp(arr):.4f}, std={np.std(arr):.4f}")

from scipy.signal import find_peaks
p_pos, _ = find_peaks(arr, distance=int(0.40*fs), prominence=0.04)
p_neg, _ = find_peaks(-arr, distance=int(0.40*fs), prominence=0.04)

print(f"\nPositive R-peaks detected: {len(p_pos)}")
if len(p_pos) >= 2:
    rr_pos = np.diff(p_pos) / fs
    print(f"  RR intervals (pos): {np.round(rr_pos, 3)} s")
    print(f"  Live Heart Rate (pos): {60.0 / np.median(rr_pos):.1f} BPM")

print(f"Negative R-peaks detected: {len(p_neg)}")
if len(p_neg) >= 2:
    rr_neg = np.diff(p_neg) / fs
    print(f"  RR intervals (neg): {np.round(rr_neg, 3)} s")
    print(f"  Live Heart Rate (neg): {60.0 / np.median(rr_neg):.1f} BPM")
