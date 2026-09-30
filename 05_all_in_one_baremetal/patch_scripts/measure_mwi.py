import serial
import json
import time
import numpy as np
import sys
sys.path.insert(0, r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650")
from sensor_gui import design_highpass_biquad, design_lowpass_biquad, design_notch_biquad

s = serial.Serial('COM3', 115200, timeout=0.5)
s.reset_input_buffer()
t0 = time.time()
fs = 179.0
hp = design_highpass_biquad(0.5, fs)
lp = design_lowpass_biquad(35.0, fs)
notch = design_notch_biquad(50.0, fs, Q=30.0)

samples = []
while time.time() - t0 < 3.0:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":['):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            samples.append(notch.process(lp.process(hp.process(e2 * 0.000048077))))
        except:
            pass
s.close()

arr = np.array(samples[60:])
deriv = np.diff(arr)
sq = deriv**2
mwi = np.convolve(sq, np.ones(25)/25.0, mode='valid')

print(f"Filtered ECG mV: min={np.min(arr):.4f}, max={np.max(arr):.4f}, ptp={np.ptp(arr):.4f}, std={np.std(arr):.4f}")
print(f"Derivative: min={np.min(deriv):.4f}, max={np.max(deriv):.4f}, ptp={np.ptp(deriv):.4f}")
print(f"MWI: min={np.min(mwi):.6f}, max={np.max(mwi):.6f}, mean={np.mean(mwi):.6f}, p95={np.percentile(mwi, 95):.6f}, p99={np.percentile(mwi, 99):.6f}")
