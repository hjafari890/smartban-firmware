import serial
import json
import time
import numpy as np

s = serial.Serial('COM3', 115200, timeout=0.5)
s.reset_input_buffer()
t0 = time.time()
samples = []
while time.time() - t0 < 3.0:
    raw = s.readline()
    if not raw: continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":['):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            samples.append(e2 * 0.000048077)
        except: pass
s.close()

arr = np.array(samples[50:])
fs = len(arr) / (3.0 - 50/179.0)
print(f"Captured {len(arr)} samples, fs = {fs:.1f} Hz")

fft_vals = np.abs(np.fft.rfft(arr))
freqs = np.fft.rfftfreq(len(arr), 1.0 / fs)

# Print top 5 frequency peaks
top_indices = np.argsort(fft_vals)[::-1][:6]
print("Top frequency components in user's signal:")
for idx in top_indices:
    if freqs[idx] > 0.2:
        print(f"  Freq: {freqs[idx]:.1f} Hz | Magnitude: {fft_vals[idx]:.1f}")
