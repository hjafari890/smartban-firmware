import serial
import json
import time
import numpy as np

s = serial.Serial('COM3', 115200, timeout=0.5)
time.sleep(0.2)
s.reset_input_buffer()

# Send '1' to enter Mode 1 Square wave
print("Switching to Mode 1 (Square wave)...")
s.write(b'1')
time.sleep(0.5)
s.reset_input_buffer()

samples = []
t0 = time.time()
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

# Switch back to Mode 4 Live ECG
s.write(b'4')
s.close()

arr = np.array(samples[20:])
print(f"Captured {len(arr)} samples in Mode 1")
print(f"Raw Mode 1 mV: min = {np.min(arr):.4f}, max = {np.max(arr):.4f}, mean = {np.mean(arr):.4f}, ptp = {np.ptp(arr):.4f}")
print("First 20 samples in Mode 1:")
for v in arr[:20]:
    print(f"  {v:+.4f} mV")
