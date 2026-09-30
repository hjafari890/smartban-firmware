import numpy as np
import serial
import json
import time

s = serial.Serial('COM3', 115200, timeout=0.5)
s.write(b'1')
time.sleep(1.0)
s.reset_input_buffer()

samples = []
t0 = time.time()
while time.time() - t0 < 3.0:
    line = s.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":['):
        try:
            d = json.loads(line)
            samples.append(d['e'][2] * 0.000048077)
        except:
            pass

s.write(b'4')
s.close()

arr = np.array(samples)
print(f"Captured {len(arr)} samples after settling")
print(f"Min: {np.min(arr):.4f}, Max: {np.max(arr):.4f}, Mean: {np.mean(arr):.4f}")
print(f"5th percentile: {np.percentile(arr, 5):.4f}")
print(f"95th percentile: {np.percentile(arr, 95):.4f}")
print("Sampling points every 10 samples:")
for i in range(0, min(len(arr), 120), 10):
    print(f"  idx {i:03d} (t={i/179.0:.2f}s): {arr[i]:+.4f} mV")
