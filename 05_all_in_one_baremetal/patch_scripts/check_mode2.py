import numpy as np
import serial
import json
import time

s = serial.Serial('COM3', 115200, timeout=0.5)
s.write(b'2') # mode 2: Input Short
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
print(f"Captured {len(arr)} samples in Mode 2")
# Filter out first 10 samples
clean = arr[20:]
print(f"Mode 2 Clean mV: Min={np.min(clean):.4f}, Max={np.max(clean):.4f}, Mean={np.mean(clean):.4f}, Std={np.std(clean):.4f}, Ptp={np.ptp(clean):.4f}")
print(f"Noise RMS in uV: {np.std(clean)*1000.0:.2f} uV RMS")
print("First 15 values:")
for v in clean[:15]:
    print(f"  {v*1000.0:+.2f} uV ({v:+.5f} mV)")
