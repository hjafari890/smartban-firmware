import serial
import time
import numpy as np

# Connect to COM3 and collect 5 seconds of live ECG data from the user
s = serial.Serial('COM3', 115200, timeout=1)
time.sleep(0.5)
s.reset_input_buffer()

print("Collecting 5 seconds of live ECG from COM3...")
t0 = time.time()
raw_samples = []
times = []

while time.time() - t0 < 5.0:
    line = s.readline().decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":'):
        try:
            parts = line[5:-2].split(',')
            stat = int(parts[0])
            e1 = int(parts[1])
            e2 = int(parts[2])
            now = time.time() - t0
            times.append(now)
            raw_samples.append((stat, e1, e2))
        except:
            pass

s.close()
print(f"Collected {len(raw_samples)} samples in 5 seconds")
if len(raw_samples) > 0:
    fs_actual = len(raw_samples) / 5.0
    print(f"Actual sampling rate: {fs_actual:.1f} Hz")
    
    stats = [x[0] for x in raw_samples]
    ch2_raw = np.array([x[2] for x in raw_samples])
    ch2_mv = ch2_raw * 0.000048077
    
    print(f"Status byte values: {set(stats)}")
    print(f"Raw CH2 mean: {np.mean(ch2_raw):.1f}, min: {np.min(ch2_raw)}, max: {np.max(ch2_raw)}")
    print(f"Physical CH2 mV: mean={np.mean(ch2_mv):.3f} mV, pk-pk={np.ptp(ch2_mv):.3f} mV, std={np.std(ch2_mv):.3f} mV")
    
    # Save a small slice to inspect
    print("\nFirst 20 physical mV values:")
    for v in ch2_mv[:20]:
        print(f"  {v:+.4f} mV")
