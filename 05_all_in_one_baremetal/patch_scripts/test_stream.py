import serial
import time
import json

try:
    s = serial.Serial('COM3', 115200, timeout=1)
    print("Opened COM3 successfully")
    time.sleep(0.5)
    s.reset_input_buffer()
    
    t0 = time.time()
    ecg_count = 0
    imu_count = 0
    samples = []
    
    while time.time() - t0 < 3.0:
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if line.startswith('{"e":'):
            ecg_count += 1
            if len(samples) < 10:
                samples.append(line)
        elif 'imu' in line:
            imu_count += 1
            
    s.close()
    elapsed = time.time() - t0
    print(f"Elapsed: {elapsed:.2f}s")
    print(f"ECG samples: {ecg_count} ({ecg_count/elapsed:.1f} Hz)")
    print(f"IMU samples: {imu_count} ({imu_count/elapsed:.1f} Hz)")
    print("First samples:")
    for sm in samples:
        print(" ", sm)
except Exception as e:
    print("Exception:", e)
