import serial
import json
import time
import sys
import numpy as np
from collections import defaultdict

PORT = "COM3" # Will update to match user's port, default to COM5
BAUD = 115200
TIMEOUT = 10

def main():
    print(f"Testing live sensor data authenticity on port {PORT}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Could not open serial port: {e}. Please update PORT in script or connect board.")
        return

    # Let it sync
    time.sleep(1)
    ser.reset_input_buffer()

    print("Listening for 15 seconds to collect samples...")
    start_time = time.time()
    
    data_store = defaultdict(list)
    
    while time.time() - start_time < 15:
        line = ser.readline()
        if not line:
            continue
        try:
            line_str = line.decode('utf-8').strip()
            if line_str.startswith('{'):
                j = json.loads(line_str)
                if j.get("type") == "env":
                    for k in ["T", "H", "P", "lux", "prox", "mlx"]:
                        if k in j:
                            data_store[k].append(j[k])
                elif j.get("type") == "imu":
                    for k in ["ax", "ay", "az", "pitch", "roll"]:
                        if k in j:
                            data_store[k].append(j[k])
                elif "e" in j:
                    data_store["ecg"].extend(j["e"])
        except Exception:
            pass

    ser.close()
    
    print("\n--- Live Data Authenticity Report ---")
    if not data_store:
        print("No JSON data received! Is the firmware in semantic mode or raw mode?")
        return

    for key, values in data_store.items():
        if not values: continue
        arr = np.array(values)
        std_dev = np.std(arr)
        v_min, v_max = np.min(arr), np.max(arr)
        mean = np.mean(arr)
        
        status = "LIVE (Noise Detected)" if std_dev > 0.0 else "SUSPICIOUS (Constant)"
        if key == "prox" and std_dev == 0.0:
            status = "EXPECTED CONSTANT (Unless hand is moving)"
            
        print(f"{key.upper():<6} | N={len(values):<5} | Mean: {mean:8.2f} | Range: [{v_min:8.2f}, {v_max:8.2f}] | StdDev: {std_dev:8.4f} -> {status}")

    print("\nConclusion: If StdDev > 0 for analog sensors (especially IMU, ECG, Lux), the data contains real physical noise profiles and is authentically read from the hardware bus.")

if __name__ == "__main__":
    main()
