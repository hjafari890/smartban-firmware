import serial
import time
import json

ser = serial.Serial('COM3', 115200, timeout=1.0)
time.sleep(0.2)
ser.reset_input_buffer()

print("Listening for 10 seconds of streaming...")
t0 = time.time()
ecg = 0
imu = 0
env = 0
last_ecg = None
last_imu = None
last_env = None

while time.time() - t0 < 10.0:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        try:
            d = json.loads(line)
            if "e" in d:
                ecg += 1
                last_ecg = d["e"]
            elif d.get("type") == "imu":
                imu += 1
                last_imu = d
            elif d.get("type") == "env":
                env += 1
                last_env = d
        except Exception:
            pass

elapsed = time.time() - t0
print(f"Elapsed: {elapsed:.2f}s")
print(f"  ECG Packets : {ecg} ({ecg/elapsed:.1f} Hz)")
print(f"  IMU Packets : {imu} ({imu/elapsed:.1f} Hz)")
print(f"  ENV Packets : {env} ({env/elapsed:.1f} Hz)")
print(f"  Last ECG    : {last_ecg}")
print(f"  Last IMU    : {last_imu}")
print(f"  Last ENV    : {last_env}")

print("\nSending DIAG command...")
ser.write(b"DIAG\r\n")
ser.flush()

t1 = time.time()
while time.time() - t1 < 2.0:
    if ser.in_waiting:
        l = ser.readline().decode('utf-8', errors='ignore').strip()
        if ">>" in l or "ECG" in l or "IMU" in l or "RingBuf" in l:
            print("  ", l)

ser.close()
