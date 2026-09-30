import serial
import time
import subprocess
import json

SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"
HEX_FILE = r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\09_tirtos_all_in_one\all_in_one.hex"

print("1. Flashing and resetting MCU via SmartRF...")
res = subprocess.run([SRFPROG, "-t", "lsidx(0)", "-e", "all", "-p", "-v", "-rs", "pin", "-f", HEX_FILE], capture_output=True, text=True)
print(f"Flash completed (code {res.returncode})")

time.sleep(0.5)

print("2. Opening COM3 with DTR=False, RTS=False...")
ser = serial.Serial('COM3', 115200, timeout=0.1, dsrdtr=False, rtscts=False)
ser.dtr = False
ser.rts = False

print("3. Streaming for 20 seconds continuously...")
t0 = time.time()
ecg_count = 0
imu_count = 0
env_count = 0
dropped_samples = 0
last_report = t0

while time.time() - t0 < 20.0:
    now = time.time()
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        try:
            d = json.loads(line)
            if "e" in d:
                ecg_count += 1
            elif d.get("type") == "imu":
                imu_count += 1
            elif d.get("type") == "env":
                env_count += 1
        except Exception:
            if ">>" in line or "RingBuf" in line:
                print(f"  [MSG] {line}")

    if now - last_report >= 3.0:
        dt = now - t0
        print(f"  [{dt:4.1f}s] ECG={ecg_count} ({ecg_count/dt:.1f} Hz) | IMU={imu_count} ({imu_count/dt:.1f} Hz) | ENV={env_count}")
        last_report = now
    time.sleep(0.002)

total_dt = time.time() - t0
print(f"\nFinished {total_dt:.1f}s test:")
print(f"  Total ECG: {ecg_count} ({ecg_count/total_dt:.1f} Hz)")
print(f"  Total IMU: {imu_count} ({imu_count/total_dt:.1f} Hz)")
print(f"  Total ENV: {env_count} ({env_count/total_dt:.1f} Hz)")

print("\n4. Sending DIAG command...")
ser.write(b"DIAG\r\n")
ser.flush()
t_diag = time.time()
while time.time() - t_diag < 2.0:
    if ser.in_waiting:
        l = ser.readline().decode('utf-8', errors='ignore').strip()
        if ">>" in l or "ECG" in l or "IMU" in l or "RingBuf" in l:
            print("  ", l)

ser.close()
