import serial
import subprocess
import time
import threading

SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"
HEX_FILE = r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\09_tirtos_all_in_one\all_in_one.hex"

print("Starting flash with -rs pin...")
res = subprocess.run([SRFPROG, "-t", "lsidx(0)", "-e", "all", "-p", "-v", "-rs", "pin", "-f", HEX_FILE], capture_output=True, text=True)
print(f"Flash return code: {res.returncode}")

time.sleep(0.1)
ser = serial.Serial('COM3', 115200, timeout=0.1)
print("Listening immediately on COM3...")

t0 = time.time()
lines = []
while time.time() - t0 < 6.0:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore')
        if line:
            lines.append(line)
            print(line, end='')
    time.sleep(0.005)

ser.close()
print(f"\nDone. Total lines captured: {len(lines)}")
