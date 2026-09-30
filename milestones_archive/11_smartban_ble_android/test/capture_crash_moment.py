import serial
import subprocess
import time

SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"
HEX_FILE = r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\09_tirtos_all_in_one\all_in_one.hex"

print("1. Flashing with -rs pin...")
subprocess.run([SRFPROG, "-t", "lsidx(0)", "-e", "all", "-p", "-v", "-rs", "pin", "-f", HEX_FILE], capture_output=True, text=True)

print("2. Opening COM3...")
time.sleep(0.05)
ser = serial.Serial('COM3', 115200, timeout=0.05)

print("3. Logging every packet with microsecond precision until stream stops...")
t0 = time.time()
last_rx_time = t0
lines = []

while time.time() - t0 < 10.0:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            now = time.time()
            rel_t = now - t0
            lines.append((rel_t, line))
            last_rx_time = now
    else:
        # If no data for more than 1.5s after having received data, stop
        if len(lines) > 50 and (time.time() - last_rx_time > 1.5):
            print(f"Stream stopped! No data received for 1.5s after {last_rx_time - t0:.3f}s")
            break
        time.sleep(0.001)

ser.close()

print(f"\nTotal lines captured: {len(lines)}")
non_ecg = [(t, l) for t, l in lines if not l.startswith('{"e"')]
print(f"Non-ECG lines ({len(non_ecg)}):")
for t, l in non_ecg:
    print(f"  [{t:6.3f}s] {l}")

print(f"\nLast 15 lines before stopping:")
for t, l in lines[-15:]:
    print(f"  [{t:6.3f}s] {l}")

