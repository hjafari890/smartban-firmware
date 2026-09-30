import serial
import time

ser = serial.Serial('COM3', 115200, timeout=0.2)
time.sleep(0.5)
ser.reset_input_buffer()

print("--- Testing SLEEP 3s ---")
ser.write(b"SLEEP 3\r\n")
ser.flush()

t0 = time.time()
while time.time() - t0 < 6.0:
    l = ser.readline().decode('utf-8', errors='ignore').strip()
    if l:
        print(f"[{time.time()-t0:.2f}s] {l}")

print("\n--- Testing DEEPSLEEP 3s ---")
ser.write(b"DEEPSLEEP 3\r\n")
ser.flush()

t0 = time.time()
while time.time() - t0 < 6.0:
    l = ser.readline().decode('utf-8', errors='ignore').strip()
    if l:
        print(f"[{time.time()-t0:.2f}s] {l}")

ser.close()
print("Sleep test complete.")
