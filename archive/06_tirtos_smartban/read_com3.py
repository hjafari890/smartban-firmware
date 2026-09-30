import serial
import time
import sys

try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    print("Connected to COM3. Listening for 10 seconds...")
    start = time.time()
    while time.time() - start < 10:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore')
            sys.stdout.write(line)
            sys.stdout.flush()
        else:
            time.sleep(0.01)
    ser.close()
    print("\nDone listening.")
except Exception as e:
    print(f"Error: {e}")
