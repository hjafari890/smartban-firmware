import serial
import subprocess
import time
import threading

SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"

ser = serial.Serial('COM3', 115200, timeout=0.1)
ser.reset_input_buffer()

boot_lines = []
stop_event = threading.Event()

def reader():
    while not stop_event.is_set():
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                boot_lines.append(line)
        time.sleep(0.01)

t = threading.Thread(target=reader)
t.start()

# Issue hardware pin reset via srfprog
print("Issuing reset via srfprog...")
subprocess.run([SRFPROG, "-t", "lsidx(0)", "-r"], capture_output=True, text=True)

time.sleep(3.0)
stop_event.set()
t.join()
ser.close()

print("\n--- CAPTURED OUTPUT ---")
for l in boot_lines:
    print(l, end='')
