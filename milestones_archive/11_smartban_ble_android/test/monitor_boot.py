import serial
import time

ser = serial.Serial('COM3', 115200, timeout=1.0)
print("Connected to COM3. Toggling RTS/DTR to reset MCU...")
ser.dtr = False
ser.rts = True
time.sleep(0.1)
ser.rts = False
time.sleep(0.1)
ser.reset_input_buffer()

print("Listening for boot messages for 6 seconds...")
start = time.time()
while time.time() - start < 6.0:
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore')
        print(line, end='')

ser.close()
