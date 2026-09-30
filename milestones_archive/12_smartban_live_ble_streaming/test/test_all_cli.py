import serial
import time

ser = serial.Serial('COM3', 115200, timeout=0.2)
time.sleep(0.1)

cmds = [
    ('STATUS', 'STATUS\r\n'),
    ('TARE', 'TARE\r\n'),
    ('DIAG', 'DIAG\r\n'),
    ('MODE 1', '1'),
    ('MODE 2', '2'),
    ('MODE 3', '3'),
    ('MODE 4', '4'),
    ('MODE SEMANTIC', 'MODE SEMANTIC\r\n'),
    ('MODE RAW', 'MODE RAW\r\n')
]

for label, cmd in cmds:
    ser.write(cmd.encode('utf-8'))
    ser.flush()
    time.sleep(0.4)
    t0 = time.time()
    responses = []
    while time.time() - t0 < 0.8:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if '>>' in line or 'Online' in line or 'RingBuf' in line or 'SEM' in line:
                responses.append(line)
    print(f"[{label}] -> {responses}")

ser.close()
