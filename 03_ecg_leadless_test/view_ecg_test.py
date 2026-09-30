import serial
import serial.tools.list_ports
import time
import sys

def find_xds110_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "XDS110" in p.description and "Application/User" in p.description:
            return p.device
        if "XDS110" in p.description:
            return p.device
    for p in ports:
        if "COM3" == p.device:
            return "COM3"
    return None

def main():
    port = find_xds110_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]

    if not port:
        print("[ERROR] Could not automatically find XDS110 COM port.")
        print("Please specify port, e.g.: python view_ecg_test.py COM3")
        sys.exit(1)

    print(f"Connecting to {port} at 115200 baud...")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("If CCS terminal is open, please disconnect it first.")
        sys.exit(1)

    print("=" * 65)
    print("  SmartBAN Leadless ECG Real-Time Telemetry Viewer")
    print("  Press Ctrl+C to exit.")
    print("  Interactive keys:")
    print("    1: 1 Hz Square Wave Mode (+/-1 mV)")
    print("    2: Input Shorted (Noise & Offset)")
    print("    3: Die Temperature Sensor")
    print("    4: Live Electrodes (Finger Touch Test)")
    print("    A: Auto-cycle through all modes")
    print("=" * 65 + "\n")

    try:
        while True:
            line = ser.readline()
            if line:
                try:
                    text = line.decode('ascii', errors='replace').rstrip()
                    print(text)
                except Exception:
                    pass
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
