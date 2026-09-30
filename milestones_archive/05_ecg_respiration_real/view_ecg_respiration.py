import serial
import time
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="View Real-time ECG and Respiration Data")
    parser.add_argument("--port", type=str, default="COM5", help="Serial port to connect to")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"Failed to open port {args.port}: {e}")
        print("Please check if the port is correct and not in use by another program.")
        sys.exit(1)

    print("\nReading data... (Press Ctrl+C to stop)\n")
    print(f"{'Idx':>8} | {'Resp(Raw)':>10} | {'ECG(Raw)':>10} | {'ECG(Filt)':>10} | {'BPM':>5} | {'RPM':>5}")
    print("-" * 65)

    try:
        # Clear buffer
        ser.reset_input_buffer()
        while True:
            line = ser.readline()
            if not line:
                continue
                
            try:
                decoded = line.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if not decoded:
                continue
                
            # Print status messages from firmware
            if not any(char.isdigit() for char in decoded):
                print(f"[FW] {decoded}")
                continue

            # Try to parse CSV
            parts = decoded.split(',')
            if len(parts) == 6:
                idx, resp_raw, ecg_raw, ecg_filt, bpm, rpm = parts
                
                bpm_val = int(bpm)
                rpm_val = int(rpm)
                
                # Add a little visual indicator for heartbeats
                heart_str = "❤️" if bpm_val > 0 and int(idx) % 25 == 0 else "  "
                breath_str = "🫁" if rpm_val > 0 and int(idx) % 100 == 0 else "  "

                print(f"{idx:>8} | {resp_raw:>10} | {ecg_raw:>10} | {ecg_filt:>10} | {bpm:>5} | {rpm:>5} {heart_str} {breath_str}")
            else:
                print(f"[FW] {decoded}")
                
    except KeyboardInterrupt:
        print("\nStopping viewer.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
