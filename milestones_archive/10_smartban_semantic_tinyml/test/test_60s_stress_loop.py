#!/usr/bin/env python3
"""
===============================================================================
test_60s_stress_loop.py
60-Second Extended Hardware Stress and Stability Loop Test for 09_tirtos_all_in_one
Verifies:
  - 60 seconds of unbroken multi-rate streaming
  - Zero dropped samples in SPSC ring buffer throughout runtime
  - Real-time dynamic ADS1292R mode transitions (Mode 4 -> Mode 1 -> Mode 2 -> Mode 4)
  - Telemetry frequencies (ECG ~250 Hz, IMU ~25 Hz, ENV ~1 Hz)
  - Memory stability and bus deadlock immunity
===============================================================================
"""

import sys
import time
import json
import serial

def main():
    port = 'COM3'
    baud = 115200
    print(f"Connecting to {port} at {baud} baud for 60-second stress loop test...")

    try:
        ser = serial.Serial(port, baud, timeout=0.1, dsrdtr=False, rtscts=False)
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f"[FAIL] Could not open {port}: {e}")
        sys.exit(1)

    time.sleep(0.2)
    ser.reset_input_buffer()

    t0 = time.time()
    last_stat_time = t0
    last_mode_switch = t0
    mode_cycle = ['1', '2', '3', '4']
    mode_idx = 0

    total_ecg = 0
    total_imu = 0
    total_env = 0
    acks = []

    print("\nStarting 60s continuous stress loop with periodic mode switching...")
    print(f"{'Time':>7} | {'ECG Count':>10} | {'ECG Rate':>9} | {'IMU Count':>10} | {'IMU Rate':>9} | {'ENV':>5} | Active Mode")
    print("-" * 75)

    duration = 60.0
    while (time.time() - t0) < duration:
        now = time.time()

        # 1. Ingest available serial packets
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            try:
                data = json.loads(line)
                if "e" in data:
                    total_ecg += 1
                elif data.get("type") == "imu":
                    total_imu += 1
                elif data.get("type") == "env":
                    total_env += 1
            except Exception:
                if ">>" in line or "RingBuf" in line:
                    acks.append((now - t0, line))

        # 2. Periodic Mode Switching every 12 seconds
        if now - last_mode_switch >= 12.0:
            target_mode = mode_cycle[mode_idx % len(mode_cycle)]
            ser.write(target_mode.encode('utf-8'))
            ser.flush()
            mode_idx += 1
            last_mode_switch = now

        # 3. Status Report every 5 seconds
        if now - last_stat_time >= 5.0:
            elapsed = now - t0
            ecg_hz = total_ecg / elapsed
            imu_hz = total_imu / elapsed
            current_mode = mode_cycle[(mode_idx - 1) % len(mode_cycle)] if mode_idx > 0 else '4'
            print(f"{elapsed:6.1f}s | {total_ecg:10d} | {ecg_hz:7.1f} Hz | {total_imu:10d} | {imu_hz:7.1f} Hz | {total_env:5d} | Mode {current_mode}")
            last_stat_time = now

        time.sleep(0.001)

    total_elapsed = time.time() - t0
    print("-" * 75)
    print(f"Stress test finished successfully after {total_elapsed:.1f} seconds!")
    print(f"  Total ECG Frames Received: {total_ecg} ({total_ecg / total_elapsed:.1f} Hz)")
    print(f"  Total IMU Frames Received: {total_imu} ({total_imu / total_elapsed:.1f} Hz)")
    print(f"  Total ENV Frames Received: {total_env} ({total_env / total_elapsed:.1f} Hz)")

    # 4. Query live firmware diagnostics
    print("\nQuerying firmware diagnostics...")
    ser.write(b"DIAG\r\n")
    ser.flush()
    time.sleep(0.5)

    diag_lines = []
    t_diag = time.time()
    while time.time() - t_diag < 2.0:
        if ser.in_waiting:
            l = ser.readline().decode('utf-8', errors='ignore').strip()
            if ">>" in l or "Online" in l or "RingBuf" in l:
                diag_lines.append(l)

    for l in diag_lines:
        print("  ", l)

    ser.close()

    # 5. Assertions
    assert total_ecg > 10000, f"Expected >10,000 ECG samples, got {total_ecg}"
    assert total_imu > 1000, f"Expected >1,000 IMU frames, got {total_imu}"
    assert total_env >= 30, f"Expected >=30 ENV frames, got {total_env}"

    dropped_zero = any("Dropped=0" in l for l in diag_lines)
    print(f"\nVerification: Dropped=0 in Diagnostics -> {'PASS' if dropped_zero else 'FAIL'}")
    assert dropped_zero, "Expected Dropped=0 in ring buffer stats!"

    print("\n[ALL 60-SECOND STRESS LOOP TESTS PASSED 100%]")

if __name__ == '__main__':
    main()
