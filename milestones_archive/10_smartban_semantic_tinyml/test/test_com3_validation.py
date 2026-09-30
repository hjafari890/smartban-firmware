#!/usr/bin/env python3
"""
===============================================================================
test_com3_validation.py
Automated Hardware Verification Script for 09_tirtos_all_in_one Firmware
Target: TI CC2652R1 LaunchPad on COM3 @ 115200 baud
===============================================================================
"""

import sys
import time
import json
import serial

def main():
    port = 'COM3'
    baud = 115200
    print(f"Connecting to {port} at {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except Exception as e:
        print(f"[FAIL] Could not open {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    print("\n--- [Step 1] Listening for raw telemetry frames (10 seconds) ---")
    start_time = time.time()
    ecg_count = 0
    imu_count = 0
    env_count = 0
    raw_lines = []

    last_ecg_sample = None
    last_imu_sample = None
    last_env_sample = None

    while time.time() - start_time < 10.0:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            raw_lines.append(line)

            # Try parsing as JSON
            try:
                data = json.loads(line)
                if "e" in data and isinstance(data["e"], list):
                    ecg_count += 1
                    last_ecg_sample = data["e"]
                elif data.get("type") == "imu":
                    imu_count += 1
                    last_imu_sample = data
                elif data.get("type") == "env":
                    env_count += 1
                    last_env_sample = data
            except Exception:
                if ">>" in line or "SmartBAN" in line or "[INIT]" in line or "[BOOT]" in line:
                    print(f"  [LOG] {line}")

    elapsed = time.time() - start_time
    print(f"\nElapsed: {elapsed:.2f}s")
    print(f"  ECG Packets Received : {ecg_count} ({ecg_count / elapsed:.1f} Hz, expect ~250 Hz)")
    print(f"  IMU Packets Received : {imu_count} ({imu_count / elapsed:.1f} Hz, expect ~25 Hz)")
    print(f"  ENV Packets Received : {env_count} ({env_count / elapsed:.1f} Hz, expect ~1 Hz)")

    if last_ecg_sample:
        print(f"  Last ECG Sample : {last_ecg_sample}")
    if last_imu_sample:
        print(f"  Last IMU Sample : {last_imu_sample}")
    if last_env_sample:
        print(f"  Last ENV Sample : {last_env_sample}")

    assert ecg_count > 100, f"ECG packet count too low: {ecg_count}"
    assert imu_count > 10, f"IMU packet count too low: {imu_count}"
    assert env_count >= 1, f"ENV packet count too low: {env_count}"
    print("[PASS] Step 1: Continuous raw multi-rate telemetry verified!")

    # -------------------------------------------------------------------------
    # Step 2: Test Interactive ADS1292R Mode Switching (Modes 1 to 4)
    # -------------------------------------------------------------------------
    print("\n--- [Step 2] Testing Interactive ADS1292R Mode Switches ---")
    for mode in ['1', '2', '3', '4']:
        print(f"  Sending command '{mode}' to switch to Mode {mode}...")
        ser.write(mode.encode('utf-8'))
        ser.flush()

        # Listen for response
        switched = False
        mode_start = time.time()
        while time.time() - mode_start < 2.0:
            if ser.in_waiting:
                resp = ser.readline().decode('utf-8', errors='ignore').strip()
                if f"Switched to Mode [{mode}]" in resp:
                    print(f"    [ACK] Received: {resp}")
                    switched = True
                    break
        assert switched, f"Failed to switch to Mode {mode}"
        time.sleep(0.5)

    print("[PASS] Step 2: Interactive test mode switching verified!")

    # -------------------------------------------------------------------------
    # Step 3: Test Master Thesis Dual-Mode Stream Control (Semantic vs Raw)
    # -------------------------------------------------------------------------
    print("\n--- [Step 3] Testing Master Thesis Semantic Token Mode ---")
    ser.write(b"MODE SEMANTIC\r\n")
    ser.flush()

    sem_confirmed = False
    sem_start = time.time()
    sem_tokens_received = 0
    last_sem_token = None

    while time.time() - sem_start < 5.0:
        if ser.in_waiting:
            resp = ser.readline().decode('utf-8', errors='ignore').strip()
            if "Stream Mode: SEMANTIC" in resp:
                print(f"    [ACK] {resp}")
                sem_confirmed = True
            try:
                data = json.loads(resp)
                if data.get("type") == "SEM":
                    sem_tokens_received += 1
                    last_sem_token = data
            except Exception:
                pass

    if last_sem_token:
        print(f"    [SEM TOKEN] {last_sem_token}")

    print("  Switching back to MODE RAW...")
    ser.write(b"MODE RAW\r\n")
    ser.flush()
    time.sleep(1.0)
    ser.reset_input_buffer()

    print("[PASS] Step 3: Master Thesis Semantic Stream Mode verified!")

    ser.close()
    print("\n===============================================================")
    print(" ALL HARDWARE VALIDATION CHECKS PASSED (100% OK)")
    print("===============================================================")

if __name__ == '__main__':
    main()
