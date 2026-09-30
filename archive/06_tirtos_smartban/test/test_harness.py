#!/usr/bin/env python3
"""
===============================================================================
test_harness.py
Master Automated Test Harness Runner for SmartBAN TI-RTOS7 Sensor Node Firmware
Platform: TI CC2652R1 Cortex-M4F @ 48 MHz + SmartBAN Shield Rev 3.5
Toolchain: tiarmclang 5.1.1.LTS / SysConfig 1.21.1 / SimpleLink SDK 8.33 / TI-RTOS7
Author: Worker M5 R2 (E2E Automated Verification & Test Harness Specialist)

Executes all verification suites across Milestones M1 through M5:
  1. test_m1_hal_math.py      - Hardware Abstraction Layer & Sensor Math
  2. test_m1_adversarial.py   - M1 Adversarial Boundary & Stress Suite
  3. test_m1_challenger2.py   - M1 Challenger 2 Memory & Hex Audits
  4. test_m2_rtos_ipc.py      - TI-RTOS7 5-Task Scheduling & SPSC IPC
  5. test_m3_edgeai.py        - Pan-Tompkins QRS, IMU Stats & Fusion
  6. test_m4_telemetry_ui.py  - Serial Telemetry, CLI & Visual UI
  7. test_m4_adversarial.py   - Telemetry Queue Stress & CLI Watchdog
  8. test_e2e_smartban.py     - Unified 4-Tier E2E Master Harness (F1-F30, Scenarios 1-8)

Exit Code:
  0 = ALL TEST SUITES PASSED (100% SUCCESS)
  1 = ONE OR MORE TEST SUITES FAILED
===============================================================================
"""

import os
import sys
import time
import subprocess
import argparse
from typing import List, Dict, Any

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(TEST_DIR, ".."))

SUITES = [
    {
        "name": "M1 HAL & Math Drivers",
        "file": "test_m1_hal_math.py",
        "desc": "ADS1292, ADXL362, MLX90632, OPT4041, VCNL4040, PCAL6408A, CH455"
    },
    {
        "name": "M1 Adversarial & Stress",
        "file": "test_m1_adversarial.py",
        "desc": "Recursive mutexes, saturation limits, 1000 SPI/I2C cycles"
    },
    {
        "name": "M1 Challenger 2 Memory & Hex",
        "file": "test_m1_challenger2.py",
        "desc": "Build pipeline, ELF headers, Intel HEX type 03/05 exclusion, SRAM headroom"
    },
    {
        "name": "M2 RTOS7 5-Task Architecture",
        "file": "test_m2_rtos_ipc.py",
        "desc": "5 POSIX threads, DRDY semaphore, SPSC ring buffers, Standby sleep"
    },
    {
        "name": "M3 Edge-AI Semantic Processing",
        "file": "test_m3_edgeai.py",
        "desc": "Pan-Tompkins QRS, IMU SMA, 4-phase fall, thermal fusion, >95% reduction"
    },
    {
        "name": "M4 Telemetry, CLI & Visual UI",
        "file": "test_m4_telemetry_ui.py",
        "desc": "UART2 JSON stream, interactive CLI parser, CH455 7-seg & mode LEDs"
    },
    {
        "name": "M4 Adversarial Queue & CLI",
        "file": "test_m4_adversarial.py",
        "desc": "SPSC queue overflow, 32-bit rollover, ANSI CSI absorption, 10s watchdog"
    },
    {
        "name": "M5 Unified E2E Test Harness",
        "file": "test_e2e_smartban.py",
        "desc": "Full 4-tier matrix (F1-F30), 8 real-world scenarios, thesis reduction proof"
    }
]

def run_suite(suite: Dict[str, Any], verbose: bool = False) -> Dict[str, Any]:
    file_path = os.path.join(TEST_DIR, suite["file"])
    if not os.path.exists(file_path):
        return {
            "name": suite["name"],
            "file": suite["file"],
            "passed": False,
            "duration": 0.0,
            "exit_code": -1,
            "error": f"File not found: {file_path}",
            "stdout": "",
            "stderr": ""
        }

    start = time.time()
    try:
        proc = subprocess.run(
            [sys.executable, file_path],
            cwd=PROJECT_DIR,
            capture_output=True,
            text=True,
            timeout=180
        )
        duration = time.time() - start
        passed = (proc.returncode == 0)
        return {
            "name": suite["name"],
            "file": suite["file"],
            "passed": passed,
            "duration": duration,
            "exit_code": proc.returncode,
            "error": None if passed else f"Exited with code {proc.returncode}",
            "stdout": proc.stdout,
            "stderr": proc.stderr
        }
    except subprocess.TimeoutExpired:
        duration = time.time() - start
        return {
            "name": suite["name"],
            "file": suite["file"],
            "passed": False,
            "duration": duration,
            "exit_code": -2,
            "error": "Execution timed out (>180s)",
            "stdout": "",
            "stderr": ""
        }
    except Exception as e:
        duration = time.time() - start
        return {
            "name": suite["name"],
            "file": suite["file"],
            "passed": False,
            "duration": duration,
            "exit_code": -3,
            "error": str(e),
            "stdout": "",
            "stderr": ""
        }

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Master Automated Test Harness Runner for SmartBAN TI-RTOS7 Sensor Node Firmware"
    )
    parser.add_argument("--suite", type=str, default=None,
                        help="Run only matching suite (substring match on filename or name)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print full standard output of each test suite")
    args = parser.parse_args()

    print("=" * 82)
    print(" SMARTBAN TI-RTOS7 SENSOR NODE: MASTER AUTOMATED TEST HARNESS RUNNER")
    print(" Target Platform: TI CC2652R1 LaunchPad (ARM Cortex-M4F @ 48 MHz) + SmartBAN Shield")
    print(f" Working Directory: {PROJECT_DIR}")
    print("=" * 82)

    suites_to_run = SUITES
    if args.suite:
        s_filter = args.suite.lower()
        suites_to_run = [s for s in SUITES if s_filter in s["name"].lower() or s_filter in s["file"].lower()]
        if not suites_to_run:
            print(f"[ERROR] No test suite matched filter: {args.suite}")
            return 1

    total_suites = len(suites_to_run)
    passed_suites = 0
    failed_suites = 0
    results = []

    master_start = time.time()

    for idx, s in enumerate(suites_to_run, 1):
        print(f"\n[{idx}/{total_suites}] Executing: {s['name']} ({s['file']})...")
        res = run_suite(s, verbose=args.verbose)
        results.append(res)

        if res["passed"]:
            passed_suites += 1
            print(f"  --> [PASS] {s['name']} completed in {res['duration']:.2f}s (Exit code 0)")
        else:
            failed_suites += 1
            print(f"  --> [FAIL] {s['name']} FAILED in {res['duration']:.2f}s ({res['error']})")
            if not args.verbose and res["stderr"]:
                print("  Stderr excerpt:")
                for line in res["stderr"].strip().split("\n")[-5:]:
                    print(f"    {line}")

        if args.verbose and res["stdout"]:
            print("-" * 60)
            print(res["stdout"].strip())
            print("-" * 60)

    total_duration = time.time() - master_start

    print("\n" + "=" * 82)
    print(" MASTER TEST HARNESS SUMMARY AUDIT TABLE")
    print("=" * 82)
    print(f"{'#':<3} {'Suite Name':<35} {'Test Script':<24} {'Time':<8} {'Result'}")
    print("-" * 82)

    for idx, r in enumerate(results, 1):
        status_str = "PASS" if r["passed"] else "FAIL"
        print(f"{idx:<3} {r['name']:<35} {r['file']:<24} {r['duration']:>5.2f}s   [{status_str}]")

    print("-" * 82)
    print(f"TOTAL SUITES EXECUTED: {total_suites}")
    print(f"TOTAL SUITES PASSED  : {passed_suites} / {total_suites} ({passed_suites/total_suites*100:.1f}%)")
    print(f"TOTAL SUITES FAILED  : {failed_suites}")
    print(f"TOTAL EXECUTION TIME : {total_duration:.2f} seconds")
    print("=" * 82)

    if failed_suites == 0 and passed_suites > 0:
        print("\n >>> [ALL TEST SUITES PASSED] FIRMWARE CERTIFIED 100% TESTBED READY <<<\n")
        return 0
    else:
        print(f"\n >>> [TEST HARNESS FAILED] {failed_suites} test suite(s) failed <<<\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())
