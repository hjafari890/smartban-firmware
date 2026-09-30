#!/usr/bin/env python3
"""
monitor.py - SmartBAN TI-RTOS7 Serial Monitor
Connects to COM3 (XDS110 Application/User UART) at 115200 baud.
Pretty-prints JSON telemetry, highlights alerts, lets you type CLI commands.

Usage:
    python monitor.py               # defaults to COM3
    python monitor.py --port COM5   # use a different port
"""

import serial
import threading
import sys
import json
import argparse
import os

# ANSI color codes
RED    = "\033[91m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
BLUE   = "\033[94m"
GRAY   = "\033[90m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# Enable ANSI colors on Windows
os.system("")

BANNER = f"""
{CYAN}{BOLD}╔══════════════════════════════════════════════════════════════╗
║     SmartBAN TI-RTOS7 Live Monitor  ·  CC2652R1 @ 115200    ║
╚══════════════════════════════════════════════════════════════╝{RESET}
{GRAY}  Ctrl+C to quit. Type commands + Enter to send to the board.{RESET}
{GRAY}  Quick commands: STATUS | MODE RAW | MODE SEMANTIC | HELP | RESET{RESET}
"""

ALERT_FLAGS = {
    0x01: ("⚠ TACHYCARDIA", RED),
    0x02: ("⚠ BRADYCARDIA", YELLOW),
    0x04: ("⚠ ARRHYTHMIA",  YELLOW),
    0x08: ("🚨 FALL DETECTED", RED),
}

POSTURE_COLORS = {
    "SIT": CYAN, "SITTING": CYAN, "SEDENTARY": CYAN,
    "WALK": GREEN, "WALKING": GREEN, "ACTIVE": GREEN,
    "RUN": YELLOW, "RUNNING": YELLOW,
    "HIGH_DYNAMIC": RED,
}


def format_json(line: str) -> str:
    try:
        d = json.loads(line)
    except json.JSONDecodeError:
        return None

    ts      = d.get("ts", 0)
    mode    = d.get("mode", "SEM")
    hr      = d.get("hr", 0)
    rr      = d.get("rr", 0)
    hrv_r   = d.get("hrv_r", d.get("hrv_rmssd", 0))
    posture = str(d.get("posture", "?")).upper()
    fall    = d.get("fall", 0)
    temp    = d.get("temp", 0.0)
    lux     = d.get("lux", 0)
    prox    = d.get("prox", 0)
    contact = d.get("contact", 0)
    alerts  = d.get("alerts", 0)

    hr_color = RED if hr > 100 else (YELLOW if hr < 50 and hr > 0 else GREEN)
    fall_str = f"{RED}{BOLD}FALL!{RESET}" if fall else f"{GREEN}OK{RESET}"
    contact_str = f"{GREEN}ON-BODY{RESET}" if contact else f"{GRAY}OFF-BODY{RESET}"
    p_color = POSTURE_COLORS.get(posture, CYAN)
    mode_str = f"{CYAN}SEMANTIC{RESET}" if "SEM" in mode.upper() else f"{YELLOW}RAW{RESET}"

    alert_parts = [f"{c}{n}{RESET}" for bit, (n, c) in ALERT_FLAGS.items() if alerts & bit]
    alert_str = "  ".join(alert_parts) if alert_parts else f"{GREEN}CLEAR{RESET}"

    return (
        f"{GRAY}[{ts/1000:>7.1f}s]{RESET} "
        f"{mode_str}  "
        f"HR:{hr_color}{BOLD}{hr:>3}bpm{RESET}  "
        f"RR:{rr:>4}ms  "
        f"HRV:{hrv_r:>3}ms  "
        f"{p_color}{posture:<12}{RESET}  "
        f"Fall:{fall_str}  "
        f"Temp:{BOLD}{temp:.1f}C{RESET}  "
        f"Lux:{lux:<5.0f}  "
        f"Prox:{prox:<5}  "
        f"{contact_str}  "
        f"Alerts:{alert_str}"
    )


def colorize(line: str) -> str:
    if any(k in line for k in ("ERROR", "FAIL", "failed")):
        return f"{RED}{line}{RESET}"
    if any(k in line for k in ("ALERT", "Tachycardia", "Bradycardia", "FALL", "DETECTED")):
        return f"{RED}{BOLD}{line}{RESET}"
    if any(k in line for k in ("ONLINE", "DONE", "Started", "Active", "SUCCESS")):
        return f"{GREEN}{line}{RESET}"
    if line.startswith(("[INIT]", "[TASK_", "[SYSTEM]", "[SELF_TEST]")):
        return f"{CYAN}{line}{RESET}"
    if line.startswith(("###", "===", "---")):
        return f"{BLUE}{BOLD}{line}{RESET}"
    return line


def reader(ser: serial.Serial, stop: threading.Event):
    print(BANNER)
    while not stop.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not line.strip():
                continue

            if line.strip().startswith("{"):
                out = format_json(line.strip())
                print(out if out else line)
            else:
                print(colorize(line))
        except serial.SerialException:
            if not stop.is_set():
                print(f"\n{RED}[MONITOR] Serial connection lost.{RESET}")
            break
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"{CYAN}Connecting to {args.port} @ {args.baud} baud...{RESET}")
    try:
        ser = serial.Serial(args.port, args.baud,
                            bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=1.0)
    except serial.SerialException as e:
        print(f"{RED}Cannot open {args.port}: {e}{RESET}")
        print(f"{YELLOW}Check Device Manager → Ports and use --port COM? to specify the right one.{RESET}")
        sys.exit(1)

    print(f"{GREEN}Connected! Waiting for firmware output...{RESET}\n")

    stop = threading.Event()
    t = threading.Thread(target=reader, args=(ser, stop), daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            if cmd.strip():
                ser.write((cmd.strip() + "\r\n").encode("utf-8"))
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Disconnecting...{RESET}")
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
