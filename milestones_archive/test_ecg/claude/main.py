"""
Usage:
    python main.py --simulate                  # no hardware needed, test the pipeline/GUI
    python main.py --port COM5                  # Windows, real hardware
    python main.py --port /dev/ttyACM0           # Linux, real hardware
    python main.py --port /dev/tty.usbmodemXXXX  # macOS, real hardware

Run `python main.py --list-ports` to see available serial ports if you're
not sure what your board enumerated as.
"""
from __future__ import annotations

import argparse
import sys

from serial_reader import SerialSource, SimulatedSource
from gui import run_gui

DEFAULT_BAUD = 115200
DEFAULT_FS = 500


def list_ports():
    import serial.tools.list_ports

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"{p.device}  -  {p.description}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="Serial port the CC2652R1 enumerated as")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--fs", type=int, default=DEFAULT_FS, help="Expected sample rate (must match ADS1292_SAMPLE_RATE_SPS in firmware)")
    parser.add_argument("--simulate", action="store_true", help="Use a synthetic ECG source instead of real hardware")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    if args.simulate:
        source = SimulatedSource(fs=args.fs)
    else:
        if not args.port:
            print("Specify --port (or use --simulate to test without hardware). "
                  "Run with --list-ports to see what's available.")
            sys.exit(1)
        source = SerialSource(args.port, args.baud)

    run_gui(source, fs=args.fs)


if __name__ == "__main__":
    main()
