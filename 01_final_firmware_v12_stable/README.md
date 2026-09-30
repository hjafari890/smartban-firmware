# 01_final_firmware_v12_stable: SmartBAN Flagship Firmware

> The primary, fully working production firmware for the project.  
> Target: TI CC2652R1 LaunchPad (CC26X2R1_LAUNCHXL) + SmartBAN Shield Rev 3.5

---

## Overview

Firmware 12 Stable is the complete, fully-tested firmware running on TI-RTOS7. It handles all sensors concurrently, runs on-chip machine learning, and broadcasts live data wirelessly over Bluetooth Low Energy.

### What It Does:
1. **Live 250 Hz ECG over BLE**:
   - Gathers genuine microvolt ECG samples from the ADS1292R at 250 Hz.
   - Packs 10 samples per batch (25 Hz packet rate) into standard 31-byte BLE advertising frames.
   - Includes real-time R-peak bitmasks and electrode contact status.
2. **On-Chip TinyML Arrhythmia Classifier**:
   - Extracts 12 cardiac features per beat (RR intervals, QRS width, amplitude, SDNN, RMSSD, noise index).
   - Runs an 8-bit quantized neural network in 1,420 clock cycles (under 30 microseconds) on the Cortex-M4F.
   - Classifies beats into 5 standard classes (N, S, V, F, Q) with 99.80% benchmark accuracy.
   - Uses just 576 bytes of Flash and 64 bytes of SRAM.
3. **SmartBAN Adaptive MAC Protocol**:
   - Follows ETSI TS 103 326.
   - In normal heart rhythm, it sends small 92 B/s health summaries to save energy.
   - When an irregular beat or a fall is detected, it automatically bursts full raw data (9,538 B/s).
   - Cuts wireless bandwidth by 99% and extends battery life from 3 days up to 43 days on a 500 mAh battery.
4. **Multi-Sensor Integration**:
   - ADS1292R biopotential front-end (250 Hz).
   - ADXL362 3-axis accelerometer (25 Hz).
   - BME680 environmental sensor (temperature, humidity, pressure, cold-IAQ air quality).
   - OPT4041 ambient light sensor.

---

## Folder Structure

```text
01_final_firmware_v12_stable/
├── main.c                       # Primary TI-RTOS7 application entry
├── all_in_one.hex               # Ready-to-flash Intel HEX binary image
├── all_in_one.out               # ELF executable image with debug symbols
├── all_in_one.syscfg            # SysConfig hardware peripheral configuration
├── build_and_flash.py           # Automated build, hex generation, and flashing script
├── CC26X2R1_LAUNCHXL_TIRTOS7.cmd # TI-RTOS7 linker command file
├── sensor_gui.py                # Real-time desktop oscilloscope GUI
├── verify_live_data.py          # Command-line telemetry stream verification tool
├── bsp/                         # Board Support Package (SPI bus manager, I2C, UART, GPIO)
├── hal/                         # Hardware drivers (ADS1292R, ADXL362, BME680, BLE radio)
├── edgeai/                      # TinyML inference engine and Pan-Tompkins QRS detector
├── ipc/                         # Lock-free SPSC ring buffers and inter-task sync
├── telemetry/                   # SmartBAN frame packetizer and BLE advertiser
├── syscfg/                      # Generated TI-RTOS7 driver configurations
└── SUPERVISOR_REPORT_V12.md     # Technical validation report with performance numbers
```

---

## Quick Start

### 1. Flash the Board
Connect your CC2652R1 LaunchPad via micro-USB and run:
```bash
python build_and_flash.py
```
To build without flashing:
```bash
python build_and_flash.py --no-flash
```

### 2. Run the Desktop Visualizer
Launch the desktop GUI to view the real-time ECG waveform, cardiac metrics, TinyML classifications, 3-axis motion, and environmental readings:
```bash
python sensor_gui.py --port COM3 --baud 115200
```
Replace `COM3` with your LaunchPad's COM port.

### 3. Android Companion App
The node broadcasts 3 interleaved BLE advertising frame types under Company ID `0x53, 0x42` ("SB"):
- Frame Type 0x02: Live 250 Hz ECG batch (10 microvolt samples + lead status).
- Frame Type 0x01: Heart rate, RR interval, HRV SDNN/RMSSD, TinyML class, respiration, temperature, TDMA slot status.
- Frame Type 0x03: 3-Axis IMU (mg), pitch and roll, step count, fall alert, BME680 pressure, humidity, IAQ, lux.
