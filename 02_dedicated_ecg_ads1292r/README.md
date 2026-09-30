# 02_dedicated_ecg_ads1292r: Dedicated ADS1292R ECG Firmware

> Dedicated biopotential acquisition firmware and real-time visualization tool.  
> Target: Texas Instruments ADS1292R 24-bit AFE + CC2652R1 LaunchPad

---

## Overview

This firmware is a standalone driver and processing setup for the Texas Instruments ADS1292R analog front-end. It runs bare-metal with interrupt-driven 250 Hz DRDY sampling to deliver clean ECG waveforms.

### Key Points:
- Low noise biopotential reading: Measured noise floor is below 1 uV RMS on shorted inputs, matching the internal 1 mV test signal within 1% error.
- Settling protocol: Waits 100 ms for the internal reference to stabilize before running offset calibration, avoiding DC saturation.
- Cross-talk fix: Shuts off the 32 kHz respiration carrier during pure ECG acquisition, cutting baseline noise by 21 times.
- Pan-Tompkins algorithm: Embedded QRS detector with adaptive thresholding, real-time heart rate estimation, and HRV metrics.
- Python visualizer (`ecg_gui.py`): Real-time desktop oscilloscope with 50/60 Hz digital notch filters, clinical grid display, and beat marker overlay.

---

## File Directory

```text
02_dedicated_ecg_ads1292r/
├── main.c                              # Dedicated bare-metal ECG sampling superloop
├── ecg_dedicated.hex                   # Precompiled Intel HEX binary image
├── ecg_dedicated.out                   # ELF binary executable
├── diagnostic.syscfg                   # TI SysConfig pin and SPI configuration
├── build_and_flash.py                  # One-click build and flashing utility
├── ecg_gui.py                          # Real-time desktop ECG oscilloscope GUI
├── edgeai_ecg.c / edgeai_ecg.h         # Pan-Tompkins QRS and feature extraction code
├── FIRMWARE_REPORT_ECG_DEDICATED.md    # Technical verification report
├── MASTER_THESIS_ECG_DIARY.md          # Laboratory bench notes and log
└── TI_RTOS_INTEGRATION_HANDOVER.md     # Architecture documentation for RTOS migration
```

---

## Quick Start

### 1. Flash the Firmware
```bash
python build_and_flash.py
```

### 2. Open the ECG Oscilloscope
```bash
python ecg_gui.py --port COM3
```
Replace `COM3` with your LaunchPad's COM port.
