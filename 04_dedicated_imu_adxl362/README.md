# 04_dedicated_imu_adxl362: Dedicated ADXL362 3-Axis IMU Firmware

> Motion tracking and attitude estimation firmware with 3D orientation visualization.  
> Target: Analog Devices ADXL362 Accelerometer + CC2652R1 LaunchPad

---

## Overview

This firmware handles bring-up, calibration, and attitude estimation for the Analog Devices ADXL362 ultra-low-power accelerometer over SPI Mode 0.

### Key Points:
- Ultra-low power: Draws under 2 uA at 100 Hz output data rate.
- Solder offset fix: Fixes a mechanical solder offset on the Rev 3.5 shield (where the Y-axis zero-g bias sat near 4.7 g) by using +-8 g dynamic range and running a 64-sample boot calibration.
- Accurate metrics: Achieves static measurement accuracy of 0.994 g with 0.3 degree attitude precision.
- Fall detection: Uses built-in motion and free-fall interrupt thresholds.
- 3D visualizer (`gui_3d_imu.py`): Real-time desktop application rendering a 3D airplane showing live pitch, roll, and acceleration vectors. Includes a recorded demo video (`Recording 2026-09-26 165950.mp4`).

---

## File Directory

```text
04_dedicated_imu_adxl362/
├── main.c                                                   # Core IMU sampling and SPI routines
├── imu_adxl362_test.hex                                     # Precompiled Intel HEX binary image
├── imu_adxl362_test.out                                     # ELF executable image
├── diagnostic.syscfg                                        # SysConfig peripheral definitions
├── build_and_flash.py                                       # Automated build and flash script
├── cc13x2_cc26x2_nortos.cmd                                 # NoRTOS linker command file
├── gui_3d_imu.py                                            # Real-time 3D flight orientation visualizer
├── gui_imu_visualizer.py                                    # Multi-channel time-series accelerometer GUI
├── run_3d_gui.bat                                           # Windows launcher for 3D visualizer
├── view_imu_test.py                                         # Serial CLI streaming check
├── Recording 2026-09-26 165950.mp4                          # Hardware video demo of 3D visualizer
└── FIRMWARE_REPORT_ADXL362_IMU.md                           # Verification report
```

---

## Quick Start

### 1. Flash the Firmware
```bash
python build_and_flash.py
```

### 2. Launch the 3D Visualizer
```bash
python gui_3d_imu.py --port COM3
```
You can also double click `run_3d_gui.bat`.
