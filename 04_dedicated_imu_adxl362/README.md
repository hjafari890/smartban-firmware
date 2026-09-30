# 04_dedicated_imu_adxl362: Dedicated ADXL362 3-Axis IMU Firmware

> Isolated motion tracking and attitude estimation suite with 3D orientation visualization.  
> Target: Analog Devices ADXL362 Ultra-Low-Power 3-Axis MEMS Accelerometer + CC2652R1 LaunchPad

---

## 🧭 Overview

This firmware provides full bring-up, static/dynamic calibration, and attitude estimation for the **Analog Devices ADXL362** ultra-low-power accelerometer over SPI Mode 0.

### Key Highlights:
- **Ultra-Low Current Consumption:** Sub-$2\,\mu\text{A}$ operation at 100 Hz output data rate (ODR).
- **PCB Solder Offset Calibration:** Resolves a mechanical solder offset on the Rev 3.5 shield (where Y-axis zero-g sat at $4.7\text{ g}$ equivalent) by dynamically switching to $\pm 8\text{ g}$ dynamic range and executing a 64-sample boot calibration.
- **Accurate Static & Dynamic Metrics:** Delivers static measurement accuracy of $0.994\text{ g}$ with $0.3^\circ$ attitude angle precision.
- **Activity & Fall Detection:** Configures internal autonomous motion-detection and free-fall thresholds.
- **Interactive 3D Visualizer (`gui_3d_imu.py`):** Real-time desktop application rendering a 3D orientation aircraft/cube matching sensor pitch, roll, and acceleration vectors.

---

## 📂 File Directory

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
├── run_3d_gui.bat                                           # Windows one-click launcher for 3D visualizer
├── view_imu_test.py                                         # Serial CLI streaming verification
├── Recording 2026-09-26 165950.mp4                          # Live hardware video demo of 3D visualizer
└── FIRMWARE_REPORT_ADXL362_IMU.md                           # Comprehensive engineering verification report
```

---

## 🚀 Quick Start

### 1. Flash the Dedicated IMU Firmware
```bash
python build_and_flash.py
```

### 2. Launch 3D Visualizer
```bash
python gui_3d_imu.py --port COM3
```
*Or double click `run_3d_gui.bat`.*
