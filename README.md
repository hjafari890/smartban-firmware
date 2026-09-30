# SmartBAN Intelligent Wearable Sensor Node Firmware

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-TI%20CC2652R1%20LaunchPad-red.svg)](https://www.ti.com/tool/LAUNCHXL-CC26X2R1)
[![Kernel](https://img.shields.io/badge/Kernel-TI--RTOS7-brightgreen.svg)]()
[![Standard](https://img.shields.io/badge/Standard-ETSI%20SmartBAN%20TS%20103%20326-orange.svg)]()
[![TinyML](https://img.shields.io/badge/TinyML-On--Chip%20Int8%20ECG-blueviolet.svg)]()

This repository contains the firmware for a wearable SmartBAN (Smart Body Area Network) sensor node built on the Texas Instruments CC2652R1 LaunchPad and a custom sensor shield.

The project combines real-time physiological sensing (ECG), 3-axis motion tracking, environmental metrics, on-chip machine learning for cardiac event detection, and wireless BLE telemetry.

---

## 🖥️ Live Telemetry GUI

Here is the real-time Python desktop visualizer running live with genuine hardware data over UART:

![SmartBAN Desktop Visualizer GUI](assets/gui_telemetry_workstation.png)

The workstation plots live 250 Hz biopotential waveforms, R-peak markers, 3-axis accelerometer dynamics, a 3D artificial horizon for posture tracking, environmental conditions (temperature, humidity, pressure, air quality), and live telemetry packets.

---

## 🔌 Hardware Setup

The system consists of a Texas Instruments CC2652R1 LaunchPad paired with a custom multi-sensor shield:

<p align="center">
  <img src="assets/hardware_shield_physical.jpg" alt="SmartBAN Sensor Shield and LaunchPad Hardware" width="48%" />
  <img src="assets/smartban_shield_pcb_layout.png" alt="SmartBAN Sensor Shield 4-Layer PCB Layout" width="48%" />
</p>

### On-Board Sensors & Components
- **ADS1292R**: 24-bit analog front-end for 2-channel ECG and respiration pneumography.
- **ADXL362**: Ultra-low-power 3-axis MEMS accelerometer for activity tracking and fall detection.
- **BME680**: Gas, pressure, humidity, and temperature sensor.
- **OPT4041**: High-precision ambient light sensor.
- **CC2652R1 LaunchPad**: 48 MHz ARM Cortex-M4F microcontroller with 2.4 GHz radio.

---

## 📁 Repository Guide

If you are new to the project, here is how the folders are organized:

```
├── 01_final_firmware_v12_stable/       # The final working version (v12 stable, start here!)
├── 02_dedicated_ecg_ads1292r/          # Dedicated ADS1292R ECG bring-up with Python GUI
├── 03_ecg_leadless_test/               # Leadless electrode impedance and contact test
├── 04_dedicated_imu_adxl362/           # Dedicated ADXL362 accelerometer test with 3D GUI
├── 05_all_in_one_baremetal/            # Multi-sensor integration running on bare metal (No-RTOS)
├── 06_final_tirtos_all_in_one/         # Base TI-RTOS7 multi-tasking firmware
└── milestones_archive/                 # Earlier milestone builds and reference drivers
```

### 1. [01_final_firmware_v12_stable/](01_final_firmware_v12_stable/) (Recommended)
This is the main, fully working firmware. It runs on TI-RTOS7 and includes:
- **Live 250 Hz hardware ECG streaming**: True physical microvolt samples broadcast over BLE advertising frames.
- **On-chip TinyML neural network**: An 8-bit quantized classifier that identifies cardiac arrhythmia types (AAMI EC57 classes N, S, V, F, Q) in under 30 microseconds right on the chip.
- **SmartBAN adaptive MAC**: An adaptive protocol (ETSI TS 103 326) that drops radio bandwidth by 99% during normal heart rhythms and bursts full data only when an irregular beat or fall occurs.
- **All sensors active**: Live ECG, IMU motion, and environmental data.
- **Companion tools**: Works directly with `sensor_gui.py` and the native Android app.

### 2. [02_dedicated_ecg_ads1292r/](02_dedicated_ecg_ads1292r/)
A clean, standalone firmware focused strictly on getting clean ECG from the ADS1292R. It handles 250 Hz DRDY interrupt sampling, internal reference settling, and real-time Pan-Tompkins QRS peak detection. Comes with its own desktop oscilloscope (`ecg_gui.py`).

### 3. [03_ecg_leadless_test/](03_ecg_leadless_test/)
A test firmware to check leadless dry-contact electrodes. It tests contact impedance and signal quality without using wet gel pads.

### 4. [04_dedicated_imu_adxl362/](04_dedicated_imu_adxl362/)
Dedicated firmware for the ADXL362 accelerometer over SPI Mode 0. It includes a boot calibration to zero out PCB mounting tilt, measures static gravity with 0.994 g accuracy, and includes an interactive 3D attitude visualizer (`gui_3d_imu.py`) plus a recorded demo video.

### 5. [05_all_in_one_baremetal/](05_all_in_one_baremetal/)
All sensors working together inside a simple bare-metal superloop (No-RTOS). Great for understanding the basic driver logic without RTOS task scheduling overhead.

### 6. [06_final_tirtos_all_in_one/](06_final_tirtos_all_in_one/)
The foundational TI-RTOS7 firmware. It sets up preemptive tasks, protects the shared SPI bus with a mutex (handling ADS1292R Mode 1 and ADXL362 Mode 0 without collisions), and passes samples through lock-free ring buffers.

### 7. [milestones_archive/](milestones_archive/)
Contains all the earlier steps from day one (v01 to v11), including initial pinout fixes, PPG experiments, and test scripts. Preserved so you can trace how the system was built step by step.

---

## ⚡ Quick Start: How to Flash and Run

### What You Need
1. TI CC2652R1 LaunchPad plugged in through USB.
2. Python 3.9+ with `pyserial`, `numpy`, and `pyqtgraph` (or `matplotlib`).
3. For compiling from source: Code Composer Studio with `tiarmclang 5.1.1.LTS`, SimpleLink SDK 8.33, and SysConfig 1.21.1.
4. If you just want to flash: UniFlash or SmartRF Flash Programmer 2 (or run the provided Python script).

### Step 1: Flash the Board in 30 Seconds
Each folder has a ready-to-use `.hex` file and a helper script. For the final stable firmware:

```bash
cd 01_final_firmware_v12_stable
python build_and_flash.py
```

*Note: You can also open UniFlash and flash `all_in_one.hex` directly.*

### Step 2: Open the Desktop GUI
Find your LaunchPad COM port (for example, `COM3` on Windows) and run:

```bash
cd 01_final_firmware_v12_stable
python sensor_gui.py --port COM3 --baud 115200
```

### Step 3: Run the Standalone Tests
To test individual sensors:

```bash
# Test ECG only
cd 02_dedicated_ecg_ads1292r
python build_and_flash.py
python ecg_gui.py --port COM3

# Test IMU with 3D orientation display
cd 04_dedicated_imu_adxl362
python build_and_flash.py
python gui_3d_imu.py --port COM3
```

---

## 📌 Pin Configuration

- **Shared SPI Bus (SSI0)**:
  - SCLK: `DIO9`
  - MOSI / PICO: `DIO8` (remapped via IO controller)
  - MISO / POCI: `DIO10` (remapped via IO controller)
  - CS ADS1292R: `DIO28`
  - CS ADXL362: `DIO27`
- **ADS1292R Control Lines**:
  - DRDY Interrupt (250 Hz): `DIO29`
  - Reset: `DIO26`
  - Start: `DIO25`
- **I2C Bus (I2C0)**:
  - SCL: `DIO4` (BME680, OPT4041)
  - SDA: `DIO5` (BME680, OPT4041)
- **UART Communication**:
  - TX: `DIO12` (115200 baud)
  - RX: `DIO13` (115200 baud)

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).
