# SmartBAN Intelligent Wearable Sensor Node Firmware

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-TI%20CC2652R1%20LaunchPad-red.svg)](https://www.ti.com/tool/LAUNCHXL-CC26X2R1)
[![Kernel](https://img.shields.io/badge/Kernel-TI--RTOS7-brightgreen.svg)]()
[![Standard](https://img.shields.io/badge/Standard-ETSI%20SmartBAN%20TS%20103%20326-orange.svg)]()
[![TinyML](https://img.shields.io/badge/TinyML-On--Chip%20Int8%20ECG-blueviolet.svg)]()

This repository contains the firmware and companion tools for a wearable SmartBAN (Smart Body Area Network) sensor node built on the Texas Instruments CC2652R1 LaunchPad and a custom multi-sensor shield.

The project brings together real-time physiological sensing (ECG), 3-axis motion tracking, environmental monitoring, on-chip machine learning for cardiac event detection, and wireless BLE telemetry with an Android companion application.

---

## 🖥️ Live Telemetry Desktop Workstation

Here is the real-time Python desktop visualizer running live with genuine hardware data over UART:

![SmartBAN Desktop Visualizer GUI](assets/gui_telemetry_workstation.png)

The workstation plots live 250 Hz biopotential waveforms, R-peak markers, 3-axis accelerometer dynamics, a 3D artificial horizon for posture tracking, environmental conditions (temperature, humidity, pressure, air quality), and live telemetry packets.

---

## 📱 Native Android Companion Application

The project includes a standalone native Android application (`android_app/`) that receives and displays live telemetry wirelessly over Bluetooth Low Energy:

- **Wireless Telemetry**: Captures live SmartBAN telemetry broadcast directly from the CC2652R1 over Bluetooth Low Energy.
- **Cardiac Electrophysiology**: Real-time heart rate (BPM), RR intervals, heart rate variability (HRV SDNN), and instantaneous visual arrhythmia warnings.
- **Edge-AI TinyML Beat Classifier**: Real-time on-chip AAMI EC57 cardiac beat classification (Normal [N], Premature Ventricular Contraction [V], Supraventricular [S], Fusion [F], Noise [Q]).
- **Biometrics and Environment**: Dual-source respiration rate (derived from thoracic impedance and ECG respiration), skin temperature, real-time posture classification (standing, sitting, supine, walking), and fall alerts.
- **SmartBAN TDMA and 5G Network Slicing**: Visualizes the 8-slot TDMA superframe (Scheduled Access S0 to S6 versus Contention Access CAP) and shows dynamic routing between high-efficiency 5G-mMTC (1 Hz, 99.03% bandwidth reduction) and low-latency 5G-URLLC (<1 ms emergency response).
- **Interactive Remote Simulation**: Includes an on-screen trigger to simulate a 5-second PVC arrhythmia event over the air, showing instant switching from 5G-mMTC to 5G-URLLC contention slots.
- **Ready to Install**: A pre-compiled APK (`SmartBAN_Monitor_v1.0.apk`) and a one-click automated USB installer script (`install_apk.bat`) are included in `android_app/`.

---

## 🔌 Hardware Setup

The system consists of a Texas Instruments CC2652R1 LaunchPad paired with a custom multi-sensor shield:

<p align="center">
  <img src="assets/hardware_shield_physical.jpg" alt="SmartBAN Sensor Shield and LaunchPad Hardware" width="48%" />
  <img src="assets/smartban_shield_pcb_layout.png" alt="SmartBAN Sensor Shield 4-Layer PCB Layout" width="48%" />
</p>

### On-Board Sensors

- **ADS1292R** (Texas Instruments): 24-bit 2-channel low-power analog front-end for ECG and respiration pneumography.
- **ADXL362** (Analog Devices): Ultra-low-power 3-axis MEMS accelerometer with integrated autonomous motion detection.
- **BME690** (Bosch Sensortec): Environmental sensor measuring gas resistance (VOC), barometric pressure, relative humidity, and ambient temperature.
- **MAX30102** (Analog Devices / Maxim Integrated): High-sensitivity optical pulse oximeter and heart-rate sensor.
- **MAX32664** (Analog Devices / Maxim Integrated): Ultra-low-power biometric sensor hub controller with embedded health algorithms.
- **VCNL4040** (Vishay): Integrated proximity sensor and high-precision ambient light sensor.
- **MLX90632** (Melexis): Medical-grade non-contact far-infrared (FIR) temperature sensor.
- **OPT4001 / OPT4041** (Texas Instruments): High-precision digital ambient light sensor.

### Power, Interface, and Control Components

- **CC2652R1 LaunchPad** (Texas Instruments): 48 MHz ARM Cortex-M4F microcontroller with 352 KB Flash, 80 KB RAM, and a multi-protocol 2.4 GHz radio.
- **TPAP2112K-1.8** (Tech Public): Ultra-low-noise 1.8V low-dropout (LDO) linear voltage regulator for analog rails.
- **PCA9306** (Texas Instruments): Dual bidirectional I2C and SMBus voltage-level translator.
- **CH455H** (WCH): 3/4-digit 7-segment LED display and keypad controller IC.
- **XL-SA2301SRWC**: 3-digit 7-segment numerical SMD LED display for on-board diagnostics.
- **Tactile Switches**: 6 push buttons for run-time modes (ECG test signals, optical modes, BT/tare, reset, and display mode cycle).
- **Status Indicator LEDs**: 11 surface-mount LEDs providing immediate visual feedback for power rails (3.3V, 1.8V), sensor states (ADS1292R heart rate lead, BME activity, ADXL motion, PPG pulse, IR channel), and system status.
- **ECG Lead Connector**: 3-pin terminal interface for Right Arm (RA), Left Arm (LA), and Right Leg Drive (RLD) electrode cables.

---

## 📁 Repository Guide

If you are new to the project, here is how the repository is structured:

```
├── 01_final_firmware_v12_stable/       # Flagship working version (v12 stable, start here!)
├── 02_dedicated_ecg_ads1292r/          # Standalone ADS1292R ECG bring-up with Python GUI
├── 03_ecg_leadless_test/               # Leadless electrode impedance and contact test
├── 04_dedicated_imu_adxl362/           # Standalone ADXL362 accelerometer test with 3D GUI
├── 05_all_in_one_baremetal/            # Multi-sensor integration on bare metal (No-RTOS)
├── 06_final_tirtos_all_in_one/         # Base TI-RTOS7 multi-tasking firmware
├── android_app/                        # Native Android telemetry app (Kotlin + Jetpack Compose + APK)
└── archive/                            # Earlier milestone builds (v01 to v11) and reference drivers
```

### 1. [01_final_firmware_v12_stable/](01_final_firmware_v12_stable/) (Recommended)
This is the main, fully working firmware. It runs on TI-RTOS7 and includes:
- **Live 250 Hz hardware ECG streaming**: True physical microvolt samples broadcast over BLE advertising frames.
- **On-chip TinyML neural network**: An 8-bit quantized classifier that identifies cardiac arrhythmia types (AAMI EC57 classes N, S, V, F, Q) directly on the microcontroller.
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

### 7. [android_app/](android_app/)
The companion Android mobile app built with Kotlin and Jetpack Compose. Provides real-time over-the-air ECG viewing, TinyML classification status, and 5G network slicing metrics on your smartphone. Includes complete source code, Gradle build files, and a pre-compiled ready-to-run APK.

### 8. [archive/](archive/)
Contains all earlier iterative builds (v01 to v11), including initial pinout adjustments, PPG experiments, and test scripts. Preserved so you can trace how the system developed step by step.

---

## ⚡ Quick Flash: No CCS Required

If you want to flash and run the pre-built firmware right away:

1. Connect the CC2652R1 LaunchPad to your PC via USB.
2. Flash the board using the automated script:
   ```bash
   cd 01_final_firmware_v12_stable
   python build_and_flash.py
   ```
   *(Or flash `all_in_one.hex` directly via TI UniFlash).*
3. Launch the desktop telemetry workstation:
   ```bash
   python sensor_gui.py --port COM3 --baud 115200
   ```
4. Optional: Install the Android mobile app:
   ```cmd
   cd android_app
   install_apk.bat
   ```

---

## 🛠️ Building with Code Composer Studio (CCS)

To compile and debug from source in CCS (version 12+ or CCS Theia with `tiarmclang`):

1. **Import**: Select **File -> Import... -> CCS Projects**, browse to any firmware directory (such as `01_final_firmware_v12_stable`), and click **Finish**.
2. **Build**: Right-click the project in Project Explorer and choose **Build Project** (generates `all_in_one.hex`).
3. **Flash / Debug**: Plug in the CC2652R1 LaunchPad, click **Debug** (`F11`), and press **Resume** (`F8`) to run.

---

## 📄 License
This project is licensed under the [MIT License](LICENSE).
