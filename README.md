# SmartBAN Intelligent Wearable Sensor Node Firmware

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-TI%20CC2652R1%20LaunchPad-red.svg)](https://www.ti.com/tool/LAUNCHXL-CC26X2R1)
[![Kernel](https://img.shields.io/badge/Kernel-TI--RTOS7-brightgreen.svg)]()
[![Standard](https://img.shields.io/badge/Standard-ETSI%20SmartBAN%20TS%20103%20326-orange.svg)]()
[![TinyML](https://img.shields.io/badge/TinyML-On--Chip%20Int8%20ECG-blueviolet.svg)]()

> **Master's Thesis Project**  
> **Author:** Hesamoddin (Hesam) Jafari  
> **Supervisor:** Prof. Konstantin Mikhaylov  
> **Affiliation:** Centre for Wireless Communications (CWC) / ITEE, University of Oulu, Finland  
> **Hardware Target:** Texas Instruments CC2652R1 (`CC26X2R1_LAUNCHXL`, ARM Cortex-M4F @ 48 MHz) + Custom SmartBAN Shield Rev 3.5  

---

## 📖 Overview

This repository contains the complete firmware development suite for an intelligent, ultra-low-power SmartBAN (Smart Body Area Network) wearable sensor node. The project addresses a fundamental challenge in digital healthcare: **can on-node semantic intelligence and adaptive communication reduce wireless channel load and power consumption enough to make clinical-grade continuous physiological monitoring viable on a miniature coin-cell battery?**

Starting from bare-metal sensor bring-up to a preemptive TI-RTOS7 multi-tasking architecture with on-chip Int8 TinyML cardiac arrhythmia classification and ETSI TS 103 326 semantic MAC protocol, this repository provides both production-ready firmware and modular verification targets.

---

## 🚀 Repository Navigation

The firmware is structured to allow immediate deployment of the **final stable release**, targeted evaluation of individual sensors, bare-metal baseline testing, and access to all historical milestones:

```
├── 01_final_firmware_v12_stable/       # 🌟 FINAL WORKING PRODUCTION FIRMWARE (Recommended)
├── 02_dedicated_ecg_ads1292r/          # 🫀 Dedicated ADS1292R 250 Hz ECG AFE & Python GUI
├── 03_ecg_leadless_test/               # ⚡ Leadless electrode contact & impedance verification
├── 04_dedicated_imu_adxl362/           # 🧭 Dedicated ADXL362 3-Axis IMU & 3D Visualizer GUI
├── 05_all_in_one_baremetal/            # ⚙️ Complete multi-sensor integration on Bare-Metal (No-RTOS)
├── 06_final_tirtos_all_in_one/         # ⏱️ Foundational TI-RTOS7 preemptive multi-tasking architecture
└── milestones_archive/                 # 📚 Chronological development progression (v01 - v12)
```

### 1. [01_final_firmware_v12_stable/](01_final_firmware_v12_stable/) — **Final Working Version (v12 Stable)**
- **What it is:** The complete, fully-tested flagship firmware running TI-RTOS7 on the CC2652R1.
- **Key Features:**
  - **Live 250 Hz Hardware ECG Streaming:** Genuine physical microvolt biopotential streaming over BLE advertising frames.
  - **On-Chip Int8 TinyML Arrhythmia Classifier:** 3-layer neural network (576 bytes Flash, 64 bytes SRAM, 29.6 µs latency) classifying AAMI EC57 classes (`N`, `S`, `V`, `F`, `Q`) with 99.80% MIT-BIH test accuracy.
  - **ETSI TS 103 326 SmartBAN MAC:** Adaptive semantic mode reducing radio throughput by 99% (92 B/s vs 9,538 B/s raw) and extending battery runtime by ~14x.
  - **Multi-Sensor Acquisition:** ADS1292R biopotential, ADXL362 3-axis motion dynamics, and BME680 environmental metrics.
  - **Android Companion App:** Seamless BLE reception and display across a 4-tab dashboard on Android (API 35+).
  - **Host Python GUI:** Interactive real-time oscilloscope (`sensor_gui.py`).

### 2. [02_dedicated_ecg_ads1292r/](02_dedicated_ecg_ads1292r/) — **Dedicated ECG Module**
- Focused standalone driver for the Texas Instruments ADS1292R 24-bit biopotential front-end.
- Features dynamic 250 Hz DRDY interrupt acquisition, Pan-Tompkins QRS peak detection, R-to-R interval calculation, heart rate variability (HRV), and desktop GUI (`ecg_gui.py`).

### 3. [03_ecg_leadless_test/](03_ecg_leadless_test/) — **ECG Leadless Test**
- Specialized firmware designed to validate leadless dry-contact electrode configurations, contact impedance monitoring, and signal-to-noise ratio (SNR) optimization.
- Accompanied by serial verification tool (`view_ecg_test.py`).

### 4. [04_dedicated_imu_adxl362/](04_dedicated_imu_adxl362/) — **Dedicated IMU Module**
- Isolated bring-up and characterization of the Analog Devices ADXL362 ultra-low-power 3-axis MEMS accelerometer.
- Includes 64-sample boot calibration (resolving PCB solder offsets), static 0.994 g measurement accuracy, 0.3° attitude precision, and a live 3D orientation visualizer (`gui_3d_imu.py`).

### 5. [05_all_in_one_baremetal/](05_all_in_one_baremetal/) — **All-in-One Bare-Metal**
- Complete multi-sensor integration operating in a deterministic bare-metal superloop (`cc13x2_cc26x2_nortos.cmd`).
- Ideal for benchmarking baseline latency, low overhead, and bare-metal resource consumption before RTOS migration.

### 6. [06_final_tirtos_all_in_one/](06_final_tirtos_all_in_one/) — **TI-RTOS All-in-One Core**
- The foundational TI-RTOS7 preemptive multi-tasking baseline architecture.
- Implements mutex-protected SPI bus arbitration (Mode 1 for ADS1292R vs Mode 0 for ADXL362 on SSI0) and lock-free single-producer single-consumer ring buffers (`ring_buffer.c`) with Cortex-M4 memory barriers.

### 7. [milestones_archive/](milestones_archive/) — **Milestone Archive**
- Contains the chronological evolution of the thesis project:
  - `01_hardware_diagnostic`: Initial GPIO, SPI, I2C, and clock bring-up.
  - `02_ppg_diagnostic` & `03_ppg_led_bringup`: Optical PPG subsystem bring-up notes.
  - `05_ecg_respiration_real`: ADS1292R impedance pneumography respiration testing.
  - `06_tirtos_smartban`: Early RTOS task decomposition prototype.
  - `10_smartban_semantic_tinyml`: TinyML classifier model training & verification.
  - `11_smartban_ble_android`: Early BLE GATT/adv integration with mobile client.
  - `12_smartban_live_ble_streaming`: Raw BLE radio broadcast engine.
  - `protocentral-ads1292r-arduino`: Reference driver library.
  - `test_ecg`: Host-side synthetic and playback test benches.

---

## 📊 Firmware Comparison Matrix

| Firmware Target | Kernel / Mode | ECG Rate | IMU Rate | TinyML AI | BLE Broadcast | Host GUI Visualizer |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **01 Final Firmware v12 Stable** | **TI-RTOS7** | **250 Hz** | **25 Hz** | **Yes (Int8)** | **Yes (3-Frame SB)** | `sensor_gui.py` |
| **02 Dedicated ECG ADS1292R** | No-RTOS | 250 Hz | — | Pan-Tompkins | UART Stream | `ecg_gui.py` |
| **03 ECG Leadless Test** | No-RTOS | 250 Hz | — | — | UART Stream | `view_ecg_test.py` |
| **04 Dedicated IMU ADXL362** | No-RTOS | — | 25 Hz / 100 Hz | — | UART Stream | `gui_3d_imu.py` |
| **05 All-In-One Bare-Metal** | No-RTOS Superloop | 250 Hz | 25 Hz | Pan-Tompkins | UART Stream | `sensor_gui.py` |
| **06 TI-RTOS All-in-One** | TI-RTOS7 Preemptive | 250 Hz | 25 Hz | — | UART Stream | `sensor_gui.py` |

---

## 🛠️ Hardware Setup & Pin Mapping

### CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + SmartBAN Shield Rev 3.5

- **SPI Bus (SSI0 Shared):**
  - `DIO9`: SCLK
  - `DIO8`: MOSI / PICO (remapped via `IOCPortConfigureSet`)
  - `DIO10`: MISO / POCI (remapped via `IOCPortConfigureSet`)
  - `DIO28`: ADS1292R Chip Select (`CS_ECG`)
  - `DIO27`: ADXL362 Chip Select (`CS_IMU`)
- **ADS1292R Biopotential Controls:**
  - `DIO29`: `DRDY_ECG` (250 Hz interrupt)
  - `DIO26`: `RESET_ECG`
  - `DIO25`: `START_ECG`
- **I2C Bus (I2C0):**
  - `DIO4`: SCL (BME680, OPT4041)
  - `DIO5`: SDA (BME680, OPT4041)
- **UART / Telemetry:**
  - `DIO13`: UART RX (115200 baud)
  - `DIO12`: UART TX (115200 baud)

---

## ⚡ Quick Start: Flashing & Running

### Prerequisites
1. **Hardware:** TI CC2652R1 LaunchPad connected via micro-USB.
2. **Software Tools:**
   - Python 3.9+ with `pyserial`, `numpy`, `pyqtgraph` / `PyQt5` (or `matplotlib` for 3D visualizers).
   - [TI UniFlash](https://www.ti.com/tool/UNIFLASH) or [SmartRF Flash Programmer 2](https://www.ti.com/tool/FLASH-PROGRAMMER-2) (optional if flashing precompiled `.hex`).
   - For recompiling: TI Code Composer Studio (CCS) with `tiarmclang 5.1.1.LTS`, SimpleLink CC13xx/CC26xx SDK 8.33, and SysConfig 1.21.1.

### 1. Flash the Final Production Firmware (v12 Stable) in 30 Seconds
Each firmware folder includes a precompiled `.hex` binary and a one-click flashing script `build_and_flash.py`:

```bash
cd 01_final_firmware_v12_stable

# Flash precompiled binary using SmartRF / UniFlash directly:
python build_and_flash.py
```

*Or flash `all_in_one.hex` directly using UniFlash GUI.*

### 2. Launch Real-Time Desktop Oscilloscope & GUI
Ensure the LaunchPad COM port is identified (e.g. `COM3` on Windows):

```bash
cd 01_final_firmware_v12_stable
python sensor_gui.py --port COM3 --baud 115200
```

### 3. Dedicated Sensor Testing
To test the ECG or IMU in isolation:

```bash
# Dedicated ECG
cd 02_dedicated_ecg_ads1292r
python build_and_flash.py
python ecg_gui.py --port COM3

# Dedicated IMU 3D Visualizer
cd 04_dedicated_imu_adxl362
python build_and_flash.py
python gui_3d_imu.py --port COM3
```

---

## 🔬 Scientific & Technical Contributions

1. **Semantic Energy-Bandwidth Pareto Frontier:** Demonstrated a **99.03% bandwidth reduction** using on-node TinyML cardiac event detection paired with ETSI TS 103 326 TDMA slot escalation, boosting calculated 500 mAh battery life from 3.1 days (continuous raw streaming) to **43.0 days** (semantic mode).
2. **Zero-Latency Bus Arbitration:** Resolved hardware SPI bus collisions between biopotential (Mode 1) and inertial (Mode 0) sensors on a shared hardware SSI peripheral via a zero-cost atomic bus manager.
3. **Cross-Rail Analog Decoupling:** Identified and eliminated thermal droop artifacts on the 1.8 V analog rail caused by BME680 gas heating elements through cold-IAQ estimation algorithms.
4. **Cortex-M4 Optimized TinyML:** Quantized 8-bit integer feedforward neural network executing in 1,420 cycles (29.6 µs at 48 MHz) occupying only 576 bytes of non-volatile storage.

---

## 📜 Citation & Academic Attribution

If you utilize this firmware codebase, shield pinout mapping, or TinyML model in your academic research or thesis, please cite:

```bibtex
@mastersthesis{jafari2026smartban,
  author       = {Hesamoddin Jafari},
  title        = {Integration and Validation of an Intelligent Sensor Node for a SmartBAN Testbed},
  school       = {University of Oulu, Faculty of Information Technology and Electrical Engineering (ITEE)},
  department   = {Centre for Wireless Communications (CWC)},
  year         = {2026},
  address      = {Oulu, Finland},
  supervisor   = {Prof. Konstantin Mikhaylov}
}
```

---

## 📄 License
This repository is licensed under the [MIT License](LICENSE).
