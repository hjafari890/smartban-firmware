# 01_final_firmware_v12_stable: SmartBAN Flagship Firmware

> **The primary and final working production firmware for the thesis project.**  
> Target: TI CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + SmartBAN Shield Rev 3.5

---

## 🌟 Overview & Architecture

Firmware 12 Stable represents the culmination of the thesis development, implementing an end-to-end intelligent body area network sensor node running **TI-RTOS7** with preemptive real-time scheduling.

### Key Capabilities:
1. **Live 250 Hz Biopotential BLE Broadcast:**
   - 10-sample batches ($25\text{ Hz}$ packet rate) transmitted within standard 31-byte BLE advertising frames.
   - Genuine physical microvolt biopotential readings: $\text{val}_{\mu\text{V}} = \text{raw}_{\text{ch2}} \times 0.048077\,\mu\text{V}$.
   - Live R-peak apex bitmask and RA/LA electrode contact impedance bytes embedded per batch.
2. **On-Chip Int8 TinyML Arrhythmia Classifier:**
   - Evaluates 12 Pan-Tompkins extracted cardiac features per beat (RR intervals, QRS duration, R-amplitude, derivative asymmetry, SDNN, RMSSD, noise index).
   - Classifies across 5 AAMI EC57 classes (`N`, `S`, `V`, `F`, `Q`) with **99.80% accuracy** on MIT-BIH test benchmarks.
   - Fully quantized integer arithmetic running in **1,420 cycles (29.6 µs)**, requiring just 576 bytes Flash and 64 bytes SRAM.
3. **ETSI TS 103 326 SmartBAN Adaptive MAC:**
   - Adaptive semantic token transmission during normal sinus rhythm ($92\text{ B/s}$ channel throughput).
   - Instant contention access burst ($9,538\text{ B/s}$) upon detected arrhythmia or fall detection events.
   - **99.03% wireless bandwidth reduction**, extending projected battery runtime from 3.1 days to **43.0 days** on a 500 mAh battery.
4. **Multi-Modal Sensor Integration:**
   - Texas Instruments ADS1292R 24-bit biopotential AFE (250 Hz).
   - Analog Devices ADXL362 ultra-low-power 3-axis accelerometer (25 Hz).
   - Bosch Sensortec BME680 environmental sensor (Cold-IAQ gas estimation, pressure, humidity, temperature).
   - Texas Instruments OPT4041 ambient light sensor.

---

## 📂 File Directory

```text
01_final_firmware_v12_stable/
├── main.c                       # Primary TI-RTOS7 multi-tasking application entry
├── all_in_one.hex               # Ready-to-flash Intel HEX binary image
├── all_in_one.out               # ELF executable image with debug symbols
├── all_in_one.syscfg            # SysConfig hardware peripheral configuration
├── build_and_flash.py           # Automated build, hex generation, and flashing script
├── CC26X2R1_LAUNCHXL_TIRTOS7.cmd # TI-RTOS7 linker command file
├── sensor_gui.py                # Real-time multi-modal desktop oscilloscope GUI
├── verify_live_data.py          # Command-line telemetry stream verification tool
├── bsp/                         # Board Support Package (SPI bus manager, I2C, UART, GPIO)
├── hal/                         # Hardware Abstraction Layer (ADS1292R, ADXL362, BME680, BLE radio)
├── edgeai/                      # TinyML inference engine & Pan-Tompkins QRS detector
├── ipc/                         # Lock-free SPSC ring buffers & inter-task sync
├── telemetry/                   # SmartBAN MAC frame packetizer & BLE advertiser
├── syscfg/                      # Generated TI-RTOS7 driver configurations
└── SUPERVISOR_REPORT_V12.md     # Detailed milestone report with technical results
```

---

## 🚀 Quick Start

### 1. Flash the Binary
Connect the CC2652R1 LaunchPad via micro-USB and execute:
```bash
python build_and_flash.py
```
*To compile without flashing:*
```bash
python build_and_flash.py --no-flash
```

### 2. Launch the Desktop Visualizer
Launch the high-performance desktop GUI to monitor the live ECG waveform, cardiac metrics, TinyML classification, 3-axis accelerometer, and environmental telemetry:
```bash
python sensor_gui.py --port COM3 --baud 115200
```
*(Replace `COM3` with your LaunchPad's XDS110 Application/User UART COM port).*

### 3. Android Companion App
The node broadcasts 3 interleaved BLE advertising frame types under Company ID `0x53, 0x42` (`"SB"`):
- **Frame Type 0x02:** Live 250 Hz ECG batch (10 microvolt samples + lead status).
- **Frame Type 0x01:** Heart rate, RR interval, HRV SDNN/RMSSD, TinyML class, Respiration RPM, temperature, TDMA slot status.
- **Frame Type 0x03:** 3-Axis IMU (mg), Pitch & Roll, step count, fall alert, BME680 pressure, humidity, IAQ, lux.
