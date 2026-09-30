# 06_final_tirtos_all_in_one: TI-RTOS7 Multi-Tasking Architecture Baseline

> Foundational preemptive RTOS kernel integration with SPI bus arbitration and lock-free ring buffers.  
> Target: Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + SmartBAN Shield Rev 3.5

---

## ⏱️ Overview

This firmware establishes the core **TI-RTOS7** preemptive real-time operating system architecture for multi-sensor data acquisition. Moving from bare-metal superloops to a preemptive kernel required engineering solutions to critical real-time embedded challenges:

### Architectural Innovations:
1. **Mutex-Protected SPI Bus Arbitration (`bsp_spi.c`):**
   - The ADS1292R operates in **SPI Mode 1** at 250 Hz; the ADXL362 operates in **SPI Mode 0** at 25 Hz.
   - Both share the CC2652R1 hardware `SSI0` peripheral.
   - An atomic bus manager protects transactions with an RTOS mutex and enforces a $2\,\mu\text{s}$ guard interval during mode reconfiguration, preventing context-switch mid-frame bus collisions.
2. **Thermal Decoupling via Cold-IAQ Estimation:**
   - The Bosch BME680 gas heater pulls $\sim 16\text{ mA}$ during gas measurement cycles, causing an $18\text{ mV}$ droop on the shared 1.8 V analog rail.
   - In bare-metal prototypes, this appeared as a false cardiac artifact at 50.8 BPM.
   - Resolved by firing the heater once at boot to establish baseline resistance, followed by cold IAQ estimation via temperature and humidity compensation models.
3. **Lock-Free SPSC Ring Buffers (`ring_buffer.c`):**
   - Single-producer single-consumer circular buffers with ARM Cortex-M4 Data Memory Barriers (`__DMB()`).
   - Guarantees zero dropped biopotential samples between the 250 Hz sensor acquisition task and telemetry task without blocking the interrupt-critical path.

---

## 📂 File Directory

```text
06_final_tirtos_all_in_one/
├── main.c                                   # Primary TI-RTOS7 multi-tasking application entry
├── all_in_one.hex                           # Precompiled Intel HEX binary
├── all_in_one.out                           # ELF executable image
├── all_in_one.syscfg                        # TI SysConfig kernel and driver definitions
├── build_and_flash.py                       # Automated build and flashing script
├── CC26X2R1_LAUNCHXL_TIRTOS7.cmd             # Linker command file for TI-RTOS7
├── sensor_gui.py                            # Production multi-sensor desktop oscilloscope
├── verify_live_data.py                      # Serial telemetry verification tool
├── bsp/                                     # Board support package (SPI bus manager, I2C, UART)
├── hal/                                     # Hardware drivers (ADS1292R, ADXL362, BME680)
├── ipc/                                     # Lock-free SPSC ring buffers
├── telemetry/                               # Binary protocol serialization
├── syscfg/                                  # Generated driver configurations
└── THESIS_FIRMWARE_REPORT_INTEGRATION.md    # Full integration engineering report
```

---

## 🚀 Quick Start

### 1. Build and Flash
```bash
python build_and_flash.py
```

### 2. Run Desktop GUI
```bash
python sensor_gui.py --port COM3 --baud 115200
```
