# 06_final_tirtos_all_in_one: TI-RTOS7 Multi-Tasking Architecture Baseline

> Preemptive RTOS kernel integration with SPI bus arbitration and lock-free ring buffers.  
> Target: Texas Instruments CC2652R1 LaunchPad (CC26X2R1_LAUNCHXL) + SmartBAN Shield Rev 3.5

---

## Overview

This firmware is the foundational TI-RTOS7 preemptive real-time operating system setup for multi-sensor data acquisition. Moving from bare-metal superloops to a preemptive kernel solves several critical real-time embedded challenges:

### Key Highlights:
1. **SPI Bus Arbitration (`bsp_spi.c`)**:
   - The ADS1292R runs in SPI Mode 1 at 250 Hz, while the ADXL362 runs in SPI Mode 0 at 25 Hz.
   - Both share the CC2652R1 hardware SSI0 peripheral.
   - A mutex protects transactions and inserts a 2 microsecond guard interval during mode switches, preventing context-switch bus collisions.
2. **Cold-IAQ Estimation for Thermal Decoupling**:
   - The Bosch BME680 gas heater draws about 16 mA during gas sensing, which caused an 18 mV droop on the shared 1.8 V analog rail in early tests.
   - In bare-metal tests, this droop looked like a false heartbeat at 50.8 BPM.
   - Fixed by firing the heater once at boot to measure baseline resistance, then estimating IAQ at runtime using temperature and humidity compensation models.
3. **Lock-Free SPSC Ring Buffers (`ring_buffer.c`)**:
   - Single-producer single-consumer circular buffers with ARM Cortex-M4 Data Memory Barriers (`__DMB()`).
   - Guarantees zero dropped biopotential samples between the 250 Hz sensor acquisition task and telemetry task without blocking high-priority interrupts.

---

## File Directory

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
└── THESIS_FIRMWARE_REPORT_INTEGRATION.md    # Integration engineering report
```

---

## Quick Start

### 1. Build and Flash
```bash
python build_and_flash.py
```

### 2. Run Desktop GUI
```bash
python sensor_gui.py --port COM3 --baud 115200
```
Replace `COM3` with your LaunchPad's COM port.
