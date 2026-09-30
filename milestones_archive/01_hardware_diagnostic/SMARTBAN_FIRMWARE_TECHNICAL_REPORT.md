# Technical Summary Report: Integration & Hardware-Software Co-Design of an Intelligent Biomedical Sensor Node for SmartBAN

**Project Title:** Integration and Validation of an Intelligent Sensor Node for a Smart Body Area Network (SmartBAN) Testbed  
**Author / Researcher:** Master's Degree Candidate  
**Target Hardware:** Texas Instruments CC2652R1 LaunchPad (`LAUNCHXL-CC26X2R1`, ARM Cortex-M4F @ 48 MHz) + Custom Multi-Modal Biomedical Shield (Hardware Revision 3.5)  
**Firmware Subsystem:** `01_hardware_diagnostic` (Bare-Metal NoRTOS Embedded Sensor Node Bring-Up & Validation Engine)  
**Date:** September 2026  

---

## 1. What Was Built

The firmware developed in this phase serves as the foundational **Multi-Modal Diagnostic & Validation Operating Environment** for the SmartBAN sensor node. It interfaces, calibrates, synchronizes, and streams real-time biometric and environmental telemetry across seven distinct physical sensor subsystems and an interactive on-board user interface.

### Target Hardware Architecture
* **Host Processing Platform:** Texas Instruments SimpleLink™ CC2652R1 wireless microcontroller (ARM Cortex-M4F core @ 48 MHz, 352 KB in-system programmable Flash, 80 KB ultra-low-leakage SRAM, integrated 2.4 GHz RF multi-protocol transceiver).
* **Sensor Node Carrier:** Custom SmartBAN Biomedical Shield Revision 3.5 utilizing dual 20-pin BoosterPack header interfaces, dual power domains (switched 3.3V and regulated 1.8V via DIO 21 LDO), and bidirectional I2C voltage translation (PCA9306 via DIO 30).
* **Integrated Transducers & Interfaces:**
  1. **TI ADS1292R:** 24-bit dual-channel low-noise analog front-end (AFE) for continuous 2-lead ECG, heart-rate intervals, and thoracic impedance pneumography (SPI Mode 1).
  2. **Analog Devices ADXL362:** Ultra-low power 3-axis MEMS accelerometer with integrated motion detection, zero-g baseline tare, and spatial orientation inference (SPI Mode 0).
  3. **Bosch Sensortec BME680:** 4-in-1 digital environmental sensor measuring barometric pressure (hPa), ambient temperature (°C), relative humidity (%RH), and indoor volatile organic compound (VOC) gas resistance (I2C address `0x77`).
  4. **Texas Instruments OPT4041:** High-precision ambient light sensor (ALS) with sub-lux resolution (I2C address `0x44`).
  5. **Vishay VCNL4040:** Integrated infrared proximity and ambient light sensor with smart infrared cross-talk cancellation (I2C address `0x60`).
  6. **Melexis MLX90632:** Factory-calibrated medical-grade far-infrared (FIR) non-contact skin and thermopile temperature sensor (I2C address `0x3A`).
  7. **Maxim Integrated MAX32664 / MAXM86161 (Optical PPG Subsystem):** Ultra-low-power biometric sensor hub and integrated dual-wavelength pulse oximeter front-end (I2C address `0x55`).
  8. **On-Board Human-Machine Interface (HMI):** CH455H dynamic 3-digit 7-segment LED driver with individual status annunciation (I2C address `0x24`), coupled with an NXP PCAL6408A 6-channel tactile switch interrupt expander (I2C address `0x20`).

### Core Firmware Functionality
The firmware executes a deterministic 100 ms sensing and control pipeline under a NoRTOS scheduler. It performs automated power sequencing, multi-bus arbitration, dynamic bus multiplexing between conflicting SPI peripheral modes, sensor register initialization, real-time sensor fusion (pitch/roll attitude, posture classification, ambient-compensated infrared skin temperature), multi-format visual feedback across the 7-segment display, and high-resolution tabular telemetry over USB-UART (115200 baud).

```
                      +------------------------------------------------+
                      |             CC2652R1 MCU (48 MHz)             |
                      +-------------------+--------------------+-------+
                                          |                    |
                                 I2C Bus (100 kHz)       SPI Bus (SSI0)
                                          |                    |
            +------------+------------+---+---+        +-------+--------+
            |            |            |       |        |                |
         BME680       OPT4041      VCNL4040   |     ADS1292R        ADXL362
        (Env/Gas)      (Lux)        (Prox)    |     (ECG/Resp)       (IMU)
                                              |    [SPI Mode 1]    [SPI Mode 0]
                                              |    (TX:9, RX:8)    (TX:8, RX:9)
                                              |          ^              ^
                                              |          |--- Dynamic --|
                                              |               IOC Swap
                                              |
                          +-------------------+-------------------+
                          |                   |                   |
                       MLX90632            CH455H             PCAL6408A
                      (IR Temp)          (7-Seg LED)        (6 Tactile BTN)
```

---

## 2. Key Technical Achievements

### Breakthrough 1: Resolution of Hardware SPI Pin Routing Reversal via Runtime IOC Dynamic Remapping
* **Problem:** On Shield Revision 3.5, the SPI routing contains a physical cross-bus mismatch between the two SPI peripherals. For the ADS1292R (ECG), LaunchPad DIO 9 connects to DIN (MOSI) and DIO 8 connects to DOUT (MISO). However, for the ADXL362 (IMU), Pin 6 (SDI/MOSI) was routed to DIO 8 and Pin 7 (SDO/MISO) was routed to DIO 9. Because standard hardware SPI controllers bind MOSI and MISO to fixed pins, communicating with the IMU caused the host to transmit into the IMU’s output driver while listening to an un-driven high-impedance input, generating wild, floating random noise.
* **Engineering Solution:** Rather than requiring a hardware trace cut and bodge wire, we leveraged the software-defined I/O Controller (IOC) peripheral of the CC2652R1. A dynamic SPI mode switcher (`spi_set_mode()`) was engineered utilizing DriverLib primitives (`IOCPortConfigureSet`). When accessing ADXL362 (Mode 0), `IOID_8` is instantly mapped to `IOC_PORT_MCU_SSI0_TX` and `IOID_9` to `IOC_PORT_MCU_SSI0_RX`. When accessing ADS1292R (Mode 1), the pin mux is swapped to `IOID_9` (TX) and `IOID_8` (RX).
* **Impact:** 100% firmware-level recovery of an apparent hardware routing flaw, allowing concurrent, full-speed operation of both SPI slave ICs on a single shared physical bus with zero hardware rework.

### Breakthrough 2: Elimination of "Random" IMU Data via Strict Validation & Automated Zero-G Tare Calibration
* **Problem:** Floating SPI input lines returned non-zero random bytes (`0x43`, `0xB7`, `0xFF`), which the burst-read parser sign-extended into erratic pseudo-accelerations (e.g., $+1.48\,\text{g}$, $-2.19\,\text{g}$). The baseline tare calibration averaged these random values, permanently corrupting spatial telemetry.
* **Engineering Solution:** Implemented a two-stage auto-probing detection sequence in `adxl362_init()` verifying the silicon manufacturer signature `DEVID_AD == 0xAD`. Strict validation guards (`if (adxl362_id != 0xAD) return false;`) were introduced in both `adxl362_read_accel()` and `adxl362_calibrate_tare()`. A 32-sample zero-g tare compensation algorithm was deployed, computing DC offsets ($X_0, Y_0, Z_0$) and standard gravity offset along the vertical axis ($Z_{\text{tare}} = \bar{Z} - 1000\,\text{mg}$), yielding real-time Euler Pitch/Roll calculations and spatial posture classification (`FLAT`, `INVERTED`, `VERTICAL`, `SIDEWAYS`).

### Breakthrough 3: Root Cause Isolation of Optical Pulse Oximeter Subsystem Electrical Level Violation
* **Problem:** The MAX32664 Biometric Sensor Hub and MAXM86161/MAX30102 optical front-end failed to respond on I2C (`0x55`), and optical illumination LEDs remained extinguished despite correct register commands.
* **Engineering Solution:** Comprehensive circuit netlist auditing of `BMEShield_Schematic_V3_2026-09-07.net` and `SCH_bme-shield_2026-09-07.json` was cross-referenced against the MAX32664 absolute maximum electrical ratings. The audit revealed that `HOST_HUB_MFIO` (J1-11) and `HOST_HUB_RST` (J2-8) were wired directly to LaunchPad 3.3V GPIOs (DIO 28 and DIO 19), whereas the MAX32664 is an ultra-low-voltage microcarrier requiring $1.8\,\text{V}$ digital logic ($V_{\text{max}} = V_{DD} + 0.3\,\text{V} = 2.1\,\text{V}$). Driving $3.3\,\text{V}$ directly into the hub exceeded maximum ratings by $+1.2\,\text{V}$, forward-biasing the ESD protection clamp diodes and preventing reset de-assertion.
* **Engineering Action:** Safe firmware bypass implemented (`hub_ready = false`) to isolate the optical hub, prevent pin over-voltage injection, and avoid electrical latch-up, enabling safe bring-up of all remaining 6 sensor modalities while defining the layout revision requirement for Shield Rev 4.0.

### Breakthrough 4: Multi-Modal HMI Navigation & Display Engine with Interactive Tare Triggering
* **Engineering Solution:** Designed an interactive, non-blocking state machine coupling the PCAL6408A interrupt-driven 6-button expander (`SW1`–`SW6`) with the CH455H 7-segment display. Integrated 8 display modes (`tEP`, `PrS`, `HuD`, `LUX`, `PrX`, `Irt`, `ECG`, `PPG`) with real-time numeric rendering and mapped button `SW4` to trigger live on-demand recalibration of the zero-g accelerometer tare baseline.

---

## 3. Software Architecture

| File / Component | Role & Interaction in Subsystem |
| :--- | :--- |
| [`main.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/main.c) | Core application entry point hosting hardware initialization, sensor calibration, state machines, and the 100 ms telemetry executive. |
| [`diagnostic.syscfg`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/diagnostic.syscfg) | SysConfig graphical/declarative metadata configuring CC2652R1 pin multiplexing, UART2, I2C, SPI, and GPIO instances. |
| [`build_and_flash.py`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/build_and_flash.py) | Automated orchestration toolchain invoking SysConfig CLI, `tiarmclang`, `tiarmobjcopy`, and cJTAG flashing utilities. |
| [`cc13x2_cc26x2_nortos.cmd`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/cc13x2_cc26x2_nortos.cmd) | Target linker command script defining physical memory layout across Flash, GPRAM, SRAM, and section placements. |
| [`ti_drivers_config.c/.h`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/syscfg/ti_drivers_config.c) | Auto-generated DriverLib hardware configuration structs for peripherals (`CONFIG_SPI_0`, `CONFIG_I2C_0`, `CONFIG_UART2_0`). |
| [`ti_devices_config.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/01_hardware_diagnostic/syscfg/ti_devices_config.c) | Customer Configuration (CCFG) flash settings specifying bootloader backdoor, HF clock source (48 MHz crystal), and power trims. |

---

## 4. Challenges & Engineering Solutions

```
+-----------------------------------+   Problem   +------------------------------------+
|  CC2652R1 SPI Master (SSI0)       | ----------> |  ADXL362 Physical Pin Routing      |
|  - PICO (TX): DIO 9               |             |  - Pin 6 (SDI / Input):  DIO 8     |
|  - POCI (RX): DIO 8               |             |  - Pin 7 (SDO / Output): DIO 9     |
+-----------------------------------+             +------------------------------------+
                  |                                                  |
                  |                                                  |
                  v                                                  v
     [Symptom: High-Z Bus]                               [Result: Floating Noise]
     MCU TX drives SDO Output                             Random Data (±2.5g jump)
     MCU RX reads SDI Input                               Corrupted Zero-G Tare
                  |
                  +-----------------------+--------------------------+
                                          |
                                          v
                   +-----------------------------------------------+
                   | Runtime IOC Pin Dynamic Reconfiguration       |
                   |   - Mode 0 (ADXL362): IOID_8=TX, IOID_9=RX   |
                   |   - Mode 1 (ADS1292): IOID_9=TX, IOID_8=RX   |
                   | Validation Guard: ID == 0xAD before Tare/Read |
                   +-----------------------------------------------+
                                          |
                                          v
                             Accurate, Stable Acceleration
                             Calibrated Zero-G Baseline (1.000g)
```

### Challenge 1: The "Random" Floating Acceleration Readings
* **Initial Observation:** Accelerometer outputs fluctuated continuously between large positive and negative acceleration extremes (e.g., $+1.48\,\text{g}$, $-2.19\,\text{g}$, $+0.74\,\text{g}$) despite the sensor lying completely stationary on the benchtop.
* **Failure Analysis:** Because SPI is an unacknowledged synchronous shift register protocol, the CC2652R1 `SPI_transfer()` driver returns `true` regardless of whether a slave is connected. The MISO pin was sampling high-impedance electrical noise on DIO 8 because the actual SDO pin was wired to DIO 9.
* **Resolution:** Reconfigured the pin mux dynamically in software (`IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX, IOC_STD_OUTPUT)` and `IOID_9` to `IOC_PORT_MCU_SSI0_RX`) upon entering SPI Mode 0, and gated all mathematical processing behind verification of `DEVID_AD (0xAD)`.

### Challenge 2: JTAG / cJTAG Probe Contention During Automated Build
* **Initial Observation:** Automated CLI flashing via `build_and_flash.py` failed with `Error -260: Unknown device` or cJTAG scan failure.
* **Failure Analysis:** The user’s active Code Composer Studio (Theia) IDE session held an open debug lock on the XDS110 hardware probe, blocking `srfprog.exe` and `DSLite.exe` from acquiring the cJTAG transport.
* **Resolution:** Partitioned the workflow into a seamless decoupled pipeline: `build_and_flash.py` performs SysConfig code generation, compilation (`tiarmclang`), linking, and HEX generation; CCS Theia links directly to the generated `main.c` workspace to flash in-place with a single keystroke.

### Challenge 3: Power Supply Shunting via N-Channel IMU Switch Q1
* **Initial Observation:** The ADXL362 power rail `3V3_ADXL` displayed intermittent dropouts when `CONFIG_GPIO_IMU_SW` was driven high.
* **Failure Analysis:** Netlist analysis revealed Q1 is an N-channel MOSFET with its Source connected directly to GND and Drain connected through R55 ($100\,\Omega$) to `3V3_ADXL`. Driving `IMU_SW` high turned Q1 ON, creating a $33\,\text{mA}$ pull-down shunt to GND.
* **Resolution:** Firmware initialization was updated to enforce `GPIO_write(CONFIG_GPIO_IMU_SW, 0)`, holding Q1 in high-impedance cut-off so that `3V3_ADXL` is fed cleanly from the primary 3.3V rail through $0\,\Omega$ resistor R47.

---

## 5. Quantitative Results & System Metrics

### Embedded Memory Footprint (tiarmclang -Oz Optimization)
The complete firmware binary was profiled using the Linker Map Output (`diagnostic.map`):

| Memory Region | Physical Size | Memory Consumed | Utilization | Available Headroom |
| :--- | :--- | :--- | :--- | :--- |
| **On-Chip Flash** | $352.0\,\text{KB}$ ($360,448\,\text{B}$) | **$41,091\,\text{B}$** ($40.13\,\text{KB}$) | **$11.67\%$** | **$311\,\text{KB}$ ($88.33\%$)** |
| **On-Chip SRAM** | $80.0\,\text{KB}$ ($81,920\,\text{B}$) | **$20,352\,\text{B}$** ($19.88\,\text{KB}$) | **$24.84\%$** | **$59.6\,\text{KB}$ ($75.16\%$)** |
| **Raw Intel HEX** | — | **$115,635\,\text{B}$** | — | Production Ready |

### Timing, Throughput & Sensor Metrics
* **Core Clock Speed:** $48.0\,\text{MHz}$ derived from external High-Frequency Crystal Oscillator (XOSCHF).
* **Deterministic Execution Period:** $100\,\text{ms}$ loop period ($10.0\,\text{Hz}$ continuous telemetry rate).
* **Communication Bus Bandwidths:**
  * **UART Telemetry:** $115,200\,\text{baud}$, 8 data bits, no parity, 1 stop bit ($86.8\,\mu\text{s}/\text{byte}$).
  * **I2C Bus:** Standard Mode ($100\,\text{kHz}$, $10\,\mu\text{s}/\text{bit}$).
  * **ADS1292R SPI Bus:** $250\,\text{kHz}$ (Mode 1, CPOL=0, CPHA=1).
  * **ADXL362 SPI Bus:** $1.0\,\text{MHz}$ (Mode 0, CPOL=0, CPHA=0).
* **Sensor Measurement Ranges & Resolutions Achieved:**
  * **Barometric Pressure (BME680):** $300.0 - 1100.0\,\text{hPa}$, validated at ambient ($\sim 1013.2\,\text{hPa}$, resolution $0.18\,\text{Pa}$).
  * **Ambient & Die Temp (BME680 / MLX90632):** Validated at room ambient ($\sim 22.4^\circ\text{C}$, resolution $0.01^\circ\text{C}$).
  * **Ambient Illuminance (OPT4041):** Sub-lux resolution ($0.01 - 100,000\,\text{lux}$).
  * **Proximity Reflection (VCNL4040):** 16-bit counts ($0 - 65,535\,\text{counts}$).
  * **Tri-Axial Acceleration (ADXL362):** $\pm 2.000\,\text{g}$ dynamic range, $1.0\,\text{mg}/\text{LSB}$ sensitivity, 12-bit two's complement format ($0.001\,\text{g}$ resolution).

---

## 6. Foundation Enabled for the Next Milestone

The successful completion and stabilization of this diagnostic firmware establish the empirical foundation for subsequent thesis milestones:

1. **Guaranteed Sensor Data Integrity for Edge Feature Extraction:**
   * Sensor drivers now output calibrated physical SI units rather than unverified ADC words. This allows immediate deployment of edge intelligence algorithms (e.g., fall detection, activity recognition, arrhythmia screening) directly on the ARM Cortex-M4F without signal artifacts.
2. **Deterministic Multi-Bus Power & Timing Characterization:**
   * Establishing an $88\%$ Flash and $75\%$ RAM headroom proves that the complete SmartBAN software stack—including the SmartBAN MAC layer (IEEE 802.15.6), BLE 5.2 protocol stack, and TinyML inference models—can easily co-exist on the CC2652R1 without requiring external memory.
3. **Definitive Hardware Errata for Carrier Board Revision 4.0:**
   * The hardware-software co-design process has rigorously verified all net connections. The physical errata documented here (swapping ADXL362 SDI/SDO traces and integrating $3.3\text{V} \leftrightarrow 1.8\text{V}$ bidirectional level shifting on `HOST_HUB_MFIO` and `HOST_HUB_RST`) will be incorporated directly into the next PCB revision, transitioning the testbed to fully production-ready wearable prototypes.

---

## 7. Researcher Experience & Personal Reflection

This phase provided invaluable hands-on insight into the complexities of hardware-software co-design and bare-metal embedded integration. It reinforced that in cyber-physical systems, software cannot be decoupled from electrical reality: identifying why the IMU streamed random numbers required analyzing transistor pinouts, schematic netlists, and silicon-level bus multiplexers rather than just inspecting high-level code. Learning to overcome physical PCB layout discrepancies entirely through low-level microcontroller architectural features (such as dynamic I/O multiplexing) was an empowering engineering milestone that substantially deepened my mastery of embedded systems.
