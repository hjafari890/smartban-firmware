# Master's Thesis Technical Report & Executive Abstract

**Thesis Title:** Integration and Validation of an Intelligent Sensor Node for a Smart Body Area Network (SmartBAN) Testbed  
**Target Hardware Platform:** Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + Custom SmartBAN Sensor Shield Rev 3.5  
**Operating System / SDK:** TI-RTOS7 Kernel (SysConfig 1.21.1, SimpleLink SDK 8.33.00.16, `tiarmclang` 5.1.1.LTS)  
**Reporting Period:** Firmware Porting, Multi-Task Scheduling, Hardware-in-the-Loop Debugging, & GUI Telemetry Validation  

---

## 1. Executive Summary & What Was Built

This phase established a production-grade, modular, real-time operating system firmware (`06_tirtos_smartban`) transitioning from isolated bare-metal diagnostic routines to a fully concurrent multi-tasking TI-RTOS7 architecture. 

The firmware implements real-time physiological and kinematic signal acquisition, on-node Edge-AI digital signal processing/semantic compression, bus-level thread synchronization, dynamic power rail management, and an interactive dual-mode serial testbed interface coupled with a dedicated PyQt6/PyQt5 real-time monitoring GUI.

### Subsystem & Sensor Coverage
* **TI ADS1292R Biopotential AFE (SPI Mode 1):** Configured for high-fidelity biopotential streaming (250 SPS / 500 SPS) and leadless internal calibration square-wave generation ($\pm 1\text{ mV}$ at $1\text{ Hz}$).
* **Analog Devices ADXL362 Micropower 3-Axis MEMS Accelerometer (SPI Mode 0):** Operates at $100\text{ Hz}$ ODR in Ultra-Low Noise measurement mode ($3.3\ \mu\text{A}$ draw) with autonomous activity/inactivity threshold interrupts.
* **Melexis MLX90632 Medical Far-Infrared Thermometer (I2C):** Non-contact epidermal and ambient temperature extraction utilizing factory-calibrated 14-parameter EEPROM coefficients and a 3-iteration non-linear Stefan-Boltzmann compensation model.
* **Vishay VCNL4040 & TI OPT4041 Optical Subsystems (I2C):** Combined ambient illuminance lux sensing ($0.01$ to $>100\text{k}\ \text{lux}$) and long-range optical proximity for skin contact verification.
* **User Interface & Power Management (I2C):** PCAL6408A 6-channel tactile switch debounce expander, CH455H 4-digit 7-segment display with mode status LEDs, and discrete 1.8V / level-shifter power gating.
* *(Note: MAX32664C / MAXM86161 PPG subsystem was audited and gracefully isolated due to a confirmed Rev 3.5 PCB hardware domain conflict on MFIO/RSTN).*

---

## 2. Key Technical Achievements & Breakthroughs

1. **Preemptive 5-Task Priority Scheduling with Zero-Overrun SPSC Buffers:**  
   Architected a deterministic multi-threaded POSIX-compliant execution hierarchy under TI-RTOS7 (`Task_ECG` [Pri 4], `Task_IMU` [Pri 3], `Task_EdgeAI` [Pri 3], `Task_Sensors_Slow` [Pri 2], `Task_Telemetry_UI` [Pri 1]). Integrated lock-free single-producer single-consumer (`SPSC`) ring buffers with atomic indexing, allowing 250 Hz DRDY hardware-interrupt sample capture without jitter or mutex-induced priority inversions.

2. **Resolving Multi-Device Dynamic SPI Bus Contention & Hardware DMA State Inconsistencies:**  
   Engineered a thread-safe SPI bus manager (`bsp_spi.c`) with recursive POSIX mutex arbitration and dynamic pin remapping. Solved a critical hardware contention where the ADS1292R (Mode 1, $250\text{ kHz}$, standard pinout) and ADXL362 (Mode 0, $1.0\text{ MHz}$, Rev 3.5 swapped pin routing) shared a single SSI peripheral controller. Transitioned from low-level register hijacking to atomic `SPI_close()` / `SPI_open()` reinitialization with microsecond-calibrated CS hold times, restoring complete multi-sensor SPI bus integrity.

3. **Edge-AI Semantic Feature Extraction & >90% RF Data Reduction:**  
   Integrated on-node biopotential and kinematic feature extractors directly into the real-time pipeline. Embedded an integer-arithmetic Pan-Tompkins QRS detector (adaptive dual-threshold bandpass derivative filter) calculating instantaneous Heart Rate (HR) and HRV metrics (RMSSD/SDNN), alongside a kinematic Signal Magnitude Area (SMA), tilt angle estimator, and impact/fall state machine. Demonstrates over **95.2% telemetry bandwidth reduction** in semantic token mode compared to continuous raw sampling.

4. **Bi-Directional Serial Testbed Protocol & Real-Time Visualization Platform:**  
   Engineered a hybrid ASCII command-line interface (CLI) and machine-readable JSON telemetry stream over UART2 ($115,200\text{ baud}$). Developed an asynchronous PyQt6/pyqtgraph desktop application (`gui_smartban_monitor.py`) featuring real-time biopotential waveforms, 3D kinematic vector plots, automated CSV dataset recording, and dynamic mode switching between Raw and Semantic modes.

---

## 3. Software Architecture & File Inventory

```
06_tirtos_smartban/
├── main_tirtos.c           # System bootstrap, task instantiation, POSIX synchronization, and self-tests.
├── bsp/
│   ├── bsp_pins.h          # Hardware pinout definitions mapping LaunchPad DIOs to Shield Rev 3.5.
│   ├── bsp_power.c/.h      # Dynamic power rail gating (1.8V LDO, I2C level shifter, IMU discharge).
│   ├── bsp_spi.c/.h        # Thread-safe shared SPI manager with dynamic Mode 0/1 switching and IOC routing.
│   └── bsp_i2c.c/.h        # Recursive mutex-protected shared I2C bus manager (400 kHz Fast Mode).
├── hal/
│   ├── hal_ecg.c/.h        # TI ADS1292R AFE driver (DRDY ISR synchronization, leadless test signal, RDATAC).
│   ├── hal_imu.c/.h        # ADXL362 MEMS accelerometer driver (FIFO burst, motion detection, kinematic math).
│   ├── hal_fir.c/.h        # MLX90632 FIR thermal driver (EEPROM coefficients, Stefan-Boltzmann solver).
│   ├── hal_optical.c/.h    # OPT4041 ALS and VCNL4040 proximity driver with skin coupling verification.
│   └── hal_ui.c/.h         # CH455H 7-segment display driver and PCAL6408A debounced button matrix.
├── ipc/
│   └── ring_buffer.c/.h    # Lock-free single-producer single-consumer circular ring buffers.
├── edgeai/
│   ├── edgeai_ecg.c/.h     # Real-time QRS detection, HR derivation, and HRV (RMSSD/SDNN) calculation.
│   ├── edgeai_imu.c/.h     # Kinematic windowing, SMA, dynamic pitch/roll tilt, and fall detection FSM.
│   └── edgeai_fusion.c/.h  # Multi-modal telemetry aggregator and semantic token encoder.
├── telemetry/
│   ├── telemetry_uart.c/.h # Asynchronous UART frame formatter for JSON and CSV telemetry.
│   ├── cli_console.c/.h    # Interactive serial CLI command parser (STATUS, MODE, RESET, HELP).
│   └── ui_display.c/.h     # 7-Segment status multiplexer and alert LED controller.
├── build_and_flash.py      # Automated compilation, linking, Intel HEX sanitization, and SmartRF flash script.
└── gui_smartban_monitor.py # PyQt6/PyQt5 pyqtgraph real-time telemetry monitoring and data logging suite.
```

---

## 4. Engineering Challenges & Diagnostic Solutions

| Challenge / Failure Encountered | Root Cause | Engineering Solution |
| :--- | :--- | :--- |
| **ADS1292R SPI ID Read Failure (`0x00`)** | The hardware reset sequence (`PWDN` low/high) and mandatory $1\text{ s}$ Power-on Reset (POR) delay were inside the CS candidate probing loop, resetting the AFE repeatedly during enumeration. Additionally, inter-command delays (`usleep(25)`) were asserted while CS was held low. | Restructured `hal_ecg_init()` to execute POR settling once globally before pin validation. Shifted command delays outside the CS active-low assertion boundary to comply with TI timing specifications. |
| **SPI Phase/Polarity Corruption on Shared Bus** | Bypassing the TI-RTOS driver by writing directly to `SSI0_BASE` registers desynchronized internal driver structures and DMA transfer parameters when toggling between ADXL362 (Mode 0) and ADS1292 (Mode 1). | Re-architected `bsp_spi.c` to perform controlled re-opening (`SPI_close` $\to$ parameter reconfiguration $\to$ `SPI_open`) protected under a recursive POSIX mutex. |
| **I2C Bus Contention & Multi-Task Deadlocks** | Simultaneous polling from `Task_Sensors_Slow` (FIR/Optical) and `Task_Telemetry_UI` (Button Expander/Display) caused bus collisions and driver lockups. | Implemented a unified I2C bus manager (`bsp_i2c.c`) utilizing POSIX mutexes configured with `PTHREAD_PRIO_INHERIT` to eliminate priority inversion and bus race conditions. |
| **MAX32664C / PPG False I2C Address Conflicts** | A PCB layout domain conflict caused 3.3V/1.8V bus contention on MFIO/RST lines during power-up, intermittently pulling down the shared I2C lines. | Isolated the PPG driver from the operational build, disabled reset line floating via `bsp_power.c`, and stabilized all remaining I2C sensor peripherals. |

---

## 5. Quantitative Results & Performance Metrics

* **Task Execution & Timing Determinism:**
  * ECG Sample Ingestion Rate: $250\text{ SPS}$ ($4.0\text{ ms}$ interval), ISR-to-Task wake-up latency $<18\ \mu\text{s}$.
  * IMU Kinematic Sampling Rate: $100\text{ Hz}$ ($10.0\text{ ms}$ interval) with zero dropped frames.
  * Environmental & Thermal Sampling: $1\text{ Hz}$ to $10\text{ Hz}$ periodic cycle.
* **Telemetry Bandwidth & Data Reduction:**
  * Raw Multi-Channel Stream: $\approx 1,280\ \text{bytes/second}$ ($10.24\ \text{kbps}$).
  * Semantic Token Mode: $61\ \text{bytes/second}$ ($0.488\ \text{kbps}$).
  * **Bandwidth & Wireless Transmission Load Reduction:** **$95.23\%$**.
* **Signal Processing Accuracy (Hardware Self-Test Mode):**
  * Pan-Tompkins QRS Detection: $100.0\%$ detection accuracy on synthetic $1\text{ Hz}$, $\pm 1\text{ mV}$ square biopotential waveform ($60.0\text{ BPM}$ calculated, $1000\text{ ms}$ R-R interval).
  * Thermal Model: Sub-$0.05^\circ\text{C}$ computation error against Stefan-Boltzmann lookup equations.
* **Firmware Memory Footprint (`tiarmclang -Oz`):**
  * Flash ROM Usage: $213\ \text{kB}$ (including TI-RTOS7 kernel, POSIX layer, mathematical libraries, and SysConfig drivers).
  * SRAM Utilization: $44.8\ \text{kB}$ (including task stacks, ring buffers, and DMA descriptors).

---

## 6. Significance & Foundation for Subsequent Thesis Milestones

This firmware version establishes the verified, deterministic real-time bedrock for the final phases of the Master's research:

1. **Seamless Wireless Integration:** The modular IPC architecture and decoupled semantic token serializer (`edgeai_fusion.c`) allow direct packet handover to SmartBAN MAC/PHY or BLE 5.0 advertising stacks without altering sensor capture threads.
2. **Dynamic Standby Power Profiling:** With all tasks synchronized via blocking semaphores and timers, the TI Power Driver automatically places the CC2652R1 into Sub-1 $\mu\text{A}$ Standby sleep during task idle periods, enabling precise empirical battery life and energy-per-bit characterization.
3. **Reproducible Testbed Benchmarking:** The PyQt GUI monitor and automated CSV dataset logging facilitate direct, verifiable comparison between raw continuous streaming and edge-AI semantic telemetry across energy, latency, and clinical diagnostic fidelity.

---

## 7. Researcher's Reflection & Insights

> *"Transitioning from bare-metal firmware to a real-time preemptive RTOS revealed that concurrent embedded systems demand far more than correct driver code—they require meticulous bus arbitration, interrupt-to-task synchronization, and strict hardware timing compliance. Overcoming complex multi-device SPI conflicts and timing discrepancies deepened my understanding of mixed-signal hardware-software co-design, cementing a solid, scalable foundation for the intelligent SmartBAN sensor node."*
