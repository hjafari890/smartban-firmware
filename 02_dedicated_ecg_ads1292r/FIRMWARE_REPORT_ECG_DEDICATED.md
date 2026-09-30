# Technical Summary: Dedicated Bare-Metal ECG & Respiration Firmware (V1.1)

**Project / project Title:** *Integration and Validation of an Intelligent Sensor Node for a Smart Body Area Network (SmartBAN) Testbed*  
**Target Platform:** Texas Instruments CC2652R1 Microcontroller (ARM Cortex-M4F) + SmartBAN Custom Sensor Shield Rev 3.5  
**Analog Front-End (AFE):** Texas Instruments ADS1292R (24-bit Low-Power Biopotential & Impedance Pneumography AFE)  
**Firmware Subdirectory:** `firmware/08_ecg_dedicated/`  
**Author / Researcher:** Hesamoddin Jafari  
**Date:** September 2026  

---

## 1. What Was Built

A specialized bare-metal firmware (`08_ecg_dedicated` V1.0 / V1.1) and companion Python desktop diagnostic station (`ecg_gui.py`) were developed to establish an isolated, gold-standard biopotential acquisition and edge-processing pipeline for the SmartBAN sensor node. Unlike previous multi-sensor firmware versions that suffered from SPI bus contention, uncalibrated PGA offsets, and incomplete AFE state machine sequencing, this firmware is **strictly dedicated to single-lead electrocardiography (ECG Lead I) and thoracic impedance pneumography (respiration)**.

The firmware operates in a deterministic bare-metal loop (NoRTOS) at an exact sampling rate of **250.0 Hz (4 ms period)**. It executes on-chip integer signal conditioning and QRS detection on the ARM Cortex-M4F core, manages ADS1292R register configurations, streams synchronized telemetry over UART at 115200 baud, and handles bi-directional runtime commands for benchtop self-testing and calibration.

---

## 2. Key Technical Achievements

1. **Resolution of ADS1292R Shadow Register & Start Pin Sequencing**:
   Discovered that asserting the hardware `START` pin (DIO 24) or leaving it floating during register programming (`WREG`) caused ongoing ADC conversion cycles to lock shadow registers, leading to silent configuration drops. Fixed this by strictly clamping `START = LOW`, executing a forced `SDATAC` (`0x11`) command, allowing $100\text{ ms}$ for internal $V_{\text{ref}}$ ($2.42\text{ V}$) capacitor charging, and performing readback verification on all 11 configuration registers before driving `START = HIGH`.
2. **Correct Physical Channel & Right-Leg Drive (RLD) Allocation**:
   Identified that prior reference implementations and firmware revisions inverted the AFE channel assignments. On the BAN Shield Rev 3.5 hardware:
   - **Channel 1 (`IN1P`/`IN1N`)** is physically wired to the $40.2\text{ k}\Omega$ series resistors and $47\text{ nF}$ filter network for Respiration (`RESP_MODP`/`RESP_MODN`). Configured with Gain = 4, $32\text{ kHz}$ internal carrier, and $112.5^\circ$ synchronous demodulation phase (`RESP1 = 0xEA`, `RESP2 = 0x03`).
   - **Channel 2 (`IN2P`/`IN2N`)** is wired to the biopotential electrode terminals (LA/RA). Configured with Gain = 6 (`CH2SET = 0x00`).
   - **RLD Loop** was routed to derive common-mode cancellation specifically from Channel 2 (`RLD_SENS = 0x2C`), eliminating $50\text{/ }60\text{ Hz}$ common-mode saturation.
3. **Clinical Integer Pan-Tompkins QRS Engine with Peak Search & Anomaly Gating**:
   Integrated a full integer-only Pan-Tompkins pipeline (`edgeai_ecg.c`, `edgeai_ecg.h`) executing at 250 Hz. Solved the "stuck at 220 BPM" runaway clamp bug by cascading a 13 Hz FIR Low-Pass filter and 5.5 Hz High-Pass filter before the 5-point derivative, and replacing level-comparison thresholding with true local-maximum peak search ($MWI[n-1] > MWI[n-2]$ and $MWI[n-1] \ge MWI[n]$) paired with a $200\text{ ms}$ refractory blanking interval.
4. **Dual-Layer Software Signal Quality Index (SQI) for Hardware Lead-Off Compensation**:
   The BAN Shield PCB includes $10\text{ M}\Omega$ pull-down bias resistors ($R_{51}, R_{52}$) on the analog input lines, causing the ADS1292R $6\text{ nA}$ DC lead-off current to produce only a $60\text{ mV}$ drop, which fails to trip the internal $2.85\text{ V}$ comparator threshold. Implemented a dual-layer strategy combining hardware status register monitoring with real-time software variance and rail-saturation checks ($< 0.0001\text{ mV}^2$ flatline detection), successfully gating false heart rate triggers.
5. **Interactive Bi-Directional Self-Test Instrumentation**:
   Engineered runtime UART command dispatching allowing benchtop switching between:
   - **Mode 1 (`1`):** Internal $1\text{ Hz}, \pm 1.0\text{ mV}$ square wave test generator.
   - **Mode 2 (`2`):** PGA inputs shorted internally for baseline noise and offset verification.
   - **Mode 3 (`3`):** Internal silicon temperature diode monitoring.
   - **Mode 4 (`4`):** Live clinical biopotential acquisition with active RLD.
   - **Calibrate (`C`):** Runtime `OFFSETCAL` trigger.

---

## 3. Software Architecture

```
firmware/08_ecg_dedicated/
├── main.c                   # Main superloop, ADS1292R SPI driver, UART telemetry & command parser
├── edgeai_ecg.h / .c        # Integer Pan-Tompkins QRS detector, FIR bandpass, RR/HRV estimator
├── diagnostic.syscfg        # SysConfig device model defining UART2_0, SPI_0, and board GPIOs
├── cc13x2_cc26x2_nortos.cmd # Linker script allocating 2 KB stack and SRAM/Flash memory sections
├── build_and_flash.py       # Automated build tool (SysConfig CLI + tiarmclang + srfprog CLI)
└── ecg_gui.py               # Python Tkinter/Matplotlib monitor, 3-stage biquad DSP, CSV logger
```

- **`main.c`**: Initializes board clocking, isolates the shared SPI bus (holding ADXL362 CS high on DIO 15), configures SSI0 in SPI Mode 1 ($250\text{ kHz}$), orchestrates the ADS1292R state machine, samples data on DRDY falling edges, feeds raw counts into `edgeai_ecg`, and streams 250 Hz data packets (`D,ts,ch1,ch2,status`) and 4-second summary packets (`S,bpm,rr,rpm,loff`).
- **`edgeai_ecg.c` / `edgeai_ecg.h`**: Modular Edge-AI biopotential processor. Executes a 13 Hz 11-tap FIR lowpass, 5.5 Hz recursive highpass, 5-point centered derivative, 64-bit squaring, 38-sample (152 ms) moving window integrator (MWI), adaptive dual-threshold peak search, 8-beat rolling RR mean, and 3-second asystole timeout.
- **`diagnostic.syscfg`**: TI SysConfig configuration binding DIO 8/9/10 to SPI, DIO 2/3 to XDS110 UART2, DIO 11/22/23/24 to ADS1292R control lines, DIO 21 to 1.8V LDO enable, and DIO 6/7 to status LEDs.
- **`cc13x2_cc26x2_nortos.cmd`**: Linker script defining flash (352 KB) and SRAM (80 KB) memory mappings, expanding stack depth to 2048 bytes for nested filter operations.
- **`build_and_flash.py`**: Python-driven build pipeline orchestrating SysConfig code generation, `tiarmclang` compilation with Cortex-M4F hardware floating point (`-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Oz`), linking, Intel HEX generation, and flashing via SmartRF Flash Programmer 2.
- **`ecg_gui.py`**: Real-time desktop GUI featuring custom biquad IIR filters (0.5 Hz HPF, 40 Hz LPF, 50/60 Hz notch), 5-second scrolling ECG strip, 30-second respiration impedance strip, live BPM/RPM badges, polarity inversion toggle, and timestamped CSV recording.

---

## 4. Challenges & Engineering Solutions

| Challenge / Symptom | Root Cause | Engineering Solution |
|:---|:---|:---|
| **ADS1292R ignored register write commands (`WREG`)** | Device was in default continuous conversion mode (`RDATAC`) with `START` pin floating/high, locking shadow registers. | Added strict pre-initialization hardware sequence: clamp `START = LOW`, issue `SDATAC` (`0x11`), write registers with readback verification, then assert `START = HIGH`. |
| **No biopotential signal on electrode attachment** | Channel mapping confusion in prior firmware versions (Channel 1 was assumed to be ECG, while Channel 2 was powered down). | Mapped PCB netlist traces: Channel 1 connects to Respiration ($40.2\text{ k}\Omega$ / $47\text{ nF}$ filter), Channel 2 connects to ECG biopotentials. Reconfigured `CH1SET = 0x40` (Gain 4) and `CH2SET = 0x00` (Gain 6). |
| **Severe 50/60 Hz powerline saturation** | Active Right-Leg Drive (RLD) was disabled or derived from the wrong channel. | Enabled internal RLD amplifier with $V_{\text{ref}}$ common-mode derived from Channel 2 inputs (`RLD_SENS = 0x2C`), driving patient ground through the $1\text{ M}\Omega / 1.5\text{ nF}$ stabilization loop. |
| **Heart rate stuck at upper clamp (220 BPM)** | Preliminary Pan-Tompkins algorithm used level-triggering on MWI output without bandpass filtering. High-frequency ambient noise kept MWI above threshold, firing a false beat every time the 200 ms refractory window expired ($15000 / 50 = 300\text{ BPM} \to 220\text{ BPM}$). | Implemented 5.5 - 13 Hz integer FIR bandpass filtering before derivative stage and enforced local maximum peak detection ($MWI[n-1] > MWI[n-2]$ and $MWI[n-1] \ge MWI[n]$). |
| **Hardware lead-off detection inoperative** | $10\text{ M}\Omega$ PCB pull-down resistors ($R_{51}, R_{52}$) on analog inputs prevented $6\text{ nA}$ DC current from reaching the $2.85\text{ V}$ comparator threshold. | Added software Signal Quality Index (SQI) monitoring signal variance ($< 0.0001\text{ mV}^2$) and rail saturation in both MCU firmware and PC GUI, cleanly reporting `0 BPM` / `--` on open leads. |

---

## 5. Quantitative & Experimental Results

- **Sampling Rate & Timing Jitter**: Exact **250.0 Hz** conversion rate verified by capturing 875 consecutive packets in $3.500\text{ s}$ with linear 4 ms timestamping ($2404, 2408, 2412\dots\text{ ms}$).
- **Measurement Linearity & Accuracy**: Benchtop verification with internal $1\text{ Hz}, \pm 1.0\text{ mV}$ square wave test generator yielded $\pm 21,250$ ADC counts:
  $$V_{\text{measured}} = 21,250 \times \frac{2.42\text{ V}}{6 \times (2^{23}-1)} = 1.021\text{ mV} \quad (V_{\text{pp}} = 2.018\text{ mV}, \text{Error} < 1.0\%)$$
- **Noise Floor & Dynamic Range**: Shorted-input mode (Mode 2) demonstrated a baseline offset of $\sim 290$ counts ($13.9\text{ }\mu\text{V}$) and a peak-to-peak noise level of $\pm 20$ counts (**$< 1.0\text{ }\mu\text{V}_{\text{RMS}}$**), confirming high analog signal-to-noise ratio.
- **QRS Detector Timing**: 1 Hz square wave transition interval detected at **512 ms**, yielding **117 BPM** ($120\text{ BPM}$ theoretical for 2 edges/s).
- **Memory Footprint**:
  - Code Size (Flash): **65,961 bytes** Intel HEX format (~28.5 KB compiled machine code).
  - SRAM Consumption: **~3.2 KB total** (statically allocated `edgeai_ecg_state_t` structure = 530 bytes, stack depth = 2048 bytes, zero dynamic `malloc` allocations).
- **Telemetry Bandwidth**: 115200 baud UART streaming 250 samples/s $\times$ 32 bytes/packet $\approx 8.0\text{ KB/s}$ ($69.4\%$ line utilization).

---

## 6. What This Version Enabled for the Next Milestone

1. **Definitive Ground Truth for AFE Hardware**: Proved that the ADS1292R silicon, power rails, SPI communications, and analog input paths on the SmartBAN Shield Rev 3.5 are fully functional and free of hardware defects.
2. **Validated Edge-AI DSP Primitives**: Created an efficient, zero-malloc integer Pan-Tompkins QRS and feature extraction engine (`edgeai_ecg`) that is ready to be ported directly into the multi-threaded RTOS environment.
3. **Foundation for TI-RTOS7 Multi-Sensor Node**: Established the exact register profile, timing delays, and SPI transaction contracts required to integrate ECG and Respiration alongside the ADXL362 accelerometer in `06_tirtos_smartban` and `09_tirtos_all_in_one`.
4. **Benchtop Validation Infrastructure**: The Python GUI and CSV data logging station provide an immediate validation harness for future wireless (BLE / SmartBAN PHY) transmission trials.

---

## 7. Personal Reflection & Research Insights

Through this phase of the research, I developed a deep appreciation for the critical importance of rigorous hardware netlist tracing and analog front-end state machine timing. I learned that biopotential acquisition failures often stem not from silicon defects, but from subtle race conditions during startup sequences, such as shadow register locking and missing filter settling margins. Furthermore, debugging the QRS detector reinforced that embedded biomedical algorithms must always couple frequency-selective filtering with strict geometric peak detection to remain resilient against real-world powerline interference.
