# 02_dedicated_ecg_ads1292r: Dedicated ADS1292R ECG Firmware

> Standalone clinical-grade biopotential acquisition firmware and real-time visualization suite.  
> Target: Texas Instruments ADS1292R 24-bit Low-Power AFE + CC2652R1 LaunchPad

---

## 🫀 Overview

This firmware provides a dedicated, highly-optimized driver and processing pipeline for the **Texas Instruments ADS1292R** biopotential analog front-end. It operates in bare-metal mode with interrupt-driven 250 Hz DRDY sampling to deliver noise-free ECG waveforms with clinical fidelity.

### Key Highlights:
- **Low Noise Biopotential Acquisition:** Hardware validated noise floor below $1\,\mu\text{V}_{\text{RMS}}$ on shorted inputs; reproduces internal $1\text{ mV}$ test signal within 1% tolerance.
- **Settling & Calibration Protocol:** Implements mandatory 100 ms internal reference stabilization prior to offset calibration (`OFFSETCAL`), resolving DC saturation.
- **Respiratory Carrier Crosstalk Mitigation:** Disables the 32 kHz high-frequency impedance pneumography carrier during pure ECG modes, delivering a 21-fold reduction in baseline noise.
- **Pan-Tompkins DSP Engine:** Embedded real-time QRS detector with dual-threshold adaptive peak tracking, instant HR estimation, and HRV metric extraction.
- **Dedicated Python Visualizer (`ecg_gui.py`):** Real-time oscilloscope with digital notch filters ($50\text{ Hz} / 60\text{ Hz}$), clinical grid display, and beat marker overlay.

---

## 📂 File Directory

```text
02_dedicated_ecg_ads1292r/
├── main.c                              # Dedicated bare-metal ECG sampling superloop
├── ecg_dedicated.hex                   # Precompiled Intel HEX binary image
├── ecg_dedicated.out                   # ELF binary executable
├── diagnostic.syscfg                   # TI SysConfig pin and SPI configuration
├── build_and_flash.py                  # One-click build and flashing utility
├── ecg_gui.py                          # Real-time desktop ECG oscilloscope GUI
├── edgeai_ecg.c / edgeai_ecg.h         # Pan-Tompkins QRS and feature extraction engine
├── FIRMWARE_REPORT_ECG_DEDICATED.md    # In-depth technical verification report
├── MASTER_THESIS_ECG_DIARY.md          # Engineering log and laboratory bench notes
└── TI_RTOS_INTEGRATION_HANDOVER.md     # Architecture documentation for RTOS migration
```

---

## 🚀 Quick Start

### 1. Flash the Dedicated ECG Firmware
```bash
python build_and_flash.py
```

### 2. Launch the Desktop ECG Oscilloscope
```bash
python ecg_gui.py --port COM3
```
*(Replace `COM3` with your LaunchPad's COM port).*
