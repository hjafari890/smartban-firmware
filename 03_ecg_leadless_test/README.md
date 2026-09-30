# 03_ecg_leadless_test: ECG Leadless Electrode Verification

> Standalone diagnostic firmware for verifying leadless dry-contact electrodes and electrode impedance.  
> Target: TI CC2652R1 LaunchPad + ADS1292R Dry Electrode Interface

---

## ⚡ Overview

This diagnostic firmware validates leadless dry-contact electrode topologies on the SmartBAN shield. In wearable cardiac patches, wet Ag/AgCl gel electrodes cause skin irritation over multi-day monitoring periods. This firmware verifies the front-end performance when coupled directly to dry leadless electrodes.

### Key Highlights:
- **Lead-Off Detection (LOD):** Continuous monitoring of electrode-skin contact impedance using ADS1292R internal current sources.
- **Dynamic Noise & Motion Artifact Characterization:** Benchmarking baseline drift and motion artifact susceptibility under physical movement.
- **Dry Contact Signal Integrity:** Verification of R-peak sharpness and baseline stability across high-impedance dry contacts.
- **Serial Verification Tool (`view_ecg_test.py`):** Real-time CLI verification tool capturing sample continuity and signal fidelity.

---

## 📂 File Directory

```text
03_ecg_leadless_test/
├── main.c                                                   # Leadless test sampling routine
├── ecg_leadless_test.hex                                    # Precompiled Intel HEX binary
├── ecg_leadless_test.out                                    # ELF binary executable
├── diagnostic.syscfg                                        # SysConfig pin definitions
├── build_and_flash.py                                       # Automated compilation and flashing script
├── cc13x2_cc26x2_nortos.cmd                                 # NoRTOS linker command file
├── ecg_leadless_test_CC26X2R1_LAUNCHXL_nortos_ticlang.projectspec # CCS project specification
├── view_ecg_test.py                                         # Serial CLI waveform viewer
└── syscfg/                                                  # Generated TI driver headers/sources
```

---

## 🚀 Quick Start

### 1. Flash the Leadless Test Firmware
```bash
python build_and_flash.py
```

### 2. Run the Verification Script
```bash
python view_ecg_test.py --port COM3
```
