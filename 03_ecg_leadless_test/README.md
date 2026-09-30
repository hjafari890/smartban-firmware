# 03_ecg_leadless_test: ECG Leadless Electrode Verification

> Test firmware for verifying leadless dry-contact electrodes and skin impedance.  
> Target: TI CC2652R1 LaunchPad + ADS1292R Dry Electrode Interface

---

## Overview

This test firmware evaluates dry-contact electrode setups on the SmartBAN shield. For wearable patches, dry electrodes avoid skin irritation caused by wet gel pads over multiple days of wear. This code tests the signal quality and contact impedance.

### Key Points:
- Lead-off detection: Tracks electrode-to-skin contact impedance using ADS1292R internal current sources.
- Motion artifact characterization: Tests baseline drift and noise during movement.
- Signal integrity: Checks R-peak sharpness and baseline stability across dry contacts.
- Serial verification utility (`view_ecg_test.py`): Quick terminal tool to check continuity and sample validity.

---

## File Directory

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
└── syscfg/                                                  # Generated TI driver headers and sources
```

---

## Quick Start

### 1. Flash the Firmware
```bash
python build_and_flash.py
```

### 2. View Live Data
```bash
python view_ecg_test.py --port COM3
```
