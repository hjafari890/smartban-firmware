# Milestones Archive: Chronological Firmware Evolution

This directory contains the chronological progression of firmware versions developed during the project. It is preserved for reference, regression testing, and completeness.

---

## Directory Index

| Folder | Stage / Milestone | Description |
| :--- | :--- | :--- |
| **`01_hardware_diagnostic`** | v01 Hardware Bring-Up | Initial pinout diagnostic, GPIO toggle, clock frequency check, and pin swapping remediation (`IOCPortConfigureSet`). |
| **`02_ppg_diagnostic`** | v02 PPG Bring-Up | Diagnostic tests for the MAX32664C biometric sensor hub. |
| **`03_ppg_led_bringup`** | v03 Optical Driver | Optical LED driver testing and hardware logic-level errata documentation. |
| **`05_ecg_respiration_real`** | v05 Respiration Pneumography | ADS1292R impedance pneumography testing with respiratory carrier modulation. |
| **`06_tirtos_smartban`** | v06 Initial RTOS Bring-up | First migration of bare-metal superloop into TI-RTOS kernel tasks. |
| **`09_tirtos_checkpoint_backup4`** | v09 Architecture Checkpoint | Stable integration checkpoint prior to edge AI feature insertion. |
| **`10_smartban_semantic_tinyml`** | v10 On-Chip TinyML | Pan-Tompkins feature extraction and 8-bit quantized integer neural network integration. |
| **`11_smartban_ble_android`** | v11 BLE Multi-Frame | Multi-frame BLE advertising packetization for the Android client. |
| **`12_smartban_live_ble_streaming`** | v12 Live BLE Streaming | 250 Hz live microvolt batch streaming over BLE radio without CPU stalling. |
| **`protocentral-ads1292r-arduino`** | Reference Driver | Upstream ProtoCentral Arduino reference library used for register validation. |
| **`test_ecg`** | Bench Testing | Host-side synthetic and recorded ECG playback test tools. |
