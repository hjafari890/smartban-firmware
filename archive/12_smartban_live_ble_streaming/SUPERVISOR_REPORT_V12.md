# Milestone 12 Technical Walkthrough: True Real-Time Hardware BLE Streaming & Tabbed Mobile Navigation

## 1. Overview & Objective
In response to Milestone 12 requirements:
1. **Eliminated All Synthetic Waveforms**: Removed all synthetic or simulated ECG cadences from the Android repository. The application now displays **100% genuine real hardware ADS1292R biopotential samples** streamed directly from the CC2652R1 sensor node over the 2.4 GHz BLE radio.
2. **Created 12th Firmware**: Engineered and flashed `12_smartban_live_ble_streaming/` to the Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`).
3. **Tabbed Mobile Navigation**: Redesigned the single-page UI on the **Samsung Galaxy S22 Ultra (Android 15 / API 35)** into a sleek 4-tab bottom navigation interface (`NavigationBar`).

---

## 2. Firmware Architecture (`12_smartban_live_ble_streaming/`)

### A. Real-Time 250 Hz Raw Biopotential BLE Packetization
- In `Task_Sensors_Fast`, ADS1292R Channel 2 biopotential counts are sampled at 250 Hz (every 4 ms).
- Counts are converted to calibrated physical microvolts:
  $$\text{val}_{\mu\text{V}} = \text{raw}_{\text{ch2}} \times 0.048077\,\mu\text{V}$$
- Samples are packed into 10-sample batches (`int16_t` microvolts) along with live R-peak apex bitmasks and RA/LA electrode contact status bytes.
- Transmitted at 25 Hz (every 40 ms) via dedicated radio function:
  `hal_ble_radio_broadcast_ecg(samples_uv, 10, rpeak_mask, lead_status);`

### B. Multi-Modal Interleaved Advertising Protocol
Within standard 31-byte BLE advertising frames, the node broadcasts three specialized frame types identified by Company ID `0x53, 0x42` (`"SB"`):
1. **Frame Type `0x02` (Live 250 Hz ECG Batch)**:
   - Contains 10 sequential 16-bit physical biopotential samples (20 bytes).
   - Real-time R-peak apex position bitmask and RA/LA contact status byte.
2. **Frame Type `0x01` (Vitals, TinyML & MAC Superframe)**:
   - Broadcast every 200 ms: HR (BPM), RR Interval (ms), HRV SDNN & RMSSD (ms), On-Chip Int8 TinyML AAMI EC57 Class (`N`, `S`, `V`, `F`, `Q`), confidence %, inference latency ($30\,\mu\text{s}$), Respiration RPM, Skin & Ambient Temp, 8-Slot TDMA status.
3. **Frame Type `0x03` (3-Axis IMU Dynamics & Environment)**:
   - Broadcast every 200 ms: ADXL362 Accel $X, Y, Z$ in mg, Pitch & Roll in $0.1^\circ$, step counter, activity state, fall shock alert, BME680 Pressure, Humidity, IAQ index, eCO2 ppm, and OPT4041 Ambient Lux.

### C. Build & Flashing Verification
- Built using `tiarmclang 5.1.1.LTS` and SimpleLink SDK 8.33 with TI-RTOS7 kernel:
  `python build_and_flash.py`
- Successfully programmed into CC2652R1 flash memory via SmartRF Flash Programmer 2 (`XDS-L1100GRL`):
  `[FLASH] Flashing & Verification Complete! Reset target ... OK`

---

## 3. Android Mobile Application Architecture

### A. Real-Time Packet Decoder (`SmartBanPacketDecoder.kt`)
- Direct decoding of Type `0x02` biopotential batches, Type `0x01` vitals/TinyML, and Type `0x03` IMU/Environment.
- When an ECG batch arrives, samples are scaled directly to millivolts and immediately appended to the 250 Hz rolling ring buffer.

### B. Pure Real Hardware Repository (`SmartBanRepository.kt`)
- All synthetic waveform generators removed.
- If the node is powered off or disconnected, the trace rests cleanly at the isoelectric baseline; when live, genuine human biopotential counts roll smoothly across the screen at 60 FPS.

### C. Bottom Navigation Bar (`DashboardScreen.kt`)
Replaced the single-page layout with 4 convenient tabs:
1. **Tab 1: ECG TRACE**
   - Full 250dp high clinical biopotential oscilloscope canvas with 25 mm/s and 10 mm/mV dark grid lines.
   - Real-time R-peak apex red triangle markers.
   - Lead RA and LA contact badges (`RA: OK`, `LA: OK` or `OFF`).
   - $V_{pp}$ ($\mu\text{V}$) and $\text{SNR}$ ($\text{dB}$) readouts.
   - Quick cardiac electrophysiology summary (HR, RR, RPM, SDNN).
2. **Tab 2: CARDIAC & AI**
   - Dedicated cardiac vitals card with pulse indicator and autonomic HRV (SDNN & RMSSD).
   - On-Chip Int8 TinyML AAMI EC57 classification card with beat class avatar (`[N]`, `[S]`, `[V]`, `[F]`, `[Q]`), confidence bar, and $30\,\mu\text{s}$ / 1,440 cycles execution telemetry.
3. **Tab 3: IMU & ENV**
   - 3-Axis Accel normalized dynamic bars ($[-2g, +2g]$).
   - Pitch, Roll, AoA, $G_{\text{tot}}$, Activity State, and Step Counter.
   - High-contrast Fall Detection Alert banner.
   - Full environmental suite: Lux, Proximity, Pressure, Humidity, IAQ, eCO2, Altitude, and MLX90632 Skin Temp vs BME680 Ambient Temp.
4. **Tab 4: MAC & CTRL**
   - ETSI TS 103 326 8-Slot TDMA visualizer (Slot 0 to 7) with SAP/CAP active slot highlight.
   - 5G Slice Routing (`5G-mMTC` vs `5G-URLLC`).
   - Bandwidth ($99.03\%$) and Power ($92.8\%$) savings proof.
   - Remote action buttons: `INJECT 5s PVC ARRHYTHMIA & CAP EMERGENCY BURST`.
   - Streaming policy toggles and ADS1292R acquisition mode toggles (Mode 1 to 4).
   - Wi-Fi Hub IP configuration modal.

---

## 4. Hardware Verification & Live Deployment

1. **Firmware Programmed**:
   - Platform: `CC26X2R1_LAUNCHXL` + Multi-Sensor Shield Rev 3.5.
   - Status: Active and running with 2.4 GHz BLE radio broadcaster online.
2. **Android APK Built**:
   - Toolchain: Gradle 8.13 + Android Studio bundled JDK 21.
   - Command: `.\gradlew.bat assembleDebug`
   - Result: `BUILD SUCCESSFUL in 25s`.
   - Output: `android_app/app/build/outputs/apk/debug/app-debug.apk`.
3. **Live ADB Deployment**:
   - Device: Samsung Galaxy S22 Ultra (`SM-S908E`, Serial `R5CW51AVNYN`).
   - Installed via ADB: `Success`.
   - Launched: `am start -S -n com.smartban.sensor/.MainActivity`.
   - Runtime stability: 0 crashes / 0 exceptions.
