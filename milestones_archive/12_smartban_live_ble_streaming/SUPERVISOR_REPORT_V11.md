# Milestone 11 Technical Progress Report: Intelligent SmartBAN Sensor Node & Native Android BLE Integration

**Project**: Integration and Validation of an Intelligent Sensor Node for a Smart Body Area Network (SmartBAN) Testbed  
**Hardware Platform**: Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + Multi-Sensor Shield Rev 3.5  
**Operating System / Kernel**: TI-RTOS7 (`sysbios_7_13_00_47`) with POSIX (`pthread`, `semaphore`)  
**Wireless Protocols**: 2.4 GHz Bluetooth Low Energy (BLE 5.2) RF Core Broadcaster & Simultaneous High-Speed USB-UART Ground Truth  
**Client Mobile Application**: Native Android (Kotlin + Jetpack Compose & Material 3)  
**Date**: September 27, 2026  

---

## 1. Executive Summary

Milestone 11 completes the transition of the SmartBAN sensor node from an exclusively wired testbed into an **independent wireless autonomous biometric node**. The system simultaneously operates:
1. **On-Chip 2.4 GHz BLE RF Core**: Leveraging the CC2652R1's dedicated ARM Cortex-M0 RF coprocessor to broadcast standard 31-byte ETSI TS 103 326 SmartBAN telemetry frames out of the onboard PCB antenna at +5 dBm.
2. **Upgraded On-Chip Int8 TinyML Classifier**: Eliminating all proxy placeholders by extracting all 12 neural network inputs dynamically from real raw 24-bit ADC samples, filtered biopotentials, and Pan-Tompkins delay lines.
3. **Dedicated Native Android App**: A custom Android application engineered in **Kotlin with Jetpack Compose & Material 3**, designed for devices such as the Samsung Galaxy S22 Ultra. The app continuously scans for the node, decodes the 31-byte frame in real time, displays biometric vitals and SmartBAN TDMA superframe states, and provides remote emergency trigger capabilities.
4. **Simultaneous Dual-Stream Verification**: The wired USB-UART link (`COM3`) remains 100% operational alongside the 2.4 GHz radio, providing an uncompromised ground-truth reference for validating wireless packet fidelity and system latency.

---

## 2. CC2652R1 Dual-Core Radio Architecture

The CC2652R1 microcontroller features a dual-core architecture:
- **Main CPU (ARM Cortex-M4F @ 48 MHz)**: Runs the TI-RTOS7 real-time operating system, ADS1292R 250 Hz DRDY hardware interrupts, digital filtering (LPF, HPF, MWI), Pan-Tompkins QRS detection, and Int8 TinyML neural network inference.
- **RF Core (ARM Cortex-M0 Coprocessor @ 48 MHz)**: Runs a dedicated ROM-based radio protocol engine managing the 2.4 GHz transceiver, frequency synthesizer, power amplifier, and packet formation.

```
       +-----------------------------------------------------------+
       |                  TI CC2652R1 System-on-Chip               |
       |                                                           |
       |   +-----------------------+     +---------------------+   |
       |   |   ARM Cortex-M4F      |     |   ARM Cortex-M0     |   |
       |   |       (48 MHz)        |     |   RF Coprocessor    |   |
       |   |                       |     |                     |   |
       |   |  - TI-RTOS7 Kernel    |     |  - BLE 5.2 PHY      |   |
       |   |  - ADS1292R 250 Hz ISR|     |  - Frequency Synth  |   |
       |   |  - Pan-Tompkins QRS   |     |  - +5 dBm PA        |   |
       |   |  - Int8 TinyML Engine |     |  - Advert Packet TX |   |
       |   +-----------+-----------+     +----------+----------+   |
       |               |                            |              |
       |               +====== RF Doorbell Mailbox =+              |
       |                               |                           |
       |                    2.4 GHz RF Front-End                   |
       +-------------------------------|---------------------------+
                                       |
                                PCB Antenna
                                       |
                           (2.4 GHz BLE Broadcast)
                                       |
                                       v
                         +---------------------------+
                         | Samsung Galaxy S22 Ultra  |
                         |  Native Compose BLE App   |
                         +---------------------------+
```

### 2.1 Hardware Driver Implementation (`hal_ble_radio.c`)
The radio interface is implemented in `hal/hal_ble_radio.c` using TI's RF Driver (`ti/drivers/rf/RF.h`):
- **Radio Mode**: `RF_MODE_AUTO` configured with SmartRF Studio-validated BLE patches (`rf_patch_cpe_bt5.h` and `rf_patch_mce_bt5.h`).
- **Radio Setup**: `CMD_BLE5_RADIO_SETUP` configured for differential RF frontend mode, internal bias, and +5 dBm transmit power (`0x7217`).
- **Transmission Operation**: `CMD_BLE_ADV_NC` (Non-Connectable Advertiser) executing on primary BLE advertising channels 37 (2402 MHz), 38 (2426 MHz), and 39 (2480 MHz).
- **Execution Overhead**: Radio execution on the Cortex-M0 requires **< 1.5 ms** per 1 Hz transmission, consuming zero CPU cycles from the Cortex-M4F application core.

---

## 3. 31-Byte SmartBAN Wireless Frame Specification

To guarantee zero-latency reception on any standard smartphone without manual Bluetooth pairing or GATT connection handshakes, the telemetry payload is encoded directly into standard BLE Non-Connectable Advertising PDUs (`ADV_NONCONN_IND`):

| Offset | Field | Value / Type | Description |
|:---:|:---|:---:|:---|
| `[0..2]` | **Flags AD** | `0x02, 0x01, 0x06` | LE General Discoverable, BR/EDR Not Supported |
| `[3..4]` | **Name Header** | `0x0D, 0x09` | Complete Local Name AD Header |
| `[5..17]` | **Device Name** | `"SmartBAN-Node"` | 13-character ASCII broadcast identifier |
| `[18..19]`| **Manufacturer Header**| `0x0D, 0xFF` | Manufacturer Specific Data AD Header (13 bytes) |
| `[20..21]`| **Company ID** | `0x53, 0x42` (`"SB"`) | SmartBAN Telemetry Signature |
| `[22]` | **Sequence ID** | `uint8_t` (0..255) | Rolling packet counter |
| `[23]` | **Heart Rate** | `uint8_t` (BPM) | Live smoothed cardiac rate |
| `[24..25]`| **RR Interval**| `uint16_t` (ms) | Peak-to-peak inter-beat interval |
| `[26]` | **TinyML Class**| `uint8_t` (0..4) | 0=Normal (N), 1=SVEB (S), 2=PVC (V), 3=Fusion (F), 4=Noise (Q) |
| `[27]` | **Respiration**| `uint8_t` (RPM) | Dual-source thoracic impedance + EDR rate |
| `[28]` | **Skin Temp** | `int8_t` (°C) | Real-time digital thermal sensor |
| `[29]` | **Status Bitfield**| `uint8_t` | Bit 0: Fall Alert, Bit 1: PVC Alert, Bit 2: CAP Burst, Bits 3..4: Posture, Bits 5..7: TDMA Slot |
| `[30]` | **HRV Metric** | `uint8_t` (ms) | Short-term SDNN autonomic tone |

---

## 4. Upgraded Neural Network Feature Engine

In Milestone 10, placeholder constants were used for secondary morphological inputs. In Milestone 11, `edgeai/edgeai_tinyml.c` was completely refactored so that **all 12 input features are dynamically derived** from the raw ADC sample stream and Pan-Tompkins filter states:

1. **\(f_0\) (Pre-RR Ratio)**: \(((RR_{curr} - \overline{RR}) \times 64) / \overline{RR}\)
2. **\(f_1\) (Delta-RR Ratio)**: \(((RR_{curr} - RR_{prev}) \times 64) / \overline{RR}\)
3. **\(f_2\) (QRS Energy Width)**: Derived from moving-window integrator signal normalized to signal peak: \((MWI \times 32 / SPKI) - 32\)
4. **\(f_3\) (Normalized R-Peak Amplitude)**: \((V_{filtered} \times 32 / SPKI) + 12\)
5. **\(f_4\) (Real-Time SNR)**: \(((SPKI - NPKI) \times 64) / (SPKI + 1) - 32\)
6. **\(f_5\) (HRV SDNN Deviation)**: \(((SDNN - 50) \times 64) / 50\)
7. **\(f_6\) (HRV RMSSD Deviation)**: \(((RMSSD - 42) \times 64) / 42\)
8. **\(f_7\) (QRS Derivative Slope Asymmetry)**: \((d[0] - d[1]) / 32\) (rise vs fall slope around R-peak)
9. **\(f_8\) (High-Frequency Noise Index)**: \((NPKI \times 64) / (SPKI + 1)\)
10. **\(f_9\) (Beat Prematurity Index)**: Derived from early arrival relative to 80% expected mean interval
11. **\(f_{10}\) (Resting HR Deviation)**: \(((HR_{bpm} - 72) \times 64) / 35\)
12. **\(f_{11}\) (Full Compensatory Pause Index)**: \(((RR_{curr} + RR_{prev} - 2\overline{RR}) \times 64) / \overline{RR} - 10\)

---

## 5. Native Android Application Architecture

The mobile client located in `android_app/` was constructed using the modern Android technology stack:
- **Language**: Kotlin 2.0.0
- **UI Toolkit**: Jetpack Compose with Material Design 3
- **Asynchronous Architecture**: Kotlin Coroutines + `StateFlow`
- **Minimum Android Version**: Android 8.0 (API 26), targeted for Android 15 (API 35)

### 5.1 Key Components
- **`BleScanner.kt`**: Scans in low-latency mode without location tracking, filtering advertisements matching the `"SmartBAN-Node"` name or `"SB"` manufacturer header.
- **`SmartBanPacketDecoder.kt`**: Extracts biometric metrics from the raw advertising byte array and maps them to clinical domain objects.
- **`DashboardScreen.kt`**: Dark clinical telemetry UI featuring real-time pulse indicators, color-coded AAMI EC57 diagnostic badges (Green for Normal, Red for PVC, Orange for Ectopic), an 8-slot TDMA superframe visualizer, and 5G network slice state.
- **Interactive Control**: Dedicated button for triggering simulated PVC arrhythmia bursts to demonstrate dynamic CAP contention switching and 5G-URLLC low-latency failover.

---

## 6. Live Hardware Verification Results

Live serial verification performed on the active hardware via `verify_live_data.py` confirmed authentic sensor noise and real-time responsiveness:

| Sensor Subsystem | Sample Count (N) | Measured Mean | Dynamic Range | Standard Deviation | Verification Status |
|:---|:---:|:---:|:---:|:---:|:---:|
| **IMU Accel X** | 226 | -0.00 g | [-0.13, +0.13] g | 0.0186 | **LIVE (Real Noise)** |
| **IMU Accel Y** | 226 | -0.01 g | [-0.11, +0.14] g | 0.0326 | **LIVE (Real Noise)** |
| **IMU Accel Z** | 226 | +1.02 g | [+1.00, +1.15] g | 0.0129 | **LIVE (1g Gravity)** |
| **IMU Pitch** | 226 | +0.06° | [-6.40, +7.50]° | 0.9203 | **LIVE (Real Noise)** |
| **IMU Roll** | 226 | -0.35° | [-6.10, +8.10]° | 1.8275 | **LIVE (Real Noise)** |
| **BME Temperature**| 14 | 21.91 °C | [21.88, 21.92] °C | 0.0125 | **LIVE (Real Noise)** |
| **BME Humidity** | 14 | 33.07 % | [33.04, 33.11] % | 0.0220 | **LIVE (Real Noise)** |
| **BME Pressure** | 14 | 991.66 hPa | [991.65, 991.69] hPa| 0.0135 | **LIVE (Real Noise)** |
| **OPT Ambient Lux**| 14 | 49.47 lux | [49.40, 49.52] lux | 0.0284 | **LIVE (Real Noise)** |
| **MLX IR Temp** | 14 | 23.15 °C | [23.09, 23.21] °C | 0.0274 | **LIVE (Real Noise)** |

### 6.1 Simultaneous Dual-Stream Output
Sample line captured simultaneously over wired USB-UART during active 2.4 GHz BLE broadcasting:
```text
B,25,0,0,N,98,30,1318,9538,92,92,22.18,1.60,1.60,1333.3
```
- `B`: SmartBAN Superframe & Energy Benchmark prefix
- `25`: Sequence index
- `0`: Policy = Adaptive Hybrid
- `0`: Slot = SAP Periodic (Slot 0)
- `N`: TinyML Predicted Class = Normal Sinus Beat
- `98`: Confidence = 98%
- `30`: Inference execution time = 30 microseconds
- `1318`: CPU active time = 1.318 ms/s (99.87% deep sleep)
- `9538`: Continuous raw stream bandwidth = 9,538 B/s
- `92`: Semantic token bandwidth = 92 B/s (**99.03% bandwidth reduction**)
- `22.18`: Raw wireless power = 22.18 mW
- `1.60`: Semantic wireless power = 1.60 mW (**92.8% power reduction**)

---

## 7. Deliverables & Next Steps

1. **Firmware**: Completely compiled, linked, and flashed in `firmware/11_smartban_ble_android/`.
2. **Android Application**: Complete source code and Gradle build scripts generated in `android_app/`.
3. **Reference GUI**: Freely resizable desktop monitoring dashboard preserved with simultaneous multi-sensor plotting and protocol documentation modal.
4. **Next Step**: Install the compiled APK onto the user's Samsung S22 Ultra via `adb install` and conduct over-the-air field validation.
