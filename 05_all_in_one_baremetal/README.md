# 05_all_in_one_baremetal: SmartBAN Integrated Bare-Metal Firmware

A unified, bare-metal (NoRTOS) embedded platform for the Texas Instruments CC26X2 / CC2650 MCU that integrates physiological biosignals, 3-axis inertial flight dynamics, multi-modal environmental sensing, and local UI controls into a single synchronized, low-latency firmware.

---

## 📁 Project Structure

All files relating to this unified firmware and visualizer are stored directly within this folder:

```text
07_all_in_one_sensor/
├── main.c              # Core bare-metal firmware (AFE drivers, NoRTOS super-loop)
├── diagnostic.syscfg   # TI SysConfig pinout and peripheral definition
├── build_and_flash.py  # Automation script: SysConfig codegen + ticlang + SmartRF flash
├── sensor_gui.py       # Production Python GUI (DSP filters, Pan-Tompkins QRS, PFD horizon)
├── JOURNAL_REPORT.md   # IEEE/ACM journal-style technical publication report
├── cc13x2_cc26x2_...   # Linker command file for Cortex-M4F memory map
├── patch_scripts/      # Architectural progression & patch scripts history
└── syscfg/             # Auto-generated TI Drivers configuration C/H files
```

---

## 🚀 Quick Start

### 1. Build and Flash the Firmware
Connect the CC26X2 LaunchPad via USB (XDS110 debugger), open a terminal in this directory, and run:
```bash
python build_and_flash.py
```
This script will automatically:
1. Run `sysconfig_cli.bat` to generate device drivers from `diagnostic.syscfg`.
2. Compile `main.c` and driver sources using `tiarmclang` (Arm LLVM).
3. Link the target image into `diagnostic.out`.
4. Generate the Intel HEX binary `diagnostic.hex`.
5. Flash and verify the CC26X2 target using SmartRF Flash Programmer 2 (`srfprog`).

### 2. Run the Clinical & Flight Visualizer GUI
Make sure any terminal serial monitors (PuTTY, Arduino, etc.) are **closed**, then run:
```bash
python sensor_gui.py
```
*(The visualizer auto-detects `COM3` at `115200 baud`).*

---

## ⚡ Key Features

### 1. High-Fidelity 250 Hz ECG & Respiration Pneumography
- **ADS1292R Analog Front-End**: Continuous conversion at 500 SPS via hardware DRDY interrupt, decimated 2:1 to 250 Hz.
- **Compact Packet Transmission**: Packets formatted as `{"e":[status, ch1_resp, ch2_ecg]}` consuming only ~56% of 115,200 baud UART bandwidth.
- **Physical Calibrated Scaling**: $V_{\text{in}} (\text{mV}) = \text{raw} \times 4.8077 \times 10^{-5}\text{ mV}$ (Gain = 6, 2.42V internal reference).
- **3-Stage Biquad Filter Pipeline**:
  - `0.5 Hz` 2nd-order Butterworth Highpass (eliminates baseline drift).
  - `40.0 Hz` 2nd-order Butterworth Lowpass (muscle EMG suppression).
  - `50.0 Hz / 60.0 Hz` selectable notch filter ($Q = 35.0$) for AC mains hum rejection.
- **Gold-Standard Pan-Tompkins QRS Detector**:
  - 5-point centered derivative, non-linear squaring, and 150 ms Moving Window Integration.
  - Dual adaptive thresholding (`SPKI` and `NPKI`) with 200 ms refractory blanking.
  - 8-beat median RR-interval smoothing for accurate Heart Rate (BPM).
- **Real-Time Lead-Off Contact Monitoring**:
  - Evaluates ADS1292R status synchronization bits (`bit 3` RA, `bit 2` LA).
  - Displays real-time electrode contact badges (`● RA: OK`, `● LA: OK`).
- **4 Hardware Analog Test Modes**:
  - `1`: 1 Hz Square Wave (+/-1 mV test generator).
  - `2`: Input Shorted (analog noise floor & offset test).
  - `3`: Internal Die Temperature Diode.
  - `4`: Live Electrodes (normal Lead I ECG).
  *(Switchable via physical button `SW1` or via GUI buttons).*

### 2. Glass-Cockpit Primary Flight Display (PFD) & Attitude Dynamics
- **ADXL362 3-Axis MEMS Accelerometer**:
  - 25 Hz live transmission: `{"type":"imu","ax":...,"ay":...,"az":...,"pitch":...,"roll":...,"act":...}`.
  - Trigonometric Roll ($\phi = \text{atan2}(a_y, a_z)$) and Pitch ($\theta = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$).
  - Angle of Attack / Total Inclination ($\alpha = \arccos(a_z / |a|)$).
- **Artificial Horizon Instrument**:
  - Sky blue and earth brown dynamic rotating horizon.
  - Pitch ladder rungs ($\pm 10^\circ, \pm 20^\circ$) and upper roll pointer arc.
  - Stationary amber aircraft reference symbol (`---•---`).
  - Digital avionics HUD readouts for Roll, Pitch, AoA, and total G-load.
- **Autonomous Motion / Attack Detector**:
  - Hardware motion threshold engine ($>250\text{ mg}$ for $40\text{ ms}$, $<150\text{ mg}$ for $500\text{ ms}$).
  - Real-time alarm banner: `🔥 ATTACK / MOTION DETECTED!` vs `● STILL / RESTING`.

### 3. Environmental Sensing Suite (1 Hz)
- **BME680**: Ambient temperature (°C), relative humidity (% RH), barometric pressure (hPa).
- **OPT4041**: High-precision ambient light lux.
- **VCNL4040**: Optical proximity counts and infrared ambient illumination.
- **MLX90632**: Far-infrared non-contact object temperature (°C).
- *(Note: MAX32664 PPG hub is cleanly bypassed due to a known 1.8V/3.3V PCB domain conflict).*

### 4. Local Hardware UI Controls
- **CH455H 3-Digit 7-Segment Display & LEDs**:
  - Press **`SW6`** to cycle display modes:
    - `tEP`: BME680 Temperature
    - `PrS`: BME680 Pressure
    - `Hud`: BME680 Humidity
    - `Lux`: OPT4041 Ambient Light
    - `Prx`: VCNL4040 Proximity
    - `Irt`: MLX90632 IR Temperature
    - `ECG`: Current ECG Mode
    - `Att`: **Attitude Mode** (shows real-time pitch angle in degrees!)
- **`SW1`**: Cycles ADS1292R analog self-test modes.
- **`SW4`**: ADXL362 Zero-G Tare calibration (displays `CAL`).

---

## 📄 Technical Documentation
A publication-grade technical manuscript describing the mathematical derivations, communication modeling, and empirical benchmarking is available in:
- **[JOURNAL_REPORT.md](JOURNAL_REPORT.md)**
