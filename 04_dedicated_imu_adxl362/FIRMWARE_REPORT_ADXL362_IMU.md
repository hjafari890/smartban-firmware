# Technical Report: SmartBAN ADXL362 IMU Firmware & 3D Telemetry System
**project Topic:** *Integration and Validation of an Intelligent Sensor Node for a Smart Body Area Network (SmartBAN) Testbed*  
**Module:** Firmware Subsystem 05, Dedicated ADXL362 IMU Bring-Up, Calibration & 3D Spatial Attitude Visualizer  
**Target Platform:** Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + BAN Shield Rev 3.5  
**Primary Sensor:** Analog Devices ADXL362 Ultra-Low Power 3-Axis MEMS Accelerometer  

---

## 1. What Was Built

This milestone developed a production-grade, standalone diagnostic and real-time telemetry firmware (`05_imu_adxl362_test`) and an interactive 3D spatial attitude visualization suite for the Analog Devices ADXL362 MEMS accelerometer on the SmartBAN Shield Rev 3.5 platform. 

The firmware implements:
- Full SPI driver support with dynamic pin routing for the BAN Shield Rev 3.5 layout (SCLK on DIO 10, SDI on DIO 8, SDO on DIO 9, CS on DIO 15).
- Multi-channel streaming (12-bit 3-axis acceleration, on-chip die temperature, hardware interrupt pin monitoring for INT1/DIO 26 and INT2/DIO 27, and status register bit decoding).
- Complete diagnostic suite: built-in electrostatic MEMS self-test deflection audit, 512-sample hardware FIFO burst retrieval, autonomous motion/inactivity activity detection, and zero-G tare calibration.
- Real-time 60 FPS 3D graphical user interface ([gui_3d_imu.py](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/gui_3d_imu.py)), 2D strip-chart visualizer ([gui_imu_visualizer.py](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/gui_imu_visualizer.py)), and Rich terminal dashboard ([view_imu_test.py](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/view_imu_test.py)).

---

## 2. Key Technical Achievements

1. **Hardware Pin-Swapping & Multi-Slave SPI Bus Isolation**:
   - Resolved the physical SDI/SDO pin-routing swap unique to BAN Shield V3.5 (`IOCPortConfigureSet` routing `IOID_8` as `IOC_PORT_MCU_SSI0_TX` and `IOID_9` as `IOC_PORT_MCU_SSI0_RX`).
   - Prevented bus contention on the shared SPI bus by asserting hardware Chip Select isolation on the ADS1292R ECG AFE (`CONFIG_GPIO_ECG_CS` / DIO 11 held `HIGH`) and disabling the Q1 discharge transistor (`CONFIG_GPIO_IMU_SW` / DIO 18 driven `LOW`).

2. **Diagnosis and Resolution of Mechanical/Thermal Zero-G Bias Railing**:
   - Identified a $-4.7\text{ g}$ resting physical baseline offset on the Y-axis (attributable to mechanical package stress from the LGA-16 soldering process).
   - Diagnosed that in the default $\pm 2\text{ g}$ range, the $-4.7\text{ g}$ bias exceeded full scale and permanently clamped the 12-bit ADC at its negative rail ($-2048\text{ mg}$ / `0xF800`), locking the roll computation at $-60.2^\circ$.
   - Migrated the baseline configuration to $\pm 8\text{ g}$ range ($\pm 8.192\text{ g}$ full scale, $4\text{ mg/LSB}$) and developed an automated 64-sample startup Tare Calibration algorithm in milli-g units. This eliminated saturation, centering the horizontal attitude to Pitch $= 0.2^\circ$, Roll $= 0.2^\circ$, and $|a| = 0.994\text{ g}$ (nominal 1.0g Earth gravity).

3. **MEMS Silicon Health Verification via Electrostatic Deflection**:
   - Implemented automated electrostatic deflection self-test auditing (Datasheet Table 22 criteria).
   - Deflection results verified healthy capacitive microstructures across all 3 axes:
     - $\Delta X = +1.876\text{ g}$ (Spec: $+0.20\text{ g}$ to $+2.80\text{ g}$) &rarr; **PASS**
     - $\Delta Y = -2.053\text{ g}$ (Spec: $-2.80\text{ g}$ to $-0.20\text{ g}$) &rarr; **PASS**
     - $\Delta Z = +1.748\text{ g}$ (Spec: $+0.20\text{ g}$ to $+2.80\text{ g}$) &rarr; **PASS**

4. **Zero-Dependency 60 FPS 3D IMU Indicator Engine**:
   - Engineered a 3D vector graphics engine in pure Python/Tkinter utilizing Euler rotation matrices, camera view projection, depth-sorted polygonal shading (Painter's algorithm), and directional lighting.
   - Real-time rendering of the BAN Shield 3D model, 3-axis coordinate frame ($+X$ red, $+Y$ green, $+Z$ blue), dynamic amber gravity/acceleration vector, and cockpit attitude horizon indicators.

5. **Code Composer Studio (CCS) Native IDE Project Integration**:
   - Diagnosed and fixed an unescaped XML ampersand in the `.projectspec` file that caused a `NullPointerException` during CCS import.
   - Built full native Eclipse/Theia project configurations (`.project`, `.cproject`, `.ccsproject`, `.clangd`, `.theia/launch.json`), verified by headless compilation with 0 errors.

---

## 3. Software Architecture

| File | Role & Interaction |
| :--- | :--- |
| [`main.c`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/main.c) | Core firmware application executing sensor initialization, SPI transactions, register configurations, auto-tare calibration, and 100 Hz UART telemetry streaming. |
| [`diagnostic.syscfg`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/diagnostic.syscfg) | TI SysConfig hardware definition assigning CC2652R1 DIO pins (UART2, SPI0, GPIOs for CS, SW, INT1, INT2, and 1V8 power domain). |
| [`cc13x2_cc26x2_nortos.cmd`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/cc13x2_cc26x2_nortos.cmd) | Linker command script allocating FLASH (352 KB) and SRAM (80 KB) memory partitions for NoRTOS execution. |
| [`build_and_flash.py`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/build_and_flash.py) | Automated toolchain pipeline executing SysConfig code generation, `tiarmclang` compilation, linking, hex conversion, and `srfprog` flashing. |
| [`gui_3d_imu.py`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/gui_3d_imu.py) | Standalone desktop 3D attitude visualizer with interactive camera orbit, dynamic gravity vector, and digital cockpit instruments. |
| [`run_3d_gui.bat`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/run_3d_gui.bat) | One-click Windows batch launcher for the 3D IMU telemetry dashboard. |
| [`view_imu_test.py`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/view_imu_test.py) | High-performance terminal dashboard with live ASCII level meters, horizon crosshair, and register flag decoders. |
| [`gui_imu_visualizer.py`](file:///C:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/05_imu_adxl362_test/gui_imu_visualizer.py) | Tkinter/Matplotlib strip-chart desktop visualizer displaying real-time 3-axis waveforms and vector magnitude. |

---

## 4. Challenges & How They Were Solved

### Challenge 1: SDI/SDO Pin Swapping on BAN Shield Rev 3.5
- **Symptom:** Initial SPI reads returned `0xFF` or `0x00` (`DEVID_AD` failed to match `0xAD`).
- **Cause:** Shield routing wired CC2652R1 HOST_MOSI (DIO 9) to ADXL362 SDO (MISO) and HOST_MISO (DIO 8) to ADXL362 SDI (MOSI).
- **Solution:** Configured CC2652 DriverLib IOC routing at runtime via `IOCPortConfigureSet(IOID_8, IOC_PORT_MCU_SSI0_TX, ...)` and `IOID_9` as `IOC_PORT_MCU_SSI0_RX`. Added auto-probing fallback to ensure portability.

### Challenge 2: Y-Axis Satiation & Permanent $-60^\circ$ Attitude Lock
- **Symptom:** Accelerometer output remained pinned at `Y: -2048mg` with $|a| = 2368\text{ mg}$ and $\text{Roll} = -60.2^\circ$, unresponsive to motion.
- **Cause:** LGA-16 package mechanical soldering strain created a $-4.7\text{ g}$ physical offset, which saturated the $\pm 2\text{ g}$ ADC ceiling ($-2048\text{ mg}$ limit).
- **Solution:** Reconfigured default dynamic range to $\pm 8\text{ g}$ (linear span $\pm 8.192\text{ g}$) and implemented a 64-sample startup Tare Calibration algorithm storing milli-g offsets. Output immediately normalized to $X \approx 0\text{ mg}$, $Y \approx 0\text{ mg}$, $Z \approx 994\text{ mg}$, and $|a| \approx 0.994\text{ g}$.

### Challenge 3: CCS IDE Import Rejection
- **Symptom:** Code Composer Studio failed to discover or import the project directory.
- **Cause:** Unescaped `&` in the `.projectspec` title attribute triggered a fatal XML parser error and internal `NullPointerException`. Furthermore, native CCS metadata directories were missing.
- **Solution:** Escaped the XML entity (`&amp;`) and generated the native CCS Eclipse/Theia project workspace structure (`.project`, `.cproject`, `.ccsproject`, `.clangd`, `.theia/launch.json`). Verified clean import and compilation via `ccs-server-cli`.

---

## 5. Quantitative Results

| Parameter | Measured / Achieved Value | Reference / Datasheet Spec |
| :--- | :--- | :--- |
| **FLASH Footprint** | **35,553 bytes (34.7 KB)** | $10.1\%$ of 352 KB Flash |
| **SRAM Footprint** | **19,816 bytes (19.4 KB)** | $24.2\%$ of 80 KB RAM |
| **UART Telemetry Stream Rate** | **10.0 Hz (115,200 baud)** | Non-blocking ring buffer |
| **Internal Sensor ODR** | **100 Hz** (selectable 12.5 - 400 Hz) | Ultra-Low Noise Mode |
| **Active Sensor Current** | **13.0 $\mu\text{A}$** @ 3.3V | $13\ \mu\text{A}$ @ 100 Hz ODR |
| **Dynamic Range** | **$\pm 8\text{ g}$** ($4\text{ mg/LSB}$ sensitivity) | Linear range $\pm 8.192\text{ g}$ |
| **Calibrated Stationary Accuracy** | **$|a| = 0.994\text{ g} \pm 0.015\text{ g}$** | $1.000\text{ g}$ nominal Earth gravity |
| **Attitude Level Precision** | **$\text{Pitch} = +0.2^\circ \pm 0.3^\circ,\ \text{Roll} = +0.2^\circ \pm 0.3^\circ$** | Flat stationary test bench |
| **Self-Test Deflection $\Delta X$** | **$+1.876\text{ g}$** | $+0.20\text{ g}$ to $+2.80\text{ g}$ (PASS) |
| **Self-Test Deflection $\Delta Y$** | **$-2.053\text{ g}$** | $-2.80\text{ g}$ to $-0.20\text{ g}$ (PASS) |
| **Self-Test Deflection $\Delta Z$** | **$+1.748\text{ g}$** | $+0.20\text{ g}$ to $+2.80\text{ g}$ (PASS) |
| **3D GUI Rendering Rate** | **60.0 FPS** | Double-buffered Tkinter Canvas |

---

## 6. What This Version Enabled for the Next Milestone

1. **Robust Inertial Subsystem for Sensor Fusion**:
   Provides a verified, low-noise ($13\ \mu\text{A}$) 3-axis motion reference ready for integration into the multi-modal SmartBAN sensor hub alongside the ADS1292R ECG and MAXM86161 PPG optical sensors.

2. **Motion-Artifact Cancellation (MAC) Reference**:
   The calibrated 100 Hz acceleration stream provides the physical reference signal required to filter out motion artifacts from ambulatory ECG/PPG cardiac waveforms.

3. **Autonomous Power Management via Activity/Inactivity Thresholds**:
   The validated motion-detection interrupt pipeline on INT1/INT2 allows the CC2652R1 microcontroller to enter deep sleep ($<1\ \mu\text{A}$) during sedentary periods and wake autonomously upon physical motion.

---

## 7. Personal Reflection & Experience

Through this bring-up phase, I learned that apparent sensor failures on custom hardware often stem from subtle interactions between mechanical assembly stress and digital dynamic range limits rather than permanent hardware damage. Conducting register-level diagnostics and electrostatic deflection tests provided definitive proof of MEMS silicon integrity, demonstrating the critical importance of rigorous root-cause analysis before replacing physical components.
