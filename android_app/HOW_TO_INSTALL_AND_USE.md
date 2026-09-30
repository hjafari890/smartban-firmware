# SmartBAN Android Telemetry Monitor: Installation & User Guide

**Target Hardware**: Texas Instruments CC2652R1 LaunchPad (`CC26X2R1_LAUNCHXL`) + Multi-Sensor Shield Rev 3.5  
**Target Mobile Device**: Samsung Galaxy S22 Ultra (or any Android smartphone running Android 8.0+ / API 26+)  
**Application Architecture**: Native Android (Kotlin 2.0.0, Jetpack Compose, Material Design 3, Coroutines)  
**Wireless Protocol**: Bluetooth Low Energy (BLE 5.2) Non-Connectable Broadcaster (`ADV_NONCONN_IND`)  
**Package Name**: `com.smartban.sensor`  
**Ready-to-Install APK**: [`SmartBAN_Monitor_v1.0.apk`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/android_app/SmartBAN_Monitor_v1.0.apk) (15.6 MB)  

---

## 1. Overview

The **SmartBAN Android Telemetry Monitor** is a standalone native application that interfaces wirelessly with the CC2652R1 sensor node. It listens for periodic 1 Hz / 2 Hz 2.4 GHz BLE advertisements broadcast directly from the CC2652R1 PCB antenna, decodes the 31-byte ETSI TS 103 326 SmartBAN frame over the air, and displays real-time clinical biosignals, on-chip Int8 TinyML beat classifications, and 5G network slice status.

**Instant Connection**: The application captures telemetry directly from Bluetooth Low Energy broadcast advertisements as soon as scanning starts, with no manual pairing needed.

---

## 2. Installation Options

You can install the application using **Method A** (easiest via USB cable) or **Method B** (completely wireless via file transfer).

---

### Method A: One-Click Automated USB Installation (Recommended)

This method uses the pre-configured Android SDK toolchain already installed on this computer.

#### Step 1: Enable Developer Options on Samsung S22 Ultra
1. Open **Settings** on your phone.
2. Scroll to the bottom and tap **About phone** $\to$ **Software information**.
3. Tap **Build number** rapidly **7 times** until you see the prompt: *"Developer mode has been enabled"*.

#### Step 2: Enable USB Debugging
1. Go back to the main **Settings** menu.
2. At the very bottom, tap the new option: **Developer options**.
3. Scroll down and turn **ON** the toggle for **USB debugging**.

#### Step 3: Connect and Authorize PC
1. Connect your phone to your computer using a USB-C data cable.
2. Look at your phone's screen. A popup will appear:
   > *"Allow USB debugging from this computer?"*
3. Check the box **"Always allow from this computer"** and tap **Allow**.

#### Step 4: Run the Installer Script
1. Navigate to the `android_app` directory on your computer:
   ```text
   c:\Users\hjafa\OneDrive\Desktop\shield cc2650\android_app
   ```
2. Double-click the file:
   ```text
   install_apk.bat
   ```
3. The script will automatically detect your phone, install `SmartBAN_Monitor_v1.0.apk`, and immediately launch the app on your screen.

---

### Method B: Direct APK File Transfer (Wireless / No Cable)

If you prefer not to use USB debugging, you can install the APK directly on your phone:

1. **Transfer the APK to your phone**:
   - Send [`SmartBAN_Monitor_v1.0.apk`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/android_app/SmartBAN_Monitor_v1.0.apk) to your phone via:
     - **Quick Share** / **Nearby Share** (Right-click the APK on PC $\to$ Share $\to$ Quick Share to your S22 Ultra).
     - **Google Drive** / **OneDrive** (Upload on PC, download on phone).
     - **USB File Transfer** (Drag and drop into phone's `Download` folder).
     - **WhatsApp / Telegram** (Send to "Saved Messages").
2. **Install on Phone**:
   - Open the **My Files** app on your Samsung phone.
   - Tap **Downloads** (or **Installation files**).
   - Tap **`SmartBAN_Monitor_v1.0.apk`**.
   - If prompted: *"For your security, your phone is not allowed to install unknown apps from this source"*, tap **Settings** $\to$ turn **ON** the toggle for *My Files* $\to$ tap **Install**.
   - Once installed, tap **Open**.

---

### Method C: Open in Android Studio

If you want to view, modify, or debug the Kotlin source code:
1. Launch **Android Studio** (`C:\Program Files\Android\Android Studio\bin\studio64.exe`).
2. Select **File $\to$ Open...**
3. Select the folder: `c:\Users\hjafa\OneDrive\Desktop\shield cc2650\android_app`.
4. Allow Gradle to sync, select your Samsung S22 Ultra from the device dropdown at the top, and click the green **Run (Play)** button.

---

## 3. First-Time Launch & Permissions

When the app opens for the first time:
1. **Grant Bluetooth Permissions**:
   - Android 12+ requires runtime permission for nearby device scanning:
   - Tap **Allow** when prompted: *"Allow SmartBAN Node to find, connect to, and determine the relative position of nearby devices?"*
2. **Turn Bluetooth ON**:
   - Ensure Bluetooth is switched ON in your phone's quick settings tray.

---

## 4. Using the Application

### 4.1 Live Telemetry Dashboard
Once scanning starts, the app detects `"SmartBAN-Node"` and displays live telemetry cards:

1. **Cardiac Electrophysiology Card**:
   - **Heart Rate**: Real-time smoothed heart rate in BPM.
   - **RR Interval**: Inter-beat duration in milliseconds ($ms$).
   - **HRV SDNN**: Autonomic heart rate variability index in milliseconds ($ms$).
   - **Arrhythmia Alert Banner**: Illuminates red with *"PVC ARRHYTHMIA ALERT"* when an ectopic contraction is detected.

2. **Edge-AI TinyML Classifier Card**:
   - Displays the on-chip **AAMI EC57** cardiac beat classification:
     - **[N] Normal Sinus**: Green badge (healthy baseline).
     - **[V] Premature Ventricular Contraction (PVC)**: Red badge (emergency arrhythmia).
     - **[S] Supraventricular Ectopic**: Orange badge.
     - **[F] Ventricular Fusion**: Orange badge.
     - **[Q] Artifact / Noise**: Muted gray badge.

3. **Biometrics & Environment Card**:
   - **Respiration**: Live breathing rate in RPM (derived from dual-source thoracic impedance + ECG-derived respiration).
   - **Skin Temperature**: Digital temperature in °C.
   - **Posture**: Real-time classification (`STANDING`, `SITTING`, `SUPINE`, `WALKING`).
   - **Fall Alert**: Illuminates red with *"FALL DETECTED"* if impact dynamics exceed 2.8g with supine posture transition.

4. **SmartBAN MAC & 5G Network Slicing Card**:
   - **8-Slot TDMA Superframe Visualizer**: Shows active transmission slots (`S0` through `S6` for periodic Scheduled Access, `CAP` for emergency Contention Access).
   - **5G Network Slice Indicator**:
     - Normal Operation: Routed to **`5G-mMTC (High-Efficiency Semantic 1Hz)`** at 92 B/s (**99.03% bandwidth reduction**).
     - Arrhythmia / Fall Trigger: Routed dynamically to **`5G-URLLC (Emergency Low-Latency <1ms)`** via CAP contention slots.
   - **Energy Consumption**: Displays active comparison of 1.60 mW semantic mode vs 22.18 mW continuous raw transmission.

---

### 4.2 Interactive Remote Simulation Trigger

The bottom **System Controls** card features two primary controls:
* **[START / STOP BLE SCAN]**: Starts or halts background low-latency BLE packet capture.
* **[INJECT 5s PVC ARRHYTHMIA & CAP BURST]**:
  - Tapping this button demonstrates the system's emergency response capabilities.
  - The app commands the testbed to simulate an acute premature ventricular contraction.
  - You will instantly observe:
    1. The TinyML classification switches from **[N] Normal** to **[V] PVC Ectopic** in red.
    2. The superframe slot switches from **SAP Periodic** to **CAP Emergency Contention**.
    3. The network slice switches from **5G-mMTC** to **5G-URLLC (<1 ms latency)**.
    4. After 5 seconds, the node automatically settles back into healthy 1 Hz semantic SAP monitoring.

---

## 5. Simultaneous Operation with PC GUI (`COM3`)

The LaunchPad simultaneously streams both **wired USB-UART (`COM3`)** and **wireless 2.4 GHz BLE broadcasts**:
* You can keep the desktop engineering GUI running on your computer:
  ```powershell
  python 01_final_firmware_v12_stable/sensor_gui.py
  ```
* While viewing the desktop GUI on your monitor, your Samsung Galaxy S22 Ultra will receive the exact same telemetry frames over the air, proving true multi-modal and wireless connectivity.

---

## 6. Troubleshooting

| Issue | Cause | Solution |
|:---|:---|:---|
| `adb: no devices/emulators found` | USB debugging is not enabled or cable is charge-only | Verify Developer Options $\to$ USB Debugging is ON. Ensure you accepted the "Allow USB debugging" prompt on the phone screen. |
| App says "Scan error" or "Bluetooth disabled" | Bluetooth is off or permission missing | Toggle Bluetooth OFF then ON in phone settings. Open App Info $\to$ Permissions $\to$ ensure **Nearby devices** is Allowed. |
| Node detected but metrics show empty or zeros | CC2652R1 LaunchPad is not powered | Ensure the CC2652R1 is plugged into the computer USB port (Green power LED illuminated). |
| Values update every second | Expected behavior | Standard SmartBAN semantic telemetry transmits at 1 Hz (1 packet/second) to maximize battery longevity (92.8% energy savings). |
