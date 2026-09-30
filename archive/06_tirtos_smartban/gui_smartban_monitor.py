"""
SmartBAN TI-RTOS7 Real-Time Testbed GUI Monitor & Telemetry Visualizer
Target Platform: TI CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5
Framework: PyQt6 / PyQt5 + pyqtgraph

Master Thesis Feature:
- Real-time physiological & kinematic dashboard
- Dual-mode streaming visualization (Semantic vs Raw Waveforms)
- High-performance scrolling plots for biopotentials & 3-axis IMU
- Interactive Serial CLI Console (STATUS, MODE RAW, MODE SEMANTIC)
- Clinical & experimental dataset recorder to CSV/JSON
"""

import sys
import os
import json
import time
import csv
from datetime import datetime
from collections import deque

try:
    from PyQt6 import QtCore, QtGui, QtWidgets
    from PyQt6.QtCore import pyqtSignal, pyqtSlot, QThread, QTimer, Qt
    PYQT_VER = 6
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread, QTimer, Qt
    PYQT_VER = 5

import pyqtgraph as pg
import serial
import serial.tools.list_ports


# =============================================================================
# Background Serial Ingestion Worker Thread
# =============================================================================
class SerialWorker(QThread):
    raw_line_received = pyqtSignal(str)
    semantic_received = pyqtSignal(dict)
    raw_sample_received = pyqtSignal(dict)
    status_changed = pyqtSignal(bool, str)

    def __init__(self, port="COM3", baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            self.status_changed.emit(True, f"Connected to {self.port} @ {self.baudrate} baud")
        except Exception as e:
            self.status_changed.emit(False, f"Connection Failed: {str(e)}")
            return

        buffer = ""
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1).decode("utf-8", errors="ignore")
                if data:
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if line:
                            self.raw_line_received.emit(line)
                            self._parse_line(line)
            except Exception as e:
                self.status_changed.emit(False, f"Serial Error: {str(e)}")
                break

        if self.ser and self.ser.is_open:
            self.ser.close()
        self.status_changed.emit(False, "Disconnected")

    def _parse_line(self, line: str):
        if line.startswith("{") and line.endswith("}"):
            try:
                payload = json.loads(line)
                p_type = payload.get("type", "")
                if p_type == "SEM":
                    self.semantic_received.emit(payload)
                elif p_type in ("RAW", "RAWB"):
                    self.raw_sample_received.emit(payload)
            except Exception:
                pass

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                full_cmd = (cmd.strip() + "\r\n").encode("utf-8")
                self.ser.write(full_cmd)
                self.ser.flush()
            except Exception as e:
                self.status_changed.emit(False, f"TX Error: {str(e)}")

    def stop(self):
        self.running = False
        self.wait(1000)


# =============================================================================
# Main Application Window
# =============================================================================
class SmartBanDashboard(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SmartBAN Intelligent Sensor Node — Real-Time Monitor (TI-RTOS7 / CC2652R1)")
        self.resize(1280, 850)
        self.setStyleSheet("""
            QMainWindow { background-color: #12141A; }
            QWidget { color: #E1E6EB; font-family: 'Segoe UI', Arial, sans-serif; }
            QGroupBox {
                border: 1px solid #2C3540;
                border-radius: 8px;
                margin-top: 12px;
                font-weight: bold;
                font-size: 13px;
                color: #58A6FF;
                padding-top: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }
            QLabel { font-size: 12px; }
            QLineEdit, QComboBox, QTextEdit {
                background-color: #1B212B;
                border: 1px solid #364252;
                border-radius: 4px;
                padding: 5px;
                color: #FFFFFF;
                font-family: 'Consolas', monospace;
            }
            QPushButton {
                background-color: #238636;
                border: none;
                border-radius: 5px;
                color: white;
                font-weight: bold;
                padding: 8px 16px;
                font-size: 12px;
            }
            QPushButton:hover { background-color: #2EA043; }
            QPushButton:pressed { background-color: #1A7F37; }
            QPushButton#btn_disconnect { background-color: #DA3633; }
            QPushButton#btn_disconnect:hover { background-color: #E5534B; }
            QPushButton#btn_record_on { background-color: #A371F7; }
        """)

        # Data buffers for plotting
        self.plot_len = 500
        self.ecg_data = deque(maxlen=self.plot_len)
        self.acc_x_data = deque(maxlen=self.plot_len)
        self.acc_y_data = deque(maxlen=self.plot_len)
        self.acc_z_data = deque(maxlen=self.plot_len)
        self.time_data = deque(maxlen=self.plot_len)

        # Recording state
        self.is_recording = False
        self.record_file = None
        self.csv_writer = None

        self.serial_worker = None
        self._init_ui()
        self._refresh_com_ports()

    def _init_ui(self):
        main_widget = QtWidgets.QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QtWidgets.QVBoxLayout(main_widget)
        main_layout.setContentsMargins(14, 14, 14, 14)
        main_layout.setSpacing(12)

        # Top Control Bar (Connection, Status, Quick Commands)
        ctrl_layout = QtWidgets.QHBoxLayout()

        lbl_port = QtWidgets.QLabel("Target Port:")
        self.combo_ports = QtWidgets.QComboBox()
        self.combo_ports.setMinimumWidth(180)

        btn_refresh = QtWidgets.QPushButton("Refresh")
        btn_refresh.setStyleSheet("background-color: #30363D; padding: 5px 10px;")
        btn_refresh.clicked.connect(self._refresh_com_ports)

        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self._toggle_connection)

        self.lbl_status = QtWidgets.QLabel("Status: Disconnected")
        self.lbl_status.setStyleSheet("color: #8B949E; font-weight: bold; margin-left: 10px;")

        ctrl_layout.addWidget(lbl_port)
        ctrl_layout.addWidget(self.combo_ports)
        ctrl_layout.addWidget(btn_refresh)
        ctrl_layout.addWidget(self.btn_connect)
        ctrl_layout.addWidget(self.lbl_status)
        ctrl_layout.addStretch()

        # Stream Mode Selector
        self.btn_mode_sem = QtWidgets.QPushButton("MODE: SEMANTIC (>90% Reduction)")
        self.btn_mode_sem.setStyleSheet("background-color: #1F6FEB;")
        self.btn_mode_sem.clicked.connect(lambda: self._send_cmd("MODE SEMANTIC"))

        self.btn_mode_raw = QtWidgets.QPushButton("MODE: RAW (100 Hz Streaming)")
        self.btn_mode_raw.setStyleSheet("background-color: #6E7681;")
        self.btn_mode_raw.clicked.connect(lambda: self._send_cmd("MODE RAW"))

        self.btn_record = QtWidgets.QPushButton("Record Dataset to CSV")
        self.btn_record.setStyleSheet("background-color: #8957E5;")
        self.btn_record.clicked.connect(self._toggle_recording)

        ctrl_layout.addWidget(self.btn_mode_sem)
        ctrl_layout.addWidget(self.btn_mode_raw)
        ctrl_layout.addWidget(self.btn_record)

        main_layout.addLayout(ctrl_layout)

        # Middle Section: Metric Cards Grid (ECG, Thermal, Optical, Kinematics)
        cards_layout = QtWidgets.QHBoxLayout()

        # Card 1: Cardiac Telemetry
        grp_ecg = QtWidgets.QGroupBox("Cardiac (TI ADS1292R)")
        ecg_form = QtWidgets.QGridLayout(grp_ecg)
        self.val_hr = self._create_card_val("72", "BPM", "#2EA043")
        self.val_rr = self._create_card_val("833", "ms", "#E1E6EB")
        self.val_hrv = self._create_card_val("38 / 42", "RMSSD/SDNN", "#7EE787")
        self.val_cardiac_flags = self._create_card_val("NORMAL", "Status", "#58A6FF")

        ecg_form.addWidget(QtWidgets.QLabel("Heart Rate:"), 0, 0)
        ecg_form.addWidget(self.val_hr, 0, 1)
        ecg_form.addWidget(QtWidgets.QLabel("R-to-R Interval:"), 1, 0)
        ecg_form.addWidget(self.val_rr, 1, 1)
        ecg_form.addWidget(QtWidgets.QLabel("HRV Metrics:"), 2, 0)
        ecg_form.addWidget(self.val_hrv, 2, 1)
        ecg_form.addWidget(QtWidgets.QLabel("Cardiac Alarm:"), 3, 0)
        ecg_form.addWidget(self.val_cardiac_flags, 3, 1)
        cards_layout.addWidget(grp_ecg)

        # Card 2: Temperature & Contact
        grp_temp = QtWidgets.QGroupBox("Thermal (MLX90632 FIR)")
        temp_form = QtWidgets.QGridLayout(grp_temp)
        self.val_skin_temp = self._create_card_val("36.4", "°C Target", "#F0883E")
        self.val_amb_temp = self._create_card_val("24.8", "°C Ambient", "#8B949E")
        self.val_contact = self._create_card_val("ON-BODY", "Coupling", "#58A6FF")
        self.val_thermal_class = self._create_card_val("Normothermia", "Class", "#7EE787")

        temp_form.addWidget(QtWidgets.QLabel("Skin Temp:"), 0, 0)
        temp_form.addWidget(self.val_skin_temp, 0, 1)
        temp_form.addWidget(QtWidgets.QLabel("Ambient Temp:"), 1, 0)
        temp_form.addWidget(self.val_amb_temp, 1, 1)
        temp_form.addWidget(QtWidgets.QLabel("Skin Contact:"), 2, 0)
        temp_form.addWidget(self.val_contact, 2, 1)
        temp_form.addWidget(QtWidgets.QLabel("Classification:"), 3, 0)
        temp_form.addWidget(self.val_thermal_class, 3, 1)
        cards_layout.addWidget(grp_temp)

        # Card 3: Optical Sensors (OPT4041 & VCNL4040)
        grp_opt = QtWidgets.QGroupBox("Optical Environment")
        opt_form = QtWidgets.QGridLayout(grp_opt)
        self.val_lux = self._create_card_val("450", "Lux (OPT4041)", "#E3B341")
        self.val_prox = self._create_card_val("7200", "Counts (VCNL)", "#D2A8FF")
        self.val_als = self._create_card_val("110", "ALS Counts", "#8B949E")
        self.val_power_red = self._create_card_val("96.7%", ">95% Thesis Goal", "#7EE787")

        opt_form.addWidget(QtWidgets.QLabel("Ambient Light:"), 0, 0)
        opt_form.addWidget(self.val_lux, 0, 1)
        opt_form.addWidget(QtWidgets.QLabel("Proximity PS:"), 1, 0)
        opt_form.addWidget(self.val_prox, 1, 1)
        opt_form.addWidget(QtWidgets.QLabel("Secondary ALS:"), 2, 0)
        opt_form.addWidget(self.val_als, 2, 1)
        opt_form.addWidget(QtWidgets.QLabel("Data Reduction:"), 3, 0)
        opt_form.addWidget(self.val_power_red, 3, 1)
        cards_layout.addWidget(grp_opt)

        # Card 4: Kinematics & Fall FSM (ADXL362)
        grp_imu = QtWidgets.QGroupBox("Motion & Fall (ADXL362)")
        imu_form = QtWidgets.QGridLayout(grp_imu)
        self.val_posture = self._create_card_val("SEDENTARY", "Posture", "#58A6FF")
        self.val_sma = self._create_card_val("0.04", "SMA (g)", "#E1E6EB")
        self.val_tilt = self._create_card_val("P: 2°  R: -1°", "Dynamic Tilt", "#E1E6EB")
        self.val_fall = self._create_card_val("CLEAR", "Impact FSM", "#2EA043")

        imu_form.addWidget(QtWidgets.QLabel("Activity State:"), 0, 0)
        imu_form.addWidget(self.val_posture, 0, 1)
        imu_form.addWidget(QtWidgets.QLabel("Dynamic SMA:"), 1, 0)
        imu_form.addWidget(self.val_sma, 1, 1)
        imu_form.addWidget(QtWidgets.QLabel("Torso Pitch/Roll:"), 2, 0)
        imu_form.addWidget(self.val_tilt, 2, 1)
        imu_form.addWidget(QtWidgets.QLabel("Fall Status:"), 3, 0)
        imu_form.addWidget(self.val_fall, 3, 1)
        cards_layout.addWidget(grp_imu)

        main_layout.addLayout(cards_layout)

        # Waveforms Section: pyqtgraph Real-Time Charts
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#161B22")

        # Top Plot: Biopotential ECG Channel
        self.p_ecg = self.plot_widget.addPlot(title="<span style='color:#58A6FF; font-weight:bold; font-size:12px;'>ADS1292R Biopotential Signal (Leadless Self-Test / Live Channel 1)</span>")
        self.p_ecg.showGrid(x=True, y=True, alpha=0.3)
        self.p_ecg.setLabel('left', 'Amplitude', units='uV')
        self.curve_ecg = self.p_ecg.plot(pen=pg.mkPen(color='#2EA043', width=2))

        self.plot_widget.nextRow()

        # Bottom Plot: 3-Axis Kinematic Acceleration
        self.p_imu = self.plot_widget.addPlot(title="<span style='color:#58A6FF; font-weight:bold; font-size:12px;'>ADXL362 3-Axis Acceleration (X: Red, Y: Green, Z: Blue @ 100 Hz)</span>")
        self.p_imu.showGrid(x=True, y=True, alpha=0.3)
        self.p_imu.setLabel('left', 'Acceleration', units='mg')
        self.curve_acc_x = self.p_imu.plot(pen=pg.mkPen(color='#FA7970', width=1.5), name='X-Axis')
        self.curve_acc_y = self.p_imu.plot(pen=pg.mkPen(color='#7CE38B', width=1.5), name='Y-Axis')
        self.curve_acc_z = self.p_imu.plot(pen=pg.mkPen(color='#77BDFB', width=1.5), name='Z-Axis')

        main_layout.addWidget(self.plot_widget, stretch=4)

        # Bottom Section: Interactive CLI Console & Log Viewer
        cli_layout = QtWidgets.QHBoxLayout()

        self.txt_log = QtWidgets.QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(130)
        self.txt_log.setStyleSheet("background-color: #0D1117; font-size: 11px; color: #8B949E;")

        cmd_panel = QtWidgets.QVBoxLayout()
        self.input_cmd = QtWidgets.QLineEdit()
        self.input_cmd.setPlaceholderText("Type CLI command (e.g., STATUS, HELP, RESET, MODE RAW)...")
        self.input_cmd.returnPressed.connect(self._send_custom_cmd)

        btn_send = QtWidgets.QPushButton("Send Command")
        btn_send.clicked.connect(self._send_custom_cmd)

        btn_status = QtWidgets.QPushButton("STATUS")
        btn_status.setStyleSheet("background-color: #30363D;")
        btn_status.clicked.connect(lambda: self._send_cmd("STATUS"))

        btn_reset = QtWidgets.QPushButton("RESET BOARD")
        btn_reset.setStyleSheet("background-color: #8B1A1A;")
        btn_reset.clicked.connect(lambda: self._send_cmd("RESET"))

        cmd_panel.addWidget(self.input_cmd)
        cmd_panel.addWidget(btn_send)
        cmd_panel.addWidget(btn_status)
        cmd_panel.addWidget(btn_reset)

        cli_layout.addWidget(self.txt_log, stretch=3)
        cli_layout.addLayout(cmd_panel, stretch=1)

        main_layout.addLayout(cli_layout)

        # Timer for smooth 30 FPS plot rendering
        self.render_timer = QTimer()
        self.render_timer.timeout.connect(self._render_plots)
        self.render_timer.start(33)

    def _create_card_val(self, val_str, sub_str, color):
        lbl = QtWidgets.QLabel(f"<span style='font-size:18px; font-weight:bold; color:{color};'>{val_str}</span> "
                               f"<span style='font-size:11px; color:#8B949E;'>{sub_str}</span>")
        lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter if PYQT_VER == 6
                         else Qt.AlignRight | Qt.AlignVCenter)
        return lbl

    def _refresh_com_ports(self):
        self.combo_ports.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.combo_ports.addItem(f"{p.device} - {p.description}", p.device)
        # Default to COM3 if found
        for i in range(self.combo_ports.count()):
            if "COM3" in self.combo_ports.itemText(i):
                self.combo_ports.setCurrentIndex(i)
                break

    def _toggle_connection(self):
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.stop()
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("background-color: #238636;")
        else:
            port = self.combo_ports.currentData()
            if not port:
                self.lbl_status.setText("Status: No COM port selected")
                return
            self.serial_worker = SerialWorker(port=port, baudrate=115200)
            self.serial_worker.raw_line_received.connect(self._on_raw_line)
            self.serial_worker.semantic_received.connect(self._on_semantic_frame)
            self.serial_worker.raw_sample_received.connect(self._on_raw_sample)
            self.serial_worker.status_changed.connect(self._on_status_changed)
            self.serial_worker.start()
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet("background-color: #DA3633;")

    @pyqtSlot(bool, str)
    def _on_status_changed(self, ok, msg):
        color = "#3FB950" if ok else "#F85149"
        self.lbl_status.setText(f"Status: <span style='color:{color}; font-weight:bold;'>{msg}</span>")

    @pyqtSlot(str)
    def _on_raw_line(self, line: str):
        # Append to log view
        self.txt_log.append(line)
        if self.is_recording and self.csv_writer:
            try:
                self.csv_writer.writerow([datetime.now().isoformat(), "RAW_LINE", line])
            except Exception:
                pass

    @pyqtSlot(dict)
    def _on_semantic_frame(self, tok: dict):
        # Update Telemetry Metric Cards
        hr = tok.get("hr", 72)
        rr = tok.get("rr", 0)
        rmssd = tok.get("rmssd", 0)
        sdnn = tok.get("sdnn", 0)
        posture = tok.get("posture", "SEDENTARY")
        fall = tok.get("fall", 0)
        contact = tok.get("contact", 0)
        temp = tok.get("temp", 0.0)
        lux = tok.get("lux", 0)

        # Cardiac
        hr_color = "#F85149" if (hr > 100 or (hr < 50 and hr > 0)) else "#2EA043"
        self.val_hr.setText(f"<span style='font-size:18px; font-weight:bold; color:{hr_color};'>{hr}</span> <span style='color:#8B949E; font-size:11px;'>BPM</span>")
        self.val_rr.setText(f"<span style='font-size:18px; font-weight:bold;'>{rr}</span> <span style='color:#8B949E; font-size:11px;'>ms</span>")
        self.val_hrv.setText(f"<span style='font-size:18px; font-weight:bold;'>{rmssd} / {sdnn}</span> <span style='color:#8B949E; font-size:11px;'>ms</span>")

        # Thermal
        temp_color = "#F85149" if temp > 38.0 else ("#58A6FF" if temp < 35.0 and contact else "#F0883E")
        self.val_skin_temp.setText(f"<span style='font-size:18px; font-weight:bold; color:{temp_color};'>{temp:.1f}</span> <span style='color:#8B949E; font-size:11px;'>°C Target</span>")
        contact_str = "ON-BODY" if contact else "DETACHED"
        contact_col = "#2EA043" if contact else "#8B949E"
        self.val_contact.setText(f"<span style='font-size:18px; font-weight:bold; color:{contact_col};'>{contact_str}</span>")

        # Optical
        self.val_lux.setText(f"<span style='font-size:18px; font-weight:bold; color:#E3B341;'>{lux}</span> <span style='color:#8B949E; font-size:11px;'>Lux</span>")

        # Posture & Fall
        fall_str = "<span style='color:#F85149; font-weight:bold; font-size:18px;'>FALL DETECTED</span>" if fall else "<span style='color:#2EA043; font-weight:bold; font-size:18px;'>CLEAR</span>"
        self.val_fall.setText(fall_str)
        self.val_posture.setText(f"<span style='font-size:18px; font-weight:bold; color:#58A6FF;'>{posture}</span>")

        # Record structured entry if active
        if self.is_recording and self.csv_writer:
            try:
                self.csv_writer.writerow([
                    datetime.now().isoformat(), "SEMANTIC",
                    tok.get("ts", 0), hr, rr, rmssd, sdnn, posture, fall, contact, temp, lux
                ])
            except Exception:
                pass

    @pyqtSlot(dict)
    def _on_raw_sample(self, sample: dict):
        p_type = sample.get("type", "")
        ts = sample.get("ts", 0)

        if p_type == "RAW":
            ecg = sample.get("ecg", 0)
            ax = sample.get("ax", 0)
            ay = sample.get("ay", 0)
            az = sample.get("az", 0)
            self.ecg_data.append(ecg)
            self.acc_x_data.append(ax)
            self.acc_y_data.append(ay)
            self.acc_z_data.append(az)
            self.time_data.append(ts)
        elif p_type == "RAWB":
            batch = sample.get("ecg_batch", [])
            ax = sample.get("ax", 0)
            ay = sample.get("ay", 0)
            az = sample.get("az", 0)
            for v in batch:
                self.ecg_data.append(v)
                self.acc_x_data.append(ax)
                self.acc_y_data.append(ay)
                self.acc_z_data.append(az)
                self.time_data.append(ts)

    def _render_plots(self):
        if len(self.ecg_data) > 0:
            self.curve_ecg.setData(list(self.ecg_data))
        if len(self.acc_x_data) > 0:
            self.curve_acc_x.setData(list(self.acc_x_data))
            self.curve_acc_y.setData(list(self.acc_y_data))
            self.curve_acc_z.setData(list(self.acc_z_data))

    def _send_custom_cmd(self):
        cmd = self.input_cmd.text().strip()
        if cmd:
            self._send_cmd(cmd)
            self.input_cmd.clear()

    def _send_cmd(self, cmd: str):
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.send_command(cmd)
            self.txt_log.append(f"<span style='color:#58A6FF;'>&gt;&gt; {cmd}</span>")
        else:
            self.txt_log.append("<span style='color:#F85149;'>Error: Serial port not connected</span>")

    def _toggle_recording(self):
        if not self.is_recording:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"smartban_telemetry_{timestamp}.csv"
            filepath = os.path.join(os.getcwd(), filename)
            try:
                self.record_file = open(filepath, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.record_file)
                self.csv_writer.writerow([
                    "SystemTime", "Type", "NodeTimestamp", "HeartRate", "RR_Interval",
                    "RMSSD", "SDNN", "Posture", "FallDetected", "SkinContact", "Temperature", "Lux"
                ])
                self.is_recording = True
                self.btn_record.setText(f"Recording: {filename}")
                self.btn_record.setStyleSheet("background-color: #DA3633;")
                self.txt_log.append(f"<span style='color:#A371F7;'>Started session recording: {filepath}</span>")
            except Exception as e:
                self.txt_log.append(f"<span style='color:#F85149;'>Failed to open recording file: {str(e)}</span>")
        else:
            self.is_recording = False
            if self.record_file:
                self.record_file.close()
                self.record_file = None
            self.csv_writer = None
            self.btn_record.setText("Record Dataset to CSV")
            self.btn_record.setStyleSheet("background-color: #8957E5;")
            self.txt_log.append("<span style='color:#A371F7;'>Session recording stopped and saved.</span>")

    def closeEvent(self, event):
        if self.serial_worker and self.serial_worker.isRunning():
            self.serial_worker.stop()
        if self.record_file:
            self.record_file.close()
        event.accept()


# =============================================================================
# Entry Point
# =============================================================================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SmartBanDashboard()
    window.show()
    sys.exit(app.exec() if PYQT_VER == 6 else app.exec_())
