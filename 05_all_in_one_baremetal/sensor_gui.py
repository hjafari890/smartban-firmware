"""
SmartBAN Multi-Modal Clinical & Aerospace Sensor Dashboard
Advanced ECG Engine:
  - Clinical 250 Hz Telemetry with ADS1292R Lead-Off Sensing
  - Physical Millivolt (mV) Calibrated Signal Processing
  - 3-Stage Cascaded Filtering (0.5Hz Highpass + 40Hz Lowpass + 50/60Hz Notch)
  - Gold-Standard Pan-Tompkins QRS R-Peak Detection (Dual Adaptive Thresholds)
  - R-Peak Waveform Overlays & Heart Pulse Indicator
  - Thoracic Impedance Respiration Pneumography & Rate (RPM)
  - Lead-Off Contact Status Badges (RA / LA)
  - Airplane Attitude & Attack Indicator (Primary Flight Display / Artificial Horizon)
  - Pitch, Roll, and Angle of Attack (AoA) Real-Time Instrumentation
  - Complete Environmental Suite (BME680, OPT4041, VCNL4040, MLX90632)
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import json
import threading
import collections
import time
import math
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

MODE_NAMES = {
    1: "INTERNAL 1Hz SQUARE WAVE (+/-1mV)",
    2: "INPUT SHORTED (Noise & Offset Test)",
    3: "INTERNAL DIE TEMPERATURE SENSOR",
    4: "LIVE ELECTRODES (Lead I Stream)"
}

# =========================================================================
# DSP Biquad Filters (Direct Form II Transposed / Canonical Form)
# =========================================================================

class BiquadFilter:
    """
    Direct-Form Transposed-II Second-Order IIR Biquad Section with
    amplitude clamping to prevent large open-circuit transients from destabilizing filters.
    """
    def __init__(self, b, a):
        self.b0, self.b1, self.b2 = float(b[0]), float(b[1]), float(b[2])
        self.a1, self.a2 = float(a[1]), float(a[2])
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0
        self.initialized = False

    def process(self, x):
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        x = max(-500.0, min(500.0, float(x)))
        if not self.initialized:
            self.initialized = True
            self.x1 = self.x2 = x
            self.y1 = self.y2 = 0.0

        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        if math.isnan(y) or math.isinf(y):
            y = 0.0
            self.reset()
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def reset(self):
        self.initialized = False
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0


def design_highpass_biquad(fc, fs):
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / np.sqrt(2.0)
    cos_w0 = np.cos(w0)
    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return BiquadFilter([b0/a0, b1/a0, b2/a0], [1.0, a1/a0, a2/a0])


def design_lowpass_biquad(fc, fs):
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / np.sqrt(2.0)
    cos_w0 = np.cos(w0)
    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return BiquadFilter([b0/a0, b1/a0, b2/a0], [1.0, a1/a0, a2/a0])


def design_notch_biquad(fc, fs, Q=30.0):
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / (2.0 * Q)
    cos_w0 = np.cos(w0)
    b0 = 1.0
    b1 = -2.0 * cos_w0
    b2 = 1.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return BiquadFilter([b0/a0, b1/a0, b2/a0], [1.0, a1/a0, a2/a0])


# =========================================================================
# Gold-Standard Pan-Tompkins QRS R-Peak Detector
# =========================================================================

class PanTompkinsQRS:
    """
    Enhanced Pan-Tompkins real-time QRS detector with dual-threshold adaptation,
    searchback for missed beats, and 450ms refractory blanking to reject prominent T-waves.
    """
    def __init__(self, fs=200.0):
        self.fs = fs
        # 120 ms Moving Window Integrator
        self.mwi_len = max(int(0.12 * fs), 4)
        self.mwi_buf = collections.deque(maxlen=self.mwi_len)
        self.mwi_sum = 0.0

        # Derivative buffer (5-point)
        self.x_buf = collections.deque([0.0] * 5, maxlen=5)

        # Dual Adaptive Thresholds (Signal and Noise running estimates in MWI units)
        self.spki = 0.0025
        self.npki = 0.00004
        self.threshold1 = 0.00035
        self.threshold2 = 0.00018

        # Timing & Refractory: 420ms blanking prevents false triggers on prominent T-waves (covers up to 142 BPM)
        self.refractory_samples = int(0.42 * fs)
        self.samples_since_peak = self.refractory_samples + 1
        self.rr_intervals = collections.deque(maxlen=8)
        self.bpm = 0.0
        self.beat_detected = False

    def update_fs(self, new_fs):
        if 100.0 <= new_fs <= 350.0:
            self.fs = new_fs
            self.mwi_len = max(int(0.12 * new_fs), 4)
            self.refractory_samples = int(0.42 * new_fs)

    def process(self, ecg_mv):
        """
        Process a single filtered ECG sample (in millivolts).
        Returns True if an R-peak was detected on this sample.
        """
        self.x_buf.append(ecg_mv)
        if len(self.x_buf) < 5:
            return False

        # 1. Five-point centered derivative:
        # H(z) = (2 + z^-1 - z^-3 - 2z^-4) / (8*T)
        deriv = (2.0*self.x_buf[4] + self.x_buf[3] - self.x_buf[1] - 2.0*self.x_buf[0]) / 8.0

        # 2. Non-linear Squaring (amplifies QRS steep slope over P and T waves)
        sq = deriv * deriv

        # 3. Moving Window Integration (MWI)
        if len(self.mwi_buf) == self.mwi_len:
            self.mwi_sum -= self.mwi_buf[0]
        self.mwi_buf.append(sq)
        self.mwi_sum += sq
        mwi = self.mwi_sum / self.mwi_len

        self.samples_since_peak += 1
        r_detected = False

        # Expected RR interval for searchback
        valid_rr_hist = [r for r in self.rr_intervals if 0.40 <= (r / self.fs) <= 1.8]
        expected_rr = np.median(valid_rr_hist) if len(valid_rr_hist) >= 2 else (0.85 * self.fs)

        # 4. Decision Rule with Refractory Period and Searchback
        if self.samples_since_peak > self.refractory_samples:
            thresh = self.threshold1
            # Searchback condition: if no beat detected for 1.4x expected interval, drop threshold
            if self.samples_since_peak > int(1.4 * expected_rr):
                thresh = self.threshold2

            if mwi > thresh:
                r_detected = True
                prev_interval = self.samples_since_peak
                self.samples_since_peak = 0

                # Update Signal Peak estimate with exponential smoothing
                self.spki = 0.125 * mwi + 0.875 * self.spki

                # Record RR interval
                if prev_interval > 0:
                    self.rr_intervals.append(prev_interval)

                # Compute robust BPM from median RR interval
                valid_rr = [r for r in self.rr_intervals if 0.42 <= (r / self.fs) <= 1.8]
                if valid_rr:
                    med_rr = float(np.median(valid_rr))
                    calc_bpm = 60.0 * self.fs / med_rr
                    if 35 <= calc_bpm <= 140:
                        if self.bpm == 0.0:
                            self.bpm = calc_bpm
                        else:
                            self.bpm = 0.25 * calc_bpm + 0.75 * self.bpm
            else:
                # Update Noise Peak estimate on sub-threshold local maxima
                if mwi > self.npki:
                    self.npki = 0.125 * mwi + 0.875 * self.npki

        # 5. Adapt Thresholds
        self.threshold1 = self.npki + 0.25 * (self.spki - self.npki)
        self.threshold2 = 0.5 * self.threshold1

        # Prevent threshold collapse or runaway
        if self.threshold1 < 0.00003:
            self.threshold1 = 0.00003
        if self.spki < self.threshold1 * 1.2:
            self.spki = self.threshold1 * 1.5

        self.beat_detected = r_detected
        return r_detected


# =========================================================================
# Respiration Rate & ECG-Derived Respiration (EDR)
# =========================================================================

class RespRateDetector:
    def __init__(self, fs=200.0):
        self.fs = fs
        self.prev_sign = 0
        self.samples_since_cross = 0
        self.half_periods = collections.deque(maxlen=16)
        self.rpm = 0.0
        self.min_half_period = int(0.6 * fs)
        self.max_half_period = int(5.0 * fs)

    def update_fs(self, new_fs):
        if 100.0 <= new_fs <= 350.0:
            self.fs = new_fs
            self.min_half_period = int(0.6 * new_fs)
            self.max_half_period = int(5.0 * new_fs)

    def process(self, filtered_sample):
        self.samples_since_cross += 1
        sign = 1 if filtered_sample >= 0 else -1

        if sign != self.prev_sign and self.prev_sign != 0:
            if self.min_half_period <= self.samples_since_cross <= self.max_half_period:
                self.half_periods.append(self.samples_since_cross)
                self.samples_since_cross = 0

                if len(self.half_periods) >= 3:
                    avg_full_period = 2.0 * float(np.median(list(self.half_periods)))
                    if avg_full_period > 0:
                        calc_rpm = 60.0 * self.fs / avg_full_period
                        if 6 <= calc_rpm <= 50:
                            if self.rpm == 0.0:
                                self.rpm = calc_rpm
                            else:
                                self.rpm = 0.20 * calc_rpm + 0.80 * self.rpm

        self.prev_sign = sign


# =========================================================================
# Airplane Attitude & Attack Indicator Widget (Glass Cockpit PFD)
# =========================================================================

class AirplaneAttitudeWidget(tk.Frame):
    def __init__(self, parent, width=320, height=310):
        super().__init__(parent, bg="#121214", width=width, height=height)
        self.w = width
        self.h = height
        self.cx = width // 2
        self.cy = height // 2
        self.r = min(width, height) // 2 - 14

        self.canvas = tk.Canvas(self, width=self.w, height=self.h, bg="#121214", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.roll = 0.0
        self.pitch = 0.0
        self.aoa = 0.0
        self.g_total = 1.0

        self.render()

    def update_attitude(self, roll, pitch, aoa, g_total):
        if (abs(self.roll - roll) < 0.2 and 
            abs(self.pitch - pitch) < 0.2 and 
            abs(self.aoa - aoa) < 0.2 and 
            abs(self.g_total - g_total) < 0.05):
            return
        self.roll = roll
        self.pitch = pitch
        self.aoa = aoa
        self.g_total = g_total
        self.render()

    def render(self):
        self.canvas.delete("all")
        cx, cy, r = self.cx, self.cy, self.r

        # Bezel Outer Ring
        self.canvas.create_oval(cx-r-10, cy-r-10, cx+r+10, cy+r+10, fill="#1c1d22", outline="#2b2d35", width=3)
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill="#0288d1", outline="")

        # Horizon geometry
        rad = math.radians(self.roll)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)

        pitch_clamped = max(-60.0, min(60.0, self.pitch))
        pitch_px = pitch_clamped * 2.2

        hx = cx - sin_r * pitch_px
        hy = cy + cos_r * pitch_px

        # Ground Polygon
        big_d = r * 3
        x1 = hx - cos_r * big_d
        y1 = hy - sin_r * big_d
        x2 = hx + cos_r * big_d
        y2 = hy + sin_r * big_d
        x3 = x2 - sin_r * big_d
        y3 = y2 + cos_r * big_d
        x4 = x1 - sin_r * big_d
        y4 = y1 + cos_r * big_d

        self.canvas.create_polygon([x1, y1, x2, y2, x3, y3, x4, y4], fill="#4e342e", outline="")
        self.canvas.create_line(x1, y1, x2, y2, fill="#ffffff", width=3)

        # Pitch Ladder
        for deg in [20, 10, -10, -20]:
            p_dist = (pitch_clamped - deg) * 2.2
            lx = cx - sin_r * p_dist
            ly = cy + cos_r * p_dist
            lw = 30 if abs(deg) == 10 else 46

            lx1 = lx - cos_r * lw
            ly1 = ly - sin_r * lw
            lx2 = lx + cos_r * lw
            ly2 = ly + sin_r * lw

            self.canvas.create_line(lx1, ly1, lx2, ly2, fill="#ffffff", width=1.5)
            self.canvas.create_text(lx1 - cos_r*10, ly1 - sin_r*10, text=f"{abs(deg)}",
                                    fill="#dddddd", font=("Segoe UI", 7, "bold"))

        # Clip mask corners
        self.canvas.create_rectangle(0, 0, self.w, cy - r, fill="#121214", outline="")
        self.canvas.create_rectangle(0, cy + r, self.w, self.h, fill="#121214", outline="")
        self.canvas.create_rectangle(0, 0, cx - r, self.h, fill="#121214", outline="")
        self.canvas.create_rectangle(cx + r, 0, self.w, self.h, fill="#121214", outline="")

        # Bezel rim
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline="#3d404d", width=4)
        self.canvas.create_oval(cx-r-8, cy-r-8, cx+r+8, cy+r+8, outline="#16171a", width=2)

        # Roll Pointer
        for tick in [-60, -30, 0, 30, 60]:
            t_rad = math.radians(tick - 90)
            tx1 = cx + (r - 2) * math.cos(t_rad)
            ty1 = cy + (r - 2) * math.sin(t_rad)
            tx2 = cx + (r - 12) * math.cos(t_rad)
            ty2 = cy + (r - 12) * math.sin(t_rad)
            self.canvas.create_line(tx1, ty1, tx2, ty2, fill="#ffffff" if tick == 0 else "#ffea00", width=2)

        # Aircraft Reference Symbol
        y_col = "#ffb703"
        self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill=y_col, outline="#000000", width=1.5)
        self.canvas.create_line(cx-50, cy, cx-16, cy, fill=y_col, width=5)
        self.canvas.create_line(cx-16, cy, cx-16, cy+10, fill=y_col, width=5)
        self.canvas.create_line(cx-50, cy, cx-16, cy, fill="#000000", width=1)
        self.canvas.create_line(cx+16, cy, cx+50, cy, fill=y_col, width=5)
        self.canvas.create_line(cx+16, cy, cx+16, cy+10, fill=y_col, width=5)
        self.canvas.create_line(cx+16, cy, cx+50, cy, fill="#000000", width=1)

        # HUD Text Readouts
        self.canvas.create_text(20, 16, text=f"ROLL: {self.roll:+.1f}°",
                                anchor="w", fill="#00f5d4", font=("Consolas", 10, "bold"))
        self.canvas.create_text(self.w - 20, 16, text=f"PITCH: {self.pitch:+.1f}°",
                                anchor="e", fill="#fee440", font=("Consolas", 10, "bold"))
        self.canvas.create_text(20, self.h - 16, text=f"AoA: {self.aoa:.1f}°",
                                anchor="w", fill="#ff758f", font=("Consolas", 10, "bold"))
        self.canvas.create_text(self.w - 20, self.h - 16, text=f"G: {self.g_total:.2f}g",
                                anchor="e", fill="#a8dadc", font=("Consolas", 10, "bold"))


# =========================================================================
# Main Dashboard Window
# =========================================================================

class SensorDashboardApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SmartBAN Clinical ECG & Aerospace Telemetry Visualizer")
        self.root.geometry("1300x900")
        self.root.minsize(1150, 780)

        self.ser = None
        self.running = False
        self.port_var = tk.StringVar(value="COM3")
        self.baud_var = tk.IntVar(value=115200)

        # Telemetry: 250 Hz for ECG/Resp, 25 Hz for IMU
        self.fs_ecg = 250.0
        self.pts_ecg = int(5.0 * self.fs_ecg)   # 5 seconds = 1250 samples
        self.pts_resp = int(20.0 * self.fs_ecg) # 20 seconds
        self.pts_imu = 250                      # 10 seconds at 25 Hz

        # Time base for ECG plot in seconds
        self.time_ecg = np.linspace(-5.0, 0.0, self.pts_ecg)

        self.ecg_filt_buf = collections.deque([0.0]*self.pts_ecg, maxlen=self.pts_ecg)
        self.resp_filt_buf = collections.deque([0.0]*self.pts_resp, maxlen=self.pts_resp)
        self.rpeak_x = collections.deque(maxlen=30)
        self.rpeak_y = collections.deque(maxlen=30)

        self.imu_x_buf = collections.deque([0.0]*self.pts_imu, maxlen=self.pts_imu)
        self.imu_y_buf = collections.deque([0.0]*self.pts_imu, maxlen=self.pts_imu)
        self.imu_z_buf = collections.deque([1.0]*self.pts_imu, maxlen=self.pts_imu)

        # Clinical 3-Stage Cascaded Biquad Filter Pipeline
        self.notch_freq = 50.0
        self.ecg_hp = design_highpass_biquad(0.5, self.fs_ecg)
        self.ecg_lp = design_lowpass_biquad(35.0, self.fs_ecg)
        self.ecg_notch = design_notch_biquad(self.notch_freq, self.fs_ecg, Q=30.0)

        # Respiration Filter
        self.resp_hp = design_highpass_biquad(0.05, self.fs_ecg)
        self.resp_lp = design_lowpass_biquad(0.8, self.fs_ecg)

        # Pan-Tompkins & Respiration Detectors
        self.qrs_detector = PanTompkinsQRS(fs=self.fs_ecg)
        self.resp_detector = RespRateDetector(fs=self.fs_ecg)
        self.prev_ecg_mv = 0.0

        # Real-time Sample Rate Tracking & Dynamic Filter Adaptation
        self.sample_times = collections.deque(maxlen=60)
        self.last_sample_t = 0.0
        self.mode1_raw_buf = collections.deque(maxlen=int(2.0 * self.fs_ecg))

        # State Variables
        self.sample_index = 0
        self.beat_pulse_timer = 0
        self.lead_status = 192
        self.ra_connected = True
        self.la_connected = True

        # 04_ecg_leadless_test Real-Time Serial Monitor State
        self.mon_lines_queue = collections.deque(maxlen=100)
        self.mon_sample_count = 0
        self.mon_stat_count = 0
        self.mon_ch1_min = 100000.0
        self.mon_ch1_max = -100000.0
        self.mon_ch2_min = 100000.0
        self.mon_ch2_max = -100000.0
        self.mon_ch1_sum = 0.0
        self.mon_ch2_sum = 0.0
        self.latest_ch1_uv = 0.0
        self.latest_ch2_uv = 0.0
        self.latest_stat = 192

        self.roll_val = 0.0
        self.pitch_val = 0.0
        self.aoa_val = 0.0
        self.g_val = 1.0

        self.env_data = {
            "T": 0.0, "H": 0.0, "P": 0.0, "lux": 0.0, "prox": 0, "mlx": 0.0, "mode": 4
        }
        self.imu_attack_state = 0
        self.active_mode = 4
        self.last_rendered_mode = None
        self.tick_count = 0
        self.txt_mon_line_count = 0
        self.mode1_dc = 0.0
        self.invert_ecg = tk.BooleanVar(value=False)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(300, self._auto_connect)

    def _make_ascii_bar(self, uv):
        bar_half = 7
        pos = int(uv / 800.0)
        pos = max(-bar_half, min(bar_half, pos))
        s = [' '] * (2 * bar_half + 1)
        s[bar_half] = '|'
        if pos > 0:
            for i in range(1, pos + 1): s[bar_half + i] = '#'
        elif pos < 0:
            for i in range(-1, pos - 1, -1): s[bar_half + i] = '#'
        return '[' + ''.join(s) + ']'

    def _reconfigure_filters(self):
        self.ecg_hp = design_highpass_biquad(0.5, self.fs_ecg)
        self.ecg_lp = design_lowpass_biquad(35.0, self.fs_ecg)
        self.ecg_notch = design_notch_biquad(self.notch_freq, self.fs_ecg, Q=25.0)
        self.resp_hp = design_highpass_biquad(0.05, self.fs_ecg)
        self.resp_lp = design_lowpass_biquad(0.8, self.fs_ecg)
        self.qrs_detector.update_fs(self.fs_ecg)
        self.resp_detector.update_fs(self.fs_ecg)

    def _build_ui(self):
        # 1. Top Bar: Port, Connection, Lead Status, Attack Banner, ECG Modes
        top_bar = tk.Frame(self.root, bg="#16171b", pady=6, padx=12)
        top_bar.pack(side=tk.TOP, fill=tk.X)

        conn_frame = tk.Frame(top_bar, bg="#16171b")
        conn_frame.pack(side=tk.LEFT)

        tk.Label(conn_frame, text="Port:", font=("Segoe UI", 9, "bold"), fg="#ffffff", bg="#16171b").pack(side=tk.LEFT, padx=3)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=8)
        self._refresh_ports()
        self.port_combo.pack(side=tk.LEFT, padx=3)

        self.btn_connect = tk.Button(conn_frame, text="Connect", font=("Segoe UI", 9, "bold"), bg="#28a745", fg="white",
                                     relief=tk.FLAT, padx=10, command=self._toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=5)

        # Electrode Contact Quality Badges
        lead_frame = tk.Frame(top_bar, bg="#16171b")
        lead_frame.pack(side=tk.LEFT, padx=10)

        self.lbl_ra = tk.Label(lead_frame, text="● RA: OK", font=("Segoe UI", 9, "bold"), fg="#28a745", bg="#22252c", padx=6, pady=2, relief=tk.SOLID, bd=1)
        self.lbl_ra.pack(side=tk.LEFT, padx=3)

        self.lbl_la = tk.Label(lead_frame, text="● LA: OK", font=("Segoe UI", 9, "bold"), fg="#28a745", bg="#22252c", padx=6, pady=2, relief=tk.SOLID, bd=1)
        self.lbl_la.pack(side=tk.LEFT, padx=3)

        # Notch Frequency Toggle
        self.notch_var = tk.StringVar(value="50 Hz Notch")
        notch_btn = ttk.Combobox(lead_frame, textvariable=self.notch_var, values=["50 Hz Notch", "60 Hz Notch"], width=11, state="readonly")
        notch_btn.bind("<<ComboboxSelected>>", self._on_notch_change)
        notch_btn.pack(side=tk.LEFT, padx=5)

        chk_inv = tk.Checkbutton(lead_frame, text="Invert", variable=self.invert_ecg,
                                 font=("Segoe UI", 8), fg="#dddddd", bg="#16171b",
                                 selectcolor="#2a2c36", activebackground="#16171b", activeforeground="white")
        chk_inv.pack(side=tk.LEFT, padx=3)

        # Attack / Motion Banner
        self.lbl_attack = tk.Label(top_bar, text="● STILL / RESTING", font=("Segoe UI", 10, "bold"),
                                   bg="#28a745", fg="white", width=22, relief=tk.RIDGE, bd=2)
        self.lbl_attack.pack(side=tk.LEFT, padx=10)

        # Mode Buttons
        mode_frame = tk.Frame(top_bar, bg="#16171b")
        mode_frame.pack(side=tk.RIGHT)

        tk.Label(mode_frame, text="ECG Modes:", font=("Segoe UI", 9, "bold"), fg="#aaa", bg="#16171b").pack(side=tk.LEFT, padx=4)

        self.mode_btns = {}
        modes = [
            ("1: Square Wave (+/-1mV)", '1', 1),
            ("2: Input Short (Noise & Offset)", '2', 2),
            ("3: Die Temp", '3', 3),
            ("4: Live Electrodes", '4', 4)
        ]
        for label, cmd, mode_id in modes:
            b = tk.Button(mode_frame, text=label, font=("Segoe UI", 8, "bold"), bg="#3a3d40", fg="white",
                          relief=tk.RAISED, bd=1, padx=6, pady=2,
                          command=lambda c=cmd, m=mode_id: self._select_mode(c, m))
            b.pack(side=tk.LEFT, padx=2)
            self.mode_btns[mode_id] = b

        # 2. Vitals & Avionics Telemetry Dashboard Cards
        dash_frame = tk.Frame(self.root, bg="#1f2026", pady=6, padx=10)
        dash_frame.pack(side=tk.TOP, fill=tk.X)

        def make_card(parent, title, val_init, unit, col, fg_color="#ffffff"):
            f = tk.Frame(parent, bg="#2a2c36", padx=8, pady=4, relief=tk.SOLID, bd=1)
            f.grid(row=0, column=col, padx=3, pady=2, sticky="nsew")
            tk.Label(f, text=title, font=("Segoe UI", 8), fg="#9da3b4", bg="#2a2c36").pack(anchor="w")
            lbl_val = tk.Label(f, text=val_init, font=("Segoe UI", 16, "bold"), fg=fg_color, bg="#2a2c36")
            lbl_val.pack(anchor="w")
            tk.Label(f, text=unit, font=("Segoe UI", 7), fg="#9da3b4", bg="#2a2c36").pack(anchor="e")
            parent.columnconfigure(col, weight=1)
            return lbl_val

        # Clinical Cards
        self.card_hr    = make_card(dash_frame, "❤️ Heart Rate", "--", "BPM (Pan-Tompkins)", 0, "#ff4d6d")
        self.card_resp  = make_card(dash_frame, "🫁 Respiration", "--", "RPM (Pneumography)", 1, "#48cae4")

        # Flight & IMU Cards
        self.card_pitch = make_card(dash_frame, "Pitch Angle", "--", "Deg (°)", 2, "#fee440")
        self.card_roll  = make_card(dash_frame, "Roll Angle", "--", "Deg (°)", 3, "#00f5d4")
        self.card_aoa   = make_card(dash_frame, "Attack / Tilt", "--", "Deg (°)", 4, "#ff758f")

        # Environmental Cards
        self.card_temp  = make_card(dash_frame, "Ambient Temp", "--", "°C (BME680)", 5, "#f77f00")
        self.card_hum   = make_card(dash_frame, "Humidity", "--", "% RH", 6, "#a8dadc")
        self.card_press = make_card(dash_frame, "Pressure", "--", "hPa", 7, "#90e0ef")
        self.card_lux   = make_card(dash_frame, "Ambient Light", "--", "Lux (OPT4041)", 8, "#fcbf49")
        self.card_prox  = make_card(dash_frame, "Proximity", "--", "Counts (VCNL)", 9, "#e0aaff")
        self.card_mlx   = make_card(dash_frame, "IR Object Temp", "--", "°C (MLX90632)", 10, "#ff9ebb")

        # 3. Lower Workspace: Left = Waveforms, Right = Airplane Attitude Indicator
        main_content = tk.Frame(self.root, bg="#121214")
        main_content.pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        # Right Panel: Airplane Attitude Indicator (Glass Cockpit)
        right_panel = tk.Frame(main_content, bg="#17181c", width=340, padx=6, pady=6, relief=tk.GROOVE, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y)
        right_panel.pack_propagate(False)

        tk.Label(right_panel, text="✈️ AIRPLANE ATTITUDE & ATTACK INDICATOR",
                 font=("Segoe UI", 10, "bold"), fg="#ffea00", bg="#17181c").pack(side=tk.TOP, pady=3)

        self.attitude_widget = AirplaneAttitudeWidget(right_panel, width=320, height=310)
        self.attitude_widget.pack(side=tk.TOP, pady=4)

        # Flight Telemetry Box
        stats_frame = tk.LabelFrame(right_panel, text="IMU / Flight Telemetry", font=("Segoe UI", 9, "bold"),
                                    bg="#17181c", fg="#a8dadc", padx=10, pady=6)
        stats_frame.pack(side=tk.TOP, fill=tk.X, padx=4, pady=6)

        def add_stat_row(parent, label_txt, r):
            tk.Label(parent, text=label_txt, font=("Segoe UI", 9), fg="#aaa", bg="#17181c").grid(row=r, column=0, sticky="w", pady=1.5)
            v_lbl = tk.Label(parent, text="--", font=("Consolas", 10, "bold"), fg="#fff", bg="#17181c")
            v_lbl.grid(row=r, column=1, sticky="e", padx=8, pady=1.5)
            parent.columnconfigure(1, weight=1)
            return v_lbl

        self.lbl_stat_pitch = add_stat_row(stats_frame, "Pitch (Elevation):", 0)
        self.lbl_stat_roll  = add_stat_row(stats_frame, "Roll (Bank):", 1)
        self.lbl_stat_aoa   = add_stat_row(stats_frame, "Attack / Tilt:", 2)
        self.lbl_stat_g     = add_stat_row(stats_frame, "Total G-Force:", 3)
        self.lbl_stat_state = add_stat_row(stats_frame, "Motion State:", 4)

        # ADS1292R Serial Monitor & Test Mode Telemetry (Leadless Verification Style)
        monitor_frame = tk.LabelFrame(right_panel, text="📟 ADS1292R Serial Monitor & Telemetry",
                                      font=("Segoe UI", 9, "bold"), bg="#17181c", fg="#00f5d4", padx=6, pady=4)
        monitor_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Quick stats line: CH1, CH2, Stat
        stats_top = tk.Frame(monitor_frame, bg="#17181c")
        stats_top.pack(side=tk.TOP, fill=tk.X, pady=2)

        self.lbl_mon_ch1 = tk.Label(stats_top, text="CH1: --", font=("Consolas", 8, "bold"), fg="#ffea00", bg="#22252c", padx=5, pady=2)
        self.lbl_mon_ch1.pack(side=tk.LEFT, padx=2)

        self.lbl_mon_ch2 = tk.Label(stats_top, text="CH2: --", font=("Consolas", 8, "bold"), fg="#00f5d4", bg="#22252c", padx=5, pady=2)
        self.lbl_mon_ch2.pack(side=tk.LEFT, padx=2)

        self.lbl_mon_stat = tk.Label(stats_top, text="Stat: --", font=("Consolas", 8, "bold"), fg="#ffffff", bg="#22252c", padx=5, pady=2)
        self.lbl_mon_stat.pack(side=tk.RIGHT, padx=2)

        # Scrolling text console (exact format of 04_ecg_leadless_test / view_ecg_test.py)
        self.txt_serial_mon = tk.Text(monitor_frame, height=14, width=38, font=("Consolas", 8),
                                      bg="#0c0d10", fg="#48cae4", insertbackground="white",
                                      relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.txt_serial_mon.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=2)

        # Left Panel: Matplotlib Waveforms
        left_panel = tk.Frame(main_content, bg="#121214")
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor="#141416")

        # 1. ECG Subplot (Clinical Millivolt Paper Scale)
        self.ax_ecg = self.fig.add_subplot(311)
        self.ax_ecg.set_facecolor("#0b0c0e")
        self.ax_ecg.set_title("ECG Lead I (ADS1292R - 250 Hz, 0.5-40 Hz Butterworth + 50Hz Notch) [mV]",
                              color="#ff4d6d", fontsize=8.5, loc="left")
        self.ax_ecg.tick_params(colors="#777788", labelsize=7.5)
        self.ax_ecg.grid(True, color="#1e2029", linestyle="-", linewidth=0.7, alpha=0.8)
        self.ax_ecg.grid(True, which='minor', color="#151720", linestyle=":", linewidth=0.5, alpha=0.5)
        self.line_ecg, = self.ax_ecg.plot(self.time_ecg, [0.0]*self.pts_ecg, color="#ff4d6d", lw=1.2, label="Lead I")
        self.scatter_rpeak, = self.ax_ecg.plot([], [], 'o', color="#ffff3f", markersize=5, label="R-Peak")
        self.ax_ecg.set_ylim(-1.5, 2.5)
        self.ax_ecg.set_ylabel("mV", color="#888", fontsize=8)
        self.ax_ecg.legend(loc="upper right", facecolor="#18181c", edgecolor="#333", labelcolor="white", fontsize=7)

        # 2. Respiration Subplot
        self.ax_resp = self.fig.add_subplot(312)
        self.ax_resp.set_facecolor("#0b0c0e")
        self.ax_resp.set_title("Thoracic Impedance Pneumography (ADS1292R Channel 1 - 0.05-0.8 Hz Bandpass)",
                               color="#48cae4", fontsize=8.5, loc="left")
        self.ax_resp.tick_params(colors="#777788", labelsize=7.5)
        self.ax_resp.grid(True, color="#1e2029", linestyle="--", alpha=0.6)
        self.line_resp, = self.ax_resp.plot(np.linspace(-20.0, 0.0, self.pts_resp), [0.0]*self.pts_resp, color="#48cae4", lw=1.2)
        self.ax_resp.set_ylim(-20000, 20000)
        self.ax_resp.set_ylabel("Raw ΔZ", color="#888", fontsize=8)

        # 3. IMU Accelerometer Subplot
        self.ax_imu = self.fig.add_subplot(313)
        self.ax_imu.set_facecolor("#0b0c0e")
        self.ax_imu.set_title("ADXL362 3-Axis Accelerometer (X: Cyan, Y: Lime, Z: Yellow) [±2g]",
                              color="#06d6a0", fontsize=8.5, loc="left")
        self.ax_imu.tick_params(colors="#777788", labelsize=7.5)
        self.ax_imu.grid(True, color="#1e2029", linestyle="--", alpha=0.6)
        t_imu = np.linspace(-10.0, 0.0, self.pts_imu)
        self.line_ax, = self.ax_imu.plot(t_imu, [0.0]*self.pts_imu, color="#00f5d4", lw=1.0, label="X")
        self.line_ay, = self.ax_imu.plot(t_imu, [0.0]*self.pts_imu, color="#52b788", lw=1.0, label="Y")
        self.line_az, = self.ax_imu.plot(t_imu, [1.0]*self.pts_imu, color="#fee440", lw=1.0, label="Z")
        self.ax_imu.set_ylim(-2.2, 2.2)
        self.ax_imu.set_ylabel("g", color="#888", fontsize=8)
        self.ax_imu.legend(loc="upper right", facecolor="#18181c", edgecolor="#333", labelcolor="white", fontsize=7)

        self.fig.tight_layout(pad=1.2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=left_panel)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.root.after(33, self._gui_tick)

    def _on_notch_change(self, event):
        val = self.notch_var.get()
        if "60" in val:
            self.notch_freq = 60.0
        else:
            self.notch_freq = 50.0
        self.ecg_notch = design_notch_biquad(self.notch_freq, self.fs_ecg, Q=35.0)
        self.ax_ecg.set_title(f"ECG Lead I (ADS1292R - 250 Hz, 0.5-40 Hz Butterworth + {int(self.notch_freq)}Hz Notch) [mV]",
                              color="#ff4d6d", fontsize=8.5, loc="left")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if "COM3" in ports:
            self.port_var.set("COM3")
        elif ports:
            self.port_var.set(ports[0])

    def _auto_connect(self):
        if not self.running and "COM3" in [p.device for p in serial.tools.list_ports.comports()]:
            self._toggle_connection()

    def _toggle_connection(self):
        if not self.running:
            port = self.port_var.get().strip()
            baud = self.baud_var.get()
            try:
                self.ser = serial.Serial(port, baud, timeout=0.2)
                self.ser.reset_input_buffer()
                self.running = True
                self.btn_connect.config(text="Disconnect", bg="#dc3545")
                self.worker_thread = threading.Thread(target=self._reader_worker, daemon=True)
                self.worker_thread.start()
            except Exception as e:
                messagebox.showerror("Connection Error", f"Could not open {port}:\n{e}")
        else:
            self.running = False
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except:
                    pass
            self.btn_connect.config(text="Connect", bg="#28a745")

    def _send_mode_cmd(self, cmd_char):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_char.encode('utf-8'))
            except Exception as e:
                print("Error sending command:", e)

    def _select_mode(self, cmd_char, mode_id):
        self.active_mode = mode_id
        self._send_mode_cmd(cmd_char)
        m_name = MODE_NAMES.get(mode_id, f"MODE {mode_id}")
        self.mon_lines_queue.append(f"\n>> Switched to Mode [{mode_id}]: {m_name}\n")
        self.mon_stat_count = 0
        self.mon_ch1_min = 100000.0
        self.mon_ch1_max = -100000.0
        self.mon_ch2_min = 100000.0
        self.mon_ch2_max = -100000.0
        self.mon_ch1_sum = 0.0
        self.mon_ch2_sum = 0.0
        # Reset filter buffers on mode switch to prevent transient spikes
        self.ecg_hp.reset()
        self.ecg_lp.reset()
        self.ecg_notch.reset()
        self.qrs_detector.rr_intervals.clear()
        self.qrs_detector.bpm = 0.0
        self.rpeak_x.clear()
        self.rpeak_y.clear()
        self.mode1_dc = 0.0
        for _ in range(self.pts_ecg):
            self.ecg_filt_buf.append(0.0)

    def _reader_worker(self):
        while self.running:
            try:
                # Catch up if buffer ever gets backlogged to keep live latency at zero
                if self.ser and self.ser.in_waiting > 1500:
                    self.ser.read(self.ser.in_waiting - 300)
                    self.ser.readline()

                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if line.startswith(">>"):
                    self.mon_lines_queue.append(line)
                    continue
                if not (line.startswith('{') and line.endswith('}')):
                    continue

                pkg = json.loads(line)

                # 1. ECG Stream: compact {"e":[status, e1, e2]} or {"type":"ecg", ...}
                if "e" in pkg:
                    stat = int(pkg["e"][0])
                    e1 = float(pkg["e"][1])
                    e2 = float(pkg["e"][2])
                    self._process_ecg_sample(stat, e1, e2)

                elif pkg.get("type") == "ecg":
                    e1 = float(pkg.get("e1", 0))
                    e2 = float(pkg.get("e2", 0))
                    stat = int(pkg.get("s", 192))
                    self._process_ecg_sample(stat, e1, e2)

                # 2. IMU Stream
                elif pkg.get("type") == "imu":
                    ax = float(pkg.get("ax", 0.0))
                    ay = float(pkg.get("ay", 0.0))
                    az = float(pkg.get("az", 0.0))
                    act = int(pkg.get("act", 0))

                    if "pitch" in pkg and "roll" in pkg:
                        pitch = float(pkg["pitch"])
                        roll = float(pkg["roll"])
                    else:
                        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
                        roll = math.degrees(math.atan2(ay, az))

                    g_tot = math.sqrt(ax*ax + ay*ay + az*az)
                    aoa = math.degrees(math.acos(max(-1.0, min(1.0, az / max(g_tot, 1e-6)))))

                    self.roll_val = roll
                    self.pitch_val = pitch
                    self.aoa_val = aoa
                    self.g_val = g_tot
                    self.imu_attack_state = act

                    self.imu_x_buf.append(ax)
                    self.imu_y_buf.append(ay)
                    self.imu_z_buf.append(az)

                # 3. Environmental Stream
                elif pkg.get("type") == "env":
                    for k in self.env_data.keys():
                        if k in pkg:
                            self.env_data[k] = pkg[k]
                    m = pkg.get("mode", self.active_mode)
                    if m in [1, 2, 3, 4]:
                        self.active_mode = m

            except Exception:
                pass

    def _process_ecg_sample(self, stat, e1, e2):
        self.sample_index += 1
        self.lead_status = stat

        # Measure real-time sampling rate and auto-calibrate notch & filter cutoffs
        now_t = time.perf_counter()
        if self.last_sample_t > 0.0:
            dt = now_t - self.last_sample_t
            if 0.001 < dt < 0.05:
                self.sample_times.append(dt)
                if len(self.sample_times) >= 50 and self.sample_index % 60 == 0:
                    avg_dt = sum(self.sample_times) / len(self.sample_times)
                    measured_fs = 1.0 / avg_dt
                    if 120.0 <= measured_fs <= 300.0 and abs(measured_fs - self.fs_ecg) > 4.0:
                        self.fs_ecg = 0.8 * self.fs_ecg + 0.2 * measured_fs
                        self._reconfigure_filters()
        self.last_sample_t = now_t

        # Physical conversion to millivolts for display:
        # 1 count = 2.42V / (6 * (2^23 - 1)) = 48.077 nV = 0.000048077 mV
        ecg_mv = e2 * 0.000048077

        # Slew-Rate Limiter: reject extreme non-physiological spikes (> 35.0 mV jump)
        if self.sample_index > 10 and abs(ecg_mv - self.prev_ecg_mv) > 35.0 and self.active_mode != 3:
            ecg_mv = self.prev_ecg_mv
        self.prev_ecg_mv = ecg_mv

        # Physiological Lead Contact Quality:
        ra_off = bool(stat & 0x08)
        la_off = bool(stat & 0x04)
        if self.active_mode == 4 and abs(ecg_mv) > 35.0:
            ra_off = True
            la_off = True
        self.ra_connected = not ra_off
        self.la_connected = not la_off

        # Optional Polarity Inversion (for inverted probe placement on chest)
        if self.invert_ecg.get() and self.active_mode == 4:
            ecg_mv = -ecg_mv

        # Microvolts conversion for serial monitor tracking
        ch1_uv = e1 * 0.0480803
        ch2_uv = e2 * 0.0480803

        self.mon_sample_count += 1
        if ch1_uv < self.mon_ch1_min: self.mon_ch1_min = ch1_uv
        if ch1_uv > self.mon_ch1_max: self.mon_ch1_max = ch1_uv
        if ch2_uv < self.mon_ch2_min: self.mon_ch2_min = ch2_uv
        if ch2_uv > self.mon_ch2_max: self.mon_ch2_max = ch2_uv
        self.mon_ch1_sum += ch1_uv
        self.mon_ch2_sum += ch2_uv
        self.mon_stat_count += 1

        self.latest_ch1_uv = ch1_uv
        self.latest_ch2_uv = ch2_uv
        self.latest_stat = stat

        # Decimate output for clean terminal viewing (print every 20th sample ~12.5 Hz display at 250Hz)
        if self.mon_sample_count % 20 == 0:
            bar = self._make_ascii_bar(ch1_uv)
            sign1 = "+" if ch1_uv >= 0 else ""
            sign2 = "+" if ch2_uv >= 0 else ""
            line_str = f"ECG | CH1={sign1}{int(ch1_uv)} uV {bar} | CH2={sign2}{int(ch2_uv)} uV | Raw: {int(e1)}"
            self.mon_lines_queue.append(line_str)

        # 1-second stats report (every 250 samples at 250 Hz)
        if self.mon_sample_count % int(self.fs_ecg) == 0:
            ch1_avg = (self.mon_ch1_sum / self.mon_stat_count) if self.mon_stat_count > 0 else 0.0
            ch1_vpp = self.mon_ch1_max - self.mon_ch1_min
            ch2_avg = (self.mon_ch2_sum / self.mon_stat_count) if self.mon_stat_count > 0 else 0.0
            ch2_vpp = self.mon_ch2_max - self.mon_ch2_min
            m_name = MODE_NAMES.get(self.active_mode, "UNKNOWN")
            stat_block = (
                f"\n>>> [1-SEC STATS] Mode: {m_name}\n"
                f"    CH1 Vpp = {ch1_vpp:.1f} uV | Offset = {ch1_avg:.1f} uV | Stat=0x{stat:02X} | DRDY=OK\n"
                f"    CH2 Vpp = {ch2_vpp:.1f} uV | Offset = {ch2_avg:.1f} uV\n"
            )
            self.mon_lines_queue.append(stat_block)
            # Reset stats window
            self.mon_stat_count = 0
            self.mon_ch1_min = 100000.0; self.mon_ch1_max = -100000.0
            self.mon_ch2_min = 100000.0; self.mon_ch2_max = -100000.0
            self.mon_ch1_sum = 0.0; self.mon_ch2_sum = 0.0

        # Mode-Aware Filtering:
        if self.active_mode == 1:
            # Mode 1: 1Hz Square Wave - bypass Highpass & Notch
            self.mode1_raw_buf.append(ecg_mv)
            if len(self.mode1_raw_buf) >= 30:
                mid = 0.5 * (max(self.mode1_raw_buf) + min(self.mode1_raw_buf))
            else:
                mid = ecg_mv
            ecg_filtered = ecg_mv - mid
        elif self.active_mode == 2:
            # Mode 2: Input Short - show raw noise & offset
            ecg_filtered = ecg_mv
        elif self.active_mode == 3:
            # Mode 3: Internal Die Temp - DC voltage
            ecg_filtered = ecg_mv
        else:
            # Mode 4: Clinical Live ECG - 0.5-35Hz Bandpass + 50/60Hz Notch
            s1 = self.ecg_hp.process(ecg_mv)
            s2 = self.ecg_lp.process(s1)
            ecg_filtered = self.ecg_notch.process(s2)

        self.ecg_filt_buf.append(ecg_filtered)

        # Pan-Tompkins QRS R-Peak Detection (Active in Mode 4 Live ECG)
        if self.active_mode == 4:
            is_rpeak = self.qrs_detector.process(ecg_filtered)
            if is_rpeak:
                recent = list(self.ecg_filt_buf)[-10:] if len(self.ecg_filt_buf) >= 10 else [ecg_filtered]
                peak_val = max(recent) if max(recent) > 0 else ecg_filtered
                self.rpeak_x.append(0.0)
                self.rpeak_y.append(peak_val)
                self.beat_pulse_timer = 6  # Flash heart for ~200ms
                # Send verified BPM to MCU to update 7-segment display & flash LEDs
                if self.serial_conn and self.serial_conn.is_open and self.qrs_detector.bpm > 30:
                    try:
                        self.serial_conn.write(bytes([min(255, 100 + int(round(self.qrs_detector.bpm)))]))
                    except Exception:
                        pass
        else:
            self.beat_pulse_timer = 0

        # Shift existing R-peak markers to the left as time advances
        if self.sample_index % 5 == 0 and len(self.rpeak_x) > 0:
            shift = 5.0 / self.fs_ecg
            for idx in range(len(self.rpeak_x)):
                self.rpeak_x[idx] -= shift

        # Respiration / Channel 1 Waveform
        if self.active_mode == 4:
            resp_input = e1 if abs(e1) > 20.0 else (ecg_filtered * 500.0)
            r1 = self.resp_hp.process(resp_input)
            resp_filtered = self.resp_lp.process(r1)
            self.resp_filt_buf.append(resp_filtered)
            self.resp_detector.process(resp_filtered)
        elif self.active_mode == 1:
            ch1_mv = e1 * 0.000048077
            self.resp_filt_buf.append(ch1_mv - mid)
        elif self.active_mode in [2, 3]:
            ch1_mv = e1 * 0.000048077
            self.resp_filt_buf.append(ch1_mv)
        else:
            self.resp_filt_buf.append(0.0)

    def _gui_tick(self):
        if self.running:
            self.tick_count += 1

            # 1. Update Lead-Off Badges
            if self.ra_connected:
                self.lbl_ra.config(text="● RA: OK", fg="#28a745", bg="#22252c")
            else:
                self.lbl_ra.config(text="✗ RA OFF", fg="#ff4d6d", bg="#3a1b24")

            if self.la_connected:
                self.lbl_la.config(text="● LA: OK", fg="#28a745", bg="#22252c")
            else:
                self.lbl_la.config(text="✗ LA OFF", fg="#ff4d6d", bg="#3a1b24")

            # 2. Update Heart Rate & Pulse Indicator
            avg_dc = float(np.mean(list(self.ecg_filt_buf)[-100:])) if len(self.ecg_filt_buf) >= 100 else 145.3
            die_temp = (avg_dc - 145.3) / 0.490 + 25.0
            die_temp = max(-10.0, min(85.0, die_temp))

            if self.active_mode == 4:
                hr_bpm = self.qrs_detector.bpm
                if hr_bpm > 0:
                    if self.beat_pulse_timer > 0:
                        self.beat_pulse_timer -= 1
                        self.card_hr.config(text=f"💓 {hr_bpm:.0f}", fg="#ffea00")
                    else:
                        self.card_hr.config(text=f"❤️ {hr_bpm:.0f}", fg="#ff4d6d")
                else:
                    self.card_hr.config(text="ACQUIRING...", fg="#ffb703")

                rpm_val = self.resp_detector.rpm
                self.card_resp.config(text=f"{rpm_val:.0f}" if rpm_val > 0 else "--", fg="#48cae4")
            elif self.active_mode == 1:
                self.card_hr.config(text="1Hz SQUARE", fg="#ffea00")
                self.card_resp.config(text="CAL TEST", fg="#ffea00")
            elif self.active_mode == 2:
                self.card_hr.config(text="SHORTED", fg="#00f5d4")
                self.card_resp.config(text="NOISE TEST", fg="#00f5d4")
            elif self.active_mode == 3:
                self.card_hr.config(text=f"{die_temp:.1f}°C", fg="#f77f00")
                self.card_resp.config(text="DIE TEMP", fg="#f77f00")

            # 3. Update Attitude Instrument & Flight Stats
            self.attitude_widget.update_attitude(self.roll_val, self.pitch_val, self.aoa_val, self.g_val)

            self.card_pitch.config(text=f"{self.pitch_val:+.1f}°")
            self.card_roll.config(text=f"{self.roll_val:+.1f}°")
            self.card_aoa.config(text=f"{self.aoa_val:.1f}°")

            self.lbl_stat_pitch.config(text=f"{self.pitch_val:+.1f}°")
            self.lbl_stat_roll.config(text=f"{self.roll_val:+.1f}°")
            self.lbl_stat_aoa.config(text=f"{self.aoa_val:.1f}°")
            self.lbl_stat_g.config(text=f"{self.g_val:.2f} g")

            if self.imu_attack_state:
                self.lbl_stat_state.config(text="ACTIVE / MOTION", fg="#ff4d6d")
                self.lbl_attack.config(text="🔥 ATTACK / MOTION DETECTED! 🔥", bg="#dc3545", fg="white")
            else:
                self.lbl_stat_state.config(text="STILL / CALM", fg="#00f5d4")
                self.lbl_attack.config(text="● STILL / RESTING", bg="#28a745", fg="white")

            # 4. ADS1292R Serial Monitor Console & Statistics Update (Batch-rendered every 4th tick ~160ms for ultra responsiveness)
            if self.mon_lines_queue and (self.tick_count % 4 == 0):
                lines_to_add = []
                while self.mon_lines_queue:
                    lines_to_add.append(self.mon_lines_queue.popleft())
                if lines_to_add:
                    batch_text = "\n".join(lines_to_add) + "\n"
                    self.txt_serial_mon.configure(state=tk.NORMAL)
                    self.txt_serial_mon.insert(tk.END, batch_text)
                    
                    # Ensure the text widget does not grow indefinitely
                    actual_lines = int(self.txt_serial_mon.index('end-1c').split('.')[0])
                    if actual_lines > 150:
                        self.txt_serial_mon.delete("1.0", f"{actual_lines - 100}.0")
                        
                    self.txt_serial_mon.see(tk.END)
                    self.txt_serial_mon.configure(state=tk.DISABLED)

            if self.tick_count % 3 == 0:
                sign1 = "+" if self.latest_ch1_uv >= 0 else ""
                sign2 = "+" if self.latest_ch2_uv >= 0 else ""
                self.lbl_mon_ch1.config(text=f"CH1: {sign1}{int(self.latest_ch1_uv)} µV")
                self.lbl_mon_ch2.config(text=f"CH2: {sign2}{int(self.latest_ch2_uv)} µV")
                self.lbl_mon_stat.config(text=f"Stat: 0x{self.latest_stat:02X}")

            # 5. Update Environmental Cards
            self.card_temp.config(text=f"{self.env_data['T']:.1f}")
            self.card_hum.config(text=f"{self.env_data['H']:.1f}")
            self.card_press.config(text=f"{self.env_data['P']:.1f}")
            self.card_lux.config(text=f"{self.env_data['lux']:.1f}")
            self.card_prox.config(text=f"{int(self.env_data['prox'])}")
            self.card_mlx.config(text=f"{self.env_data['mlx']:.1f}")

            # 6. Highlight Active Mode Button
            for mid, btn in self.mode_btns.items():
                if mid == self.active_mode:
                    btn.config(bg="#ff0054", fg="white", relief=tk.SUNKEN)
                else:
                    btn.config(bg="#3a3d40", fg="#dddddd", relief=tk.RAISED)

            # 7. Update Waveform Plots with Mode-Aware Scaling
            self.line_ecg.set_ydata(self.ecg_filt_buf)
            self.line_resp.set_ydata(self.resp_filt_buf)
            self.line_ax.set_ydata(self.imu_x_buf)
            self.line_ay.set_ydata(self.imu_y_buf)
            self.line_az.set_ydata(self.imu_z_buf)

            # Update axes titles and limits ONLY when mode changes (avoids massive re-layout overhead!)
            if self.active_mode != self.last_rendered_mode:
                self.last_rendered_mode = self.active_mode
                if self.active_mode == 1:
                    self.ax_ecg.set_title("ECG Test CH2: 1 Hz Internal Square Wave (±1.0 mV Test Signal) [mV]",
                                          color="#ffea00", fontsize=8.5, loc="left")
                    self.ax_ecg.set_ylim(-2.0, 2.0)
                    self.ax_ecg.set_ylabel("mV", color="#888", fontsize=8)
                    self.ax_resp.set_title("Channel 1 (CH1): 1 Hz Internal Square Wave (±1.0 mV Test Signal) [mV]",
                                           color="#ffea00", fontsize=8.5, loc="left")
                    self.ax_resp.set_ylim(-2.0, 2.0)
                    self.ax_resp.set_ylabel("mV", color="#888", fontsize=8)
                elif self.active_mode == 2:
                    self.ax_ecg.set_title("ECG Test CH2: Internal Input Short (Noise Floor & ADC Offset) [±0.5 mV Full-Scale]",
                                          color="#00f5d4", fontsize=8.5, loc="left")
                    self.ax_ecg.set_ylim(-0.5, 0.5)
                    self.ax_ecg.set_ylabel("mV", color="#888", fontsize=8)
                    self.ax_resp.set_title("Channel 1 (CH1): Internal Input Short (Noise Floor & Offset) [±0.5 mV]",
                                           color="#00f5d4", fontsize=8.5, loc="left")
                    self.ax_resp.set_ylim(-0.5, 0.5)
                    self.ax_resp.set_ylabel("mV", color="#888", fontsize=8)
                elif self.active_mode == 3:
                    self.ax_ecg.set_title(f"ECG Test CH2: Internal Die Temp Diode ({die_temp:.1f}°C) [mV]",
                                          color="#f77f00", fontsize=8.5, loc="left")
                    self.ax_ecg.set_ylim(avg_dc - 5.0, avg_dc + 5.0)
                    self.ax_ecg.set_ylabel("mV", color="#888", fontsize=8)
                    self.ax_resp.set_title(f"Channel 1 (CH1): Internal Die Temp Diode [mV]",
                                           color="#f77f00", fontsize=8.5, loc="left")
                    self.ax_resp.set_ylim(avg_dc - 5.0, avg_dc + 5.0)
                    self.ax_resp.set_ylabel("mV", color="#888", fontsize=8)
                else:
                    self.ax_ecg.set_title(f"ECG Lead I CH2 (ADS1292R - 0.5-35 Hz Bandpass + {int(self.notch_freq)}Hz Notch) [mV]",
                                          color="#ff4d6d", fontsize=8.5, loc="left")
                    self.ax_ecg.set_ylim(-1.5, 2.5)
                    self.ax_ecg.set_ylabel("mV", color="#888", fontsize=8)
                    self.ax_resp.set_title("Thoracic Impedance Pneumography (ADS1292R Channel 1 - 0.05-0.8 Hz Bandpass)",
                                           color="#48cae4", fontsize=8.5, loc="left")
                    self.ax_resp.set_ylabel("Raw ΔZ", color="#888", fontsize=8)
                    self.ax_resp.set_ylim(-15000, 15000)

            # R-Peak Markers (Only in Mode 4)
            if self.active_mode == 4:
                valid_x = [x for x in self.rpeak_x if -5.0 <= x <= 0.0]
                valid_y = [self.rpeak_y[i] for i, x in enumerate(self.rpeak_x) if -5.0 <= x <= 0.0]
                self.scatter_rpeak.set_data(valid_x, valid_y)
            else:
                self.scatter_rpeak.set_data([], [])

            if self.tick_count % 3 == 0:
                self.canvas.draw_idle()

        self.root.after(40, self._gui_tick)

    def _on_close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = SensorDashboardApp(root)
    root.mainloop()
