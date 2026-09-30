#!/usr/bin/env python3
"""
SmartBAN ECG & Respiration Real-Time Monitor GUI
=================================================
Connects to the ADS1292R firmware (V2) over UART and displays:
  - Scrolling ECG waveform (5 seconds, filtered)
  - Scrolling Respiration waveform (30 seconds, filtered)
  - Live BPM and RPM readouts
  - Electrode contact (lead-off) indicators
  - Signal quality meter

Usage:
    python ecg_monitor_gui.py --port COM3
"""

import sys
import time
import argparse
import threading
import collections
import numpy as np

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

import tkinter as tk
from tkinter import font as tkfont

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# =========================================================================
# DSP Filters (Pure numpy — no scipy dependency)
# =========================================================================

class BiquadFilter:
    """Second-order IIR (biquad) filter section."""
    def __init__(self, b, a):
        self.b0, self.b1, self.b2 = b
        self.a1, self.a2 = a[1], a[2]
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0

    def process(self, x):
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


def design_highpass_biquad(fc, fs):
    """Design a 2nd-order Butterworth high-pass biquad."""
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / (2.0 * np.sqrt(2.0))  # Q = sqrt(2)/2
    cos_w0 = np.cos(w0)
    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return BiquadFilter([b0/a0, b1/a0, b2/a0], [1.0, a1/a0, a2/a0])


def design_lowpass_biquad(fc, fs):
    """Design a 2nd-order Butterworth low-pass biquad."""
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / (2.0 * np.sqrt(2.0))
    cos_w0 = np.cos(w0)
    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return BiquadFilter([b0/a0, b1/a0, b2/a0], [1.0, a1/a0, a2/a0])


def design_notch_biquad(fc, fs, Q=30.0):
    """Design a notch (band-reject) biquad filter."""
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
# R-Peak Detector (Pan-Tompkins inspired)
# =========================================================================

class RPeakDetector:
    """Derivative-based QRS detector with adaptive thresholding."""
    def __init__(self, fs):
        self.fs = fs
        # Moving window integrator length (~150 ms)
        self.mwi_len = max(int(0.15 * fs), 3)
        self.mwi_buf = collections.deque(maxlen=self.mwi_len)
        self.mwi_sum = 0.0
        # Threshold adaptation
        self.threshold = 0.0
        self.peak_history = collections.deque(maxlen=8)
        # Refractory: no two beats within 200ms
        self.refractory_samples = int(0.20 * fs)
        self.samples_since_peak = self.refractory_samples + 1
        # Derivative delay
        self.prev_samples = collections.deque([0.0] * 4, maxlen=4)
        # Output
        self.rr_intervals = collections.deque(maxlen=12)
        self.bpm = 0.0

    def process(self, filtered_sample):
        """Process one filtered ECG sample. Returns True if R-peak detected."""
        # 1. Derivative (5-point)
        deriv = (filtered_sample - self.prev_samples[0]) / 4.0
        self.prev_samples.append(filtered_sample)

        # 2. Squaring
        sq = deriv * deriv

        # 3. Moving window integration
        if len(self.mwi_buf) == self.mwi_len:
            self.mwi_sum -= self.mwi_buf[0]
        self.mwi_buf.append(sq)
        self.mwi_sum += sq
        mwi_val = self.mwi_sum / self.mwi_len

        self.samples_since_peak += 1
        detected = False

        # 4. Threshold crossing
        if mwi_val > self.threshold and self.samples_since_peak > self.refractory_samples:
            detected = True
            self.peak_history.append(mwi_val)

            # Record R-R interval
            self.rr_intervals.append(self.samples_since_peak)
            self.samples_since_peak = 0

            # Update BPM (median of recent R-R intervals for robustness)
            if len(self.rr_intervals) >= 3:
                rr_arr = np.array(list(self.rr_intervals))
                median_rr = np.median(rr_arr)
                if median_rr > 0:
                    self.bpm = 60.0 * self.fs / median_rr

        # 5. Adaptive threshold
        if len(self.peak_history) > 0:
            self.threshold = 0.4 * np.mean(list(self.peak_history))
        # Prevent threshold from collapsing to zero
        if self.threshold < 1e-6:
            self.threshold = 1e-6

        return detected


# =========================================================================
# Respiration Rate Detector
# =========================================================================

class RespRateDetector:
    """Simple zero-crossing detector for respiration rate."""
    def __init__(self, fs):
        self.fs = fs
        self.prev_val = 0.0
        self.prev_sign = 0
        self.samples_since_cross = 0
        self.half_periods = collections.deque(maxlen=20)
        self.rpm = 0.0
        # Min ~3 RPM, max ~60 RPM → period 1-20 sec → half-period 0.5-10 sec
        self.min_half_period = int(0.5 * fs)

    def process(self, filtered_sample):
        self.samples_since_cross += 1
        sign = 1 if filtered_sample >= 0 else -1

        if sign != self.prev_sign and self.prev_sign != 0:
            if self.samples_since_cross > self.min_half_period:
                self.half_periods.append(self.samples_since_cross)
                self.samples_since_cross = 0
                # Full period = 2 half-periods
                if len(self.half_periods) >= 4:
                    hp_arr = np.array(list(self.half_periods))
                    avg_full_period = 2.0 * np.median(hp_arr)
                    if avg_full_period > 0:
                        self.rpm = 60.0 * self.fs / avg_full_period

        self.prev_sign = sign
        self.prev_val = filtered_sample


# =========================================================================
# Serial Reader Thread
# =========================================================================

class SerialReader:
    """Background thread that reads UART data into shared buffers."""
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.running = False
        self.lock = threading.Lock()

        # Shared data buffers
        self.ch1_buf = collections.deque(maxlen=15000)  # ~60 sec at 250 Hz
        self.ch2_buf = collections.deque(maxlen=15000)
        self.status_buf = collections.deque(maxlen=250)  # Last 1 sec of status

        # Metadata
        self.sample_count = 0
        self.data_started = False
        self.boot_messages = []
        self.connected = False
        self.error_msg = ""

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def _reader_loop(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
            self.connected = True
            ser.reset_input_buffer()
        except Exception as e:
            self.error_msg = str(e)
            self.connected = False
            return

        while self.running:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Detect DATA_START marker from firmware
                if line == "DATA_START":
                    self.data_started = True
                    continue

                if not self.data_started:
                    with self.lock:
                        self.boot_messages.append(line)
                    continue

                # Parse: status,ch1,ch2
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        status = int(parts[0])
                        ch1 = int(parts[1])
                        ch2 = int(parts[2])
                        with self.lock:
                            self.ch1_buf.append(ch1)
                            self.ch2_buf.append(ch2)
                            self.status_buf.append(status)
                            self.sample_count += 1
                    except ValueError:
                        pass

                # Also support V1 format: idx,resp,ecg,filt,bpm,rpm
                elif len(parts) == 6:
                    try:
                        ch1 = int(parts[1])
                        ch2 = int(parts[2])
                        with self.lock:
                            self.ch1_buf.append(ch1)
                            self.ch2_buf.append(ch2)
                            self.status_buf.append(192)  # fake OK status
                            self.sample_count += 1
                            if not self.data_started:
                                self.data_started = True
                    except ValueError:
                        pass

            except serial.SerialException:
                self.error_msg = "Serial connection lost"
                self.connected = False
                break
            except Exception:
                pass

        try:
            ser.close()
        except Exception:
            pass


# =========================================================================
# Main GUI Application
# =========================================================================

class ECGMonitorApp:
    # Firmware sample rate after 2x decimation
    FS = 250.0

    # Colors (dark medical monitor theme)
    BG = '#0a0a0a'
    FG = '#cccccc'
    ECG_COLOR = '#00ff41'     # Classic green
    RESP_COLOR = '#00bfff'    # Cyan-blue
    HEART_COLOR = '#ff3333'   # Red
    BREATH_COLOR = '#66ccff'  # Light blue
    OK_COLOR = '#00cc00'      # Green
    WARN_COLOR = '#ffaa00'    # Amber
    BAD_COLOR = '#ff3333'     # Red
    GRID_COLOR = '#1a1a1a'

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud

        # ---- DSP chain ----
        # ECG filters: 0.5 Hz HP → 40 Hz LP → 50 Hz notch
        self.ecg_hp = design_highpass_biquad(0.5, self.FS)
        self.ecg_lp = design_lowpass_biquad(40.0, self.FS)
        self.ecg_notch = design_notch_biquad(50.0, self.FS, Q=35.0)

        # Respiration filters: 0.05 Hz HP → 0.8 Hz LP
        self.resp_hp = design_highpass_biquad(0.05, self.FS)
        self.resp_lp = design_lowpass_biquad(0.8, self.FS)

        # Detectors
        self.rpeak = RPeakDetector(self.FS)
        self.resp_rate = RespRateDetector(self.FS)

        # ---- Data Buffers for Plotting ----
        self.ecg_plot_len = int(5.0 * self.FS)    # 5 seconds
        self.resp_plot_len = int(30.0 * self.FS)   # 30 seconds

        self.ecg_filt_buf = collections.deque(maxlen=self.ecg_plot_len)
        self.resp_filt_buf = collections.deque(maxlen=self.resp_plot_len)
        self.rpeak_markers = collections.deque(maxlen=50)

        self.last_processed = 0
        self.total_processed = 0

        # ---- Serial ----
        self.reader = SerialReader(port, baud)

        # ---- Build GUI ----
        self._build_gui()

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title(f"SmartBAN ECG Monitor — {self.port}")
        self.root.configure(bg=self.BG)
        self.root.geometry("1200x800")
        self.root.minsize(900, 600)

        # ---- Top Info Bar ----
        top_frame = tk.Frame(self.root, bg='#111111', pady=6)
        top_frame.pack(fill=tk.X)

        title_font = tkfont.Font(family='Consolas', size=14, weight='bold')
        tk.Label(top_frame, text="  SmartBAN ECG & Respiration Monitor",
                 font=title_font, fg=self.ECG_COLOR, bg='#111111').pack(side=tk.LEFT)

        self.conn_label = tk.Label(top_frame, text=f"  ● {self.port}  ",
                                   font=('Consolas', 11), fg=self.WARN_COLOR, bg='#111111')
        self.conn_label.pack(side=tk.RIGHT)

        # ---- Main Content: plots + vitals ----
        content = tk.Frame(self.root, bg=self.BG)
        content.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        # Left: Plots
        plot_frame = tk.Frame(content, bg=self.BG)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Right: Vitals Panel
        vitals_frame = tk.Frame(content, bg='#111111', width=200)
        vitals_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(8, 0))
        vitals_frame.pack_propagate(False)

        self._build_vitals_panel(vitals_frame)
        self._build_plots(plot_frame)

        # ---- Bottom Status Bar ----
        status_frame = tk.Frame(self.root, bg='#111111', pady=4)
        status_frame.pack(fill=tk.X)

        self.status_label = tk.Label(status_frame, text="  Connecting...",
                                      font=('Consolas', 9), fg=self.FG, bg='#111111',
                                      anchor='w')
        self.status_label.pack(fill=tk.X, padx=8)

    def _build_vitals_panel(self, parent):
        """Build the right-side vitals display."""
        # BPM Section
        tk.Label(parent, text="HEART RATE", font=('Consolas', 9),
                 fg='#666666', bg='#111111').pack(pady=(20, 0))

        self.bpm_label = tk.Label(parent, text="--",
                                   font=('Consolas', 48, 'bold'),
                                   fg=self.HEART_COLOR, bg='#111111')
        self.bpm_label.pack()

        tk.Label(parent, text="BPM", font=('Consolas', 12),
                 fg='#666666', bg='#111111').pack()

        # Divider
        tk.Frame(parent, bg='#333333', height=1).pack(fill=tk.X, padx=20, pady=15)

        # RPM Section
        tk.Label(parent, text="RESP RATE", font=('Consolas', 9),
                 fg='#666666', bg='#111111').pack(pady=(5, 0))

        self.rpm_label = tk.Label(parent, text="--",
                                   font=('Consolas', 48, 'bold'),
                                   fg=self.BREATH_COLOR, bg='#111111')
        self.rpm_label.pack()

        tk.Label(parent, text="RPM", font=('Consolas', 12),
                 fg='#666666', bg='#111111').pack()

        # Divider
        tk.Frame(parent, bg='#333333', height=1).pack(fill=tk.X, padx=20, pady=15)

        # Lead-off indicators
        tk.Label(parent, text="ELECTRODES", font=('Consolas', 9),
                 fg='#666666', bg='#111111').pack(pady=(5, 2))

        lead_frame = tk.Frame(parent, bg='#111111')
        lead_frame.pack()

        self.ra_dot = tk.Label(lead_frame, text="● RA", font=('Consolas', 11),
                                fg=self.WARN_COLOR, bg='#111111')
        self.ra_dot.pack(anchor='w', padx=15)

        self.la_dot = tk.Label(lead_frame, text="● LA", font=('Consolas', 11),
                                fg=self.WARN_COLOR, bg='#111111')
        self.la_dot.pack(anchor='w', padx=15)

        # Divider
        tk.Frame(parent, bg='#333333', height=1).pack(fill=tk.X, padx=20, pady=15)

        # Signal quality
        tk.Label(parent, text="SIGNAL", font=('Consolas', 9),
                 fg='#666666', bg='#111111').pack(pady=(5, 2))

        self.quality_label = tk.Label(parent, text="Waiting...",
                                       font=('Consolas', 12),
                                       fg=self.WARN_COLOR, bg='#111111')
        self.quality_label.pack()

        # Sample counter
        tk.Frame(parent, bg='#333333', height=1).pack(fill=tk.X, padx=20, pady=15)
        self.sample_label = tk.Label(parent, text="Samples: 0",
                                      font=('Consolas', 9),
                                      fg='#444444', bg='#111111')
        self.sample_label.pack(pady=(5, 0))

    def _build_plots(self, parent):
        """Build the matplotlib plots embedded in tkinter."""
        self.fig, (self.ax_ecg, self.ax_resp) = plt.subplots(
            2, 1, figsize=(9, 6), facecolor=self.BG,
            gridspec_kw={'height_ratios': [2, 1], 'hspace': 0.25}
        )

        for ax in [self.ax_ecg, self.ax_resp]:
            ax.set_facecolor(self.BG)
            ax.tick_params(colors=self.FG, labelsize=8)
            ax.spines['bottom'].set_color('#333333')
            ax.spines['left'].set_color('#333333')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(True, color=self.GRID_COLOR, linewidth=0.5, alpha=0.5)

        # ECG plot
        self.ax_ecg.set_title("ECG — Lead I", fontsize=11, color=self.ECG_COLOR,
                               fontweight='bold', loc='left')
        self.ax_ecg.set_ylabel("mV", fontsize=9, color=self.FG)
        self.ax_ecg.set_xlim(0, 5.0)
        self.ax_ecg.set_ylim(-1.5, 2.0)
        self.ecg_line, = self.ax_ecg.plot([], [], color=self.ECG_COLOR, linewidth=1.0)
        self.rpeak_scatter = self.ax_ecg.scatter([], [], color=self.HEART_COLOR,
                                                   s=30, zorder=5, marker='v')

        # Respiration plot
        self.ax_resp.set_title("Respiration — Impedance Pneumography", fontsize=11,
                                color=self.RESP_COLOR, fontweight='bold', loc='left')
        self.ax_resp.set_ylabel("A.U.", fontsize=9, color=self.FG)
        self.ax_resp.set_xlabel("Time (sec)", fontsize=9, color=self.FG)
        self.ax_resp.set_xlim(0, 30.0)
        self.resp_line, = self.ax_resp.plot([], [], color=self.RESP_COLOR, linewidth=1.2)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _process_new_samples(self):
        """Pull new raw samples from serial reader, filter them, run detectors."""
        with self.reader.lock:
            n = self.reader.sample_count
            if n <= self.last_processed:
                return

            # Grab only the newest samples
            all_ch1 = list(self.reader.ch1_buf)
            all_ch2 = list(self.reader.ch2_buf)
            statuses = list(self.reader.status_buf)

        new_count = n - self.last_processed
        # Only process the tail
        start_idx = max(0, len(all_ch2) - new_count)
        new_ch1 = all_ch1[start_idx:]
        new_ch2 = all_ch2[start_idx:]

        for i in range(len(new_ch2)):
            # ---- ECG filtering ----
            # Convert to microvolts first, then to mV
            # At Gain=6, Vref=2.42V: 1 LSB = 2.42 / (6 * 2^23) = 48.1 nV
            ecg_uv = new_ch2[i] * 0.0481  # microvolts
            ecg_mv = ecg_uv / 1000.0       # millivolts

            ecg_hp = self.ecg_hp.process(ecg_mv)
            ecg_lp = self.ecg_lp.process(ecg_hp)
            ecg_out = self.ecg_notch.process(ecg_lp)

            self.ecg_filt_buf.append(ecg_out)
            self.total_processed += 1

            # R-peak detection
            if self.rpeak.process(ecg_out):
                t = self.total_processed / self.FS
                self.rpeak_markers.append((t, ecg_out))

            # ---- Respiration filtering ----
            # CH1 at Gain=1: 1 LSB = 2.42 / 2^23 = 288.4 nV
            resp_val = float(new_ch1[i])
            resp_hp = self.resp_hp.process(resp_val)
            resp_out = self.resp_lp.process(resp_hp)
            self.resp_filt_buf.append(resp_out)

            # Respiration rate detection
            self.resp_rate.process(resp_out)

        self.last_processed = n

        # Update lead-off from latest status
        if statuses:
            self._update_leadoff(statuses[-1])

    def _update_leadoff(self, status_byte):
        """Parse the ADS1292R status byte for lead-off flags."""
        # Status byte: 1100_xxxx where x bits are lead-off
        # bit3 = IN2N (RA) off, bit2 = IN2P (LA) off
        # Status should start with 0xC0 (192)
        if status_byte < 192:
            # Invalid status — SPI error
            self.ra_dot.config(fg=self.BAD_COLOR, text="✗ RA")
            self.la_dot.config(fg=self.BAD_COLOR, text="✗ LA")
            return

        ra_off = bool(status_byte & 0x08)  # bit 3
        la_off = bool(status_byte & 0x04)  # bit 2

        if ra_off:
            self.ra_dot.config(fg=self.BAD_COLOR, text="✗ RA OFF")
        else:
            self.ra_dot.config(fg=self.OK_COLOR, text="● RA OK")

        if la_off:
            self.la_dot.config(fg=self.BAD_COLOR, text="✗ LA OFF")
        else:
            self.la_dot.config(fg=self.OK_COLOR, text="● LA OK")

    def _update_animation(self, frame):
        """Called by FuncAnimation ~30 FPS."""
        self._process_new_samples()

        # ---- ECG plot ----
        ecg_data = list(self.ecg_filt_buf)
        n_ecg = len(ecg_data)
        if n_ecg > 2:
            t_ecg = np.arange(n_ecg) / self.FS
            # Shift so right edge = now
            t_ecg = t_ecg - t_ecg[-1] + 5.0  # Right-align to 5 seconds
            self.ecg_line.set_data(t_ecg, ecg_data)

            # Auto-scale Y with some headroom
            visible = np.array(ecg_data)
            if len(visible) > 50:
                p5, p95 = np.percentile(visible, [2, 98])
                margin = max(abs(p95 - p5) * 0.3, 0.3)
                self.ax_ecg.set_ylim(p5 - margin, p95 + margin)

            # R-peak markers
            now_t = self.total_processed / self.FS
            visible_peaks = [(t - now_t + 5.0, v) for t, v in self.rpeak_markers
                             if (now_t - t) < 5.0]
            if visible_peaks:
                px, py = zip(*visible_peaks)
                self.rpeak_scatter.set_offsets(np.column_stack([px, py]))
            else:
                self.rpeak_scatter.set_offsets(np.empty((0, 2)))

        # ---- Respiration plot ----
        resp_data = list(self.resp_filt_buf)
        n_resp = len(resp_data)
        if n_resp > 2:
            t_resp = np.arange(n_resp) / self.FS
            t_resp = t_resp - t_resp[-1] + 30.0  # Right-align to 30 seconds
            self.resp_line.set_data(t_resp, resp_data)

            visible_r = np.array(resp_data)
            if len(visible_r) > 50:
                p5, p95 = np.percentile(visible_r, [5, 95])
                margin = max(abs(p95 - p5) * 0.3, 100)
                self.ax_resp.set_ylim(p5 - margin, p95 + margin)

        # ---- Vitals readouts ----
        bpm = self.rpeak.bpm
        rpm = self.resp_rate.rpm

        if bpm > 0 and 30 < bpm < 220:
            self.bpm_label.config(text=f"{int(bpm)}")
            if bpm < 50 or bpm > 120:
                self.bpm_label.config(fg=self.WARN_COLOR)
            else:
                self.bpm_label.config(fg=self.HEART_COLOR)
        else:
            self.bpm_label.config(text="--", fg='#444444')

        if rpm > 0 and 3 < rpm < 60:
            self.rpm_label.config(text=f"{int(rpm)}")
        else:
            self.rpm_label.config(text="--", fg='#444444')

        # ---- Signal quality ----
        if n_ecg > 100:
            recent = np.array(ecg_data[-250:]) if n_ecg >= 250 else np.array(ecg_data)
            rms = np.sqrt(np.mean(recent ** 2))
            if rms < 0.01:
                self.quality_label.config(text="No Signal", fg=self.BAD_COLOR)
            elif rms > 5.0:
                self.quality_label.config(text="Noisy", fg=self.WARN_COLOR)
            else:
                self.quality_label.config(text="Good", fg=self.OK_COLOR)
        else:
            self.quality_label.config(text="Settling...", fg=self.WARN_COLOR)

        # ---- Connection status ----
        if self.reader.connected:
            if self.reader.data_started:
                self.conn_label.config(text=f"  ● {self.port}  ", fg=self.OK_COLOR)
            else:
                self.conn_label.config(text=f"  ◐ {self.port} booting  ", fg=self.WARN_COLOR)
        else:
            self.conn_label.config(text=f"  ✗ {self.port}  ", fg=self.BAD_COLOR)

        self.sample_label.config(text=f"Samples: {self.last_processed:,}")

        # ---- Status bar ----
        boot_msgs = ""
        with self.reader.lock:
            if self.reader.boot_messages:
                boot_msgs = self.reader.boot_messages[-1]
        if boot_msgs and not self.reader.data_started:
            self.status_label.config(text=f"  {boot_msgs}")
        elif self.reader.error_msg:
            self.status_label.config(text=f"  ERROR: {self.reader.error_msg}",
                                      fg=self.BAD_COLOR)
        elif self.reader.data_started:
            self.status_label.config(
                text=f"  Streaming: {self.last_processed:,} samples | "
                     f"BPM: {int(bpm) if bpm > 0 else '--'} | "
                     f"RPM: {int(rpm) if rpm > 0 else '--'}",
                fg=self.FG)

        return self.ecg_line, self.resp_line

    def run(self):
        """Start the application."""
        print(f"Starting ECG Monitor on {self.port}...")
        self.reader.start()

        # Give serial time to connect
        time.sleep(0.5)

        self.anim = FuncAnimation(self.fig, self._update_animation,
                                   interval=33, blit=False, cache_frame_data=False)

        try:
            self.root.protocol("WM_DELETE_WINDOW", self._on_close)
            self.root.mainloop()
        except KeyboardInterrupt:
            self._on_close()

    def _on_close(self):
        self.reader.stop()
        self.root.quit()
        self.root.destroy()


# =========================================================================
# Entry Point
# =========================================================================

def main():
    parser = argparse.ArgumentParser(description="SmartBAN ECG & Respiration Monitor GUI")
    parser.add_argument("--port", type=str, default="COM3",
                        help="Serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    # Auto-detect ports if needed
    try:
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if args.port not in ports:
            print(f"WARNING: {args.port} not found. Available ports: {ports}")
            if ports:
                args.port = ports[-1]
                print(f"  Using {args.port} instead.")
    except Exception:
        pass

    app = ECGMonitorApp(args.port, args.baud)
    app.run()


if __name__ == "__main__":
    main()
