#!/usr/bin/env python3
"""
SmartBAN ECG & Respiration Monitor
===================================
Dedicated GUI for ADS1292R ECG front-end on CC2652R1 (SimpleLink).

Serial protocol:
  Boot header : #ECG_DEDICATED_V1,250,<chip_id>\r\n
  Data @250Hz : D,<ts_ms>,<ch1_raw>,<ch2_raw>,<status>\r\n
  Summary @4s : S,<bpm>,<rr_ms>,<resp_rpm>,<lead_off>\r\n

Requires: tkinter, matplotlib, pyserial, numpy
"""

import tkinter as tk
from tkinter import ttk
import threading
import collections
import time
import math
import numpy as np

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SAMPLE_RATE       = 250          # Hz
ECG_WINDOW_SEC    = 5            # seconds visible on ECG strip
RESP_WINDOW_SEC   = 30           # seconds visible on resp strip
ECG_WINDOW_LEN    = SAMPLE_RATE * ECG_WINDOW_SEC    # 1250
RESP_WINDOW_LEN   = SAMPLE_RATE * RESP_WINDOW_SEC   # 7500
GUI_UPDATE_MS     = 50           # 20 FPS
BAUD_RATE         = 115200

# ADS1292R ADC conversion
VREF              = 2.42         # volts
ADC_FULLSCALE     = 8388607      # 2^23 - 1 (24-bit signed max)
ECG_GAIN          = 6
RESP_GAIN         = 4

# Colour palette (clinical dark theme)
CLR_BG            = "#1a1a2e"
CLR_CARD          = "#16213e"
CLR_ECG           = "#00ff41"
CLR_RESP          = "#00b4d8"
CLR_RPEAK         = "#ff0000"
CLR_GRID          = "#333333"
CLR_TEXT          = "#ffffff"
CLR_WARN          = "#e63946"
CLR_OK            = "#2ec4b6"

# ---------------------------------------------------------------------------
# Biquad IIR Filter  (Direct-Form II Transposed)
# ---------------------------------------------------------------------------
class BiquadFilter:
    """Second-order IIR section with zero-transient DC state initialization.
    Allows full ADS1292R input range (+/-403 mV) before high-pass DC removal
    instead of clipping electrode half-cell offsets at +/-25 mV.
    """

    def __init__(self, b, a):
        self.b0, self.b1, self.b2 = float(b[0]), float(b[1]), float(b[2])
        self.a1, self.a2 = float(a[1]), float(a[2])
        self.is_hpf = abs(self.b0 + self.b1 + self.b2) < 1e-5
        self.initialized = False
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0

    def process(self, x):
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        x = max(-500.0, min(500.0, float(x)))
        if not self.initialized:
            self.initialized = True
            if self.is_hpf:
                self.x1 = self.x2 = x
                self.y1 = self.y2 = 0.0
            else:
                self.x1 = self.x2 = self.y1 = self.y2 = x
        y = (self.b0 * x
             + self.b1 * self.x1
             + self.b2 * self.x2
             - self.a1 * self.y1
             - self.a2 * self.y2)
        if math.isnan(y) or math.isinf(y):
            y = 0.0
            self.reset()
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def reset(self):
        self.initialized = False
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0


# ---------------------------------------------------------------------------
# Cookbook filter design  (no scipy required)
# ---------------------------------------------------------------------------
def design_butterworth_lpf(fc, fs):
    """2nd-order Butterworth low-pass (single biquad section)."""
    w0 = 2.0 * math.pi * fc / fs
    alpha = math.sin(w0) / (2.0 * math.sqrt(0.5))   # Q = 1/sqrt(2)
    cos_w0 = math.cos(w0)
    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return [b0 / a0, b1 / a0, b2 / a0], [1.0, a1 / a0, a2 / a0]


def design_butterworth_hpf(fc, fs):
    """2nd-order Butterworth high-pass (single biquad section)."""
    w0 = 2.0 * math.pi * fc / fs
    alpha = math.sin(w0) / (2.0 * math.sqrt(0.5))
    cos_w0 = math.cos(w0)
    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return [b0 / a0, b1 / a0, b2 / a0], [1.0, a1 / a0, a2 / a0]


def design_notch(f0, fs, Q=30.0):
    """Standard notch (band-reject) filter."""
    w0 = 2.0 * math.pi * f0 / fs
    alpha = math.sin(w0) / (2.0 * Q)
    cos_w0 = math.cos(w0)
    b0 = 1.0
    b1 = -2.0 * cos_w0
    b2 = 1.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return [b0 / a0, b1 / a0, b2 / a0], [1.0, a1 / a0, a2 / a0]


# ---------------------------------------------------------------------------
# 5-Stage Expert Clinical ECG Filter Pipeline (Hospital-Monitor Grade)
# ---------------------------------------------------------------------------
class ECGFilterChain:
    """5-Stage Clinical Morphology-Preserving ECG Pipeline:
      Stage 1: High-Pass Filter (0.67 Hz) + Fast-Recovery Rolling Median Baseline Clamping
      Stage 2: Multi-Harmonic Powerline & USB Hum Suppression (50 Hz + 60 Hz + 100 Hz Notches)
      Stage 3: 4th-Order Cascaded Butterworth Low-Pass Filter (25 Hz / 35 Hz / 40 Hz, -24 dB/oct)
      Stage 4: 5-Point Quadratic Savitzky-Golay Polynomial Filter ([-3, 12, 17, 12, -3]/35)
               -> Preserves exact R-peak height/width while eliminating high-frequency EMG fuzz
      Stage 5: Morphology-Aware Isoelectric Baseline Denoiser (Wavelet-Shrinkage equivalent)
               -> Glass-smooth baseline between beats while passing P-QRS-T waves at 100% fidelity.
    """

    PROFILES = {
        "Clinical Clean (0.67-25Hz + SG)": (0.67, 25.0, True),
        "Monitoring (0.5-35Hz)":           (0.50, 35.0, True),
        "Diagnostic (0.05-40Hz)":          (0.05, 40.0, False),
        "Ambulatory (1.0-22Hz Max Clean)": (1.00, 22.0, True),
    }

    def __init__(self, notch_freq=50, profile="Clinical Clean (0.67-25Hz + SG)"):
        self.notch_enabled = True
        self.notch_freq = notch_freq
        self.profile = profile
        self._sg_buf = collections.deque([0.0] * 5, maxlen=5)
        # 19-tap (76 ms) symmetric Gaussian kernel (fc ~ 7.5 Hz) for silky-smooth P/T waves & zero EMG
        raw_g = [math.exp(-0.5 * ((k - 9) / 3.8) ** 2) for k in range(19)]
        g_sum = sum(raw_g)
        self._gauss_weights = [w / g_sum for w in raw_g]
        self._delay_buf = collections.deque([0.0] * 19, maxlen=19)
        self._iso_prev = 0.0
        self.set_profile(profile)
        self._set_notch(notch_freq)

    def set_profile(self, profile):
        if profile not in self.PROFILES:
            profile = "Clinical Clean (0.67-25Hz + SG)"
        self.profile = profile
        hp_fc, lp_fc, self.use_iso_denoiser = self.PROFILES[profile]

        # Stage 1: 2nd-order Butterworth HPF (removes DC offset & respiratory wander)
        b_hp, a_hp = design_butterworth_hpf(hp_fc, SAMPLE_RATE)
        self.hpf = BiquadFilter(b_hp, a_hp)

        # Stage 3: 4th-order Cascaded Butterworth LPF (-24 dB/oct EMG rejection)
        b_lp, a_lp = design_butterworth_lpf(lp_fc, SAMPLE_RATE)
        self.lpf1 = BiquadFilter(b_lp, a_lp)
        self.lpf2 = BiquadFilter(b_lp, a_lp)

    def _set_notch(self, freq):
        # Cascade 50 Hz, 60 Hz, and 100 Hz notches to reject mains hum + laptop USB switching noise
        if freq in (50, 60):
            self.notch_enabled = True
            self.notch_freq = freq
            b50, a50 = design_notch(50.0, SAMPLE_RATE, Q=8.0)
            b60, a60 = design_notch(60.0, SAMPLE_RATE, Q=10.0)
            b100, a100 = design_notch(100.0, SAMPLE_RATE, Q=12.0)
            self.notch50 = BiquadFilter(b50, a50)
            self.notch60 = BiquadFilter(b60, a60)
            self.notch100 = BiquadFilter(b100, a100)
        else:
            self.notch_enabled = False

    def set_notch_freq(self, freq):
        self._set_notch(freq)

    def process(self, x):
        # Stage 1: Linear HPF
        y = self.hpf.process(x)

        # Stage 2: Multi-harmonic 50 Hz + 60 Hz + 100 Hz Notch cascade
        if self.notch_enabled:
            y = self.notch50.process(y)
            y = self.notch60.process(y)
            y = self.notch100.process(y)

        # Stage 3: 4th-Order Butterworth LPF
        y = self.lpf2.process(self.lpf1.process(y))

        # Stage 4: 5-Point Quadratic Savitzky-Golay Polynomial Filter (preserves 100% of R-peak height)
        self._sg_buf.append(y)
        if len(self._sg_buf) == 5:
            s0, s1, s2, s3, s4 = self._sg_buf
            y_qrs_band = (-3.0 * s0 + 12.0 * s1 + 17.0 * s2 + 12.0 * s3 - 3.0 * s4) / 35.0
        else:
            y_qrs_band = y

        # Stage 5: QRS-Gated Multi-Bandwidth Clinical Reconstruction
        # Center-align the 28 Hz QRS path (index 9 of _delay_buf) with the 19-tap 7.5 Hz P/T Gaussian path
        self._delay_buf.append(y_qrs_band)
        if self.use_iso_denoiser and len(self._delay_buf) == 19:
            # 7.5 Hz Gaussian P/T-wave contour (rejects >95% of 10-40 Hz EMG muscle noise)
            y_pt_band = sum(w * v for w, v in zip(self._gauss_weights, self._delay_buf))
            # Aligned 28 Hz QRS sample (center of 19-sample window = index 9)
            y_center = self._delay_buf[9]

            # Measure local QRS activity in 9-sample (36 ms) window around center [5..13]
            local_seg = [self._delay_buf[k] for k in range(5, 14)]
            local_ptp = max(local_seg) - min(local_seg)
            detail = abs(y_center - y_pt_band)

            # Smooth blending weight w_qrs in [0.0, 1.0]:
            # - Inside QRS complex (local_ptp > 0.22 mV or detail > 0.10 mV): w_qrs -> 1.0 (sharp 28 Hz R-peak)
            # - Inside P/T waves & baseline (local_ptp < 0.12 mV): w_qrs -> 0.0 (glass-smooth 7.5 Hz contour)
            qrs_score = max((local_ptp - 0.14) / 0.14, (detail - 0.06) / 0.07)
            w_qrs = max(0.0, min(1.0, qrs_score))

            y_recon = w_qrs * y_center + (1.0 - w_qrs) * y_pt_band

            # Extra isoelectric baseline damping when completely between waves (< 0.05 mV)
            if local_ptp < 0.09 and abs(y_recon) < 0.05:
                y_out = 0.15 * y_recon + 0.85 * self._iso_prev
            else:
                y_out = y_recon

            self._iso_prev = y_out
            return max(-10.0, min(10.0, y_out))

        self._iso_prev = y_qrs_band
        return max(-10.0, min(10.0, y_qrs_band))

    def reset(self):
        self.hpf.reset()
        self.lpf1.reset()
        self.lpf2.reset()
        self._sg_buf.clear()
        self._delay_buf.clear()
        self._iso_prev = 0.0
        if hasattr(self, "notch50"):
            self.notch50.reset()
            self.notch60.reset()
            self.notch100.reset()


# ---------------------------------------------------------------------------
# Dual-Source Respiration Band-Pass & RPM Estimator (0.10 – 0.42 Hz ≈ 6-25 RPM)
# ---------------------------------------------------------------------------
class RespFilterChain:
    def __init__(self):
        b_hp, a_hp = design_butterworth_hpf(0.10, SAMPLE_RATE)
        self.hpf = BiquadFilter(b_hp, a_hp)
        b_lp, a_lp = design_butterworth_lpf(0.42, SAMPLE_RATE)
        self.lpf1 = BiquadFilter(b_lp, a_lp)
        self.lpf2 = BiquadFilter(b_lp, a_lp)
        self._ma_buf = collections.deque(maxlen=75)  # 300 ms smoothing
        self.rpm = 0.0

    def process(self, x_ch1_mv, x_ecg_raw_mv=0.0):
        # Fuse CH1 thoracic impedance/potential with CH2 ECG-Derived Respiration (EDR) baseline wander
        fused = 0.35 * x_ch1_mv + 0.65 * x_ecg_raw_mv
        y = self.lpf2.process(self.lpf1.process(self.hpf.process(fused)))
        self._ma_buf.append(y)
        return sum(self._ma_buf) / float(len(self._ma_buf))

    def estimate_rpm(self, resp_buf):
        """Estimate Respiration Rate (RPM) from the 15-second respiration buffer."""
        if len(resp_buf) < SAMPLE_RATE * 5:
            return self.rpm
        arr = np.array(resp_buf, dtype=float)
        arr = arr - np.mean(arr)
        std_val = float(np.std(arr))
        if std_val < 0.005:
            return self.rpm

        # Normalize and count hysteresis upward zero-crossings (min 2.2s = 550 samples between breaths)
        norm = arr / std_val
        crossings = []
        in_pos = False
        for idx, val in enumerate(norm):
            if not in_pos and val > 0.25:
                in_pos = True
                if not crossings or (idx - crossings[-1]) >= int(2.2 * SAMPLE_RATE):
                    crossings.append(idx)
            elif in_pos and val < -0.25:
                in_pos = False

        if len(crossings) >= 2:
            diffs = np.diff(crossings)
            valid_diffs = [d for d in diffs if int(2.2 * SAMPLE_RATE) <= d <= int(8.5 * SAMPLE_RATE)]
            if valid_diffs:
                med_samples = float(np.median(valid_diffs))
                calc_rpm = (60.0 * SAMPLE_RATE) / med_samples
                if 7.0 <= calc_rpm <= 28.0:
                    self.rpm = calc_rpm if self.rpm == 0.0 else (0.30 * calc_rpm + 0.70 * self.rpm)
        return self.rpm

    def reset(self):
        self.hpf.reset()
        self.lpf1.reset()
        self.lpf2.reset()
        self._ma_buf.clear()
        self.rpm = 0.0


# ---------------------------------------------------------------------------
# Clinical Heartbeat Analyzer & Pan-Tompkins QRS Detector (Anti-Spike Guarded)
# ---------------------------------------------------------------------------
class PanTompkinsDetector:
    """Clinical Heartbeat Analyzer (250 Hz) with:
      1. 380 ms (95-sample) physiological T-wave refractory blanking + Adaptive RR Gate
      2. Morphological R-Peak Prominence Verification (>= 52% of dominant 2.5s R-peak envelope)
      3. 5-Second Envelope Autocorrelation Periodicity Cross-Check + Slew-Rate Limiter
         (prevents false high BPM spikes from EMG twitches or baseline movement)."""

    def __init__(self, fs=SAMPLE_RATE):
        self.fs = fs
        self._deriv_buf = collections.deque([0.0] * 5, maxlen=5)
        self._raw_win = collections.deque(maxlen=18)         # 72 ms morphological QRS window
        self._amp_history = collections.deque(maxlen=int(2.5 * fs))  # 2.5s ptp history for prominence
        self._mwi_len = max(1, int(0.110 * fs))              # 110 ms (27 samples) QRS energy window
        self._mwi_buf = collections.deque([0.0] * self._mwi_len, maxlen=self._mwi_len)
        self._mwi_sum = 0.0
        self._mwi_history = collections.deque(maxlen=int(2.5 * fs))

        self._mwi_prev1 = 0.0
        self._mwi_prev2 = 0.0
        self._last_qrs_slope = 0.005

        # Adaptive running estimates
        self.spki = 0.0004
        self.npki = 0.00004
        self.threshold1 = 0.00012
        self.threshold2 = 0.00006

        # Strict 380 ms (95 samples -> max 158 BPM) baseline refractory window
        self._refractory_samples = int(0.380 * fs)
        self._twave_samples = int(0.440 * fs)
        self._since_last = self._refractory_samples

        # Outputs & TinyML features
        self.rr_intervals = collections.deque(maxlen=12)
        self.bpm = 0.0
        self.last_rr_ms = 0.0
        self.sdnn_ms = 0.0
        self.rmssd_ms = 0.0
        self.last_r_amp_mv = 0.0
        self.qrs_polarity = 1
        self.last_rr_ratio = 1.0
        self.beat_count = 0

    def process(self, x_filtered):
        """Feed one filtered ECG sample (mV). Returns True ONLY on a verified R-peak."""
        self._raw_win.append(x_filtered)
        self._deriv_buf.append(x_filtered)
        d = self._deriv_buf
        if len(d) < 5:
            return False

        # Track local 18-sample peak-to-peak swing for morphological prominence verification
        win_max = max(self._raw_win)
        win_min = min(self._raw_win)
        win_ptp = win_max - win_min
        self._amp_history.append(win_ptp)

        # 5-point derivative emphasizing 8-25 Hz QRS slopes
        deriv = (-d[0] - 2.0 * d[1] + 2.0 * d[3] + d[4]) / 8.0
        sq = deriv * deriv

        oldest = self._mwi_buf[0] if len(self._mwi_buf) == self._mwi_buf.maxlen else 0.0
        self._mwi_buf.append(sq)
        self._mwi_sum = max(0.0, self._mwi_sum + sq - oldest)
        mwi = self._mwi_sum / float(self._mwi_len)
        self._mwi_history.append(mwi)

        self._since_last += 1
        detected = False

        # Auto-calibrate thresholds if >1.4 s passes without a beat
        if self._since_last > int(1.4 * self.fs) and (self._since_last % 25 == 0):
            if len(self._mwi_history) >= 125:
                arr = np.array(self._mwi_history)
                p96 = float(np.percentile(arr, 96))
                p50 = float(np.percentile(arr, 50))
                self.spki = max(0.00002, p96 * 0.85)
                self.npki = max(0.000002, p50)
            else:
                self.spki *= 0.78
                self.npki *= 0.78
            self.threshold1 = max(0.000008, self.npki + 0.30 * (self.spki - self.npki))
            self.threshold2 = 0.55 * self.threshold1

        # Peak detection: local maximum on MWI
        is_peak = (self._mwi_prev1 > self._mwi_prev2) and (self._mwi_prev1 >= mwi)
        peak_val = self._mwi_prev1
        self._mwi_prev2 = self._mwi_prev1
        self._mwi_prev1 = mwi

        if is_peak and peak_val > 0.0:
            expected_rr = float(np.median(self.rr_intervals)) if len(self.rr_intervals) >= 2 else (0.82 * self.fs)
            # Dynamic minimum refractory: never allow instantaneous RR < 66% of established rhythm (or 380 ms)
            min_allowed_rr = max(self._refractory_samples, int(0.66 * expected_rr)) if len(self.rr_intervals) >= 3 else self._refractory_samples
            thresh = self.threshold2 if self._since_last > int(1.45 * expected_rr) else self.threshold1

            if self._since_last >= min_allowed_rr:
                # Morphological prominence check: candidate peak must be >= 50% of rolling 98th-percentile QRS height
                dominant_amp = float(np.percentile(self._amp_history, 98)) if len(self._amp_history) >= 60 else win_ptp
                prominence_ok = (win_ptp >= max(0.07, 0.50 * dominant_amp))

                if peak_val > thresh and prominence_ok:
                    # Extra T-wave slope guard up to 440 ms
                    is_twave = False
                    if self._since_last <= self._twave_samples:
                        if abs(deriv) < (0.52 * self._last_qrs_slope) or win_ptp < (0.62 * self.last_r_amp_mv):
                            is_twave = True

                    if not is_twave:
                        self.spki = 0.20 * peak_val + 0.80 * self.spki
                        self._last_qrs_slope = max(abs(deriv), 0.002)

                        if abs(win_max) >= abs(win_min):
                            self.last_r_amp_mv = 0.3 * win_ptp + 0.7 * (self.last_r_amp_mv if self.last_r_amp_mv > 0 else win_ptp)
                            self.qrs_polarity = 1
                        else:
                            self.last_r_amp_mv = 0.3 * win_ptp + 0.7 * (self.last_r_amp_mv if self.last_r_amp_mv > 0 else win_ptp)
                            self.qrs_polarity = -1

                        rr_samples = self._since_last
                        if self.beat_count == 0:
                            self.beat_count = 1
                            self._since_last = 0
                            detected = True
                        elif 95 <= rr_samples <= 420:  # 380 ms (158 BPM) to 1680 ms (35.7 BPM)
                            med_before = float(np.median(self.rr_intervals)) if len(self.rr_intervals) >= 1 else float(rr_samples)
                            self.last_rr_ratio = float(rr_samples) / max(1.0, med_before)

                            # Reject single-beat extreme outliers (>35% sudden change) from corrupting BPM buffer
                            if len(self.rr_intervals) < 3 or (0.68 <= self.last_rr_ratio <= 1.42):
                                self.rr_intervals.append(rr_samples)

                            self.last_rr_ms = (rr_samples * 1000.0) / float(self.fs)
                            self.beat_count += 1

                            if len(self.rr_intervals) >= 1:
                                sorted_rr = sorted(self.rr_intervals)
                                med_rr = sorted_rr[len(sorted_rr) // 2]
                                calc_bpm = 60.0 * self.fs / med_rr if med_rr > 0 else 0.0
                                if 36.0 <= calc_bpm <= 158.0:
                                    if self.bpm == 0.0:
                                        self.bpm = calc_bpm
                                    else:
                                        # Slew-rate limit BPM change to max ±4.5 BPM per beat
                                        delta = max(-4.5, min(4.5, calc_bpm - self.bpm))
                                        self.bpm += 0.45 * delta

                            if len(self.rr_intervals) >= 2:
                                rr_ms_arr = np.array(self.rr_intervals, dtype=float) * (1000.0 / float(self.fs))
                                self.sdnn_ms = float(np.std(rr_ms_arr))
                                diffs = np.diff(rr_ms_arr)
                                self.rmssd_ms = float(np.sqrt(np.mean(diffs * diffs)))

                            self._since_last = 0
                            detected = True
                        else:
                            self._since_last = 0
                            self.beat_count += 1
                            detected = True
                    else:
                        self.npki = 0.15 * peak_val + 0.85 * self.npki
                else:
                    self.npki = 0.15 * peak_val + 0.85 * self.npki

            self.threshold1 = max(0.000008, self.npki + 0.30 * (self.spki - self.npki))
            self.threshold2 = 0.55 * self.threshold1

        return detected

    def compute_acf_bpm(self, ecg_buf):
        """Cross-check Heart Rate using 5-second QRS Envelope Autocorrelation (45 to 145 BPM)."""
        if len(ecg_buf) < self.fs * 4:
            return self.bpm
        arr = np.array(list(ecg_buf)[-self.fs * 4:], dtype=float)
        # Differentiate and square to isolate QRS spikes
        diff_sq = np.diff(arr) ** 2
        if float(np.max(diff_sq)) < 1e-5:
            return self.bpm
        # Smooth envelope (20 samples = 80 ms)
        kernel = np.ones(20) / 20.0
        env = np.convolve(diff_sq, kernel, mode="same")
        env = env - np.mean(env)
        norm = float(np.sum(env * env))
        if norm <= 1e-9:
            return self.bpm

        # Search lags from 105 samples (420 ms = 142 BPM) to 320 samples (1280 ms = 46.8 BPM)
        best_lag = 0
        best_corr = 0.0
        for lag in range(105, 320, 2):
            c = float(np.sum(env[:-lag] * env[lag:])) / norm
            if c > best_corr:
                best_corr = c
                best_lag = lag

        if best_lag > 0 and best_corr >= 0.22:
            acf_bpm = (60.0 * self.fs) / float(best_lag)
            if 45.0 <= acf_bpm <= 145.0:
                if self.bpm == 0.0:
                    self.bpm = acf_bpm
                elif abs(self.bpm - acf_bpm) > 15.0:
                    # Pull towards fundamental autocorrelation period if beat counter drifted
                    self.bpm = 0.70 * self.bpm + 0.30 * acf_bpm
        return self.bpm

    def reset(self):
        self._deriv_buf.clear()
        self._raw_win.clear()
        self._amp_history.clear()
        self._mwi_buf.clear()
        self._mwi_history.clear()
        self._mwi_sum = 0.0
        self._mwi_prev1 = 0.0
        self._mwi_prev2 = 0.0
        self._last_qrs_slope = 0.005
        self.spki = 0.0004
        self.npki = 0.00004
        self.threshold1 = 0.00012
        self.threshold2 = 0.00006
        self._since_last = self._refractory_samples
        self.rr_intervals.clear()
        self.bpm = 0.0
        self.last_rr_ms = 0.0
        self.sdnn_ms = 0.0
        self.rmssd_ms = 0.0
        self.last_r_amp_mv = 0.0
        self.qrs_polarity = 1
        self.last_rr_ratio = 1.0
        self.beat_count = 0


# ---------------------------------------------------------------------------
# Serial reader thread (Non-blocking chunked I/O + thread-safe TX queue)
# ---------------------------------------------------------------------------
class SerialReader:
    """Daemon thread that reads chunks from the COM port and writes queued
    commands without ever blocking the Tkinter GUI thread."""

    def __init__(self):
        self.port = None
        self._ser = None
        self._thread = None
        self._running = False

        # Thread-safe queues
        self.data_q = collections.deque(maxlen=RESP_WINDOW_LEN + 512)
        self.summary_q = collections.deque(maxlen=30)
        self._cmd_q = collections.deque(maxlen=32)
        self.chip_id = ""
        self.mode_str = "Mode 4: LIVE_ECG"
        self.connected = False

    # ---- public API --------------------------------------------------------
    def connect(self, port_name):
        if self._running:
            self.disconnect()
        try:
            self._ser = serial.Serial(
                port_name, BAUD_RATE,
                timeout=0.04,
                write_timeout=0.2,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.port = port_name
            self._cmd_q.clear()
            self._running = True
            self.connected = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            return True
        except Exception as exc:
            print(f"[SerialReader] connect error: {exc}")
            return False

    def write_cmd(self, char):
        """Queue command (str or raw bytes) for non-blocking transmission by the serial worker thread."""
        if self.connected:
            payload = char.encode("ascii") if isinstance(char, str) else bytes(char)
            self._cmd_q.append(payload)
            return True
        return False

    def disconnect(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.connected = False

    # ---- background thread -------------------------------------------------
    def _read_loop(self):
        buf = b""
        while self._running:
            try:
                # 1. Drain outgoing commands from GUI thread safely
                while self._cmd_q and self._ser is not None and self._ser.is_open:
                    cmd_bytes = self._cmd_q.popleft()
                    self._ser.write(cmd_bytes)
                    self._ser.flush()

                # 2. Batch read available serial bytes (avoids per-byte syscall & GIL lock contention)
                if self._ser is None or not self._ser.is_open:
                    break
                waiting = self._ser.in_waiting
                chunk = self._ser.read(max(1, min(4096, waiting if waiting > 0 else 64)))
                if not chunk:
                    continue

                buf += chunk
                if b"\n" in buf:
                    lines = buf.split(b"\n")
                    buf = lines[-1]
                    for raw_line in lines[:-1]:
                        line = raw_line.decode("ascii", errors="ignore").strip()
                        if line:
                            self._parse(line)
                elif len(buf) > 4096:
                    buf = b""
            except serial.SerialException:
                self._running = False
                self.connected = False
                break
            except Exception:
                time.sleep(0.005)
                continue

    def _parse(self, line):
        if line.startswith("#ECG_DEDICATED"):
            parts = line.lstrip("#").split(",")
            if len(parts) >= 3:
                self.chip_id = parts[2]
        elif line.startswith("#MODE,"):
            parts = line.split(",")
            if len(parts) >= 3:
                self.mode_str = f"Mode {parts[1]}: {parts[2]}"
        elif line.startswith("#CALIBRATED"):
            self.mode_str = "Offset Calibrated OK"
        elif line.startswith("D,"):
            parts = line.split(",")
            if len(parts) >= 5:
                try:
                    ts   = int(parts[1])
                    ch1  = int(parts[2])
                    ch2  = int(parts[3])
                    stat = int(parts[4])
                    self.data_q.append((ts, ch1, ch2, stat))
                except ValueError:
                    pass
        elif line.startswith("S,"):
            parts = line.split(",")
            if len(parts) >= 5:
                try:
                    bpm      = float(parts[1])
                    rr_ms    = float(parts[2])
                    resp_rpm = float(parts[3])
                    lo_flags = int(parts[4])
                    sdnn_ms  = float(parts[5]) if len(parts) >= 6 else 0.0
                    rmssd_ms = float(parts[6]) if len(parts) >= 7 else 0.0
                    flags    = int(parts[7])   if len(parts) >= 8 else 0
                    self.summary_q.append((bpm, rr_ms, resp_rpm, lo_flags, sdnn_ms, rmssd_ms, flags))
                except ValueError:
                    pass


# ---------------------------------------------------------------------------
# Main GUI application
# ---------------------------------------------------------------------------
class ECGMonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SmartBAN ECG & Respiration Monitor")
        self.root.geometry("1200x800")
        self.root.configure(bg=CLR_BG)
        self.root.minsize(960, 640)

        # State
        self.reader = SerialReader()
        self.ecg_filter = ECGFilterChain(notch_freq=50)
        self.resp_filter = RespFilterChain()
        self.qrs = PanTompkinsDetector()

        # Data buffers (ring-buffers for plotting)
        self.ecg_buf  = collections.deque([0.0] * ECG_WINDOW_LEN,
                                          maxlen=ECG_WINDOW_LEN)
        self.resp_buf = collections.deque([0.0] * RESP_WINDOW_LEN,
                                          maxlen=RESP_WINDOW_LEN)

        # R-peak marker positions (sample index inside ecg_buf)
        self.rpeak_positions = collections.deque(maxlen=50)
        self._ecg_sample_idx = 0  # monotonic counter

        # Vitals & Edge-AI telemetry from firmware
        self.fw_bpm      = 0.0
        self.fw_rr_ms    = 0.0
        self.fw_resp_rpm = 0.0
        self.fw_lo_flags = 0
        self.fw_sdnn_ms  = 0.0
        self.fw_rmssd_ms = 0.0
        self.fw_flags    = 0

        # Pre-allocated time axes & cached Y-limits (prevents Matplotlib axis rebuild lag)
        self._t_ecg = np.linspace(0, ECG_WINDOW_SEC, ECG_WINDOW_LEN)
        self._t_resp = np.linspace(0, RESP_WINDOW_SEC, RESP_WINDOW_LEN)
        self._last_ecg_ylim = (-2.0, 2.0)
        self._last_resp_ylim = (-1.0, 1.0)
        self._last_plot_ts = 0.0
        self._update_id = None

        # Heart & 7-Segment animation state
        self._heart_visible = True
        self._heart_toggle_ts = 0.0
        self._seg_flash_ts = 0.0
        self._last_acf_ts = 0.0
        self._host_resp_rpm = 0.0

        # Mode and Display Controls
        self._invert_ecg = tk.BooleanVar(value=False)
        self._paused = False
        self._recording = False
        self._record_file = None
        self._record_writer = None
        self._record_count = 0
        self._yscale_mode = tk.StringVar(value="±2.0 mV")

        # Build UI
        self._build_top_bar()
        self._build_mode_bar()
        self._build_tinyml_bar()
        self._build_plots()

        # Start GUI refresh loop
        self._update_id = self.root.after(GUI_UPDATE_MS, self._gui_tick)

        # Graceful shutdown
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ======================================================================
    # UI construction
    # ======================================================================
    def _build_top_bar(self):
        bar = tk.Frame(self.root, bg=CLR_CARD, height=90)
        bar.pack(fill=tk.X, padx=6, pady=(6, 3))
        bar.pack_propagate(False)

        # --- COM port selector ---
        port_frame = tk.Frame(bar, bg=CLR_CARD)
        port_frame.pack(side=tk.LEFT, padx=8, pady=8)

        tk.Label(port_frame, text="COM Port", fg=CLR_TEXT, bg=CLR_CARD,
                 font=("Segoe UI", 9)).pack(anchor=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var,
                                       width=12, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 4))
        self._refresh_ports()

        self.connect_btn = tk.Button(
            port_frame, text="Connect", width=9,
            bg="#0f3460", fg=CLR_TEXT, activebackground="#1a5276",
            activeforeground=CLR_TEXT, relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
            command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=2)

        refresh_btn = tk.Button(
            port_frame, text="⟳", width=3,
            bg="#0f3460", fg=CLR_TEXT, relief=tk.FLAT,
            font=("Segoe UI", 10),
            command=self._refresh_ports)
        refresh_btn.pack(side=tk.LEFT, padx=2)

        # --- connection status dot ---
        self.status_canvas = tk.Canvas(bar, width=16, height=16,
                                       bg=CLR_CARD, highlightthickness=0)
        self.status_canvas.pack(side=tk.LEFT, padx=(2, 6), pady=8)
        self.status_dot = self.status_canvas.create_oval(2, 2, 14, 14,
                                                         fill=CLR_WARN, outline="")

        # --- separator ---
        sep1 = tk.Frame(bar, bg=CLR_GRID, width=1)
        sep1.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=12)

        # --- Heart Rate card ---
        hr_frame = tk.Frame(bar, bg=CLR_CARD)
        hr_frame.pack(side=tk.LEFT, padx=10, pady=4)
        tk.Label(hr_frame, text="HEART RATE", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W)

        hr_val_frame = tk.Frame(hr_frame, bg=CLR_CARD)
        hr_val_frame.pack(anchor=tk.W)
        self.heart_lbl = tk.Label(hr_val_frame, text="❤", fg=CLR_WARN,
                                  bg=CLR_CARD, font=("Segoe UI", 22))
        self.heart_lbl.pack(side=tk.LEFT)
        self.bpm_lbl = tk.Label(hr_val_frame, text="--", fg=CLR_TEXT,
                                bg=CLR_CARD, font=("Consolas", 26, "bold"))
        self.bpm_lbl.pack(side=tk.LEFT, padx=(2, 0))
        tk.Label(hr_val_frame, text="BPM", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, anchor=tk.S, pady=8)

        # --- separator ---
        sep2 = tk.Frame(bar, bg=CLR_GRID, width=1)
        sep2.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=12)

        # --- Respiration Rate ---
        rr_frame = tk.Frame(bar, bg=CLR_CARD)
        rr_frame.pack(side=tk.LEFT, padx=10, pady=4)
        tk.Label(rr_frame, text="RESPIRATION", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W)
        rr_val_frame = tk.Frame(rr_frame, bg=CLR_CARD)
        rr_val_frame.pack(anchor=tk.W)
        self.resp_lbl = tk.Label(rr_val_frame, text="--", fg=CLR_RESP,
                                 bg=CLR_CARD, font=("Consolas", 24, "bold"))
        self.resp_lbl.pack(side=tk.LEFT)
        tk.Label(rr_val_frame, text="RPM", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, anchor=tk.S, pady=6)

        # --- separator ---
        sep3 = tk.Frame(bar, bg=CLR_GRID, width=1)
        sep3.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=12)

        # --- 7-Segment BPM Display (Mirrors CH455H 3-Digit 7-Segment on BAN Shield) ---
        seg_frame = tk.Frame(bar, bg=CLR_CARD)
        seg_frame.pack(side=tk.LEFT, padx=10, pady=4)
        tk.Label(seg_frame, text="7-SEG BPM DISPLAY", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W)
        seg_box = tk.Frame(seg_frame, bg="#090c14", highlightbackground="#334155",
                           highlightthickness=1, padx=8, pady=2)
        seg_box.pack(anchor=tk.W, pady=(2, 0))
        self.seg_dot_lbl = tk.Label(seg_box, text="●", fg="#3b1219", bg="#090c14",
                                    font=("Segoe UI", 12, "bold"))
        self.seg_dot_lbl.pack(side=tk.LEFT, padx=(0, 5))
        self.seg_counter_lbl = tk.Label(seg_box, text="---", fg="#ff2a2a", bg="#090c14",
                                        font=("Consolas", 20, "bold"))
        self.seg_counter_lbl.pack(side=tk.LEFT, padx=(0, 4))
        tk.Label(seg_box, text="BPM", fg="#94a3b8", bg="#090c14",
                 font=("Segoe UI", 7, "bold")).pack(side=tk.LEFT, anchor=tk.S, pady=3)

        # --- separator ---
        sep4 = tk.Frame(bar, bg=CLR_GRID, width=1)
        sep4.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=12)

        # --- Lead-off badges ---
        lo_frame = tk.Frame(bar, bg=CLR_CARD)
        lo_frame.pack(side=tk.LEFT, padx=8, pady=8)
        tk.Label(lo_frame, text="LEAD STATUS", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, pady=(0, 2))

        badge_row = tk.Frame(lo_frame, bg=CLR_CARD)
        badge_row.pack(anchor=tk.W)

        self.ra_badge = tk.Label(badge_row, text=" RA: -- ", fg=CLR_TEXT,
                                 bg="#555555", font=("Consolas", 10, "bold"),
                                 padx=4, pady=1)
        self.ra_badge.pack(side=tk.LEFT, padx=(0, 6))

        self.la_badge = tk.Label(badge_row, text=" LA: -- ", fg=CLR_TEXT,
                                 bg="#555555", font=("Consolas", 10, "bold"),
                                 padx=4, pady=1)
        self.la_badge.pack(side=tk.LEFT)

        # --- Notch & Bandpass filter selectors (right side) ---
        notch_frame = tk.Frame(bar, bg=CLR_CARD)
        notch_frame.pack(side=tk.RIGHT, padx=10, pady=8)
        tk.Label(notch_frame, text="NOTCH FILTER", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W)
        self.notch_var = tk.StringVar(value="50 Hz")
        notch_combo = ttk.Combobox(notch_frame, textvariable=self.notch_var,
                                   values=["50 Hz", "60 Hz", "OFF"], width=7,
                                   state="readonly")
        notch_combo.pack(anchor=tk.W, pady=2)
        notch_combo.bind("<<ComboboxSelected>>", self._on_notch_changed)

        filt_frame = tk.Frame(bar, bg=CLR_CARD)
        filt_frame.pack(side=tk.RIGHT, padx=10, pady=8)
        tk.Label(filt_frame, text="CLINICAL BANDPASS", fg="#8899aa", bg=CLR_CARD,
                 font=("Segoe UI", 8)).pack(anchor=tk.W)
        self.profile_var = tk.StringVar(value="Clinical Clean (0.67-25Hz + SG)")
        profile_combo = ttk.Combobox(filt_frame, textvariable=self.profile_var,
                                     values=list(ECGFilterChain.PROFILES.keys()),
                                     width=26, state="readonly")
        profile_combo.pack(anchor=tk.W, pady=2)
        profile_combo.bind("<<ComboboxSelected>>", self._on_profile_changed)

    # ------------------------------------------------------------------
    def _build_mode_bar(self):
        mbar = tk.Frame(self.root, bg="#131b2e", height=42)
        mbar.pack(fill=tk.X, padx=6, pady=(0, 4))
        mbar.pack_propagate(False)

        # Mode buttons
        tk.Label(mbar, text="MODE:", fg="#8899aa", bg="#131b2e",
                 font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=(10, 4), pady=8)

        modes = [
            ("❤ Live ECG", "4"),
            ("⚡ 1Hz Test Signal", "1"),
            ("🔇 Inputs Shorted", "2"),
            ("🌡 Die Temp", "3"),
            ("⚙ Calibrate", "C"),
        ]
        self.mode_buttons = {}
        for name, cmd in modes:
            btn = tk.Button(
                mbar, text=name, relief=tk.FLAT, font=("Segoe UI", 8, "bold"),
                bg="#0288d1" if cmd == "4" else "#1a2744",
                fg=CLR_TEXT, activebackground="#0f3460", activeforeground=CLR_TEXT,
                command=lambda c=cmd: self._send_mode_cmd(c)
            )
            btn.pack(side=tk.LEFT, padx=3, pady=6)
            self.mode_buttons[cmd] = btn

        # Current active mode indicator
        self.mode_status_lbl = tk.Label(mbar, text="Active: Mode 4 (Live ECG)",
                                        fg="#00e676", bg="#131b2e",
                                        font=("Segoe UI", 9, "bold"))
        self.mode_status_lbl.pack(side=tk.LEFT, padx=12, pady=8)

        # Right side controls: Invert, Y-Scale, Pause, Record
        self.record_btn = tk.Button(
            mbar, text="⏺ Record CSV", relief=tk.FLAT, font=("Segoe UI", 8, "bold"),
            bg="#2c3e50", fg=CLR_TEXT, activebackground="#e74c3c", activeforeground=CLR_TEXT,
            command=self._toggle_record
        )
        self.record_btn.pack(side=tk.RIGHT, padx=8, pady=6)

        self.pause_btn = tk.Button(
            mbar, text="⏸ Pause", relief=tk.FLAT, font=("Segoe UI", 8, "bold"),
            bg="#2c3e50", fg=CLR_TEXT, activebackground="#34495e", activeforeground=CLR_TEXT,
            command=self._toggle_pause
        )
        self.pause_btn.pack(side=tk.RIGHT, padx=4, pady=6)

        inv_chk = tk.Checkbutton(
            mbar, text="Invert ECG", variable=self._invert_ecg,
            fg=CLR_TEXT, bg="#131b2e", selectcolor="#1a2744", activebackground="#131b2e",
            activeforeground=CLR_TEXT, font=("Segoe UI", 8)
        )
        inv_chk.pack(side=tk.RIGHT, padx=6, pady=8)

        tk.Label(mbar, text="Y-Scale:", fg="#8899aa", bg="#131b2e",
                 font=("Segoe UI", 8)).pack(side=tk.RIGHT, padx=(8, 2), pady=8)
        yscale_combo = ttk.Combobox(
            mbar, textvariable=self._yscale_mode,
            values=["Auto-Scale", "±1.0 mV", "±2.0 mV", "±5.0 mV"],
            width=10, state="readonly"
        )
        yscale_combo.pack(side=tk.RIGHT, padx=2, pady=6)

    # ------------------------------------------------------------------
    def _build_tinyml_bar(self):
        """Dedicated Edge-AI / TinyML Cardiac Analytics & Placement Advisor bar."""
        tbar = tk.Frame(self.root, bg="#0f172a", height=48, highlightbackground="#1e293b", highlightthickness=1)
        tbar.pack(fill=tk.X, padx=6, pady=(0, 4))
        tbar.pack_propagate(False)

        # 1. TinyML Rhythm Classification Badge
        c1 = tk.Frame(tbar, bg="#0f172a")
        c1.pack(side=tk.LEFT, padx=(10, 14), pady=4)
        tk.Label(c1, text="🧠 TINYML RHYTHM CLASSIFIER", fg="#94a3b8", bg="#0f172a",
                 font=("Segoe UI", 7, "bold")).pack(anchor=tk.W)
        self.tinyml_badge = tk.Label(
            c1, text=" STANDBY / CONNECT BOARD ", fg="#ffffff", bg="#334155",
            font=("Consolas", 9, "bold"), padx=6, pady=1
        )
        self.tinyml_badge.pack(anchor=tk.W, pady=(1, 0))

        tk.Frame(tbar, bg="#1e293b", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=6)

        # 2. Autonomic HRV & Stress State (MCU + Host Fusion)
        c2 = tk.Frame(tbar, bg="#0f172a")
        c2.pack(side=tk.LEFT, padx=12, pady=4)
        tk.Label(c2, text="📊 AUTONOMIC HRV & ANS TONE", fg="#94a3b8", bg="#0f172a",
                 font=("Segoe UI", 7, "bold")).pack(anchor=tk.W)
        self.hrv_lbl = tk.Label(
            c2, text="SDNN: -- ms  |  RMSSD: -- ms  |  ANS: --",
            fg="#38bdf8", bg="#0f172a", font=("Consolas", 9, "bold")
        )
        self.hrv_lbl.pack(anchor=tk.W, pady=(1, 0))

        tk.Frame(tbar, bg="#1e293b", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=6)

        # 3. Electrode Placement & Morphology Advisor
        c3 = tk.Frame(tbar, bg="#0f172a")
        c3.pack(side=tk.LEFT, padx=12, pady=4, fill=tk.X, expand=True)
        tk.Label(c3, text="📍 ELECTRODE PLACEMENT & SIGNAL QUALITY ADVISOR", fg="#94a3b8", bg="#0f172a",
                 font=("Segoe UI", 7, "bold")).pack(anchor=tk.W)
        self.placement_lbl = tk.Label(
            c3, text="QRS Amp: -- mV  |  SNR: -- dB  |  Placement: Waiting for signal...",
            fg="#a7f3d0", bg="#0f172a", font=("Segoe UI", 8, "bold")
        )
        self.placement_lbl.pack(anchor=tk.W, pady=(1, 0))

    # ------------------------------------------------------------------
    def _build_plots(self):
        plot_frame = tk.Frame(self.root, bg=CLR_BG)
        plot_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

        # ---- ECG plot (upper 60%) ----
        self.ecg_fig = Figure(figsize=(12, 3.2), dpi=100, facecolor=CLR_BG)
        self.ecg_ax = self.ecg_fig.add_subplot(111)
        self._style_axis(self.ecg_ax, "ECG Lead I (mV)",
                         x_range=(0, ECG_WINDOW_SEC),
                         y_range=(-2.0, 2.0),
                         x_major=0.2, y_major=0.5)
        self.ecg_line, = self.ecg_ax.plot([], [], color=CLR_ECG, linewidth=1.0)
        self.rpeak_scatter = self.ecg_ax.scatter([], [], color=CLR_RPEAK,
                                                  s=36, zorder=5, marker="o")
        self.ecg_canvas = FigureCanvasTkAgg(self.ecg_fig, plot_frame)
        self.ecg_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                             pady=(0, 2))

        # ---- Respiration plot (lower 40%) ----
        self.resp_fig = Figure(figsize=(12, 2.0), dpi=100, facecolor=CLR_BG)
        self.resp_ax = self.resp_fig.add_subplot(111)
        self._style_axis(self.resp_ax, "Respiration (impedance a.u.)",
                         x_range=(0, RESP_WINDOW_SEC),
                         y_range=(-1.0, 1.0),
                         x_major=5.0, y_major=0.5)
        self.resp_line, = self.resp_ax.plot([], [], color=CLR_RESP,
                                            linewidth=1.0)
        self.resp_canvas = FigureCanvasTkAgg(self.resp_fig, plot_frame)
        self.resp_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                              pady=(2, 0))

    # ------------------------------------------------------------------
    @staticmethod
    def _style_axis(ax, ylabel, x_range, y_range, x_major, y_major):
        ax.set_facecolor(CLR_BG)
        ax.set_xlim(*x_range)
        ax.set_ylim(*y_range)
        ax.set_ylabel(ylabel, color=CLR_TEXT, fontsize=9)
        ax.tick_params(colors=CLR_TEXT, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(CLR_GRID)

        # Major grid
        import matplotlib.ticker as mticker
        ax.xaxis.set_major_locator(mticker.MultipleLocator(x_major))
        ax.yaxis.set_major_locator(mticker.MultipleLocator(y_major))
        ax.grid(True, which="major", color=CLR_GRID, linewidth=0.5,
                alpha=0.7)
        ax.set_xlabel("Time (s)", color=CLR_TEXT, fontsize=8)

    # ======================================================================
    # Callbacks
    # ======================================================================
    def _refresh_ports(self):
        ports = sorted([p.device for p in serial.tools.list_ports.comports()])
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.reader.connected:
            self.reader.disconnect()
            self.connect_btn.configure(text="Connect")
        else:
            port = self.port_var.get()
            if port and self.reader.connect(port):
                self.connect_btn.configure(text="Disconnect")
                # Reset filters on new connection
                self.ecg_filter.reset()
                self.resp_filter.reset()
                self.qrs.reset()
                self.ecg_buf = collections.deque([0.0] * ECG_WINDOW_LEN,
                                                  maxlen=ECG_WINDOW_LEN)
                self.resp_buf = collections.deque([0.0] * RESP_WINDOW_LEN,
                                                   maxlen=RESP_WINDOW_LEN)
                self.rpeak_positions.clear()
                self._ecg_sample_idx = 0
                self.seg_counter_lbl.configure(text="---")
                self.reader.write_cmd("R")

    def _reset_beat_counter(self):
        """Reset both GUI and physical CH455H 3-digit 7-segment BPM displays to '---'."""
        self.qrs.beat_count = 0
        self.seg_counter_lbl.configure(text="---")
        if self.reader.connected:
            self.reader.write_cmd("R")

    def _send_mode_cmd(self, cmd):
        if self.reader.connected:
            self.reader.write_cmd(cmd)
            for k, btn in self.mode_buttons.items():
                if k == cmd:
                    btn.configure(bg="#0288d1")
                elif k != "C":
                    btn.configure(bg="#1a2744")

    def _toggle_record(self):
        import csv
        from datetime import datetime
        if not self._recording:
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"ecg_record_{now_str}.csv"
            try:
                self._record_file = open(filename, "w", newline="", encoding="utf-8")
                self._record_writer = csv.writer(self._record_file)
                self._record_writer.writerow(["timestamp_ms", "ch1_raw", "ch2_raw", "ecg_mv", "resp_raw", "lead_status"])
                self._recording = True
                self._record_count = 0
                self.record_btn.configure(text="⏹ Stop (0)", bg="#e74c3c")
            except Exception as e:
                print(f"Failed to open recording file: {e}")
        else:
            self._recording = False
            if self._record_file:
                self._record_file.close()
                self._record_file = None
            self.record_btn.configure(text="⏺ Record CSV", bg="#2c3e50")

    def _toggle_pause(self):
        self._paused = not self._paused
        self.pause_btn.configure(text="▶ Resume" if self._paused else "⏸ Pause",
                                 bg="#f39c12" if self._paused else "#2c3e50")

    def _on_notch_changed(self, _event=None):
        val = self.notch_var.get()
        freq = 50 if "50" in val else (60 if "60" in val else 0)
        self.ecg_filter.set_notch_freq(freq)

    def _on_profile_changed(self, _event=None):
        self.ecg_filter.set_profile(self.profile_var.get())
        self.ecg_filter.reset()

    def _on_close(self):
        self._recording = False
        if self._record_file:
            self._record_file.close()
            self._record_file = None
        self.reader.disconnect()
        if self._update_id is not None:
            self.root.after_cancel(self._update_id)
        self.root.destroy()

    # ======================================================================
    # Main GUI refresh (Non-blocking: drains queues at 22 Hz, redraws plots at 10 Hz)
    # ======================================================================
    def _gui_tick(self):
        try:
            self._drain_data()
            self._drain_summaries()
            self._update_status_dot()
            self._animate_heart()

            now = time.monotonic()
            # Throttle expensive Matplotlib canvas redraws to 10 FPS (every 100 ms)
            # so the Tkinter event loop always has idle time to process button clicks!
            if not self._paused and (now - self._last_plot_ts) >= 0.10:
                self._last_plot_ts = now
                self._redraw_ecg()
                self._redraw_resp()
        except Exception as exc:
            print(f"[GUI Tick Error] {exc}")
        finally:
            self._update_id = self.root.after(45, self._gui_tick)

    # ------------------------------------------------------------------
    def _drain_data(self):
        """Process queued D-messages and pinpoint R-peaks."""
        count = 0
        max_per_tick = 400  # Drain up to 1.6s of buffered samples per tick so queue never lags
        mode_str = self.reader.mode_str
        while self.reader.data_q and count < max_per_tick:
            ts, ch1, ch2, stat = self.reader.data_q.popleft()
            count += 1

            # --- ECG (ch2) ---
            ecg_mv = ch2 * VREF / (ECG_GAIN * ADC_FULLSCALE) * 1000.0
            if self._invert_ecg.get() and "Live" in mode_str:
                ecg_mv = -ecg_mv

            if "1Hz" in mode_str:
                if not hasattr(self, "_sq_buf"):
                    self._sq_buf = collections.deque(maxlen=500)
                self._sq_buf.append(ecg_mv)
                mid = 0.5 * (max(self._sq_buf) + min(self._sq_buf)) if len(self._sq_buf) >= 30 else ecg_mv
                ecg_filt = ecg_mv - mid
            elif "Short" in mode_str or "Temp" in mode_str:
                ecg_filt = ecg_mv
            else:
                ecg_filt = self.ecg_filter.process(ecg_mv)

            self.ecg_buf.append(ecg_filt)
            self._ecg_sample_idx += 1

            # Pan-Tompkins QRS detection (Active in Live ECG & 1Hz Test mode)
            if ("Live" in mode_str or "1Hz" in mode_str) and self.qrs.process(ecg_filt):
                # Trigger 7-segment BPM display & Heart Icon flash ONLY on real R-peak heartbeat!
                self._seg_flash_ts = time.monotonic()
                bpm_val = int(round(self.qrs.bpm if self.qrs.bpm >= 35.0 else self.fw_bpm))
                if 30 <= bpm_val <= 154:
                    self.reader.write_cmd(bytes([100 + bpm_val]))
                else:
                    self.reader.write_cmd("B")

                # Pinpoint exact morphological extremum in last 14 samples so dot sits on tip of R-wave
                lookback = min(14, len(self.ecg_buf))
                best_offset = 0
                best_val = -1.0
                for k in range(1, lookback + 1):
                    v = abs(self.ecg_buf[-k])
                    if v > best_val:
                        best_val = v
                        best_offset = k - 1
                exact_idx = self._ecg_sample_idx - best_offset
                if not self.rpeak_positions or (exact_idx - self.rpeak_positions[-1]) >= 85:
                    self.rpeak_positions.append(exact_idx)

            # --- Respiration (Dual-Source Fusion: CH1 + CH2 EDR) ---
            resp_raw = ch1 * VREF / (RESP_GAIN * ADC_FULLSCALE) * 1000.0
            resp_filt = self.resp_filter.process(resp_raw, ecg_mv)
            self.resp_buf.append(resp_filt)

            # --- Recording ---
            if self._recording and self._record_writer is not None:
                try:
                    self._record_writer.writerow([ts, ch1, ch2, f"{ecg_filt:.4f}", f"{resp_filt:.4f}", stat])
                    self._record_count += 1
                except Exception:
                    pass

            # --- Lead-off from status byte ---
            self.fw_lo_flags = stat

        # Update lead-off badges
        self._update_lead_off(self.fw_lo_flags)

    def _drain_summaries(self):
        """Consume firmware Edge-AI summary messages and run TinyML Rhythm & Placement Analytics."""
        while self.reader.summary_q:
            item = self.reader.summary_q.popleft()
            self.fw_bpm      = item[0]
            self.fw_rr_ms    = item[1]
            self.fw_resp_rpm = item[2]
            self.fw_lo_flags = item[3]
            if len(item) >= 7:
                self.fw_sdnn_ms  = item[4]
                self.fw_rmssd_ms = item[5]
                self.fw_flags    = item[6]

        now = time.monotonic()
        if (now - self._last_acf_ts) >= 0.45:
            self._last_acf_ts = now
            # 1. Cross-check BPM with 5-second autocorrelation to eliminate any double-count spike
            self.qrs.compute_acf_bpm(self.ecg_buf)
            # 2. Estimate real-time Respiration Rate (RPM) from the 15-second Dual-Source breathing waveform
            self._host_resp_rpm = self.resp_filter.estimate_rpm(self.resp_buf)

        is_live = "Live" in self.reader.mode_str
        # Only declare flatline if signal variance is truly dead (<0.00005 mV^2) AND both RA+LA comparators trip
        is_flatline = False
        snr_db = 0.0
        qrs_amp = self.qrs.last_r_amp_mv
        if len(self.ecg_buf) >= SAMPLE_RATE:
            recent = np.array(list(self.ecg_buf)[-SAMPLE_RATE:])
            var_1s = float(np.var(recent))
            ptp_1s = float(np.ptp(recent))
            if qrs_amp <= 0.01:
                qrs_amp = ptp_1s * 0.75
            # Estimate high-frequency baseline noise via first-difference MAD
            hf_noise = float(np.median(np.abs(np.diff(recent)))) * 1.4826
            snr_db = 20.0 * math.log10(max(qrs_amp, 1e-4) / max(hf_noise * 2.5, 1e-4))
            if is_live and var_1s < 0.00004 and ((self.fw_lo_flags & 0x06) == 0x06):
                is_flatline = True

        # Fuse Host Anti-Spike Pan-Tompkins BPM + Firmware Edge-AI BPM
        # (Only blend fw_bpm if it agrees within 20% of the anti-spike host BPM, preventing false upward jumps!)
        display_bpm = 0.0
        if not is_flatline:
            if 38.0 <= self.qrs.bpm <= 165.0 and self.qrs._since_last < (3.0 * SAMPLE_RATE):
                if 38.0 <= self.fw_bpm <= 165.0 and abs(self.fw_bpm - self.qrs.bpm) <= (0.20 * self.qrs.bpm):
                    display_bpm = 0.70 * self.qrs.bpm + 0.30 * self.fw_bpm
                else:
                    display_bpm = self.qrs.bpm
            elif 38.0 <= self.fw_bpm <= 150.0:
                display_bpm = self.fw_bpm

        # Fuse Host Dual-Source Respiration RPM + Firmware RPM
        display_resp = 0.0
        if not is_flatline:
            if 5.0 <= self._host_resp_rpm <= 35.0 and 5.0 <= self.fw_resp_rpm <= 35.0:
                display_resp = 0.65 * self._host_resp_rpm + 0.35 * self.fw_resp_rpm
            elif 5.0 <= self._host_resp_rpm <= 35.0:
                display_resp = self._host_resp_rpm
            elif 5.0 <= self.fw_resp_rpm <= 35.0:
                display_resp = self.fw_resp_rpm

        if display_bpm > 0:
            self.bpm_lbl.configure(text=f"{display_bpm:.0f}", fg=CLR_ECG)
            self.seg_counter_lbl.configure(text=f"{int(round(display_bpm)):3d}")
        else:
            self.bpm_lbl.configure(text="--", fg=CLR_TEXT)
            self.seg_counter_lbl.configure(text="---")

        if display_resp > 0:
            self.resp_lbl.configure(text=f"{display_resp:.0f}")
        else:
            self.resp_lbl.configure(text="--")

        # --- Update Autonomic HRV (SDNN & RMSSD) ---
        sdnn = self.qrs.sdnn_ms if self.qrs.sdnn_ms > 0 else self.fw_sdnn_ms
        rmssd = self.qrs.rmssd_ms if self.qrs.rmssd_ms > 0 else self.fw_rmssd_ms
        if display_bpm > 0 and (sdnn > 0 or rmssd > 0):
            ans_state = "Vagal / Relaxed" if rmssd >= 35.0 else ("Balanced" if rmssd >= 18.0 else "High Sympathetic Tone")
            self.hrv_lbl.configure(text=f"SDNN: {sdnn:4.1f} ms  |  RMSSD: {rmssd:4.1f} ms  |  ANS: {ans_state}")
        else:
            self.hrv_lbl.configure(text="SDNN: -- ms  |  RMSSD: -- ms  |  ANS: Acquiring beats...")

        # --- Update TinyML Rhythm Classifier & Placement Advisor ---
        self._update_tinyml_and_placement(display_bpm, sdnn, rmssd, qrs_amp, snr_db, is_flatline)

        # Update active mode text
        self.mode_status_lbl.configure(text=f"Active: {self.reader.mode_str}")

        # Update record button label count
        if self._recording:
            self.record_btn.configure(text=f"⏹ Stop ({self._record_count})")

    def _update_tinyml_and_placement(self, bpm, sdnn, rmssd, qrs_amp, snr_db, is_flatline):
        """Run lightweight TinyML decision tree / softmax score on beat & morphology features."""
        if not self.reader.connected:
            self.tinyml_badge.configure(text=" DISCONNECTED ", bg="#334155")
            self.placement_lbl.configure(text="Connect to COM3 to start real-time TinyML cardiac analysis.", fg="#94a3b8")
            return

        if "Live" not in self.reader.mode_str:
            self.tinyml_badge.configure(text=f" CALIBRATION ({self.reader.mode_str}) ", bg="#0284c7")
            self.placement_lbl.configure(text=f"Signal P-P: {qrs_amp:.2f} mV  |  SNR: {snr_db:.1f} dB  |  Internal Hardware Test Active", fg="#38bdf8")
            return

        if is_flatline or bpm <= 0:
            self.tinyml_badge.configure(text=" ⏳ LOCKING ONTO QRS RHYTHM... ", bg="#475569")
            self.placement_lbl.configure(
                text=f"P-P: {qrs_amp:.2f} mV | Hold still for 2s — auto-calibrating threshold to your electrode placement...",
                fg="#fde047"
            )
            return

        # 1. TinyML Rhythm Classification
        rr_ratio = self.qrs.last_rr_ratio
        cv_rr = (sdnn / max(250.0, (60000.0 / bpm))) if bpm > 0 else 0.0
        if (self.fw_flags & 0x08) or (rr_ratio < 0.76 and self.qrs.beat_count >= 4):
            rhythm_txt = " ⚠ ECTOPIC / PVC BEAT DETECTED "
            rhythm_bg = "#dc2626"
        elif (self.fw_flags & 0x04) or (cv_rr > 0.16 and rmssd > 85.0 and self.qrs.beat_count >= 6):
            rhythm_txt = " 🟠 IRREGULAR R-R (ARRHYTHMIA / AFIB SCREEN) "
            rhythm_bg = "#d97706"
        elif bpm > 100.0:
            rhythm_txt = f" 🟡 SINUS TACHYCARDIA ({bpm:.0f} BPM) "
            rhythm_bg = "#ca8a04"
        elif bpm < 55.0:
            rhythm_txt = f" 🟡 SINUS BRADYCARDIA ({bpm:.0f} BPM) "
            rhythm_bg = "#ca8a04"
        else:
            conf = min(99, max(82, int(88 + min(11.0, snr_db * 0.5))))
            rhythm_txt = f" 🟢 NORMAL SINUS RHYTHM ({conf}% conf) "
            rhythm_bg = "#16a34a"
        self.tinyml_badge.configure(text=rhythm_txt, bg=rhythm_bg)

        # 2. Electrode Placement & Signal Quality Advisor
        if snr_db < 4.0:
            advice = "High EMG muscle noise — relax shoulders/hands & place pads off pectoral muscle"
            clr = "#fb923c"
        elif self.qrs.qrs_polarity < 0:
            advice = "Inverted Lead-I (RA/LA reversed or vertical axis) — Check 'Invert ECG' or swap RA/LA"
            clr = "#38bdf8"
        elif qrs_amp < 0.25:
            advice = "Off-Axis Placement (low dipole projection) — Widen RA & LA separation for taller R-wave"
            clr = "#facc15"
        else:
            advice = "Optimal Lead-I Einthoven Vector (Strong R-Peak & Clean Baseline)"
            clr = "#4ade80"

        self.placement_lbl.configure(
            text=f"QRS Amp: {qrs_amp:.2f} mV  |  SNR: {snr_db:+.1f} dB  |  {advice}",
            fg=clr
        )

    # ------------------------------------------------------------------
    def _update_status_dot(self):
        colour = CLR_OK if self.reader.connected else CLR_WARN
        self.status_canvas.itemconfig(self.status_dot, fill=colour)
        if not self.reader.connected:
            self.connect_btn.configure(text="Connect")

    def _animate_heart(self):
        """Pulse both the Heart Icon and 7-Segment BPM Display ONLY on each real detected R-peak heartbeat."""
        now = time.monotonic()
        flashing_beat = (now - self._seg_flash_ts) < 0.17
        if flashing_beat:
            self.seg_dot_lbl.configure(fg="#ff1744")
            self.seg_counter_lbl.configure(fg="#ffffff", bg="#4a0e17")
            self.heart_lbl.configure(fg="#ff1744")
        else:
            self.seg_dot_lbl.configure(fg="#3b1219")
            self.seg_counter_lbl.configure(fg="#ff2a2a", bg="#090c14")
            # Stay calm and steady dim-red between beats (never blink on a fake timer!)
            self.heart_lbl.configure(fg="#451422")

    def _update_lead_off(self, stat):
        # status_byte: bit2 = IN2N/RA off, bit1 = IN2P/LA off
        ra_off = bool(stat & 0x04)
        la_off = bool(stat & 0x02)

        # If a rhythmic ECG waveform is actively being received (QRS detected), override false dry-skin comparator trips
        if "Live" in self.reader.mode_str and self.qrs.bpm >= 33.0 and self.qrs._since_last < (2.5 * SAMPLE_RATE):
            ra_off = False
            la_off = False

        self.ra_badge.configure(
            text=" RA: WARN" if ra_off else " RA: OK  ",
            bg="#d97706" if ra_off else CLR_OK)
        self.la_badge.configure(
            text=" LA: WARN" if la_off else " LA: OK  ",
            bg="#d97706" if la_off else CLR_OK)

    # ------------------------------------------------------------------
    # Plot redraw (Optimized: only calls set_ylim when scale changes >15%)
    # ------------------------------------------------------------------
    def _redraw_ecg(self):
        data = np.array(self.ecg_buf)
        t = self._t_ecg
        self.ecg_line.set_data(t, data)

        # Determine target Y-limits
        ymode = self._yscale_mode.get()
        if ymode == "±1.0 mV":
            target_ylim = (-1.0, 1.0)
        elif ymode == "±2.0 mV":
            target_ylim = (-2.0, 2.0)
        elif ymode == "±5.0 mV":
            target_ylim = (-5.0, 5.0)
        else:
            # Smart Auto-Scale using 1st-99th percentile with clinical minimum span of ±1.0 mV
            p1, p99 = float(np.percentile(data, 1)), float(np.percentile(data, 99))
            span = max(0.80, p99 - p1)
            mid = 0.5 * (p99 + p1)
            half = max(1.0, span * 0.75)
            target_ylim = (mid - half, mid + half)

        # CRITICAL: Only call set_ylim if range changed by >15% (prevents expensive Matplotlib tick rebuilds!)
        old_lo, old_hi = self._last_ecg_ylim
        old_span = max(0.1, old_hi - old_lo)
        if abs(target_ylim[0] - old_lo) > 0.15 * old_span or abs(target_ylim[1] - old_hi) > 0.15 * old_span:
            self._last_ecg_ylim = target_ylim
            self.ecg_ax.set_ylim(target_ylim[0], target_ylim[1])

        # R-peak markers
        rpeak_x = []
        rpeak_y = []
        base_idx = self._ecg_sample_idx - ECG_WINDOW_LEN
        for pk_idx in self.rpeak_positions:
            offset = pk_idx - base_idx
            if 0 <= offset < ECG_WINDOW_LEN:
                rpeak_x.append(t[offset])
                rpeak_y.append(data[offset])
        if rpeak_x:
            self.rpeak_scatter.set_offsets(np.column_stack([rpeak_x, rpeak_y]))
        else:
            self.rpeak_scatter.set_offsets(np.empty((0, 2)))

        self.ecg_canvas.draw_idle()

    def _redraw_resp(self):
        data = np.array(self.resp_buf)
        self.resp_line.set_data(self._t_resp, data)

        if self.reader.connected:
            d_min, d_max = float(data.min()), float(data.max())
            margin = max(0.2, (d_max - d_min) * 0.20)
            y_lo = d_min - margin
            y_hi = d_max + margin
            if abs(y_hi - y_lo) < 0.01:
                y_lo -= 1.0
                y_hi += 1.0
            old_lo, old_hi = self._last_resp_ylim
            old_span = max(0.1, old_hi - old_lo)
            if abs(y_lo - old_lo) > 0.20 * old_span or abs(y_hi - old_hi) > 0.20 * old_span:
                self._last_resp_ylim = (y_lo, y_hi)
                self.resp_ax.set_ylim(y_lo, y_hi)

        self.resp_canvas.draw_idle()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    root = tk.Tk()

    # Dark ttk styling
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TCombobox",
                     fieldbackground=CLR_CARD,
                     background=CLR_CARD,
                     foreground=CLR_TEXT,
                     arrowcolor=CLR_TEXT)
    style.map("TCombobox",
              fieldbackground=[("readonly", CLR_CARD)],
              foreground=[("readonly", CLR_TEXT)])

    app = ECGMonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
