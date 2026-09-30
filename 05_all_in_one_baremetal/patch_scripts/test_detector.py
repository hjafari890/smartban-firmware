import numpy as np
import collections
import math

class BiquadFilter:
    def __init__(self, b, a):
        self.b = b
        self.a = a
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def process(self, x):
        if math.isnan(x) or math.isinf(x):
            x = 0.0
        x = max(-25.0, min(25.0, x))
        y = self.b[0]*x + self.b[1]*self.x1 + self.b[2]*self.x2 - self.a[1]*self.y1 - self.a[2]*self.y2
        if math.isnan(y) or math.isinf(y):
            y = 0.0
            self.reset()
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y

    def reset(self):
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


class RobustECGDetector:
    def __init__(self, fs=180.0):
        self.fs = fs
        # 140 ms MWI
        self.mwi_len = max(int(0.14 * fs), 4)
        self.mwi_buf = collections.deque(maxlen=self.mwi_len)
        self.mwi_sum = 0.0
        self.x_buf = collections.deque([0.0] * 5, maxlen=5)
        
        self.spki = 0.001
        self.npki = 0.00005
        self.threshold1 = 0.0002
        self.threshold2 = 0.0001
        
        # 280 ms Refractory Period (max 214 BPM)
        self.refractory_samples = int(0.28 * fs)
        self.samples_since_peak = self.refractory_samples + 1
        self.rr_intervals = collections.deque(maxlen=8)
        self.bpm = 0.0
        self.beat_detected = False

    def process_sample(self, ecg_mv):
        self.x_buf.append(ecg_mv)
        if len(self.x_buf) < 5:
            return False
            
        # 5-point centered derivative
        deriv = (2.0*self.x_buf[4] + self.x_buf[3] - self.x_buf[1] - 2.0*self.x_buf[0]) / 8.0
        sq = deriv * deriv
        
        if len(self.mwi_buf) == self.mwi_len:
            self.mwi_sum -= self.mwi_buf[0]
        self.mwi_buf.append(sq)
        self.mwi_sum += sq
        mwi = self.mwi_sum / self.mwi_len
        
        self.samples_since_peak += 1
        r_detected = False
        
        # Expected RR
        valid_rr_hist = [r for r in self.rr_intervals if 0.30 <= (r / self.fs) <= 1.8]
        expected_rr = np.median(valid_rr_hist) if len(valid_rr_hist) >= 2 else (0.75 * self.fs)
        
        if self.samples_since_peak > self.refractory_samples:
            thresh = self.threshold1
            # Searchback condition: missed beat
            if self.samples_since_peak > int(1.35 * expected_rr):
                thresh = self.threshold2
                
            if mwi > thresh:
                r_detected = True
                prev_interval = self.samples_since_peak
                self.samples_since_peak = 0
                
                self.spki = 0.125 * mwi + 0.875 * self.spki
                
                if prev_interval > 0:
                    self.rr_intervals.append(prev_interval)
                    
                valid_rr = [r for r in self.rr_intervals if 0.30 <= (r / self.fs) <= 1.8]
                if valid_rr:
                    med_rr = float(np.median(valid_rr))
                    calc_bpm = 60.0 * self.fs / med_rr
                    if 40 <= calc_bpm <= 200:
                        if self.bpm == 0.0:
                            self.bpm = calc_bpm
                        else:
                            self.bpm = 0.30 * calc_bpm + 0.70 * self.bpm
            else:
                if mwi > self.npki:
                    self.npki = 0.125 * mwi + 0.875 * self.npki
                    
        self.threshold1 = self.npki + 0.25 * (self.spki - self.npki)
        self.threshold2 = 0.5 * self.threshold1
        if self.threshold1 < 0.00003:
            self.threshold1 = 0.00003
            
        self.beat_detected = r_detected
        return r_detected
