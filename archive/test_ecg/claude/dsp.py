"""
Signal conditioning + R-peak detection, run on a rolling window rather
than sample-by-sample -- simplest way to get filtfilt's zero-phase
filtering (no lag/distortion on the plotted waveform) without implementing
causal IIR state persistence between GUI update ticks.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import signal


@dataclass
class DspResult:
    filtered: np.ndarray          # filtered ECG, same length as input
    peak_indices: np.ndarray      # indices into `filtered` where an R-peak was found
    heart_rate_bpm: Optional[float]


class ECGProcessor:
    def __init__(self, fs: int, mains_hz: float = 50.0):
        self.fs = fs
        nyq = fs / 2.0

        # 0.5 Hz high-pass: removes baseline wander
        self._b_hp, self._a_hp = signal.butter(2, 0.5 / nyq, btype="high")
        # 40 Hz low-pass: removes EMG/muscle noise and high-frequency junk
        self._b_lp, self._a_lp = signal.butter(4, 40.0 / nyq, btype="low")
        # notch at mains frequency (50 Hz in Finland; pass 60.0 for US-wired setups)
        self._b_notch, self._a_notch = signal.iirnotch(mains_hz / nyq, Q=30.0)

        # Keep last few detected peak timestamps (in samples, absolute count)
        # so heart rate is computed from real RR intervals across window
        # boundaries, not just whatever fits in one window.
        self._recent_peak_times: list[float] = []
        self._total_samples_seen = 0

    def _filter(self, x: np.ndarray) -> np.ndarray:
        # filtfilt needs the window to be a few times longer than the
        # filter's settling time; skip filtering (return raw) on windows
        # that are still too short, e.g. right after startup.
        min_len = 3 * max(len(self._b_hp), len(self._b_lp), len(self._b_notch)) * 3
        if len(x) < min_len:
            return x.copy()
        x = signal.filtfilt(self._b_hp, self._a_hp, x)
        x = signal.filtfilt(self._b_notch, self._a_notch, x)
        x = signal.filtfilt(self._b_lp, self._a_lp, x)
        return x

    def _detect_peaks(self, filtered: np.ndarray) -> np.ndarray:
        if len(filtered) < self.fs:  # need at least ~1s of data
            return np.array([], dtype=int)

        # Simplified Pan-Tompkins: derivative -> square -> moving-window
        # integration -> adaptive threshold via scipy's find_peaks.
        deriv = np.diff(filtered, prepend=filtered[0])
        squared = deriv ** 2
        win = max(1, int(0.15 * self.fs))  # ~150 ms integration window
        kernel = np.ones(win) / win
        integrated = np.convolve(squared, kernel, mode="same")

        peak_max = np.max(integrated)
        if peak_max <= 0:
            return np.array([], dtype=int)

        threshold = 0.4 * peak_max
        min_distance = max(1, int(0.3 * self.fs))  # refractory period, caps ~200 BPM

        peaks, _ = signal.find_peaks(integrated, height=threshold, distance=min_distance)
        return peaks

    def _update_heart_rate(self, filtered: np.ndarray, peak_indices: np.ndarray, window_start_sample: int) -> Optional[float]:
        # Convert this window's peak indices to absolute sample counts so
        # RR intervals are correct across successive (overlapping) windows.
        abs_times = [(idx + window_start_sample) / self.fs for idx in peak_indices]

        # Overlapping windows re-detect the same beat at slightly different
        # sample offsets (filtfilt edge effects shift peaks by a few ms
        # depending on where the window boundary falls) -- dedupe by
        # proximity, not exact equality, or near-duplicates inflate HR.
        min_gap = 0.3  # seconds; same refractory period used in peak detection
        merged = sorted(self._recent_peak_times + abs_times)
        deduped: list[float] = []
        for t in merged:
            if not deduped or (t - deduped[-1]) > min_gap:
                deduped.append(t)

        # Only keep the last ~10 s of peak history to bound memory and let
        # HR adapt if the subject's rate changes.
        cutoff = self._total_samples_seen / self.fs - 10.0
        self._recent_peak_times = [t for t in deduped if t > cutoff]

        if len(self._recent_peak_times) < 2:
            return None

        rr = np.diff(self._recent_peak_times)
        median = np.median(rr)
        good = rr[(rr > 0.5 * median) & (rr < 1.5 * median)]
        if len(good) == 0:
            return None
        return 60.0 / float(np.mean(good))

    def process(self, raw_window: np.ndarray, window_start_sample: int) -> DspResult:
        self._total_samples_seen = window_start_sample + len(raw_window)
        filtered = self._filter(raw_window)
        peaks = self._detect_peaks(filtered)
        hr = self._update_heart_rate(filtered, peaks, window_start_sample)
        return DspResult(filtered=filtered, peak_indices=peaks, heart_rate_bpm=hr)
