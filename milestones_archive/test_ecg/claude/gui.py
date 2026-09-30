"""
Live scrolling ECG trace + heart-rate readout.

Pulls samples from whatever source is passed in (SerialSource or
SimulatedSource, see serial_reader.py -- both expose the same poll()
interface), re-filters + re-runs peak detection on a rolling window on a
timer tick, and updates the plot.
"""
from __future__ import annotations

import sys

import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from dsp import ECGProcessor

WINDOW_SECONDS = 4.0      # how much history to show on screen
PROCESS_WINDOW_SECONDS = 3.0  # how much history to (re-)filter each tick -- shorter than
                               # the display window is fine since filtfilt only needs
                               # enough samples to settle, not the whole visible trace
UPDATE_INTERVAL_MS = 100  # GUI refresh rate


class EcgWindow(QtWidgets.QWidget):
    def __init__(self, source, fs: int):
        super().__init__()
        self.source = source
        self.fs = fs
        self.processor = ECGProcessor(fs=fs)

        max_samples = int(WINDOW_SECONDS * fs)
        self._raw = np.zeros(max_samples)
        self._sample_count = 0  # total samples ever received, for windowing math

        self._build_ui()

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._on_tick)
        self._timer.start(UPDATE_INTERVAL_MS)

    def _build_ui(self):
        self.setWindowTitle("ECG monitor")
        layout = QtWidgets.QVBoxLayout(self)

        self.hr_label = QtWidgets.QLabel("HR: -- bpm")
        font = self.hr_label.font()
        font.setPointSize(28)
        self.hr_label.setFont(font)
        self.hr_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.hr_label)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status_label)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.setLabel("left", "ECG (raw ADC code)")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot_widget.plot(pen=pg.mkPen(width=1.5))
        self.peak_scatter = pg.ScatterPlotItem(size=10, brush=pg.mkBrush("r"))
        self.plot_widget.addItem(self.peak_scatter)
        layout.addWidget(self.plot_widget)

        self.resize(900, 500)

    def _on_tick(self):
        # Drain everything currently available from the source.
        got_error = False
        n_new = 0
        while True:
            s = self.source.poll()
            if s is None:
                break
            if s.is_error:
                got_error = True
                continue
            self._raw = np.roll(self._raw, -1)
            self._raw[-1] = s.value
            self._sample_count += 1
            n_new += 1

        if got_error:
            self.status_label.setText(
                "Firmware reported a self-test failure -- check SPI/CS/reset wiring "
                "before trusting anything else on screen."
            )
        elif n_new == 0:
            self.status_label.setText("Waiting for data...")
        else:
            self.status_label.setText("")

        # Process only the most recent PROCESS_WINDOW_SECONDS for filtering/
        # peak detection -- keeps filtfilt cheap regardless of how much
        # history is shown on screen.
        proc_len = int(PROCESS_WINDOW_SECONDS * self.fs)
        proc_window = self._raw[-proc_len:]
        window_start_sample = max(0, self._sample_count - proc_len)

        result = self.processor.process(proc_window, window_start_sample)

        t = (np.arange(len(self._raw)) - len(self._raw)) / self.fs
        self.curve.setData(t, self._raw)

        # Map peak indices (within proc_window) back onto the displayed
        # array's time axis so the red dots land on the right samples.
        offset_into_display = len(self._raw) - len(proc_window)
        peak_t = t[offset_into_display + result.peak_indices] if len(result.peak_indices) else []
        peak_y = (
            self._raw[offset_into_display + result.peak_indices]
            if len(result.peak_indices)
            else []
        )
        self.peak_scatter.setData(peak_t, peak_y)

        if result.heart_rate_bpm:
            self.hr_label.setText(f"HR: {result.heart_rate_bpm:.0f} bpm")
        elif not got_error:
            self.hr_label.setText("HR: -- bpm")

    def closeEvent(self, event):
        self.source.close()
        super().closeEvent(event)


def run_gui(source, fs: int):
    app = QtWidgets.QApplication(sys.argv)
    win = EcgWindow(source, fs)
    win.show()
    sys.exit(app.exec_())
