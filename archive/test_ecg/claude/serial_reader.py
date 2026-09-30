"""
Decodes the firmware's 8-byte UART frame protocol (see
firmware/uart_stream.h for the authoritative layout) and exposes two
interchangeable sample sources:

- SerialSource: reads real frames from the CC2652R1 over a serial port.
- SimulatedSource: generates a synthetic ECG-like waveform with no
  hardware attached, so you can build/test the filtering, peak-detection
  and GUI code before the firmware side is even flashed.

Both are simple iterators that yield (seq, sample) tuples, or None if
nothing is available yet (non-blocking).
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from typing import Optional, Tuple

SYNC0 = 0xAA
SYNC1 = 0x55
FRAME_LEN = 8
ERROR_SEQ = 0xFFFF


@dataclass
class Sample:
    seq: int
    value: int
    is_error: bool = False


class FrameDecoder:
    """Feed raw bytes in with push(); pop completed, checksum-valid frames
    out with poll(). Handles partial reads and resyncs on a bad checksum
    or a corrupted sync sequence instead of getting permanently stuck."""

    def __init__(self):
        self._buf = bytearray()

    def push(self, data: bytes) -> None:
        self._buf.extend(data)

    def poll(self) -> Optional[Sample]:
        while True:
            # Resync: drop bytes until we see the sync pair at buf[0:2].
            while len(self._buf) >= 2 and not (
                self._buf[0] == SYNC0 and self._buf[1] == SYNC1
            ):
                del self._buf[0]

            if len(self._buf) < FRAME_LEN:
                return None

            frame = self._buf[:FRAME_LEN]
            seq = (frame[2] << 8) | frame[3]
            raw24 = (frame[4] << 16) | (frame[5] << 8) | frame[6]
            checksum = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]

            if checksum != frame[7]:
                # Bad frame -- drop the leading sync byte and try resyncing
                # from the next byte rather than discarding the whole thing.
                del self._buf[0]
                continue

            del self._buf[:FRAME_LEN]

            if seq == ERROR_SEQ:
                return Sample(seq=seq, value=raw24, is_error=True)

            # sign-extend 24-bit two's complement
            if raw24 & 0x800000:
                raw24 -= 1 << 24
            return Sample(seq=seq, value=raw24, is_error=False)


class SerialSource:
    """Reads frames from a real serial port. Requires `pyserial`."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # local import so SimulatedSource works without pyserial installed

        self._ser = serial.Serial(port, baud, timeout=0)
        self._decoder = FrameDecoder()

    def poll(self) -> Optional[Sample]:
        n = self._ser.in_waiting
        if n:
            self._decoder.push(self._ser.read(n))
        return self._decoder.poll()

    def close(self):
        self._ser.close()


class SimulatedSource:
    """Generates a synthetic ECG-like waveform at a configurable sample
    rate and heart rate, with a little amplitude/timing jitter so the
    filtering and peak-detection code has something non-trivial to chew
    on. Not a substitute for real electrode data -- just enough to
    validate the PC-side pipeline end to end without hardware."""

    def __init__(self, fs: int = 500, bpm: float = 70.0):
        self.fs = fs
        self.bpm = bpm
        self._t = 0.0
        self._seq = 0
        self._sample_period = 1.0 / fs
        self._wall_start = time.monotonic()

    def _beat_shape(self, phase: float) -> float:
        # Crude PQRST approximation: a tall narrow spike for the R wave
        # plus small P/T bumps, built from a few gaussians. Not clinically
        # meaningful -- purely so the waveform "looks like" ECG for testing.
        def gauss(center, width, height):
            return height * math.exp(-((phase - center) ** 2) / (2 * width ** 2))

        return (
            gauss(0.15, 0.02, 0.15)   # P wave
            + gauss(0.30, 0.006, -0.10)  # Q dip
            + gauss(0.32, 0.006, 1.0)    # R spike
            + gauss(0.34, 0.008, -0.25)  # S dip
            + gauss(0.55, 0.04, 0.20)    # T wave
        )

    def poll(self) -> Optional[Sample]:
        # Pace ourselves against wall-clock time, same as a real serial
        # stream would -- otherwise this generates samples as fast as the
        # GUI's drain loop calls poll(), which never terminates.
        if (time.monotonic() - self._wall_start) < self._t:
            return None

        rr = 60.0 / (self.bpm + random.uniform(-1.5, 1.5))
        phase = (self._t % rr) / rr
        value = self._beat_shape(phase)
        value += random.gauss(0, 0.01)          # sensor noise
        value += 0.03 * math.sin(2 * math.pi * 50 * self._t)  # mains hum, left in on purpose

        sample = int(value * 200000)  # scale roughly into ADC-code range
        seq = self._seq & 0xFFFF
        self._seq += 1
        self._t += self._sample_period

        return Sample(seq=seq, value=sample, is_error=False)

    def close(self):
        pass
