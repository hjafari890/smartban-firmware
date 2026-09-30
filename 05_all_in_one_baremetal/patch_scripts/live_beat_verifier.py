import serial
import json
import time
import numpy as np
from test_detector import RobustECGDetector, design_highpass_biquad, design_lowpass_biquad, design_notch_biquad

s = serial.Serial('COM3', 115200, timeout=0.5)
time.sleep(0.2)
s.reset_input_buffer()

print('Reading live ECG stream for 7 seconds with clinical cascaded filters...')
t0 = time.time()
fs = 180.0
hp = design_highpass_biquad(0.5, fs)
lp = design_lowpass_biquad(35.0, fs)
notch = design_notch_biquad(50.0, fs, Q=30.0)
det = RobustECGDetector(fs=fs)

samples = []
beats = 0

while time.time() - t0 < 7.0:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":'):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            raw_mv = e2 * 0.000048077
            # Cascade: HP (0.5Hz) -> LP (35Hz) -> Notch (50Hz)
            s1 = hp.process(raw_mv)
            s2 = lp.process(s1)
            filt_mv = notch.process(s2)
            
            samples.append(filt_mv)
            # Skip first 100 samples to let filter settle
            if len(samples) > 100:
                is_beat = det.process_sample(filt_mv)
                if is_beat:
                    beats += 1
                    print(f'  💓 HEARTBEAT #{beats:02d} at {time.time()-t0:.2f}s | Instant BPM: {det.bpm:.1f} | Peak: {filt_mv:+.3f} mV')
        except Exception as err:
            pass

s.close()
print(f'\n========================================')
print(f'Summary: Total samples: {len(samples)} ({len(samples)/7.0:.1f} Hz)')
print(f'Total cardiac beats detected: {beats}')
print(f'Average Heart Rate: {det.bpm:.1f} BPM')
print(f'========================================')
