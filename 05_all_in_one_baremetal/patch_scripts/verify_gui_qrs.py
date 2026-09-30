import serial
import json
import time
import sys, os
sys.path.insert(0, r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650")
from sensor_gui import design_highpass_biquad, design_lowpass_biquad, design_notch_biquad, PanTompkinsQRS

s = serial.Serial('COM3', 115200, timeout=0.5)
time.sleep(0.2)
s.reset_input_buffer()
t0 = time.time()
fs = 180.0
hp = design_highpass_biquad(0.5, fs)
lp = design_lowpass_biquad(35.0, fs)
notch = design_notch_biquad(50.0, fs, Q=30.0)
qrs = PanTompkinsQRS(fs=fs)

samples = []
detected_beats = []

print("Running live test with sensor_gui.py DSP engine for 6 seconds...")
while time.time() - t0 < 6.0:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode('utf-8', errors='ignore').strip()
    if line.startswith('{"e":['):
        try:
            d = json.loads(line)
            e2 = d['e'][2]
            v = notch.process(lp.process(hp.process(e2 * 0.000048077)))
            samples.append(v)
            if len(samples) > 60:
                is_r = qrs.process(v)
                if is_r:
                    detected_beats.append((time.time() - t0, qrs.bpm))
                    print(f"  💓 R-PEAK #{len(detected_beats):02d} at {time.time()-t0:.2f}s | Live BPM: {qrs.bpm:.1f} | Peak: {v:+.3f} mV")
        except Exception as e:
            pass

s.close()
print("\n" + "="*45)
print(f"Total samples captured: {len(samples)} ({len(samples)/6.0:.1f} Hz)")
print(f"Total R-peaks detected: {len(detected_beats)}")
print(f"Final Locked Heart Rate: {qrs.bpm:.1f} BPM")
print("="*45)
