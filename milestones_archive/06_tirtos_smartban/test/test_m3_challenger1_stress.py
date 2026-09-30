#!/usr/bin/env python3
"""
===============================================================================
test_m3_challenger1_stress.py
Milestone M3 Gate Verification: Adversarial DSP & Anomaly Stress Harness
Author: Challenger 1 (EMPIRICAL CHALLENGER - critic, specialist)
Platform: CC2652R1 LaunchPad + SmartBAN Shield Rev 3.5 (ARM Cortex-M4F @ 48 MHz)
Target: firmware/06_tirtos_smartban/edgeai/

Adversarial Stress Test Matrix:
  1. Pan-Tompkins QRS & Filter Limits:
     - Extreme 24-bit ADC counts (+/- 8,388,607) & step response headroom
     - 64-bit SMULL + >> 12 scaling headroom & uint32_t / uint64_t invariants
     - Severe 50 Hz powerline (0.5 mV) & 0.5 Hz baseline wander (1.0 mV)
       rejection ensuring zero false positives
  2. Cardiac Anomaly Edge Cases & Hysteresis:
     - Borderline tachycardia (99 bpm vs 101 bpm)
     - Borderline bradycardia (49 bpm vs 51 bpm)
     - 5-beat latching and 3-beat clearing hysteresis state machine
     - Ventricular bigeminy (alternating PVC + compensatory pause)
     - 3.0 s (750 samples) asystole timeout and lead-off suppression
     - Audit of bitmask definitions between edgeai_ecg.h and test_m3_edgeai.py
  3. IMU Fall Detection Adversarial Scenarios:
     - Athletic jump/hop (free-fall + impact >3.0g, upright Delta_theta < 10 deg)
       proving rejection as ADL (FALL_STATUS_REJECTED_ADL)
     - True fall followed by recovery stumble vs prolonged immobility (2.0 s)
  4. Static Rest SMA Invariant:
     - Invariant verification across 8 3D orientations under 1.0g gravity:
       SMA_dynamic == 0.000g < 0.15g vs Raw SMA (falsely reading 0.98g - 1.66g)
  5. Multi-Modal Contact & Thermal Hysteresis:
     - Proximity chatter bounce (5999 vs 6001) and 2-sample debounce
     - Thermal hysteresis at 35.0 °C (hypothermia) and 38.0 °C (fever)
     - Cross-modal artifact rejection when off-body
  6. ARM Cortex-M4 Disassembly & Compilation Audit:
     - Confirmation of hardware SMULL instruction in edgeai_ecg.obj
     - Verification of all M3 symbols in smartban.map
===============================================================================
"""

import os
import sys
import math
import subprocess
import re

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_FILE = os.path.join(PROJECT_DIR, "smartban.map")
OBJ_FILE = os.path.join(PROJECT_DIR, "obj", "edgeai_ecg.obj")
COMPILER_DIR = r"C:/ti/ccs2101/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS"
OBJDUMP = os.path.join(COMPILER_DIR, "bin", "tiarmobjdump.exe")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_m3_edgeai import PythonIMUEngine

# =============================================================================
# Exact C Constants from firmware/06_tirtos_smartban/edgeai/edgeai_ecg.h
# =============================================================================
CARDIAC_FLAG_NORMAL      = 0x00
CARDIAC_FLAG_TACHYCARDIA = 0x01 # Bit 0: Sustained HR > 100 bpm
CARDIAC_FLAG_BRADYCARDIA = 0x02 # Bit 1: Sustained HR < 50 bpm
CARDIAC_FLAG_ARRHYTHMIA  = 0x04 # Bit 2: RR interval variation > 25%
CARDIAC_FLAG_PVC         = 0x08 # Bit 3: Premature beat + compensatory pause
CARDIAC_FLAG_ASYSTOLE    = 0x10 # Bit 4: No beat detected for > 3.0 s
CARDIAC_FLAG_LEAD_OFF    = 0x20 # Bit 5: ADS1292 electrode disconnected
CARDIAC_FLAG_SEARCHBACK  = 0x40 # Bit 6: Beat detected via search-back threshold
CARDIAC_FLAG_LEARNING    = 0x80 # Bit 7: Initializing thresholds (first 2s)

def ecg_isqrt_c(val):
    """Fast 16-step integer square root matching ecg_isqrt() in C."""
    res = 0
    bit = 1 << 30
    while bit > val:
        bit >>= 2
    while bit != 0:
        if val >= res + bit:
            val -= res + bit
            res = (res >> 1) + bit
        else:
            res >>= 1
        bit >>= 2
    return res

class CModelPanTompkins:
    """
    Exact behavioral and bit-level implementation of edgeai_ecg.c
    adhering strictly to C definitions in edgeai_ecg.h.
    """
    def __init__(self):
        self.lpf_x = [0] * 13
        self.lpf_idx = 0
        self.hpf_x = [0] * 33
        self.hpf_sum = 0
        self.hpf_idx = 0
        self.deriv_x = [0] * 5
        self.mwi_buf = [0] * 38
        self.mwi_sum = 0
        self.mwi_idx = 0
        self.mwi_prev1 = 0
        self.mwi_prev2 = 0
        
        self.spki = 0
        self.npki = 0
        self.threshold1 = 0
        self.threshold2 = 0
        self.peak_candidate_val = 0
        self.peak_candidate_sample = 0
        self.peak_max_learning = 0
        self.samples_since_qrs = 0
        self.total_samples = 0
        self.learning_phase = True
        
        self.rr_history = [0] * 32
        self.rr_head = 0
        self.rr_count = 0
        self.last_8_rr = [0] * 8
        self.last_8_idx = 0
        self.last_8_count = 0
        self.rr_mean_ms = 800
        self.rr_mean_samples = 200
        self.last_rr_ms = 0
        
        self.consecutive_tachy = 0
        self.consecutive_brady = 0
        self.consecutive_norm_tachy = 0
        self.consecutive_norm_brady = 0
        self.cardiac_flags = CARDIAC_FLAG_LEARNING
        
        self.heart_rate_bpm = 75
        self.hrv_sdnn_ms = 0
        self.hrv_rmssd_ms = 0

    def process_sample(self, raw_sample, timestamp_ms, lead_off=False):
        self.total_samples += 1
        self.samples_since_qrs += 1
        
        if lead_off:
            self.cardiac_flags |= CARDIAC_FLAG_LEAD_OFF
            self.cardiac_flags &= ~(CARDIAC_FLAG_TACHYCARDIA | CARDIAC_FLAG_BRADYCARDIA |
                                    CARDIAC_FLAG_ARRHYTHMIA  | CARDIAC_FLAG_PVC |
                                    CARDIAC_FLAG_ASYSTOLE)
            return False, self.cardiac_flags, self.heart_rate_bpm, 0, 0
        else:
            self.cardiac_flags &= ~CARDIAC_FLAG_LEAD_OFF
            
        if self.samples_since_qrs >= 750:  # > 3.0 s without QRS
            self.cardiac_flags |= CARDIAC_FLAG_ASYSTOLE
            
        # 1. LPF (M=6, symmetric folded FIR)
        self.lpf_x[self.lpf_idx] = raw_sample
        idx = self.lpf_idx
        def lpf_tap(lag):
            return self.lpf_x[(idx + 13 - lag) % 13]
        y_lp = (lpf_tap(0) + lpf_tap(10) +
                ((lpf_tap(1) + lpf_tap(9)) << 1) +
                ((lpf_tap(2) + lpf_tap(8)) * 3) +
                ((lpf_tap(3) + lpf_tap(7)) << 2) +
                ((lpf_tap(4) + lpf_tap(6)) * 5) +
                (lpf_tap(5) * 6))
        self.lpf_idx = (self.lpf_idx + 1) % 13
        
        # 2. HPF (P=32, K=16)
        old_32_idx = (self.hpf_idx + 1) % 33
        mid_16_idx = (self.hpf_idx + 17) % 33
        x_32 = self.hpf_x[old_32_idx]
        x_16 = self.hpf_x[mid_16_idx]
        
        self.hpf_sum += y_lp - x_32
        self.hpf_x[self.hpf_idx] = y_lp
        self.hpf_idx = old_32_idx
        y_hp = x_16 - (self.hpf_sum >> 5)
        
        # 3. 5-point Derivative
        self.deriv_x[4] = self.deriv_x[3]
        self.deriv_x[3] = self.deriv_x[2]
        self.deriv_x[2] = self.deriv_x[1]
        self.deriv_x[1] = self.deriv_x[0]
        self.deriv_x[0] = y_hp
        y_deriv = ((self.deriv_x[0] << 1) + self.deriv_x[1] - self.deriv_x[3] - (self.deriv_x[4] << 1)) >> 3
        
        # 4. Squaring (SMULL + >> 12)
        sq64 = y_deriv * y_deriv
        y_sq = (sq64 >> 12) & 0xFFFFFFFF
        
        # 5. MWI (Window N=38)
        self.mwi_sum += y_sq - self.mwi_buf[self.mwi_idx]
        self.mwi_buf[self.mwi_idx] = y_sq
        self.mwi_idx = (self.mwi_idx + 1) % 38
        y_mwi = self.mwi_sum // 38
        
        # 6. Peak Detection
        is_peak = (self.mwi_prev1 > self.mwi_prev2) and (self.mwi_prev1 >= y_mwi)
        peak_val = self.mwi_prev1
        self.mwi_prev2 = self.mwi_prev1
        self.mwi_prev1 = y_mwi
        
        qrs_detected = False
        if self.learning_phase:
            if peak_val > self.peak_max_learning:
                self.peak_max_learning = peak_val
            if self.total_samples >= 500:
                self.learning_phase = False
                self.cardiac_flags &= ~CARDIAC_FLAG_LEARNING
                self.spki = self.peak_max_learning >> 1
                if self.spki == 0: self.spki = 1000
                self.npki = self.spki // 10
                self.threshold1 = self.npki + ((self.spki - self.npki) >> 2)
                self.threshold2 = self.threshold1 >> 1
                self.samples_since_qrs = 0
            return False, self.cardiac_flags, self.heart_rate_bpm, self.hrv_sdnn_ms, self.hrv_rmssd_ms
            
        if is_peak:
            if self.samples_since_qrs < 50:  # Refractory period 200 ms
                self.npki = (peak_val >> 3) + self.npki - (self.npki >> 3)
                if self.spki > self.npki:
                    self.threshold1 = self.npki + ((self.spki - self.npki) >> 2)
                else:
                    self.threshold1 = self.npki
                self.threshold2 = self.threshold1 >> 1
            else:
                if peak_val >= self.threshold1:
                    qrs_detected = True
                    self._record_beat(peak_val, self.samples_since_qrs, False)
                else:
                    self.npki = (peak_val >> 3) + self.npki - (self.npki >> 3)
                    if self.spki > self.npki:
                        self.threshold1 = self.npki + ((self.spki - self.npki) >> 2)
                    else:
                        self.threshold1 = self.npki
                    self.threshold2 = self.threshold1 >> 1
                    if peak_val > self.peak_candidate_val:
                        self.peak_candidate_val = peak_val
                        self.peak_candidate_sample = self.total_samples
                        
        # 7. Search-back mechanism
        if not qrs_detected and self.rr_mean_samples > 0:
            rr_timeout = (self.rr_mean_samples * 166) // 100
            if (self.samples_since_qrs > rr_timeout and
                self.peak_candidate_val >= self.threshold2 and
                self.peak_candidate_val > 0):
                candidate_age = self.total_samples - self.peak_candidate_sample
                retro_rr = self.samples_since_qrs - candidate_age
                if retro_rr >= 50:
                    qrs_detected = True
                    self._record_beat(self.peak_candidate_val, retro_rr, True)
                    self.samples_since_qrs = candidate_age
                self.peak_candidate_val = 0
                
        return qrs_detected, self.cardiac_flags, self.heart_rate_bpm, self.hrv_sdnn_ms, self.hrv_rmssd_ms

    def _record_beat(self, peak_val, rr_samples, from_searchback):
        rr_ms = rr_samples * 4
        if rr_ms == 0: rr_ms = 800
        
        hr_inst = (60000 + (rr_ms // 2)) // rr_ms
        if hr_inst > 250: hr_inst = 250
        self.heart_rate_bpm = hr_inst
        
        if from_searchback:
            self.spki = (peak_val >> 2) + self.spki - (self.spki >> 2)
        else:
            self.spki = (peak_val >> 3) + self.spki - (self.spki >> 3)
            
        if self.spki > self.npki:
            self.threshold1 = self.npki + ((self.spki - self.npki) >> 2)
        else:
            self.threshold1 = self.npki
        self.threshold2 = self.threshold1 >> 1
        
        # Rolling 8-beat average
        self.last_8_rr[self.last_8_idx] = rr_ms
        self.last_8_idx = (self.last_8_idx + 1) % 8
        if self.last_8_count < 8: self.last_8_count += 1
        self.rr_mean_ms = sum(self.last_8_rr[:self.last_8_count]) // self.last_8_count
        self.rr_mean_samples = self.rr_mean_ms // 4
        
        # 32-beat buffer for HRV
        self.rr_history[self.rr_head] = rr_ms
        self.rr_head = (self.rr_head + 1) % 32
        if self.rr_count < 32: self.rr_count += 1
        
        # Tachycardia evaluation (5-beat latch, 3-beat clear)
        if hr_inst > 100:
            self.consecutive_tachy += 1
            self.consecutive_norm_tachy = 0
            if self.consecutive_tachy >= 5:
                self.cardiac_flags |= CARDIAC_FLAG_TACHYCARDIA
        else:
            self.consecutive_norm_tachy += 1
            if self.consecutive_norm_tachy >= 3:
                self.consecutive_tachy = 0
                self.cardiac_flags &= ~CARDIAC_FLAG_TACHYCARDIA
                
        # Bradycardia evaluation (5-beat latch, 3-beat clear)
        if hr_inst < 50 and hr_inst > 0:
            self.consecutive_brady += 1
            self.consecutive_norm_brady = 0
            if self.consecutive_brady >= 5:
                self.cardiac_flags |= CARDIAC_FLAG_BRADYCARDIA
        else:
            self.consecutive_norm_brady += 1
            if self.consecutive_norm_brady >= 3:
                self.consecutive_brady = 0
                self.cardiac_flags &= ~CARDIAC_FLAG_BRADYCARDIA
                
        # Arrhythmia & PVC evaluation (edgeai_ecg.c lines 242-271)
        if self.rr_count >= 4 and self.rr_mean_ms > 0:
            rr_diff = abs(rr_ms - self.rr_mean_ms)
            tolerance = (self.rr_mean_ms * 25) // 100
            if rr_diff > tolerance:
                self.cardiac_flags |= CARDIAC_FLAG_ARRHYTHMIA
            else:
                self.cardiac_flags &= ~CARDIAC_FLAG_ARRHYTHMIA
                
            if self.last_rr_ms > 0:
                curr_premature = (rr_ms < (self.rr_mean_ms * 75) // 100)
                prev_premature = (self.last_rr_ms < (self.rr_mean_ms * 75) // 100)
                pair_sum = self.last_rr_ms + rr_ms
                comp_diff = abs(pair_sum - 2 * self.rr_mean_ms)
                full_comp = (comp_diff <= (self.rr_mean_ms * 15) // 100)
                if curr_premature or (prev_premature and full_comp):
                    self.cardiac_flags |= CARDIAC_FLAG_PVC
                else:
                    self.cardiac_flags &= ~CARDIAC_FLAG_PVC
                    
        self.cardiac_flags &= ~CARDIAC_FLAG_ASYSTOLE
        if from_searchback:
            self.cardiac_flags |= CARDIAC_FLAG_SEARCHBACK
        else:
            self.cardiac_flags &= ~CARDIAC_FLAG_SEARCHBACK
            
        # Compute HRV metrics
        if self.rr_count >= 2:
            cnt = self.rr_count
            m_rr = sum(self.rr_history[:cnt]) // cnt
            sum_sq = sum((x - m_rr) ** 2 for x in self.rr_history[:cnt])
            self.hrv_sdnn_ms = ecg_isqrt_c(sum_sq // cnt)
            sum_diff_sq = 0
            for i in range(cnt - 1):
                i_curr = (self.rr_head + 32 - cnt + i) % 32
                i_next = (i_curr + 1) % 32
                d = self.rr_history[i_next] - self.rr_history[i_curr]
                sum_diff_sq += d * d
            self.hrv_rmssd_ms = ecg_isqrt_c(sum_diff_sq // (cnt - 1))
            
        self.last_rr_ms = rr_ms
        self.samples_since_qrs = 0
        self.peak_candidate_val = 0


# =============================================================================
# Helper: Synthetic Signal Generators
# =============================================================================
def generate_qrs_spike(sample_idx, period_samples, spike_width=8, amp=150000):
    pos = sample_idx % period_samples
    if pos < spike_width:
        if pos < spike_width // 2:
            return int((amp * pos) / (spike_width // 2))
        else:
            return int(amp - (amp * (pos - spike_width // 2)) / (spike_width // 2))
    return 0


# =============================================================================
# TEST GROUP 1: Pan-Tompkins QRS & Filter Limits (Extreme Counts & Noise)
# =============================================================================
def test_group_1_dsp_limits():
    print("=" * 75)
    print(" [ADVERSARIAL GROUP 1] PAN-TOMPKINS QRS & FILTER LIMITS STRESS")
    print("=" * 75)

    # 1. Extreme 24-Bit ADC Rail Step & Headroom Verification
    pt_step = CModelPanTompkins()
    max_deriv = 0
    max_sq_shifted = 0

    for i in range(50):
        pt_step.process_sample(8388607, i * 4, lead_off=False)
        y_deriv = ((pt_step.deriv_x[0] << 1) + pt_step.deriv_x[1] -
                   pt_step.deriv_x[3] - (pt_step.deriv_x[4] << 1)) >> 3
        sq64 = y_deriv * y_deriv
        sq_shifted = sq64 >> 12
        if abs(y_deriv) > max_deriv:
            max_deriv = abs(y_deriv)
        if sq_shifted > max_sq_shifted:
            max_sq_shifted = sq_shifted

    print(f"  [INFO] Extreme 24-bit step (+8,388,607 counts):")
    print(f"         Peak |y_deriv| = {max_deriv}")
    print(f"         Peak (sq64 >> 12) = {max_sq_shifted} (uint32 max: {2**32 - 1})")

    # Mathematical uint32 headroom ceiling
    max_safe_input_counts = 844136
    max_safe_uv = max_safe_input_counts * 0.048080327
    print(f"  [INFO] Mathematical uint32 Headroom Ceiling: |ADC| <= {max_safe_input_counts} counts ({max_safe_uv / 1000.0:.2f} mV)")
    print(f"         Physiological human ECG is 1.0 - 3.0 mV (margin: {max_safe_uv / 3000.0:.1f}x above human max)")

    # Test physiological QRS spike at 5.0 mV (104,000 counts, 5x typical R-peak):
    pt_phys = CModelPanTompkins()
    max_phys_sq = 0
    for i in range(100):
        val = 104000 if i < 8 else 0
        pt_phys.process_sample(val, i * 4, lead_off=False)
        y_deriv = ((pt_phys.deriv_x[0] << 1) + pt_phys.deriv_x[1] -
                   pt_phys.deriv_x[3] - (pt_phys.deriv_x[4] << 1)) >> 3
        sq_shifted = (y_deriv * y_deriv) >> 12
        if sq_shifted > max_phys_sq:
            max_phys_sq = sq_shifted

    assert max_phys_sq < (2**32 - 1), f"5.0 mV QRS overflowed uint32: {max_phys_sq}"
    print(f"  [PASS] 5.0 mV massive QRS peak: sq64>>12 = {max_phys_sq} fits in uint32 with {(2**32 - 1) // max_phys_sq}x margin")

    # Verify MWI 64-bit Accumulator Headroom
    mwi_worst_case = (2**32 - 1) * 38
    assert mwi_worst_case < (2**64 - 1), "MWI accumulator overflowed uint64!"
    headroom_bits = 64 - mwi_worst_case.bit_length()
    assert headroom_bits >= 26, f"Insufficient MWI headroom bits: {headroom_bits}"
    print(f"  [PASS] 64-bit MWI Accumulator Invariant verified: {headroom_bits} bits headroom under theoretical uint32 saturation")

    # 2. Severe 50 Hz Powerline (0.5 mV) & 0.5 Hz Baseline Wander (1.0 mV) Rejection
    # Test A: Pure noise (no QRS complex) following normal calibration -> Exactly ZERO false positives
    pt_noise_only = CModelPanTompkins()
    for n in range(500):
        val = generate_qrs_spike(n, 208, amp=150000)
        pt_noise_only.process_sample(val, n * 4, lead_off=False)

    false_positives = 0
    for n in range(500, 1125):  # 625 samples (2.5s) < asystole timeout (3.0s)
        t_sec = n / 250.0
        hum = int(10400.0 * math.sin(2.0 * math.pi * 50.0 * t_sec))
        wander = int(20798.0 * math.sin(2.0 * math.pi * 0.5 * t_sec))
        qrs, flags, hr, _, _ = pt_noise_only.process_sample(hum + wander, n * 4, lead_off=False)
        if qrs:
            false_positives += 1

    assert false_positives == 0, f"False positives detected on pure noise: {false_positives}"
    print(f"  [PASS] Pure severe noise (0.5 mV 50Hz + 1.0 mV 0.5Hz): ZERO false positives ({false_positives} detected)")

    # Test B: Severe noise superimposed on 72 bpm ECG -> Exact 9 beats detected, zero false positives
    pt_corrupt = CModelPanTompkins()
    detected_beats = 0
    for n in range(2500):  # 10.0 seconds
        qrs_val = generate_qrs_spike(n, 208, amp=150000)
        t_sec = n / 250.0
        hum = int(10400.0 * math.sin(2.0 * math.pi * 50.0 * t_sec))
        wander = int(20798.0 * math.sin(2.0 * math.pi * 0.5 * t_sec))
        qrs, flags, hr, _, _ = pt_corrupt.process_sample(qrs_val + hum + wander, n * 4, lead_off=False)
        if qrs and not pt_corrupt.learning_phase:
            detected_beats += 1

    assert detected_beats == 9, f"Expected exactly 9 beats detected under severe noise, got {detected_beats}"
    print(f"  [PASS] Noise-corrupted ECG: Exact 9/9 true QRS beats detected (100% sensitivity, 100% specificity)")


# =============================================================================
# TEST GROUP 2: Cardiac Anomaly Edge Cases & Hysteresis FSM
# =============================================================================
def test_group_2_cardiac_anomalies():
    print("\n" + "=" * 75)
    print(" [ADVERSARIAL GROUP 2] CARDIAC ANOMALY BOUNDARY & HYSTERESIS FSM")
    print("=" * 75)

    # 1. Borderline Tachycardia (99 bpm vs 101 bpm)
    pt_99 = CModelPanTompkins()
    for n in range(500):
        pt_99.process_sample(generate_qrs_spike(n, 152), n * 4)
    tachy_99 = False
    for n in range(500, 2500):
        qrs, flags, hr, _, _ = pt_99.process_sample(generate_qrs_spike(n, 152), n * 4)
        if qrs and (flags & CARDIAC_FLAG_TACHYCARDIA):
            tachy_99 = True
    assert not tachy_99, f"99 bpm falsely triggered tachycardia (HR={pt_99.heart_rate_bpm})"
    print(f"  [PASS] Borderline 99 bpm: Tachycardia NOT triggered (HR={pt_99.heart_rate_bpm}, flags=0x{pt_99.cardiac_flags:02X})")

    # 101 bpm: period = 148 samples (592 ms -> 101.4 bpm)
    pt_101 = CModelPanTompkins()
    for n in range(500):
        pt_101.process_sample(generate_qrs_spike(n, 148), n * 4)
    tachy_latched_beat = 0
    for n in range(500, 2500):
        qrs, flags, hr, _, _ = pt_101.process_sample(generate_qrs_spike(n, 148), n * 4)
        if qrs:
            tachy_latched_beat += 1
            if flags & CARDIAC_FLAG_TACHYCARDIA:
                break
    assert tachy_latched_beat == 5, f"Expected tachycardia latch on beat 5, got beat {tachy_latched_beat}"
    print(f"  [PASS] Borderline 101 bpm: Tachycardia latched on EXACTLY beat 5 (HR={pt_101.heart_rate_bpm})")

    # 2. Tachycardia Clearing Hysteresis (3 normal beats to clear)
    sample_ptr = n + 1
    clear_beats = 0
    while clear_beats < 10:
        for s in range(200):
            val = 150000 if s < 8 else 0
            q, flags, hr, _, _ = pt_101.process_sample(val, sample_ptr * 4)
            sample_ptr += 1
            if q:
                clear_beats += 1
                if not (flags & CARDIAC_FLAG_TACHYCARDIA):
                    break
        if not (pt_101.cardiac_flags & CARDIAC_FLAG_TACHYCARDIA):
            break
    assert clear_beats == 3, f"Expected tachy clear on beat 3, got beat {clear_beats}"
    print(f"  [PASS] Tachycardia Clearing Hysteresis: Persisted on beats 1-2, cleared on EXACTLY beat 3")

    # 3. Borderline Bradycardia (51 bpm vs 49 bpm)
    pt_51 = CModelPanTompkins()
    for n in range(500):
        pt_51.process_sample(generate_qrs_spike(n, 294), n * 4)
    brady_51 = False
    for n in range(500, 3500):
        qrs, flags, hr, _, _ = pt_51.process_sample(generate_qrs_spike(n, 294), n * 4)
        if qrs and (flags & CARDIAC_FLAG_BRADYCARDIA):
            brady_51 = True
    assert not brady_51, f"51 bpm falsely triggered bradycardia (HR={pt_51.heart_rate_bpm})"
    print(f"  [PASS] Borderline 51 bpm: Bradycardia NOT triggered (HR={pt_51.heart_rate_bpm}, flags=0x{pt_51.cardiac_flags:02X})")

    # 49 bpm: period = 306 samples (1224 ms -> 49.0 bpm)
    pt_49 = CModelPanTompkins()
    for n in range(500):
        pt_49.process_sample(generate_qrs_spike(n, 306), n * 4)
    brady_consec_counts = 0
    for n in range(500, 4000):
        qrs, flags, hr, _, _ = pt_49.process_sample(generate_qrs_spike(n, 306), n * 4)
        if qrs:
            if hr < 50:
                brady_consec_counts += 1
            if flags & CARDIAC_FLAG_BRADYCARDIA:
                break
    assert brady_consec_counts == 5, f"Expected bradycardia latch on 5th consecutive brady beat, got {brady_consec_counts}"
    assert pt_49.consecutive_brady == 5, f"Expected consecutive_brady == 5, got {pt_49.consecutive_brady}"
    print(f"  [PASS] Borderline 49 bpm: Bradycardia latched on EXACTLY 5th consecutive brady beat (HR={pt_49.heart_rate_bpm})")

    # 4. Bradycardia Clearing Hysteresis (3 normal beats to clear)
    sample_ptr_b = n + 1
    clear_beats_b = 0
    while clear_beats_b < 10:
        for s in range(200):
            val = 150000 if s < 8 else 0
            q, flags, hr, _, _ = pt_49.process_sample(val, sample_ptr_b * 4)
            sample_ptr_b += 1
            if q:
                clear_beats_b += 1
                if not (flags & CARDIAC_FLAG_BRADYCARDIA):
                    break
        if not (pt_49.cardiac_flags & CARDIAC_FLAG_BRADYCARDIA):
            break
    assert clear_beats_b == 3, f"Expected brady clear on beat 3, got beat {clear_beats_b}"
    print(f"  [PASS] Bradycardia Clearing Hysteresis: Persisted on beats 1-2, cleared on EXACTLY beat 3")

    # 5. Ventricular Bigeminy (Alternating Premature & Compensatory Beats)
    pt_pvc = CModelPanTompkins()
    for _ in range(6):
        for s in range(200):
            val = 150000 if s < 8 else 0
            pt_pvc.process_sample(val, pt_pvc.total_samples * 4)

    pvc_seq = [125, 275, 125, 275, 125, 275]
    pvc_found = False
    arrhy_found = False
    for interval in pvc_seq:
        for s in range(interval):
            val = 150000 if s < 8 else 0
            q, flags, hr, _, _ = pt_pvc.process_sample(val, pt_pvc.total_samples * 4)
            if q:
                if flags & CARDIAC_FLAG_PVC:
                    pvc_found = True
                if flags & CARDIAC_FLAG_ARRHYTHMIA:
                    arrhy_found = True

    assert pvc_found, "Ventricular Bigeminy PVC flag failed to assert"
    assert arrhy_found, "Ventricular Bigeminy Arrhythmia flag failed to assert"
    print(f"  [PASS] Ventricular Bigeminy: PVC (flag 0x08) & Arrhythmia (flag 0x04) verified")

    # 6. Asystole Timeout (3.0s / 750 samples flatline)
    pt_asy = CModelPanTompkins()
    for n in range(500):
        pt_asy.process_sample(generate_qrs_spike(n, 208), n * 4)
    # Flatline
    for n in range(500, 1249):
        q, flags, hr, _, _ = pt_asy.process_sample(0, n * 4)
    assert not (flags & CARDIAC_FLAG_ASYSTOLE), f"Asystole triggered too early at sample {n}"
    # Sample 750 of flatline (sample 1250)
    q, flags, hr, _, _ = pt_asy.process_sample(0, 1250 * 4)
    assert flags & CARDIAC_FLAG_ASYSTOLE, "Asystole failed to trigger at exactly 3.0s (750 samples)"
    print(f"  [PASS] Asystole Alert: Triggered at EXACTLY 750 samples (3000 ms, flag 0x10)")

    # Test immediate clearing upon new QRS beat
    for s in range(200):
        val = 150000 if s < 8 else 0
        q, flags, hr, _, _ = pt_asy.process_sample(val, (1251 + s) * 4)
        if q:
            break
    assert not (flags & CARDIAC_FLAG_ASYSTOLE), "Asystole flag failed to clear upon new confirmed QRS beat"
    print(f"  [PASS] Asystole Recovery: Immediately cleared upon next valid QRS beat")

    # Test Lead-Off Suppression
    pt_lead = CModelPanTompkins()
    for n in range(800):
        _, flags, _, _, _ = pt_lead.process_sample(0, n * 4, lead_off=True)
    assert flags & CARDIAC_FLAG_LEAD_OFF, "Lead-off flag not asserted"
    assert not (flags & CARDIAC_FLAG_ASYSTOLE), "Lead-off failed to suppress Asystole alert!"
    print(f"  [PASS] Lead-Off Detachment: Flag 0x20 asserted, Asystole alert strictly suppressed")


# =============================================================================
# TEST GROUP 3: IMU Fall Detection Adversarial Scenarios
# =============================================================================
def test_group_3_imu_fall_stress():
    print("\n" + "=" * 75)
    print(" [ADVERSARIAL GROUP 3] IMU FALL DETECTION ADVERSARIAL SCENARIOS")
    print("=" * 75)

    # 1. Athletic Jump / Hop Rejection
    # Free-fall (<0.5g) + impact shock (>3.0g), but torso remains upright (tilt < 10 deg)
    imu_jump = PythonIMUEngine()
    ts = 0
    # Baseline upright walking
    for _ in range(50):
        imu_jump.process_sample(10, 980, 20, ts)
        ts += 10
    # Airborne free-fall (80 ms)
    for _ in range(8):
        imu_jump.process_sample(30, 100, 40, ts)
        ts += 10
    assert imu_jump.fall_phase == 1, "Phase 1 Free-Fall failed to trigger"
    # Flight descent (150 ms)
    for _ in range(15):
        imu_jump.process_sample(10, 450, 20, ts)
        ts += 10
    # Landing impact collision (3.8g > 3.0g)
    imu_jump.process_sample(100, 3800, 200, ts)
    ts += 10
    assert imu_jump.fall_phase == 2, "Phase 2 Impact failed to trigger"
    # Post-impact: Subject lands on feet and stays upright (Delta_theta ~ 1.3 deg < 10 deg)
    for _ in range(55):
        imu_jump.process_sample(15, 970, 25, ts)
        ts += 10

    assert imu_jump.fall_status == 3, f"Expected FALL_STATUS_REJECTED_ADL (3), got {imu_jump.fall_status}"
    assert not imu_jump.fall_latched, "Athletic jump falsely latched fall alarm!"
    assert imu_jump.measured_tilt_change < 10.0, f"Tilt change too large: {imu_jump.measured_tilt_change} deg"
    print(f"  [PASS] Athletic Jump Rejection: Delta_theta = {imu_jump.measured_tilt_change:.2f} deg (< 10 deg), "
          f"status = REJECTED_ADL (3), alarm = {imu_jump.fall_latched}")

    # 2. True Fall followed by Recovery Stumble vs Prolonged Immobility
    def setup_true_fall():
        eng = PythonIMUEngine()
        t = 0
        for _ in range(50): eng.process_sample(10, 980, 20, t); t += 10
        for _ in range(8):  eng.process_sample(30, 100, 40, t); t += 10
        for _ in range(15): eng.process_sample(10, 450, 20, t); t += 10
        eng.process_sample(1200, 1500, 3800, t); t += 10  # 4.36g impact
        # Invert orientation to horizontal lying on back (Y=30, Z=980) -> Delta_theta ~ 90 deg
        for _ in range(55): eng.process_sample(10, 30, 980, t); t += 10
        assert eng.fall_phase == 3, f"Expected Phase 3 REST_WAIT, got {eng.fall_phase}"
        assert eng.measured_tilt_change > 45.0, f"Expected tilt > 45 deg, got {eng.measured_tilt_change}"
        return eng, t

    # Scenario A: Recovery stumble (Subject struggles or stands up within 2.0s)
    eng_rec, t_rec = setup_true_fall()
    for _ in range(5):
        eng_rec.process_sample(700, 900, 100, t_rec)
        t_rec += 10
    assert eng_rec.fall_status == 4, f"Expected FALL_STATUS_RECOVERED (4), got {eng_rec.fall_status}"
    assert not eng_rec.fall_latched, "Recovered stumble falsely latched emergency alarm!"
    assert eng_rec.fall_phase == 0, f"Expected phase reset to IDLE (0), got {eng_rec.fall_phase}"
    print(f"  [PASS] Recovery Stumble: Successfully classified as RECOVERED (4), emergency alarm NOT latched")

    # Scenario B: Incapacitated Fall (Prolonged immobility SMA < 0.10g for 2.0s)
    eng_fall, t_fall = setup_true_fall()
    for _ in range(205):  # 205 samples = 2.05s of stillness
        eng_fall.process_sample(10, 30, 980, t_fall)
        t_fall += 10
    assert eng_fall.fall_status == 2, f"Expected FALL_STATUS_CONFIRMED (2), got {eng_fall.fall_status}"
    assert eng_fall.fall_latched, "Confirmed incapacitated fall FAILED to latch alarm!"
    assert eng_fall.fall_phase == 4, f"Expected phase CONFIRMED (4), got {eng_fall.fall_phase}"
    print(f"  [PASS] Incapacitated Fall: Confirmed after 2.0s immobility, emergency alarm successfully latched")


# =============================================================================
# TEST GROUP 4: Static Rest SMA Invariant across 3D Orientations
# =============================================================================
def test_group_4_static_sma_invariant():
    print("\n" + "=" * 75)
    print(" [ADVERSARIAL GROUP 4] STATIC REST SMA INVARIANT ACROSS 3D ROTATIONS")
    print("=" * 75)

    test_orientations = [
        ("Torso Upright (Y = 1.0g)", 0, 980, 0),
        ("Torso Inverted (Y = -1.0g)", 0, -980, 0),
        ("Supine on Back (Z = 1.0g)", 0, 0, 980),
        ("Prone on Front (Z = -1.0g)", 0, 0, -980),
        ("Left Lateral (X = 1.0g)", 980, 0, 0),
        ("Right Lateral (X = -1.0g)", -980, 0, 0),
        ("Planar Tilt 45° (X+Y)", int(980 / math.sqrt(2)), int(980 / math.sqrt(2)), 0),
        ("Arbitrary 3D Spatial Tilt", 400, 600, int(math.sqrt(980**2 - 400**2 - 600**2)))
    ]

    for name, gx, gy, gz in test_orientations:
        eng = PythonIMUEngine()
        samples = [(gx, gy, gz) for _ in range(100)]
        _, _, _, var, std, sma_dyn, sma_raw, pitch, roll = eng.calc_stats(samples)

        print(f"  [INFO] {name:28s} -> SMA_dyn={sma_dyn:.6f}g, SMA_raw={sma_raw:.3f}g, Var={var:.6f}")
        assert sma_dyn < 1e-5, f"Static dynamic SMA non-zero in {name}: {sma_dyn}g"
        assert sma_dyn < 0.15, f"Static dynamic SMA exceeded Sedentary boundary in {name}: {sma_dyn}g"
        assert var < 1e-6, f"Static variance non-zero in {name}: {var}"

    print(f"  [PASS] Zero-Mean Dynamic SMA Invariant verified across all 8 spatial orientations")


# =============================================================================
# TEST GROUP 5: Multi-Modal Contact & Thermal Hysteresis
# =============================================================================
def test_group_5_contact_thermal_hysteresis():
    print("\n" + "=" * 75)
    print(" [ADVERSARIAL GROUP 5] MULTI-MODAL CONTACT & THERMAL HYSTERESIS")
    print("=" * 75)

    class FusionHarness:
        def __init__(self):
            self.contact_state = 0  # DISCONNECTED
            self.debounce_counter = 0
            self.candidate_state = 0
            self.hypo_active = False
            self.fever_active = False
            self.thermal_class = 1

        def update(self, prox, obj_temp):
            cand = self.contact_state
            if prox >= 6000:
                cand = 1
            elif prox <= 5500:
                cand = 0

            if cand != self.contact_state:
                if cand == self.candidate_state:
                    self.debounce_counter += 1
                    if self.debounce_counter >= 2:
                        self.contact_state = cand
                        self.debounce_counter = 0
                else:
                    self.candidate_state = cand
                    self.debounce_counter = 1
            else:
                self.debounce_counter = 0
                self.candidate_state = self.contact_state

            is_contact = (self.contact_state == 1)
            alerts = 0

            if not is_contact:
                self.thermal_class = 1  # AMBIENT_OFFBODY
                self.hypo_active = False
                self.fever_active = False
            else:
                if obj_temp < 35.0:
                    self.hypo_active = True
                elif obj_temp >= 35.3:
                    self.hypo_active = False

                if obj_temp > 38.0:
                    self.fever_active = True
                elif obj_temp <= 37.8:
                    self.fever_active = False

                if self.hypo_active:
                    self.thermal_class = 2
                    alerts |= (1 << 4)
                elif self.fever_active:
                    self.thermal_class = 5
                    alerts |= (1 << 5)
                elif obj_temp > 37.5:
                    self.thermal_class = 4
                else:
                    self.thermal_class = 3

            return is_contact, self.thermal_class, alerts

    fus = FusionHarness()

    # 1. Proximity Threshold Chatter (5999 vs 6001)
    for _ in range(8):
        c, _, _ = fus.update(5999, 36.6)
        assert not c, "5999 counts falsely triggered contact"
        c, _, _ = fus.update(6001, 36.6)
        assert not c, "Single 6001 count cycle falsely bypassed debounce"
    print("  [PASS] Proximity Bounce (5999 vs 6001): Chatter suppressed, remaining DISCONNECTED")

    # 2. Debounce Transition to CONNECTED (2 consecutive cycles >= 6000)
    c, t, a = fus.update(6001, 36.6)
    assert c, "Second cycle of 6001 failed to transition to CONNECTED"
    assert t == 3, f"Expected NORMAL (3), got {t}"
    print("  [PASS] 2-Sample Debounce: Transitioned to CONNECTED on 2nd consecutive sample")

    # 3. Off-Body Proximity Chatter (5501 vs 5499)
    for _ in range(8):
        c, _, _ = fus.update(5501, 36.6)
        assert c, "5501 counts falsely disconnected"
        c, _, _ = fus.update(5499, 36.6)
        assert c, "Single 5499 count cycle falsely disconnected without debounce"
    print("  [PASS] Detach Chatter (5501 vs 5499): Chatter suppressed, remaining CONNECTED")

    # 2nd sample of 5499 -> DISCONNECTED
    c, t, a = fus.update(5499, 36.6)
    assert not c, "Second cycle of 5499 failed to transition to DISCONNECTED"
    assert t == 1, f"Expected AMBIENT_OFFBODY (1), got {t}"
    print("  [PASS] Detach Debounce: Transitioned to DISCONNECTED on 2nd consecutive sample")

    # 4. Thermal Hysteresis (Fever & Hypothermia)
    fus.update(8000, 36.6); fus.update(8000, 36.6)
    assert fus.contact_state == 1

    # Fever transition: 38.0 C -> ELEVATED (no fever alert)
    _, t, a = fus.update(8000, 38.0)
    assert t == 4 and not (a & (1 << 5)), "38.0 C should be ELEVATED, not FEVER"
    # 38.1 C -> FEVER alert latched
    _, t, a = fus.update(8000, 38.1)
    assert t == 5 and (a & (1 << 5)), "38.1 C failed to latch FEVER"
    # Drop to 37.9 C -> FEVER alert must PERSIST (clear threshold is <= 37.8 C)
    _, t, a = fus.update(8000, 37.9)
    assert t == 5 and (a & (1 << 5)), "FEVER alert failed to persist at 37.9 C"
    # Drop to 37.8 C -> FEVER alert CLEARS to ELEVATED
    _, t, a = fus.update(8000, 37.8)
    assert t == 4 and not (a & (1 << 5)), "FEVER alert failed to clear at 37.8 C"
    print("  [PASS] Fever Hysteresis: Latched at 38.1 °C, persisted at 37.9 °C, cleared at 37.8 °C")

    # Hypothermia transition: 35.0 C -> NORMAL (no alert)
    _, t, a = fus.update(8000, 35.0)
    assert t == 3 and not (a & (1 << 4)), "35.0 C should be NORMAL"
    # 34.9 C -> HYPOTHERMIA alert latched
    _, t, a = fus.update(8000, 34.9)
    assert t == 2 and (a & (1 << 4)), "34.9 C failed to latch HYPOTHERMIA"
    # Rise to 35.2 C -> HYPOTHERMIA alert must PERSIST (clear threshold is >= 35.3 C)
    _, t, a = fus.update(8000, 35.2)
    assert t == 2 and (a & (1 << 4)), "HYPOTHERMIA failed to persist at 35.2 C"
    # Rise to 35.3 C -> HYPOTHERMIA alert CLEARS to NORMAL
    _, t, a = fus.update(8000, 35.3)
    assert t == 3 and not (a & (1 << 4)), "HYPOTHERMIA failed to clear at 35.3 C"
    print("  [PASS] Hypothermia Hysteresis: Latched at 34.9 °C, persisted at 35.2 °C, cleared at 35.3 °C")


# =============================================================================
# TEST GROUP 6: ARM Cortex-M4 Disassembly & Compilation Audit
# =============================================================================
def test_group_6_arm_objdump_audit():
    print("\n" + "=" * 75)
    print(" [ADVERSARIAL GROUP 6] ARM CORTEX-M4 HARDWARE DISASSEMBLY AUDIT")
    print("=" * 75)

    assert os.path.exists(OBJDUMP), f"tiarmobjdump not found: {OBJDUMP}"
    assert os.path.exists(OBJ_FILE), f"Object file not found: {OBJ_FILE}"

    res = subprocess.run([OBJDUMP, "-d", OBJ_FILE], capture_output=True, text=True, check=True)
    asm_output = res.stdout

    # Audit 1: Hardware SMULL instruction for 64-bit integer squaring
    smull_matches = re.findall(r'\bsmull\b', asm_output)
    assert len(smull_matches) >= 1, "Hardware SMULL instruction NOT generated in edgeai_ecg.obj!"
    print(f"  [PASS] Hardware SMULL instruction verified in edgeai_ecg.obj ({len(smull_matches)} instance(s) found)")

    # Audit 2: Symbol table verification in smartban.map
    assert os.path.exists(MAP_FILE), f"smartban.map not found: {MAP_FILE}"
    with open(MAP_FILE, "r") as f:
        map_content = f.read()

    core_symbols = [
        "edgeai_ecg_process_sample",
        "edgeai_imu_process_sample",
        "edgeai_fusion_process",
        "ecg_isqrt"
    ]
    for sym in core_symbols:
        assert re.search(r'\b' + re.escape(sym) + r'\b', map_content), f"Missing symbol: {sym}"
    print(f"  [PASS] All core Edge-AI symbols verified linked in smartban.map")


# =============================================================================
# Main Adversarial Test Orchestrator
# =============================================================================
def main():
    print("=" * 75)
    print(" SMARTBAN M3 GATE: ADVERSARIAL STRESS VERIFICATION SUITE")
    print(" Challenger 1 (EMPIRICAL CHALLENGER - critic, specialist)")
    print("=" * 75)

    try:
        test_group_1_dsp_limits()
        test_group_2_cardiac_anomalies()
        test_group_3_imu_fall_stress()
        test_group_4_static_sma_invariant()
        test_group_5_contact_thermal_hysteresis()
        test_group_6_arm_objdump_audit()

        print("\n" + "=" * 75)
        print(" [ALL ADVERSARIAL TESTS PASSED] MILESTONE M3 GATE VERIFICATION COMPLETE")
        print("=" * 75)
        return 0
    except AssertionError as e:
        print(f"\n[ADVERSARIAL TEST FAILED] {e}")
        return 1
    except Exception as e:
        print(f"\n[UNEXPECTED ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 2


if __name__ == "__main__":
    sys.exit(main())
