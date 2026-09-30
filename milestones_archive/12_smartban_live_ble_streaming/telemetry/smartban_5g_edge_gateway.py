#!/usr/bin/env python3
"""
===============================================================================
smartban_5g_edge_gateway.py
SmartBAN Milestone M4 (Pillar 5): 5G RedCap / UPF Network Slicing Edge Gateway
Can run on Primary PC, Secondary PC, or Raspberry Pi 5.
Routes ETSI TS 103 326 SmartBAN frames into 5G Network Slices:
  - Slice 1 (5G-mMTC, SST=3): 1 Hz Scheduled Access Period (SAP) Semantic Tokens
  - Slice 2 (5G-URLLC, SST=2): Contention Access Period (CAP) Anomaly Bursts
===============================================================================
"""

import time
import json
import argparse

def process_smartban_b_frame(line: str) -> dict:
    """Parse a live `B,...` frame from the CC2652R1 SmartBAN Sensor Node."""
    parts = line.strip().split(",")
    if len(parts) < 15 or parts[0] != "B":
        return {}
    slot = int(parts[3])
    tinyml_cls = parts[4].strip()
    is_urllc = (slot == 1) or (tinyml_cls in ("S", "V", "F"))
    return {
        "timestamp_utc": round(time.time(), 3),
        "ibi_sequence": int(parts[1]),
        "smartban_slot": "CAP_EMERGENCY" if slot == 1 else "SAP_SCHEDULED",
        "5g_upf_slice": "URLLC (Ultra-Reliable Low-Latency, SST=2, QoS 5QI=82)" if is_urllc else "mMTC (Massive Medical Telemetry, SST=3, QoS 5QI=9)",
        "tinyml_aami_class": tinyml_cls,
        "tinyml_confidence_pct": int(parts[5]),
        "tinyml_inference_us": int(parts[6]),
        "cortex_m4f_active_us_s": int(parts[7]),
        "raw_bps": int(parts[8]),
        "semantic_bps": int(parts[9]),
        "adaptive_bps": int(parts[10]),
        "power_raw_mw": float(parts[11]),
        "power_semantic_mw": float(parts[12]),
        "power_adaptive_mw": float(parts[13]),
        "energy_uj_per_beat": float(parts[14])
    }

def run_demo_simulation():
    sample_frames = [
        "B,101,0,0,N,98,30,1318,9538,92,92,22.18,1.60,1.60,1333.3",
        "B,102,0,0,N,99,29,1317,9538,92,92,22.18,1.60,1.60,1333.3",
        "B,103,0,1,V,96,30,1318,9538,92,9538,22.18,1.60,22.18,18483.3"
    ]
    print("=========================================================================")
    print(" SmartBAN Coordinator (BC) -> 5G UPF Multi-Access Edge Computing Gateway")
    print("=========================================================================")
    for f in sample_frames:
        pkt = process_smartban_b_frame(f)
        print(json.dumps(pkt, indent=2))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", default=True)
    args = parser.parse_args()
    run_demo_simulation()
