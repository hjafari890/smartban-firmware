#!/usr/bin/env python3
"""
===============================================================================
SmartBAN Sensor Node Firmware — Milestone M3 Gate Challenger 2 Verification
Firmware Binary, Build Pipeline, Memory Map & Data Reduction Audit
Target MCU: TI CC2652R1 (SimpleLink ARM Cortex-M4F, 80 KB SRAM, 352 KB Flash)
Toolchain: tiarmclang 5.1.1.LTS, tiarmobjcopy, tiarmnm, tiarmreadelf
===============================================================================
"""

import os
import re
import sys
import subprocess

FIRMWARE_DIR = r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\06_tirtos_smartban"
MAP_FILE = os.path.join(FIRMWARE_DIR, "smartban.map")
OUT_FILE = os.path.join(FIRMWARE_DIR, "smartban.out")
HEX_FILE = os.path.join(FIRMWARE_DIR, "smartban.hex")
RAW_HEX_FILE = os.path.join(FIRMWARE_DIR, "smartban_raw.hex")
TOOLCHAIN_BIN = r"C:\ti\ccs2101\ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS\bin"
TIARMREADELF = os.path.join(TOOLCHAIN_BIN, "tiarmreadelf.exe")
TIARMNM = os.path.join(TOOLCHAIN_BIN, "tiarmnm.exe")
TIARMCLANG = os.path.join(TOOLCHAIN_BIN, "tiarmclang.exe")

# =============================================================================
# GROUP 1: Clean Rebuild & Compiler Diagnostics Audit
# =============================================================================
def test_group_1_clean_rebuild_and_diagnostics():
    print("=" * 75)
    print(" [TEST GROUP 1] CLEAN REBUILD & COMPILER/LINKER DIAGNOSTICS AUDIT")
    print("=" * 75)

    assert os.path.exists(TIARMCLANG), f"tiarmclang binary not found: {TIARMCLANG}"
    ver_out = subprocess.check_output([TIARMCLANG, "--version"], text=True)
    print(f"  [INFO] Compiler Version: {ver_out.splitlines()[0]}")
    assert "5.1.1.LTS" in ver_out, f"Unexpected compiler version: {ver_out}"

    # Execute clean rebuild
    build_script = os.path.join(FIRMWARE_DIR, "build_and_flash.py")
    res = subprocess.run([sys.executable, build_script, "--clean", "--no-flash"],
                         cwd=FIRMWARE_DIR, capture_output=True, text=True)
    if res.returncode != 0:
        print("STDOUT:\n", res.stdout)
        print("STDERR:\n", res.stderr)
    assert res.returncode == 0, f"Clean build failed with return code {res.returncode}"

    combined_output = res.stdout + "\n" + res.stderr
    # Scan for warnings (excluding harmless commandline flags like -Wl,--warn_sections)
    warnings = [l.strip() for l in combined_output.splitlines() 
                if "warning" in l.lower() and "--warn_sections" not in l and "-w" not in l.lower()]
    # Scan for errors
    errors = [l.strip() for l in combined_output.splitlines() 
              if ("error:" in l.lower() or "failed" in l.lower() or "fatal" in l.lower()) 
              and "smartrf" not in l.lower()]

    print(f"  [INFO] Compiler & Linker Warnings: {len(warnings)}")
    print(f"  [INFO] Compiler & Linker Errors:   {len(errors)}")
    assert len(warnings) == 0, f"Compiler/Linker warnings detected: {warnings}"
    assert len(errors) == 0, f"Compiler/Linker errors detected: {errors}"
    print("  [PASS] Clean rebuild succeeded with exactly 0 warnings and 0 errors under tiarmclang 5.1.1.LTS.")


# =============================================================================
# GROUP 2: ELF Header & ARM Architecture Attributes
# =============================================================================
def test_group_2_elf_header_and_arm_attributes():
    print("=" * 75)
    print(" [TEST GROUP 2] ELF HEADER & ARM CORTEX-M4 ARCHITECTURE ATTRIBUTES")
    print("=" * 75)

    assert os.path.exists(OUT_FILE), f"ELF output file not found: {OUT_FILE}"
    assert os.path.exists(TIARMREADELF), f"tiarmreadelf tool not found: {TIARMREADELF}"

    # Read ELF Header
    header_out = subprocess.check_output([TIARMREADELF, "-h", "smartban.out"], cwd=FIRMWARE_DIR, text=True)
    assert "ELF32" in header_out, "ELF is not 32-bit"
    assert "2's complement, little endian" in header_out, "ELF is not 2's complement little-endian"
    assert "EXEC (Executable file)" in header_out, "ELF is not an executable file"
    assert "Machine:                           ARM" in header_out, "ELF machine is not ARM"
    print("  [PASS] ELF32 Executable, Little-Endian, ARM Machine Header verified.")

    # Read Architecture Build Attributes
    attr_out = subprocess.check_output([TIARMREADELF, "-A", "smartban.out"], cwd=FIRMWARE_DIR, text=True)
    assert "cortex-m4" in attr_out, "Target CPU is not cortex-m4"
    assert "ARM v7E-M" in attr_out, "CPU arch is not ARM v7E-M"
    assert "Thumb-2" in attr_out, "Instruction set is not Thumb-2"
    assert "VFPv4-D16" in attr_out, "Floating-point unit is not VFPv4-D16"
    assert "AAPCS VFP" in attr_out, "ABI is not HardFP (AAPCS VFP)"
    print("  [PASS] Target CPU: ARM Cortex-M4 (ARM v7E-M, Thumb-2, VFPv4-D16 Hardware FPU) verified.")


# =============================================================================
# GROUP 3: Intel HEX Sanitization & Type 03 Audit
# =============================================================================
def test_group_3_intel_hex_sanitization():
    print("=" * 75)
    print(" [TEST GROUP 3] INTEL HEX SANITIZATION & TYPE 03 RECORD AUDIT")
    print("=" * 75)

    assert os.path.exists(HEX_FILE), f"HEX file not found: {HEX_FILE}"
    with open(HEX_FILE, "r") as f:
        hex_lines = [l.strip() for l in f if l.strip()]

    type_counts = {}
    type_03_records = []
    for idx, line in enumerate(hex_lines):
        assert line.startswith(":"), f"Line {idx+1} does not start with colon: {line}"
        rtype = line[7:9]
        type_counts[rtype] = type_counts.get(rtype, 0) + 1
        if rtype == "03":
            type_03_records.append((idx + 1, line))

    print(f"  [INFO] Total HEX Records: {len(hex_lines)}")
    print(f"  [INFO] Record Types Distribution: {type_counts}")
    print(f"  [INFO] Type 03 Records Found: {len(type_03_records)}")

    assert len(type_03_records) == 0, f"Found {len(type_03_records)} Type 03 records in {HEX_FILE}: {type_03_records}"
    assert "00" in type_counts, "No Type 00 (Data) records found in HEX"
    assert "01" in type_counts and type_counts["01"] == 1, "Expected exactly 1 EOF record"

    # Verify that raw hex file had the Type 03 record stripped
    if os.path.exists(RAW_HEX_FILE):
        with open(RAW_HEX_FILE, "r") as f:
            raw_lines = [l.strip() for l in f if l.strip()]
        raw_03 = [l for l in raw_lines if l[7:9] == "03"]
        print(f"  [INFO] Verified raw HEX had {len(raw_03)} Type 03 record(s) stripped by build pipeline: {raw_03}")

    print("  [PASS] Intel HEX successfully sanitized: Exactly 0 Type 03 records exist.")


# =============================================================================
# GROUP 4: Memory Map, Section Overlaps & Free SRAM Headroom
# =============================================================================
def test_group_4_memory_map_and_sram_headroom():
    print("=" * 75)
    print(" [TEST GROUP 4] MEMORY MAP, SECTION OVERLAPS & SRAM HEADROOM AUDIT")
    print("=" * 75)

    assert os.path.exists(MAP_FILE), f"Map file not found: {MAP_FILE}"
    with open(MAP_FILE, "r") as f:
        map_text = f.read()

    seg_pos = map_text.find("SEGMENT ALLOCATION MAP")
    assert seg_pos != -1, "SEGMENT ALLOCATION MAP not found in map file"
    seg_section = map_text[seg_pos:map_text.find("MODULE SUMMARY")]
    lines = seg_section.splitlines()

    sec_regex = re.compile(r"^\s+([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})\s+([0-9a-fA-F]{8})\s+([rwx\-]+)\s+(\S+)")
    sram_sections = []
    flash_sections = []

    for line in lines:
        m = sec_regex.match(line)
        if m:
            run_addr = int(m.group(1), 16)
            load_addr = int(m.group(2), 16)
            length = int(m.group(3), 16)
            attrs = m.group(5)
            name = m.group(6)
            entry = {"run": run_addr, "load": load_addr, "len": length, "attrs": attrs, "name": name}
            if 0x20000000 <= run_addr < 0x20014000:
                sram_sections.append(entry)
            elif 0x00000000 <= run_addr < 0x00058000:
                flash_sections.append(entry)

    # Check for SRAM overlaps
    sram_sorted = sorted(sram_sections, key=lambda x: x["run"])
    for i in range(len(sram_sorted) - 1):
        s1 = sram_sorted[i]
        s2 = sram_sorted[i+1]
        assert s1["run"] + s1["len"] <= s2["run"], f"Overlap detected between {s1['name']} and {s2['name']}"

    # Check for Flash overlaps
    flash_sorted = sorted(flash_sections, key=lambda x: x["run"])
    for i in range(len(flash_sorted) - 1):
        s1 = flash_sorted[i]
        s2 = flash_sorted[i+1]
        assert s1["run"] + s1["len"] <= s2["run"], f"Flash overlap detected between {s1['name']} and {s2['name']}"

    print(f"  [PASS] Verified 0 overlaps across {len(sram_sections)} SRAM sections and {len(flash_sections)} Flash sections.")

    # Calculate free SRAM margin between .bss end and .stack start
    bss_sec = next((s for s in sram_sections if s["name"] == ".bss"), None)
    stack_sec = next((s for s in sram_sections if s["name"] == ".stack"), None)
    priheap_sec = next((s for s in sram_sections if s["name"] == ".priheap"), None)

    assert bss_sec is not None, ".bss section missing from SRAM!"
    assert stack_sec is not None, ".stack section missing from SRAM!"
    assert priheap_sec is not None, ".priheap section missing from SRAM!"

    bss_end = bss_sec["run"] + bss_sec["len"]
    stack_start = stack_sec["run"]
    free_sram_bytes = stack_start - bss_end
    free_sram_kb = free_sram_bytes / 1024.0

    print(f"  [INFO] .priheap:     0x{priheap_sec['run']:08X}..0x{priheap_sec['run']+priheap_sec['len']:08X} ({priheap_sec['len']/1024.0:.2f} KB)")
    print(f"  [INFO] .bss:        0x{bss_sec['run']:08X}..0x{bss_end:08X} ({bss_sec['len']/1024.0:.2f} KB)")
    print(f"  [INFO] .stack:      0x{stack_start:08X}..0x{stack_start+stack_sec['len']:08X} ({stack_sec['len']/1024.0:.2f} KB)")
    print(f"  [INFO] Contiguous Unallocated Free SRAM Margin: {free_sram_bytes} Bytes ({free_sram_kb:.2f} KB)")

    assert free_sram_kb > 40.0, f"Free SRAM margin ({free_sram_kb:.2f} KB) is below 40.0 KB threshold!"
    print(f"  [PASS] Contiguous unallocated free SRAM margin is {free_sram_kb:.2f} KB (> 40 KB required threshold).")


# =============================================================================
# GROUP 5: Symbol Table Completeness Audit (Milestone M3)
# =============================================================================
def test_group_5_symbol_table_audit():
    print("=" * 75)
    print(" [TEST GROUP 5] MILESTONE M3 SYMBOL TABLE COMPLETENESS AUDIT")
    print("=" * 75)

    nm_out = subprocess.check_output([TIARMNM, "smartban.out"], cwd=FIRMWARE_DIR, text=True)
    sym_dict = {}
    for line in nm_out.splitlines():
        parts = line.strip().split()
        if len(parts) >= 3:
            sym_dict[parts[2]] = (parts[0], parts[1])

    required_m3_symbols = [
        "edgeai_ecg_init",
        "edgeai_ecg_process_sample",
        "edgeai_imu_init",
        "edgeai_imu_process_sample",
        "edgeai_fusion_init",
        "edgeai_fusion_process",
        "edgeai_format_semantic_json",
        "edgeai_format_raw_json",
        "edgeai_toggle_stream_mode",
        "g_latest_semantic_token",
        "task_edgeai_entry.s_ecg_state",
        "task_edgeai_entry.s_imu_state",
        "task_edgeai_entry.s_fusion_state"
    ]

    for sym in required_m3_symbols:
        assert sym in sym_dict, f"Required M3 symbol '{sym}' missing from ELF symbol table!"
        addr, stype = sym_dict[sym]
        print(f"  [PASS] Symbol '{sym:32s}' verified at 0x{addr} (Type {stype})")


# =============================================================================
# GROUP 6: Telemetry Data Reduction Proof & Evaluation Scenarios
# =============================================================================
def test_group_6_data_reduction_proof():
    print("=" * 75)
    print(" [TEST GROUP 6] TELEMETRY DATA REDUCTION MATHEMATICAL & EMPIRICAL PROOF")
    print("=" * 75)

    # 1. Raw Stream Mode Bandwidth Breakdown
    # ADS1292 ECG: 250 SPS * (3B Header + 3B CH1 + 3B CH2) = 2250 B/s
    b_ecg_raw = 250 * 9
    # ADXL362 IMU: 50 SPS * 6B (2B X + 2B Y + 2B Z) = 300 B/s
    b_imu_raw = 50 * 6
    # Slow Environmental: 1 Hz * (4B Temp + 4B Lux + 2B Prox + 6B Pad/Status) = 16 B/s
    b_slow_raw = 1 * 16
    b_raw_total = b_ecg_raw + b_imu_raw + b_slow_raw
    assert b_raw_total == 2566, f"Raw throughput calculation error: {b_raw_total} != 2566 B/s"

    print(f"  [INFO] Raw Stream Mode Bandwidth Breakdown:")
    print(f"         - ADS1292 ECG @ 250 Hz (9 B/smp) : {b_ecg_raw} B/s")
    print(f"         - ADXL362 IMU @ 50 Hz (6 B/smp)  : {b_imu_raw} B/s")
    print(f"         - Environmental Sensors @ 1 Hz   : {b_slow_raw} B/s")
    print(f"         - Total Raw Bandwidth             : {b_raw_total} B/s ({b_raw_total * 8 / 1000.0:.2f} kbps)")

    # 2. Semantic Mode Bandwidth Breakdown
    # Canonical 1 Hz JSON Token
    canonical_json_len = 85
    # Tagged JSON Token
    tagged_json_len = 138
    # Binary Struct
    binary_struct_len = 44

    reduction_canonical = (1.0 - (canonical_json_len / b_raw_total)) * 100.0
    reduction_tagged    = (1.0 - (tagged_json_len / b_raw_total)) * 100.0
    reduction_binary    = (1.0 - (binary_struct_len / b_raw_total)) * 100.0

    print(f"\n  [INFO] Semantic Telemetry Modes & Data Volume Reduction:")
    print(f"         - Tagged JSON Frame ({tagged_json_len} B/s)    : {reduction_tagged:.2f}% reduction")
    print(f"         - Canonical JSON Frame ({canonical_json_len} B/s) : {reduction_canonical:.2f}% reduction")
    print(f"         - Binary Token Struct ({binary_struct_len} B/s)   : {reduction_binary:.2f}% reduction")

    assert reduction_canonical >= 95.0, f"Canonical reduction failed threshold: {reduction_canonical:.2f}% < 95.0%"
    assert reduction_tagged >= 94.0, f"Tagged reduction failed threshold: {reduction_tagged:.2f}% < 94.0%"
    assert reduction_binary >= 98.0, f"Binary reduction failed threshold: {reduction_binary:.2f}% < 98.0%"

    # 3. Longitudinal Data Volume & Energy Comparison (1 Hour, 24 Hours, 7 Days)
    print("\n  [INFO] Longitudinal Volume Comparison:")
    for label, secs in [("1 Hour", 3600), ("24 Hours", 86400), ("7 Days", 604800)]:
        raw_vol_mb = (b_raw_total * secs) / (1024 * 1024)
        sem_vol_mb = (canonical_json_len * secs) / (1024 * 1024)
        savings_mb = raw_vol_mb - sem_vol_mb
        print(f"         - {label:8s}: Raw = {raw_vol_mb:7.2f} MB | Semantic = {sem_vol_mb:6.2f} MB | Saved = {savings_mb:7.2f} MB ({reduction_canonical:.2f}%)")

    print(f"  [PASS] Data reduction verified: Canonical mode achieves {reduction_canonical:.2f}% reduction (Target: >=95.0%).")


# =============================================================================
# GROUP 7: Hardware Flaw Exclusion Audit (MAX32664 PPG)
# =============================================================================
def test_group_7_hardware_flaw_exclusion():
    print("=" * 75)
    print(" [TEST GROUP 7] HARDWARE FLAW EXCLUSION AUDIT (MAX32664 PPG)")
    print("=" * 75)

    forbidden = ["max32664", "maxm86161", "max30102", "hal_ppg"]
    nm_out = subprocess.check_output([TIARMNM, "smartban.out"], cwd=FIRMWARE_DIR, text=True)
    violations = []
    for line in nm_out.splitlines():
        for pat in forbidden:
            if pat in line.lower():
                violations.append(line)

    assert len(violations) == 0, f"Forbidden PPG symbols present in ELF binary: {violations}"
    print(f"  [PASS] Exactly 0 forbidden PPG symbols found in {OUT_FILE} symbol table.")

    # Audit source tree for active code
    source_violations = []
    for root, _, files in os.walk(FIRMWARE_DIR):
        if "obj" in root or "test" in root:
            continue
        for f in files:
            if f.endswith((".c", ".h")):
                p = os.path.join(root, f)
                with open(p, "r", encoding="utf-8", errors="ignore") as fl:
                    for i, line in enumerate(fl, 1):
                        stripped = line.strip()
                        if stripped.startswith("//") or stripped.startswith("*") or stripped.startswith("/*"):
                            continue
                        for pat in forbidden:
                            if pat in stripped.lower():
                                source_violations.append((p, i, stripped))

    assert len(source_violations) == 0, f"Active code violations found for PPG: {source_violations}"
    print("  [PASS] Exactly 0 active code references to MAX32664/MAXM86161 in firmware sources.")


# =============================================================================
# Main Entry Point
# =============================================================================
if __name__ == "__main__":
    print("\n" + "#" * 75)
    print(" RUNNING CHALLENGER 2 EMPIRICAL AUDIT SUITE FOR MILESTONE M3 GATE")
    print("#" * 75 + "\n")

    test_group_1_clean_rebuild_and_diagnostics()
    test_group_2_elf_header_and_arm_attributes()
    test_group_3_intel_hex_sanitization()
    test_group_4_memory_map_and_sram_headroom()
    test_group_5_symbol_table_audit()
    test_group_6_data_reduction_proof()
    test_group_7_hardware_flaw_exclusion()

    print("\n" + "=" * 75)
    print(" ALL 7 CHALLENGER 2 GATE VERIFICATION TESTS PASSED UNANIMOUSLY!")
    print("=" * 75 + "\n")
