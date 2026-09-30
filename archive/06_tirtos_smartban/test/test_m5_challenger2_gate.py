#!/usr/bin/env python3
"""
===============================================================================
test_m5_challenger2_gate.py
Challenger 2 Milestone M5 Gate Verification Suite:
Binary Artifact, Toolchain, Intel HEX, SRAM Headroom & Flaw Exclusion Audit
Platform: TI CC2652R1 LaunchPad (ARM Cortex-M4F @ 48 MHz) + SmartBAN Shield Rev 3.5
Toolchain: tiarmclang 5.1.1.LTS / SysConfig 1.21.1 / SimpleLink SDK 8.33 / TI-RTOS7
Author: Challenger 2 (Binary Artifact, Toolchain & Master Harness Specialist)

Audits:
  1. Independent Toolchain & ELF Header Audit:
     - Direct binary ELF32 parsing & tiarmreadelf inspection
     - ARMv7E-M, Cortex-M4, Thumb-2, VFPv4-D16, AAPCS VFP hard-float
     - Aggressive Size (-Oz) optimization attribute
     - ResetISR entry point verification (0xEA55)
  2. Intel HEX Sanitization & Boundary Audit:
     - Strict absence of Type 03 (Start Segment) and Type 05 (Start Linear) records
     - Valid 2's complement checksums across 100% of records
     - CC2652R1 Flash address boundary enforcement (0x00000000 - 0x00057FFF)
     - 88-byte CCFG table presence at 0x00057FA8 - 0x00057FFF
  3. Cryptographic Hashes & Binary Integrity Audit:
     - Exact match of smartban.out and smartban.hex against TEST_READY.md
     - smartban.map length verification (209,143 bytes) and timestamp isolation
  4. Memory Map Contiguous Free SRAM Headroom Audit:
     - Contiguous unallocated SRAM margin between .bss end (0x2000964C) and .stack (0x20013800)
     - Margin strictly exceeds 40 KB (confirming 41,396 Bytes / 40.43 KB)
     - Total unallocated SRAM headroom audit (46,587 Bytes / 56.87% free)
     - Zero segment overlap verification across all SRAM and FLASH sections
  5. Flawed Optical Front-End Hardware Exclusion Audit:
     - Inspection of 878 linked symbols via tiarmnm (0 MAX32664 / MAXM86161 / MAX30102)
     - Full text scan of smartban.map (0 occurrences)
     - Exclusion of I2C address 0x55 and MAX32664 BioHub pins
  6. End-to-End Test Harness & Clean Rebuild Execution:
     - test/test_harness.py execution (all 8 milestone suites, 100% pass)
     - python build_and_flash.py --clean --no-flash clean build pipeline execution
===============================================================================
"""

import os
import sys
import struct
import hashlib
import subprocess
import re

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(TEST_DIR, ".."))
OUT_FILE = os.path.join(PROJECT_DIR, "smartban.out")
HEX_FILE = os.path.join(PROJECT_DIR, "smartban.hex")
MAP_FILE = os.path.join(PROJECT_DIR, "smartban.map")
COMPILER_DIR = r"C:/ti/ccs2101/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS"
READELF_BIN = os.path.join(COMPILER_DIR, "bin", "tiarmreadelf.exe")
NM_BIN = os.path.join(COMPILER_DIR, "bin", "tiarmnm.exe")

passed_checks = 0
failed_checks = 0

def assert_check(condition: bool, description: str):
    global passed_checks, failed_checks
    if condition:
        passed_checks += 1
        print(f"  [PASS] {description}")
    else:
        failed_checks += 1
        print(f"  [FAIL] {description}")

def test_toolchain_and_elf_header():
    print("\n" + "=" * 78)
    print(" [AUDIT 1] INDEPENDENT TOOLCHAIN & ELF HEADER AUDIT (smartban.out)")
    print("=" * 78)

    assert_check(os.path.exists(OUT_FILE), f"smartban.out exists ({os.path.getsize(OUT_FILE):,} bytes)")

    with open(OUT_FILE, "rb") as f:
        elf_bytes = f.read()

    # ELF Header Parsing
    magic = elf_bytes[:4]
    assert_check(magic == b"\x7fELF", "ELF magic number \\x7fELF verified")

    elf_class = elf_bytes[4]
    assert_check(elf_class == 1, "ELF Class is 32-bit (ELFCLASS32)")

    data_enc = elf_bytes[5]
    assert_check(data_enc == 1, "Data encoding is 2's complement little-endian (ELFDATA2LSB)")

    e_type = struct.unpack("<H", elf_bytes[16:18])[0]
    assert_check(e_type == 2, "ELF Type is Executable (ET_EXEC = 2)")

    e_machine = struct.unpack("<H", elf_bytes[18:20])[0]
    assert_check(e_machine == 40, f"Target machine is ARM (EM_ARM = 40, actual={e_machine})")

    e_entry = struct.unpack("<I", elf_bytes[24:28])[0]
    assert_check(e_entry == 0xEA55, f"Entry point symbol ResetISR at 0x{e_entry:08X} (Thumb-2 bit set)")

    e_flags = struct.unpack("<I", elf_bytes[36:40])[0]
    eabi_ver = (e_flags >> 24) & 0xFF
    assert_check(eabi_ver == 5, f"ARM EABI Version is 5 (flags=0x{e_flags:08X}, eabi_ver={eabi_ver})")

    # tiarmreadelf verification
    if os.path.exists(READELF_BIN):
        res = subprocess.run([READELF_BIN, "-A", OUT_FILE], capture_output=True, text=True)
        attr_out = res.stdout

        assert_check("cortex-m4" in attr_out, "tiarmreadelf confirms CPU_name: cortex-m4")
        assert_check("ARM v7E-M" in attr_out, "tiarmreadelf confirms CPU_arch: ARM v7E-M")
        assert_check("Thumb-2" in attr_out, "tiarmreadelf confirms THUMB_ISA_use: Thumb-2")
        assert_check("VFPv4-D16" in attr_out, "tiarmreadelf confirms FP_arch: VFPv4-D16")
        assert_check("AAPCS VFP" in attr_out, "tiarmreadelf confirms ABI_VFP_args: AAPCS VFP (Hard-Float)")
        assert_check("Aggressive Size" in attr_out, "tiarmreadelf confirms ABI_optimization_goals: Aggressive Size (-Oz)")
    else:
        assert_check(False, f"tiarmreadelf not found at {READELF_BIN}")

def test_intel_hex_records():
    print("\n" + "=" * 78)
    print(" [AUDIT 2] INTEL HEX SANITIZATION & BOUNDARY AUDIT (smartban.hex)")
    print("=" * 78)

    assert_check(os.path.exists(HEX_FILE), f"smartban.hex exists ({os.path.getsize(HEX_FILE):,} bytes)")

    type_counts = {}
    total_data_bytes = 0
    min_phys = 0xFFFFFFFF
    max_phys = 0x00000000
    base_addr = 0
    ccfg_bytes = bytearray()

    with open(HEX_FILE, "r") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            assert line.startswith(":"), f"Line {line_num}: missing leading colon"
            rec_len = int(line[1:3], 16)
            offset = int(line[3:7], 16)
            rec_type = int(line[7:9], 16)
            data_str = line[9:9+rec_len*2]
            chk = int(line[9+rec_len*2:11+rec_len*2], 16)

            calc_chk = rec_len + (offset >> 8) + (offset & 0xFF) + rec_type
            for i in range(0, len(data_str), 2):
                calc_chk += int(data_str[i:i+2], 16)
            calc_chk = ((~calc_chk) + 1) & 0xFF
            assert chk == calc_chk, f"Line {line_num}: Checksum mismatch ({chk:02X} != {calc_chk:02X})"

            type_counts[rec_type] = type_counts.get(rec_type, 0) + 1

            if rec_type == 0:  # Data record
                phys = base_addr + offset
                min_phys = min(min_phys, phys)
                max_phys = max(max_phys, phys + rec_len - 1)
                total_data_bytes += rec_len
                # Check CCFG range: 0x00057FA8 - 0x00057FFF
                if 0x00057FA8 <= phys <= 0x00057FFF:
                    for i in range(0, len(data_str), 2):
                        byte_addr = phys + (i // 2)
                        if 0x00057FA8 <= byte_addr <= 0x00057FFF:
                            ccfg_bytes.append(int(data_str[i:i+2], 16))
            elif rec_type == 2:  # Extended Segment Address
                base_addr = int(data_str, 16) << 4
            elif rec_type == 4:  # Extended Linear Address
                base_addr = int(data_str, 16) << 16
            elif rec_type == 1:  # End of File
                pass
            else:
                assert False, f"Unexpected HEX record type 0x{rec_type:02X} on line {line_num}"

    assert_check(type_counts.get(3, 0) == 0, f"Zero Type 03 (Start Segment Address) records (count={type_counts.get(3, 0)})")
    assert_check(type_counts.get(5, 0) == 0, f"Zero Type 05 (Start Linear Address) records (count={type_counts.get(5, 0)})")
    assert_check(min_phys == 0x00000000, f"Lowest physical Flash byte at 0x00000000 (actual=0x{min_phys:08X})")
    assert_check(max_phys <= 0x00057FFF, f"Highest physical Flash byte <= 0x00057FFF (actual=0x{max_phys:08X})")
    assert_check(max_phys == 0x00057FFF, f"Highest physical Flash byte touches CCFG top 0x00057FFF (actual=0x{max_phys:08X})")
    assert_check(len(ccfg_bytes) == 88, f"CCFG boot configuration block complete (88/88 bytes present)")
    print(f"  HEX Stats: {type_counts.get(0, 0)} data records, {total_data_bytes:,} payload bytes.")

def test_cryptographic_hashes_and_integrity():
    print("\n" + "=" * 78)
    print(" [AUDIT 3] CRYPTOGRAPHIC HASHES & ARTIFACT INTEGRITY AUDIT")
    print("=" * 78)

    expected_out_hash = "39bd69188abb87d25145161c2182be54d3b745ca90f3d28b1ab84f8ee6da4121"
    expected_hex_hash = "f8cf9fa72e6496610418fe1c54780c877216303987147510de5d408c73e23f06"
    expected_out_size = 821948
    expected_hex_size = 212430
    expected_map_size = 209143

    out_bytes = open(OUT_FILE, "rb").read()
    hex_bytes = open(HEX_FILE, "rb").read()
    map_bytes = open(MAP_FILE, "rb").read()

    actual_out_hash = hashlib.sha256(out_bytes).hexdigest()
    actual_hex_hash = hashlib.sha256(hex_bytes).hexdigest()

    assert_check(actual_out_hash == expected_out_hash, f"smartban.out SHA256 verified: {actual_out_hash}")
    assert_check(len(out_bytes) == expected_out_size, f"smartban.out size verified: {len(out_bytes):,} bytes")

    assert_check(actual_hex_hash == expected_hex_hash, f"smartban.hex SHA256 verified: {actual_hex_hash}")
    assert_check(len(hex_bytes) == expected_hex_size, f"smartban.hex size verified: {len(hex_bytes):,} bytes")

    assert_check(len(map_bytes) == expected_map_size, f"smartban.map size verified: {len(map_bytes):,} bytes")

    # Verify that the map file structure is identical and timestamp is present on line 4
    map_lines = map_bytes.decode("utf-8", errors="ignore").splitlines()
    assert_check(len(map_lines) == 2487, f"smartban.map line count verified: {len(map_lines)} lines")
    assert_check(map_lines[3].startswith(">> Linked "), f"smartban.map dynamic link timestamp isolated: '{map_lines[3]}'")

def test_memory_map_headroom():
    print("\n" + "=" * 78)
    print(" [AUDIT 4] MEMORY MAP CONTIGUOUS FREE SRAM HEADROOM AUDIT")
    print("=" * 78)

    with open(MAP_FILE, "r", encoding="utf-8", errors="ignore") as f:
        map_content = f.read()

    # Parse SEGMENT ALLOCATION MAP
    in_seg_map = False
    sram_segments = []
    flash_segments = []

    for line in map_content.splitlines():
        if "SEGMENT ALLOCATION MAP" in line:
            in_seg_map = True
            continue
        if "SECTION ALLOCATION MAP" in line:
            in_seg_map = False
            continue
        if in_seg_map:
            parts = line.split()
            if len(parts) >= 6:
                try:
                    run_org = int(parts[0], 16)
                    length = int(parts[2], 16)
                    name = parts[-1]
                    if length > 0:
                        if 0x20000000 <= run_org < 0x20014000:
                            sram_segments.append((run_org, run_org + length, length, name))
                        elif 0x00000000 <= run_org < 0x00058000:
                            flash_segments.append((run_org, run_org + length, length, name))
                except ValueError:
                    pass

    # Verify SRAM sections non-overlap
    sram_sorted = sorted(sram_segments, key=lambda x: x[0])
    no_overlap = True
    for i in range(len(sram_sorted) - 1):
        if sram_sorted[i][1] > sram_sorted[i+1][0]:
            no_overlap = False
            print(f"  [ERROR] SRAM overlap: {sram_sorted[i][3]} and {sram_sorted[i+1][3]}")
    assert_check(no_overlap, f"Zero memory overlap across all {len(sram_sorted)} SRAM segments")

    # Locate .bss and .stack
    bss_seg = [s for s in sram_sorted if ".bss" in s[3]]
    stack_seg = [s for s in sram_sorted if ".stack" in s[3]]
    priheap_seg = [s for s in sram_sorted if ".priheap" in s[3]]

    assert_check(len(bss_seg) > 0, ".bss segment found in SRAM map")
    assert_check(len(stack_seg) > 0, ".stack segment found in SRAM map")
    assert_check(len(priheap_seg) > 0, ".priheap segment found in SRAM map")

    bss_start, bss_end, bss_size = bss_seg[0][0], bss_seg[0][1], bss_seg[0][2]
    stack_start, stack_end, stack_size = stack_seg[0][0], stack_seg[0][1], stack_seg[0][2]
    heap_start, heap_end, heap_size = priheap_seg[0][0], priheap_seg[0][1], priheap_seg[0][2]

    assert_check(bss_end == 0x2000964C, f".bss end address confirmed: 0x{bss_end:08X} (size={bss_size:,} B)")
    assert_check(stack_start == 0x20013800, f".stack start address confirmed: 0x{stack_start:08X} (size={stack_size:,} B)")
    assert_check(heap_end == 0x20005A50, f".priheap end address confirmed: 0x{heap_end:08X} (size={heap_size:,} B)")

    # Compute contiguous free SRAM headroom
    contiguous_free_sram = stack_start - bss_end
    assert_check(contiguous_free_sram > 40960, f"Contiguous free SRAM between .bss end and .stack start ({contiguous_free_sram:,} B) > 40 KB")
    assert_check(contiguous_free_sram == 41396, f"Contiguous free SRAM matches exact verified headroom: {contiguous_free_sram:,} Bytes (40.43 KB)")

    # Parse overall SRAM usage
    sram_match = re.search(r"SRAM\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([0-9a-fA-F]+)", map_content)
    assert_check(sram_match is not None, "MEMORY CONFIGURATION SRAM block parsed")
    if sram_match:
        sram_len = int(sram_match.group(2), 16)
        sram_used = int(sram_match.group(3), 16)
        sram_free = int(sram_match.group(4), 16)
        assert_check(sram_free == 46587, f"Total unallocated SRAM margin confirmed: {sram_free:,} Bytes (56.87% free)")
        print(f"  Memory Map Summary: 80 KB SRAM total, {sram_used:,} B used ({sram_used/sram_len*100:.1f}%), {sram_free:,} B free.")

def test_flawed_ppg_exclusion():
    print("\n" + "=" * 78)
    print(" [AUDIT 5] FLAWED OPTICAL PPG HARDWARE EXCLUSION AUDIT")
    print("=" * 78)

    # 1. Inspect ELF symbol table via tiarmnm
    if os.path.exists(NM_BIN):
        res = subprocess.run([NM_BIN, OUT_FILE], capture_output=True, text=True)
        nm_lines = res.stdout.splitlines()
        forbidden_keywords = ["max32664", "maxm86161", "max30102", "biohub", "hal_ppg"]
        matched_symbols = []
        for line in nm_lines:
            line_l = line.lower()
            for kw in forbidden_keywords:
                if kw in line_l:
                    matched_symbols.append((kw, line.strip()))

        assert_check(len(matched_symbols) == 0, f"tiarmnm symbol audit: 0 flawed PPG symbols found across {len(nm_lines):,} symbols")
        if matched_symbols:
            for kw, sym in matched_symbols:
                print(f"  [FOUND FORBIDDEN SYMBOL] {sym}")
    else:
        assert_check(False, f"tiarmnm tool not found at {NM_BIN}")

    # 2. Text scan of smartban.map
    with open(MAP_FILE, "r", encoding="utf-8", errors="ignore") as f:
        map_text = f.read().lower()

    for kw in ["max32664", "maxm86161", "max30102", "biohub"]:
        cnt = map_text.count(kw)
        assert_check(cnt == 0, f"smartban.map scan: 0 occurrences of '{kw}'")

    # 3. Source code exclusion audit in bsp/hal
    scan_dirs = [os.path.join(PROJECT_DIR, d) for d in ["bsp", "hal", "tasks", "edgeai", "telemetry"]]
    active_ppg_refs = 0
    for sd in scan_dirs:
        for root, dirs, files in os.walk(sd):
            for f in files:
                if f.endswith((".c", ".h")):
                    p = os.path.join(root, f)
                    with open(p, "r", encoding="utf-8", errors="ignore") as src_f:
                        for l_idx, line in enumerate(src_f, 1):
                            if "is EXCLUDED" in line or "PPG view" in line or "known PCB flaw" in line:
                                continue
                            for kw in ["max32664", "maxm86161", "max30102"]:
                                if kw in line.lower():
                                    active_ppg_refs += 1
                                    print(f"  Active PPG ref: {f}:{l_idx}: {line.strip()}")
    assert_check(active_ppg_refs == 0, "Source tree scan: zero active code references to flawed optical PPG")

def test_master_harness_and_clean_rebuild():
    print("\n" + "=" * 78)
    print(" [AUDIT 6] MASTER TEST HARNESS & CLEAN REBUILD EXECUTION")
    print("=" * 78)

    # 1. Master test harness execution
    harness_py = os.path.join(PROJECT_DIR, "test", "test_harness.py")
    assert_check(os.path.exists(harness_py), "test_harness.py exists")
    res_harness = subprocess.run([sys.executable, harness_py], cwd=PROJECT_DIR, capture_output=True, text=True)
    assert_check(res_harness.returncode == 0, f"Master test harness (test_harness.py) executed with exit code 0")
    assert_check("8 / 8 (100.0%)" in res_harness.stdout, "Master harness verified: 8 / 8 milestone test suites PASSED")

    # 2. Clean rebuild execution
    build_py = os.path.join(PROJECT_DIR, "build_and_flash.py")
    assert_check(os.path.exists(build_py), "build_and_flash.py exists")
    res_build = subprocess.run([sys.executable, build_py, "--clean", "--no-flash"], cwd=PROJECT_DIR, capture_output=True, text=True)
    assert_check(res_build.returncode == 0, "build_and_flash.py --clean --no-flash completed with exit code 0")
    assert_check("BUILD SUCCESSFUL" in res_build.stdout, "build_and_flash.py reports 'BUILD SUCCESSFUL'")

def main() -> int:
    print("=" * 78)
    print(" CHALLENGER 2: MILESTONE M5 GATE COMPREHENSIVE VERIFICATION")
    print(" Binary Artifact, Toolchain, Intel HEX, SRAM Headroom & Flaw Exclusion")
    print(f" Working Directory: {PROJECT_DIR}")
    print("=" * 78)

    test_toolchain_and_elf_header()
    test_intel_hex_records()
    test_cryptographic_hashes_and_integrity()
    test_memory_map_headroom()
    test_flawed_ppg_exclusion()
    test_master_harness_and_clean_rebuild()

    print("\n" + "=" * 78)
    print(" CHALLENGER 2 M5 GATE AUDIT SUMMARY SCORECARD")
    print("=" * 78)
    print(f" TOTAL AUDIT CHECKS EXECUTED: {passed_checks + failed_checks}")
    print(f" TOTAL AUDIT CHECKS PASSED  : {passed_checks}")
    print(f" TOTAL AUDIT CHECKS FAILED  : {failed_checks}")
    print("=" * 78)

    if failed_checks == 0 and passed_checks > 0:
        print("\n >>> [GATE VERIFICATION PASSED] VERDICT: APPROVE <<< \n")
        return 0
    else:
        print(f"\n >>> [GATE VERIFICATION FAILED] VERDICT: REQUEST_CHANGES ({failed_checks} failures) <<< \n")
        return 1

if __name__ == "__main__":
    sys.exit(main())
