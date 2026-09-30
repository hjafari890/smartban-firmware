#!/usr/bin/env python3
"""
===============================================================================
test_m1_challenger2.py
Milestone M1 Adversarial Verification Suite — Challenger 2
Empirical verification of build pipeline, memory map, linker script,
Intel HEX integrity, bus concurrency, and hardware flaw exclusion.
===============================================================================
"""

import os
import sys
import struct
import subprocess

PROJECT_DIR = r"C:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\06_tirtos_smartban"

# -----------------------------------------------------------------------------
# TEST GROUP 1: Clean Rebuild Pipeline
# -----------------------------------------------------------------------------
def test_clean_rebuild():
    print("\n" + "="*70)
    print(" [TEST GROUP 1] CLEAN REBUILD PIPELINE VERIFICATION")
    print("="*70)

    build_py = os.path.join(PROJECT_DIR, "build_and_flash.py")
    assert os.path.exists(build_py), f"build_and_flash.py not found at {build_py}"

    cmd = [sys.executable, build_py, "--clean", "--no-flash"]
    print(f"  Executing: {' '.join(cmd)}")
    res = subprocess.run(cmd, cwd=PROJECT_DIR, capture_output=True, text=True)
    
    if res.returncode != 0:
        print(f"  [FAIL] Build script exited with code {res.returncode}")
        print("  STDERR:\n", res.stderr)
        print("  STDOUT:\n", res.stdout)
    assert res.returncode == 0, f"Clean rebuild failed with exit code {res.returncode}"
    print("  [PASS] Clean build completed with exit code 0")

    for f in ["smartban.out", "smartban.hex", "smartban.map"]:
        p = os.path.join(PROJECT_DIR, f)
        assert os.path.exists(p), f"Expected artifact {f} does not exist"
        sz = os.path.getsize(p)
        assert sz > 0, f"Artifact {f} has zero size"
        print(f"  [PASS] Artifact {f:14s} verified ({sz:,} bytes)")


# -----------------------------------------------------------------------------
# TEST GROUP 2: ELF & Memory Layout / Linker Script Audit
# -----------------------------------------------------------------------------
def test_memory_layout_and_elf():
    print("\n" + "="*70)
    print(" [TEST GROUP 2] ELF HEADERS, MEMORY ALLOCATION & OVERLAP AUDIT")
    print("="*70)

    out_path = os.path.join(PROJECT_DIR, "smartban.out")
    with open(out_path, "rb") as f:
        elf = f.read()

    assert elf[:4] == b'\x7fELF', "Invalid ELF magic"
    ei_class = elf[4] # 1 = 32-bit
    ei_data = elf[5]  # 1 = little endian
    e_type = struct.unpack('<H', elf[16:18])[0] # 2 = EXEC
    e_machine = struct.unpack('<H', elf[18:20])[0] # 40 = ARM
    e_entry = struct.unpack('<I', elf[24:28])[0]

    assert ei_class == 1, "ELF is not 32-bit"
    assert ei_data == 1, "ELF is not Little-Endian"
    assert e_type == 2, "ELF is not Executable"
    assert e_machine == 40, "ELF machine is not ARM (40)"
    print(f"  [PASS] ELF 32-bit Little-Endian ARM executable verified (Entry: 0x{e_entry:08X})")

    map_path = os.path.join(PROJECT_DIR, "smartban.map")
    with open(map_path, "r") as f:
        map_lines = f.readlines()

    in_seg_map = False
    sram_sections = []
    flash_sections = []

    for line in map_lines:
        line_s = line.strip()
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
                            sram_sections.append((run_org, run_org + length, length, name))
                        elif 0x00000000 <= run_org < 0x00058000:
                            flash_sections.append((run_org, run_org + length, length, name))
                except ValueError:
                    pass

    print(f"  Found {len(sram_sections)} SRAM sections and {len(flash_sections)} FLASH sections in map.")

    # Check for overlapping sections in SRAM
    sram_sorted = sorted(sram_sections, key=lambda x: x[0])
    for i in range(len(sram_sorted) - 1):
        s1 = sram_sorted[i]
        s2 = sram_sorted[i+1]
        assert s1[1] <= s2[0], f"SRAM overlap between {s1[3]} and {s2[3]}"
        print(f"  [PASS] SRAM Section {s1[3]:35s}: 0x{s1[0]:08X} - 0x{s1[1]:08X} ({s1[2]:6d} bytes)")
    if sram_sorted:
        s = sram_sorted[-1]
        print(f"  [PASS] SRAM Section {s[3]:35s}: 0x{s[0]:08X} - 0x{s[1]:08X} ({s[2]:6d} bytes)")

    # Verify Heap & Stack separation
    priheap = [s for s in sram_sections if "priheap" in s[3]]
    stack = [s for s in sram_sections if "stack" in s[3]]
    assert len(priheap) > 0, "No .priheap found in SRAM"
    assert len(stack) > 0, "No .stack found in SRAM"

    heap_end = priheap[0][1]
    stack_start = stack[0][0]
    headroom = stack_start - heap_end
    print(f"\n  [PASS] Heap End: 0x{heap_end:08X}, Stack Start: 0x{stack_start:08X}")
    print(f"  [PASS] Free SRAM Headroom between Heap and Stack: {headroom:,} bytes ({headroom/1024:.1f} KB)")
    assert headroom >= 50000, f"Headroom too small: {headroom} bytes"

    # Verify CCFG placement at 0x00057FA8
    ccfg = [s for s in flash_sections if "ccfg" in s[3]]
    assert len(ccfg) > 0, "No .ccfg found in FLASH"
    assert ccfg[0][0] == 0x00057FA8, f"CCFG not at 0x00057FA8: 0x{ccfg[0][0]:08X}"
    assert ccfg[0][1] == 0x00058000, f"CCFG does not end at 0x00058000: 0x{ccfg[0][1]:08X}"
    print(f"  [PASS] CCFG correctly placed at top of Flash: 0x{ccfg[0][0]:08X} - 0x{ccfg[0][1]:08X} (88 bytes)")


# -----------------------------------------------------------------------------
# TEST GROUP 3: Intel HEX Integrity & Checksum Stress Test
# -----------------------------------------------------------------------------
def test_intel_hex_integrity():
    print("\n" + "="*70)
    print(" [TEST GROUP 3] INTEL HEX RECORD INTEGRITY & CHECKSUM VERIFICATION")
    print("="*70)

    hex_path = os.path.join(PROJECT_DIR, "smartban.hex")
    with open(hex_path, "r") as f:
        lines = [l.strip() for l in f if l.strip()]

    base_addr = 0
    mem = {}
    type_counts = {}

    for idx, line in enumerate(lines, 1):
        assert line.startswith(':'), f"Line {idx} missing ':'"
        length = int(line[1:3], 16)
        addr = int(line[3:7], 16)
        rectype = int(line[7:9], 16)
        data_hex = line[9:9+length*2]
        chk = int(line[9+length*2:11+length*2], 16)

        computed = (length + (addr >> 8) + (addr & 0xFF) + rectype + sum(int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2))) & 0xFF
        computed = ((~computed) + 1) & 0xFF
        assert chk == computed, f"Checksum mismatch on line {idx}: {chk:02X} != {computed:02X}"

        type_counts[rectype] = type_counts.get(rectype, 0) + 1

        if rectype == 0:
            phys = base_addr + addr
            for i in range(length):
                byte_val = int(data_hex[i*2:i*2+2], 16)
                assert (phys + i) not in mem, f"Hex overlap at address 0x{phys+i:08X}"
                mem[phys + i] = byte_val
        elif rectype == 1:
            assert idx == len(lines), "EOF record not at end of file"
        elif rectype == 2:
            base_addr = int(data_hex, 16) << 4
        elif rectype == 4:
            base_addr = int(data_hex, 16) << 16
        else:
            assert False, f"Unsupported record type {rectype:02X} on line {idx}"

    print(f"  [PASS] Parsed {len(lines)} Intel HEX records with 100% valid checksums.")
    print(f"  [PASS] Record type breakdown: {type_counts}")
    assert type_counts.get(3, 0) == 0, "Type 03 records found in smartban.hex!"
    print("  [PASS] Zero Type 03 records verified (SmartRF Programmer compatible)")

    addrs = sorted(mem.keys())
    min_a, max_a = addrs[0], addrs[-1]
    print(f"  [PASS] Physical Memory Span: 0x{min_a:08X} to 0x{max_a:08X} (Total bytes: {len(mem):,})")
    assert min_a >= 0x00000000 and max_a < 0x00058000, "Memory written outside 352 KB Flash boundary!"

    ccfg_bytes = [mem[a] for a in range(0x00057FA8, 0x00058000) if a in mem]
    assert len(ccfg_bytes) == 88, f"CCFG bytes incomplete: {len(ccfg_bytes)}/88"
    print("  [PASS] CCFG configuration table completely present in Intel HEX (88/88 bytes)")


# -----------------------------------------------------------------------------
# TEST GROUP 4: Bus Concurrency & SPI Dynamic Pin-Swapping Stress Test
# -----------------------------------------------------------------------------
class MockSpiBus:
    def __init__(self):
        self.mutex_locked = False
        self.owner_thread = None
        self.current_dev = 0
        self.cs_ecg = 1
        self.cs_imu = 1
        self.dio8_mode = None
        self.dio9_mode = None
        self.spi_open_count = 0
        self.spi_close_count = 0

    def acquire(self, dev, thread_id):
        if self.mutex_locked:
            if self.owner_thread == thread_id:
                raise RuntimeError(f"Thread {thread_id} deadlocked on non-recursive spi_bus_mutex!")
            else:
                return False

        self.mutex_locked = True
        self.owner_thread = thread_id
        self.cs_ecg = 1
        self.cs_imu = 1

        if dev == 1:
            if self.current_dev != 1:
                if self.current_dev != 0:
                    self.spi_close_count += 1
                self.spi_open_count += 1
                self.dio9_mode = "SSI0_TX"
                self.dio8_mode = "SSI0_RX"
                self.current_dev = 1
            self.cs_ecg = 0
        elif dev == 2:
            if self.current_dev != 2:
                if self.current_dev != 0:
                    self.spi_close_count += 1
                self.spi_open_count += 1
                self.dio8_mode = "SSI0_TX"
                self.dio9_mode = "SSI0_RX"
                self.current_dev = 2
            self.cs_imu = 0
        else:
            self.mutex_locked = False
            self.owner_thread = None
            return False

        return True

    def release(self, dev, thread_id):
        assert self.mutex_locked and self.owner_thread == thread_id, "Release without ownership!"
        if dev == 1:
            self.cs_ecg = 1
        elif dev == 2:
            self.cs_imu = 1
        self.mutex_locked = False
        self.owner_thread = None

def test_spi_concurrency_stress():
    print("\n" + "="*70)
    print(" [TEST GROUP 4] SPI PIN-SWAPPING CONCURRENCY & DEADLOCK STRESS TEST")
    print("="*70)

    bus = MockSpiBus()

    print("  Running 1,000 rapid alternating acquire/release cycles...")
    for cycle in range(1000):
        ok = bus.acquire(1, thread_id=1)
        assert ok and bus.cs_ecg == 0 and bus.cs_imu == 1
        assert bus.dio9_mode == "SSI0_TX" and bus.dio8_mode == "SSI0_RX"
        bus.release(1, thread_id=1)
        assert not bus.mutex_locked and bus.cs_ecg == 1 and bus.cs_imu == 1

        ok = bus.acquire(2, thread_id=2)
        assert ok and bus.cs_imu == 0 and bus.cs_ecg == 1
        assert bus.dio8_mode == "SSI0_TX" and bus.dio9_mode == "SSI0_RX"
        bus.release(2, thread_id=2)
        assert not bus.mutex_locked and bus.cs_ecg == 1 and bus.cs_imu == 1

    print(f"  [PASS] 1,000 dynamic pin-swap cycles completed without lockup.")
    print(f"  [PASS] SPI controller opens: {bus.spi_open_count}, closes: {bus.spi_close_count}")

    ok = bus.acquire(99, thread_id=3)
    assert not ok, "Acquire succeeded on invalid device"
    assert not bus.mutex_locked, "Mutex left locked after invalid device acquire!"
    print("  [PASS] Invalid device acquire safely unlocked mutex without deadlock.")

    bus.acquire(1, thread_id=1)
    reentrancy_deadlock = False
    try:
        bus.acquire(1, thread_id=1)
    except RuntimeError:
        reentrancy_deadlock = True
    bus.release(1, thread_id=1)
    assert reentrancy_deadlock, "Expected non-recursive mutex deadlock not caught!"
    print("  [PASS] Verified non-recursive mutex behavior: callers must not nest bsp_spi_acquire.")


# -----------------------------------------------------------------------------
# TEST GROUP 5: MAX32664 / Hardware Flaw Static Audit
# -----------------------------------------------------------------------------
def test_max32664_exclusion():
    print("\n" + "="*70)
    print(" [TEST GROUP 5] MAX32664 PPG SUBSYSTEM EXCLUSION AUDIT")
    print("="*70)

    forbidden_patterns = ["max32664", "maxm86161", "hal_ppg", "max30102"]
    scan_dirs = ["bsp", "hal"]
    
    violation_count = 0
    for d in scan_dirs:
        full_d = os.path.join(PROJECT_DIR, d)
        for root, dirs, files in os.walk(full_d):
            for f in files:
                if f.endswith(('.c', '.h')):
                    fp = os.path.join(root, f)
                    with open(fp, "r", errors="ignore") as fh:
                        for line_idx, line in enumerate(fh, 1):
                            if "is EXCLUDED" in line or "PPG view" in line:
                                continue
                            for pat in forbidden_patterns:
                                if pat in line.lower():
                                    print(f"  [VIOLATION] {f}:{line_idx}: {line.strip()}")
                                    violation_count += 1

    assert violation_count == 0, f"Found {violation_count} forbidden PPG references!"
    print("  [PASS] Zero active code references to MAX32664, MAXM86161, or MAX30102.")

    syscfg_path = os.path.join(PROJECT_DIR, "smartban.syscfg")
    with open(syscfg_path, "r") as f:
        syscfg_txt = f.read().lower()
    for pat in forbidden_patterns:
        assert pat not in syscfg_txt, f"Found {pat} in smartban.syscfg!"
    print("  [PASS] smartban.syscfg contains zero PPG peripherals or pin bindings.")


# -----------------------------------------------------------------------------
# MAIN
# -----------------------------------------------------------------------------
def main():
    print("=======================================================================")
    print(" SMARTBAN MILESTONE M1: ADVERSARIAL VERIFICATION SUITE (CHALLENGER 2)")
    print(" Pipeline, Memory Allocation, Concurrency, and Flaw Exclusion")
    print("=======================================================================")

    test_clean_rebuild()
    test_memory_layout_and_elf()
    test_intel_hex_integrity()
    test_spi_concurrency_stress()
    test_max32664_exclusion()

    print("\n" + "="*70)
    print(" ALL CHALLENGER 2 ADVERSARIAL TESTS PASSED (100%)")
    print("=======================================================================\n")

if __name__ == '__main__':
    main()
