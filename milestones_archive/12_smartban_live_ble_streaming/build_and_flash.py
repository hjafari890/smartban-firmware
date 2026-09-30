#!/usr/bin/env python3
"""
===============================================================================
build_and_flash.py
One-Click Build, Hex Generation, and Flashing Pipeline for SmartBAN TI-RTOS7
Platform: CC2652R1 LaunchPad (CC26X2R1_LAUNCHXL) + SmartBAN Shield Rev 3.5
Toolchain: tiarmclang 5.1.1.LTS, SysConfig 1.21.1, SimpleLink SDK 8.33
===============================================================================
"""

import os
import sys
import argparse
import subprocess
import time

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = r"C:/ti/simplelink_cc13xx_cc26xx_sdk_8_33_00_16"
COMPILER_DIR = r"C:/ti/ccs2101/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS"
SYSCONFIG_BAT = r"C:/ti/sysconfig_1.21.1/sysconfig_cli.bat"
SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"

CC = os.path.join(COMPILER_DIR, "bin", "tiarmclang.exe")
OBJCOPY = os.path.join(COMPILER_DIR, "bin", "tiarmobjcopy.exe")

def run(cmd, cwd=PROJECT_DIR, check=True):
    cmd_str = cmd if isinstance(cmd, str) else ' '.join(f'"{c}"' if ' ' in c else c for c in cmd)
    print(f"\n[EXEC] {cmd_str}")
    res = subprocess.run(cmd, cwd=cwd, shell=isinstance(cmd, str), capture_output=True, text=True)
    if res.stdout:
        print(res.stdout.strip())
    if res.stderr:
        print("[STDERR]", res.stderr.strip())
    if check and res.returncode != 0:
        print(f"\n[ERROR] Command failed with exit code {res.returncode}")
        sys.exit(res.returncode)
    return res

def main():
    parser = argparse.ArgumentParser(description="SmartBAN All-in-One TI-RTOS7 Build and Flash Pipeline")
    parser.add_argument("--no-flash", action="store_true", help="Compile and link only, do not invoke srfprog")
    parser.add_argument("--build-only", action="store_true", help="Alias for --no-flash")
    parser.add_argument("--clean", action="store_true", help="Clean build artifacts before compiling")
    args = parser.parse_args()

    flash_target = not (args.no_flash or args.build_only)

    print("=====================================================================")
    print(" SmartBAN All-in-One TI-RTOS7 Firmware Build System")
    print(f" Target Directory : {PROJECT_DIR}")
    print(f" Toolchain        : tiarmclang 5.1.1.LTS")
    print(f" Kernel Target    : TI-RTOS7 (--rtos tirtos7)")
    print(f" Flashing Mode    : {'ENABLED' if flash_target else 'DISABLED (Build Only)'}")
    print("=====================================================================")

    syscfg_out = os.path.join(PROJECT_DIR, "syscfg")
    obj_dir = os.path.join(PROJECT_DIR, "obj")
    os.makedirs(syscfg_out, exist_ok=True)
    os.makedirs(obj_dir, exist_ok=True)

    if args.clean:
        print("[CLEAN] Removing previous build artifacts...")
        for root, dirs, files in os.walk(obj_dir):
            for f in files:
                os.remove(os.path.join(root, f))
        for f in ["all_in_one.out", "all_in_one.map", "all_in_one_raw.hex", "all_in_one.hex"]:
            p = os.path.join(PROJECT_DIR, f)
            if os.path.exists(p):
                os.remove(p)

    # -------------------------------------------------------------------------
    # Step 1: SysConfig Generation (--rtos tirtos7)
    # -------------------------------------------------------------------------
    print("\n--- [Step 1/5] Running SysConfig 1.21.1 CLI ---")
    syscfg_file = os.path.join(PROJECT_DIR, "all_in_one.syscfg")
    syscfg_cmd = [
        SYSCONFIG_BAT,
        "--compiler", "ticlang",
        "--product", f"{SDK_DIR}/.metadata/product.json",
        "--output", syscfg_out,
        syscfg_file
    ]
    run(syscfg_cmd)

    # -------------------------------------------------------------------------
    # Step 2: Compile C Sources
    # -------------------------------------------------------------------------
    print("\n--- [Step 2/5] Compiling Sources with tiarmclang ---")
    cflags = [
        "-mcpu=cortex-m4",
        "-march=armv7e-m",
        "-mthumb",
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16",
        "-Oz",
        "-gdwarf-3",
        f"-I{PROJECT_DIR}",
        f"-I{syscfg_out}",
        f"-I{SDK_DIR}/source",
        f"-I{SDK_DIR}/kernel/tirtos7/packages",
        f"-I{SDK_DIR}/source/ti/posix/ticlang",
        f"@{syscfg_out}/ti_utils_build_compiler.opt"
    ]

    sources = [
        # SysConfig generated sources
        os.path.join(syscfg_out, "ti_devices_config.c"),
        os.path.join(syscfg_out, "ti_drivers_config.c"),
        os.path.join(syscfg_out, "ti_sysbios_config.c"),
        # BSP sources
        os.path.join(PROJECT_DIR, "bsp", "bsp_power.c"),
        os.path.join(PROJECT_DIR, "bsp", "bsp_spi.c"),
        os.path.join(PROJECT_DIR, "bsp", "bsp_i2c.c"),
        # HAL sources
        os.path.join(PROJECT_DIR, "hal", "hal_ecg.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_imu.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_bme680.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_optical.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_fir.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_ui.c"),
        os.path.join(PROJECT_DIR, "hal", "hal_ble_radio.c"),
        # IPC sources
        os.path.join(PROJECT_DIR, "ipc", "ring_buffer.c"),
        # Edge-AI sources
        os.path.join(PROJECT_DIR, "edgeai", "edgeai_ecg.c"),
        os.path.join(PROJECT_DIR, "edgeai", "edgeai_imu.c"),
        os.path.join(PROJECT_DIR, "edgeai", "edgeai_fusion.c"),
        os.path.join(PROJECT_DIR, "edgeai", "edgeai_tinyml.c"),
        # Telemetry & SmartBAN MAC sources
        os.path.join(PROJECT_DIR, "telemetry", "smartban_mac.c"),
        # Main entry
        os.path.join(PROJECT_DIR, "main.c"),
    ]

    objects = []
    for src in sources:
        rel_name = os.path.splitext(os.path.basename(src))[0] + ".obj"
        obj = os.path.join(obj_dir, rel_name)
        compile_cmd = [CC] + cflags + ["-c", src, "-o", obj]
        run(compile_cmd)
        objects.append(obj)
        print(f"  [CC] {os.path.relpath(src, PROJECT_DIR)} -> {rel_name}")

    # -------------------------------------------------------------------------
    # Step 3: Link Firmware (.out)
    # -------------------------------------------------------------------------
    print("\n--- [Step 3/5] Linking all_in_one.out with TI-RTOS7 Kernel ---")
    out_file = os.path.join(PROJECT_DIR, "all_in_one.out")
    map_file = os.path.join(PROJECT_DIR, "all_in_one.map")
    cmd_file = os.path.join(PROJECT_DIR, "CC26X2R1_LAUNCHXL_TIRTOS7.cmd")
    genlibs = os.path.join(syscfg_out, "ti_utils_build_linker.cmd.genlibs")

    lflags = [
        "-Wl,-u,_c_int00",
        f"-Wl,-m,{map_file}",
        "-Wl,--rom_model",
        "-Wl,--warn_sections",
        f"-L{SDK_DIR}/source",
        f"-L{SDK_DIR}/kernel/tirtos7/packages",
        genlibs,
        cmd_file,
        f"-L{COMPILER_DIR}/lib",
        "-llibc.a",
        "-o", out_file
    ]

    link_cmd = [CC] + objects + lflags
    run(link_cmd)
    out_size = os.path.getsize(out_file)
    print(f"  [LINK] Linked successfully: {out_file} ({out_size:,} bytes)")

    # -------------------------------------------------------------------------
    # Step 4: Generate & Clean Intel HEX
    # -------------------------------------------------------------------------
    print("\n--- [Step 4/5] Generating & Sanitizing Intel HEX for SmartRF ---")
    raw_hex = os.path.join(PROJECT_DIR, "all_in_one_raw.hex")
    clean_hex = os.path.join(PROJECT_DIR, "all_in_one.hex")

    run([OBJCOPY, "-O", "ihex", out_file, raw_hex])

    # Strip Type 03 record (Start Segment Address) to satisfy srfprog parser
    with open(raw_hex, 'r') as f:
        lines = f.readlines()
    clean_lines = [l for l in lines if l[7:9] != '03']
    with open(clean_hex, 'w') as f:
        f.writelines(clean_lines)

    hex_size = os.path.getsize(clean_hex)
    print(f"  [HEX] Generated clean hex: {clean_hex} ({hex_size:,} bytes)")

    # -------------------------------------------------------------------------
    # Step 5: Flashing (Optional / Conditional)
    # -------------------------------------------------------------------------
    if flash_target:
        print("\n--- [Step 5/5] Flashing Target via SmartRF Flash Programmer 2 ---")
        if not os.path.exists(SRFPROG):
            print(f"[WARN] SmartRF Flash Programmer 2 executable not found at {SRFPROG}")
            print("       Skipping flashing step.")
        else:
            flash_cmd = [
                SRFPROG,
                "-t", "soc(XDS-L1100GRL, CC2652R)",
                "-e", "all",
                "-p",
                "-v",
                "-f", clean_hex
            ]
            run(flash_cmd)
            print("  [FLASH] Flashing & Verification Complete!")
    else:
        print("\n--- [Step 5/5] Flashing Skipped (--no-flash / --build-only selected) ---")

    print("\n=====================================================================")
    print(" BUILD SUCCESSFUL")
    print(f" Firmware Image: {out_file}")
    print(f" Intel HEX     : {clean_hex}")
    print("=====================================================================")

if __name__ == '__main__':
    main()
