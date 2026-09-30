import os
import sys
import subprocess
import argparse

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
SDK_DIR = r"C:/ti/simplelink_cc13xx_cc26xx_sdk_8_33_00_16"
COMPILER_DIR = r"C:/ti/ccs2101/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS"
SYSCONFIG_BAT = r"C:/ti/sysconfig_1.21.1/sysconfig_cli.bat"
SRFPROG = r"C:/Program Files (x86)/Texas Instruments/SmartRF Tools/Flash Programmer 2/bin/srfprog.exe"

CC = os.path.join(COMPILER_DIR, "bin", "tiarmclang.exe")
OBJCOPY = os.path.join(COMPILER_DIR, "bin", "tiarmobjcopy.exe")

def run(cmd, cwd=PROJECT_DIR, check=True):
    print(f"\n[EXEC] {cmd if isinstance(cmd, str) else ' '.join(cmd)}")
    res = subprocess.run(cmd, cwd=cwd, shell=isinstance(cmd, str), capture_output=True, text=True)
    if res.stdout:
        print(res.stdout)
    if res.stderr:
        print("[STDERR]", res.stderr)
    if check and res.returncode != 0:
        print(f"FAILED with exit code {res.returncode}")
        sys.exit(res.returncode)
    return res

def main():
    parser = argparse.ArgumentParser(description="Build and Flash Dedicated ECG Firmware")
    parser.add_argument("--flash", action="store_true", help="Flash the target board after building")
    args = parser.parse_args()

    print("=" * 65)
    print("  SmartBAN ECG Dedicated — Build Tool")
    print("=" * 65)

    print("\n=== Step 1: Running SysConfig ===")
    syscfg_out = os.path.join(PROJECT_DIR, "syscfg")
    os.makedirs(syscfg_out, exist_ok=True)
    syscfg_cmd = [
        SYSCONFIG_BAT,
        "--compiler", "ticlang",
        "--product", f"{SDK_DIR}/.metadata/product.json",
        "--output", syscfg_out,
        os.path.join(PROJECT_DIR, "diagnostic.syscfg")
    ]
    run(syscfg_cmd)

    print("\n=== Step 2: Compiling Source Files ===")
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
        f"-I{SDK_DIR}/kernel/nortos",
        f"-I{SDK_DIR}/kernel/nortos/posix",
        f"@{syscfg_out}/ti_utils_build_compiler.opt"
    ]

    srcs = [
        os.path.join(syscfg_out, "ti_devices_config.c"),
        os.path.join(syscfg_out, "ti_drivers_config.c"),
        os.path.join(PROJECT_DIR, "edgeai_ecg.c"),
        os.path.join(PROJECT_DIR, "main.c")
    ]

    objs = []
    for src in srcs:
        obj = os.path.join(PROJECT_DIR, os.path.splitext(os.path.basename(src))[0] + ".obj")
        compile_cmd = [CC] + cflags + ["-c", src, "-o", obj]
        run(compile_cmd)
        objs.append(obj)

    print("\n=== Step 3: Linking ecg_dedicated.out ===")
    out_file = os.path.join(PROJECT_DIR, "ecg_dedicated.out")
    map_file = os.path.join(PROJECT_DIR, "ecg_dedicated.map")
    cmd_file = os.path.join(PROJECT_DIR, "cc13x2_cc26x2_nortos.cmd")
    genlibs = os.path.join(syscfg_out, "ti_utils_build_linker.cmd.genlibs")

    lflags = [
        "-Wl,-u,_c_int00",
        f"-Wl,-m,{map_file}",
        "-Wl,--rom_model",
        "-Wl,--warn_sections",
        f"-L{SDK_DIR}/source",
        f"-L{SDK_DIR}/kernel/nortos",
        genlibs,
        cmd_file,
        f"-L{COMPILER_DIR}/lib",
        "-llibc.a",
        "-o", out_file
    ]

    link_cmd = [CC] + objs + lflags
    run(link_cmd)

    print("\n=== Step 4: Generating Intel HEX ===")
    raw_hex = os.path.join(PROJECT_DIR, "ecg_dedicated_raw.hex")
    clean_hex = os.path.join(PROJECT_DIR, "ecg_dedicated.hex")
    run([OBJCOPY, "-O", "ihex", out_file, raw_hex])

    with open(raw_hex, 'r') as f:
        lines = f.readlines()
    clean_lines = [l for l in lines if l[7:9] != '03']
    with open(clean_hex, 'w') as f:
        f.writelines(clean_lines)

    hex_size = os.path.getsize(clean_hex)
    print(f"Generated clean HEX: {clean_hex} ({hex_size:,} bytes)")

    if args.flash:
        print("\n=== Step 5: Flashing Target via SmartRF Flash Programmer 2 ===")
        flash_cmd = [
            SRFPROG,
            "-t", "lsidx(0)",
            "-e", "all",
            "-p",
            "-v",
            "-f", clean_hex
        ]
        run(flash_cmd)
        print("\nFlashing Complete!")
    else:
        print("\n" + "=" * 65)
        print("  BUILD COMPLETE! (Firmware ready, not flashed)")
        print(f"  Firmware Binary : {clean_hex}")
        print("  To flash whenever you are ready, run:")
        print("    python build_and_flash.py --flash")
        print("=" * 65)

if __name__ == '__main__':
    main()
