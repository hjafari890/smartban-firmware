# Firmware: ECG-only bring-up (CC2652R1, bare metal)

## What this is

A minimal, no-RTOS firmware that talks to the ADS1292R over SPI, reads one
ECG channel at 500 SPS via a DRDY interrupt, and streams samples to the PC
over UART as fixed 8-byte binary frames. Everything else on the shield
(IMU, PPG, BME690, etc.) is left untouched — the only other thing this
firmware drives is the IMU chip-select line, and only to hold it high so it
doesn't interfere with the shared SPI bus.

## Files

| File | Purpose |
|---|---|
| `pins.h` | DIO/IOID definitions for every pin this firmware touches |
| `ads1292.h/.c` | ADS1292R driver: reset, register config, DRDY-driven sample read |
| `uart_stream.h/.c` | 8-byte framed UART protocol to the PC |
| `main.c` | Superloop tying it together |

## Integrating into your existing CCS project

You already have a SysConfig-based CCS project (`main.c`, `ti_drivers_config.*`,
`*.syscfg`) targeting the CC2652R1. Two ways to bring this in, pick one:

**Option A — keep it truly bare metal (what this code assumes):**
1. Copy `pins.h`, `ads1292.h`, `ads1292.c`, `uart_stream.h`, `uart_stream.c`
   into your project's source folder.
2. Replace your existing `main.c` with this one (or merge — the important
   part is that nothing else calls `UART_Init()`/configures UART0, and
   nothing else configures SSI0, before this code runs).
3. In SysConfig, you can leave GPIO/SPI/UART modules out entirely for the
   pins listed in `pins.h` — this code configures them directly via
   DriverLib, not through TI Drivers. If SysConfig has already claimed
   `DIO_8/9/10/11/15/21/22/23/24/2/3` for something else in your `.syscfg`
   file, free them there first or you'll get pin-conflict build errors.
4. Build. If you get undeclared-function errors, it's almost certainly a
   DriverLib function that was renamed between SDK versions — the fix is a
   one-line lookup in the matching header under
   `<SDK>/source/ti/devices/cc13x2_cc26x2/driverlib/`.

**Option B — keep using TI Drivers (SPI.h/GPIO.h/UART2.h) instead:**
Possible, but means rewriting `ads1292.c`/`uart_stream.c` against the TI
Drivers API and adding the 7+2 pins to your `.syscfg` file as GPIO/SPI/UART
resources with SysConfig-generated `CONFIG_*` names. Only worth it if you
plan to add RTOS-based work (BLE stack, multiple peripherals with drivers
fighting over priority) on top later. For a single-purpose ECG bring-up,
option A is less code and easier to debug.

## Bring-up order (do this, don't skip steps)

1. **Flash and check `ADS1292_SelfTest()` first**, before wiring up
   electrodes. If it fails, the firmware loops sending diagnostic frames
   with `seq = 0xFFFF` and the raw ID byte as the payload — you'll see this
   immediately in the GUI (see `gui/README` usage notes) or on a raw serial
   terminal. A failure here means SPI/CS/reset wiring, not signal quality —
   don't move on to filtering/HR code until this passes.
2. Once self-test passes, connect ECG electrodes (RA/LA/RLD per the
   schematic's ECG+Respiration block) and confirm you're getting a
   plausible, changing waveform — even noisy — over UART.
3. Only then move to the PC-side filtering/HR pipeline in `gui/`.

## Known unknowns / things to verify yourself

- `PIN_UART_TX`/`PIN_UART_RX` in `pins.h` (`IOID_3`/`IOID_2`) are the
  standard XDS110 backchannel pins on CC13xx/CC26xx LaunchPads, but were
  **not** independently re-verified against your shield's netlist the way
  the 7 ADS1292R pins were. Confirm against your board before assuming it.
- SPI clock is set conservatively to 1 MHz in `ads1292.c`. The ADS1292R
  supports faster; only push it up once you have clean reads at 1 MHz.
- Register values in `ADS1292_Init()` are commonly-used reference-design
  defaults, not derived bit-by-bit from the datasheet in this pass — fine
  to get a working signal, but cross-check against SBAS502 before you
  start tuning gain/lead-off detection/data rate.
