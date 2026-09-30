# ECG-only bring-up: CC2652R1 + ADS1292R

Two halves:

- **`firmware/`**, bare-metal CC2652R1 code. Reads channel 1 of the
  ADS1292R at 500 SPS via a DRDY interrupt and streams samples to the PC
  over UART. See `firmware/README.md` for how to fold it into your
  existing CCS project and the bring-up order to follow.
- **`gui/`**, Python app. Decodes the UART stream, filters it (baseline
  wander + mains notch + high-frequency noise), finds R-peaks with a
  simplified Pan-Tompkins detector, and shows a live scrolling trace plus
  a heart-rate readout.

## Try the GUI before touching hardware

The GUI works against a synthetic ECG-like waveform with no board
attached, so you can sanity-check the whole PC-side pipeline (filtering,
peak detection, plotting) independently of firmware bring-up:

```bash
cd gui
python -m venv .venv
source .venv/bin/activate        # .venv\Scripts\activate on Windows
pip install -r requirements.txt
python main.py --simulate
```

You should see a scrolling waveform, red dots on detected R-peaks, and an
HR readout that settles near 70 bpm within a few seconds (the simulator's
target rate, this is a synthetic test signal, not a real measurement).

## Running against real hardware

1. Flash the firmware (see `firmware/README.md`), power the board, and
   note which serial port it enumerates as.
2. `python main.py --list-ports` if you're not sure.
3. `python main.py --port <your-port>` (add `--baud` if you changed
   `UART_BAUD` in `firmware/uart_stream.c`, they must match).

If the firmware's self-test fails, the GUI's status line will say so
explicitly instead of silently showing a flat or garbage trace, that
means go back to `firmware/README.md`'s bring-up order and check SPI/CS/
reset wiring before looking at the DSP side at all.

## Protocol reference

8-byte binary frame per sample, defined once and shared by both sides, see `firmware/uart_stream.h` (source of truth) and
`gui/serial_reader.py` (decoder). If you ever change one, change the
other.

## Where to go from here

- **Signal too noisy even with RLD enabled:** check electrode contact/gel
  first, it's the most common cause, more than firmware or filter
  settings. Confirm the ECG+Respiration block's RA/LA/RLD wiring against
  the schematic.
- **HR readout jumps around:** the peak-detection threshold (`0.4 * peak`
  in `gui/dsp.py`) and refractory window (`0.3 s`) are reasonable
  defaults, not tuned to your specific electrode setup, adjust if a
  particular subject's R-wave amplitude is unusually low relative to
  noise.
- **Want faster iteration on the DSP side:** everything in `gui/dsp.py`
  runs fine offline against a recorded `.csv` of raw samples too, if you
  want to log a session and tune filters without re-running hardware each
  time, not wired up here, but `ECGProcessor.process()` takes a plain
  numpy array so it's a small addition.
