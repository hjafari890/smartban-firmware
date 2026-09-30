# TI-RTOS Integration Handover Guide (From Verified `08_ecg_dedicated` Checkpoint)

**Checkpoint Directory**: `c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\08_ecg_dedicated\checkpoint_v1_clinical_verified\`  
**Full Engineering Diary**: [`MASTER_THESIS_ECG_DIARY.md`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/MASTER_THESIS_ECG_DIARY.md)

This document summarizes the exact hardware register settings, MCU DSP algorithms (`main.c`, `edgeai_ecg.c`), CH455H 7-segment display driver, and host GUI (`ecg_gui.py`) features verified in `08_ecg_dedicated` so they can be ported directly into `09_tirtos_all_in_one`.

---

## 1. Critical ADS1292R Register Fixes (`ads_init()` & `ads_set_mode()`)
Located in [`main.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/main.c#L502-L610):
1. **Disable 32 kHz Respiration Carrier in Live ECG Mode (`REG_RESP1 = 0x00`)**:
   * Setting `REG_RESP1 = 0xEA` (32 kHz excitation) injected 250 mVpp carrier modulation onto `IN2P`/`IN2N`, swamping the 6 nA DC lead-off comparators (`0x06`) and adding demodulation noise to `CH2`. Keep `REG_RESP1 = 0x00` in Live ECG mode.
2. **Keep Internal RLD Reference (`RLDREF_INT`) Active After Offset Calibration (`REG_RESP2 = 0x07`)**:
   * During `OFFSETCAL`, write `REG_RESP2 = 0x87` (`CALIB_ON=1, RLDREF_INT=1`), wait 500 ms, and restore **`REG_RESP2 = 0x07`** (`CALIB_ON=0, RLDREF_INT=1`). Writing `0x03` disables the internal $(AVDD+AVSS)/2 = 1.21\text{ V}$ RLD reference and causes Common-Mode / Right-Leg-Drive saturation.
3. **Optimal Live ECG Register Map (Mode 4)**:
   * `CONFIG1 (0x01)` = `0x01` (250 SPS continuous conversion)
   * `CONFIG2 (0x02)` = `0xE0` (Internal 2.42V reference + Lead-off comparators enabled)
   * `LOFF (0x03)` = `0x10` (95%/5% comparator threshold, 6 nA DC lead-off current)
   * `CH1SET (0x04)` = `0x10` (Normal electrode input, PGA Gain = 1)
   * `CH2SET (0x05)` = `0x00` (Normal electrode input, PGA Gain = 6)
   * `RLD_SENS (0x06)` = `0x2C` (RLD buffer ON, closed-loop feedback from `RLDIN/RLD_SENSE` + `IN2P` + `IN2N`)
   * `LOFF_SENS (0x07)` = `0x0C` (DC lead-off sensing on `CH2` `IN2P`/`IN2N`)
   * `RESP1 (0x09)` = `0x00`
   * `RESP2 (0x0A)` = `0x07`

---

## 2. Anti-Spike Heart Rate Analyzer (`edgeai_ecg.c` & `PanTompkinsDetector`)
Located in [`edgeai_ecg.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/edgeai_ecg.c#L195-L310) and [`ecg_gui.py`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/ecg_gui.py#L442-L585):
* **Why BPM Previously Spiked to High Numbers (`160–210 BPM`)**:
  * A `200–220 ms` refractory window allowed tall T-waves (`260–340 ms` after R-peak) or EMG muscle twitches to trigger a false second beat (`300 ms` interval = `200 BPM`).
* **5-Layer Anti-Spike Solution**:
  1. **`380 ms` (`95 samples` at 250 Hz) Physiological Refractory Period** + **Adaptive RR Gate** ($\text{RR}_{\text{new}} \ge 0.66 \times \overline{\text{RR}}_{\text{median}}$).
  2. **Morphological QRS Prominence Check**: Local 96 ms peak-to-peak excursion must be $\ge 50\%$ of the 98th-percentile QRS amplitude.
  3. **Single-Beat Outlier Quarantine**: Any beat $>25\%$ shorter than the median RR is quarantined for 1 beat unless confirmed by the next consecutive interval.
  4. **5-Second Autocorrelation Cross-Check (`compute_acf_bpm`)**: Computes normalized autocorrelation $R_{xx}(\tau)$ across $\tau \in [110, 340]\text{ samples}$ ($44\text{--}136\text{ BPM}$) every 450 ms.
  5. **Slew-Rate Limiter**: Caps beat-to-beat displayed BPM transitions to $\pm 4.5\text{ BPM/beat}$.

---

## 3. Dual-Source ECG-Derived Respiration (`EDR`) Rate & Waveform
Located in [`main.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/main.c#L337-L422) (`resp_process(ch1_raw, ch2_raw)`) and [`ecg_gui.py`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/ecg_gui.py#L316-L435) (`RespFilterChain`):
* Fuses **`0.35 * CH1 + 0.65 * CH2_raw` (ECG-Derived Respiration `EDR`)** in the **`0.10–0.42 Hz` ($6\text{--}25\text{ RPM}$)** passband.
* Normalizes the breathing wave into `[-1.0, +1.0]` for live display and estimates `RPM` continuously via zero-crossing peak hysteresis cross-checked by 15-second respiratory autocorrelation.

---

## 4. WCH CH455H 3-Digit 7-Segment BPM Display & Event-Driven Beat Flash
Located in [`main.c`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/main.c#L282-L336) (`ch455_init`, `ch455_update_bpm`) and [`ecg_gui.py`](file:///c:/Users/hjafa/OneDrive/Desktop/shield%20cc2650/firmware/08_ecg_dedicated/ecg_gui.py#L872-L892):
* **I2C0 Bus (`SCL = DIO_4`, `SDA = DIO_5`, `CONFIG_GPIO_I2C_EN = DIO_7 = 1`)**:
  * System register `0x24 <- 0x71` (8-segment mode, max brightness, display ON).
  * `DIG0..DIG2` (`0x34..0x36`): Displays 3-digit **Heart Rate per minute (`BPM`, e.g. ` 72` or `---`)**.
  * `DIG3` (`0x37`): Flashes all 4 discrete Shield LEDs (`0x0F`) + 7-segment decimal points (`0x80`) for `140 ms` (`35 samples`) on every real R-peak heartbeat.
* **UART Beat + BPM Sync Protocol**:
  * On every real detected R-peak heartbeat, `ecg_gui.py` sends a single non-blocking byte `100 + int(round(BPM))` (`rx_char >= 0x80`), which immediately sets the displayed BPM on the `CH455H` 7-segment display and triggers the `140 ms` hardware LED flash in unison with the GUI's `7-SEG BPM DISPLAY` and `❤` Heart Icon.
