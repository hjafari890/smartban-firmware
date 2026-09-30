# Original User Request

## Initial Request, 2026-09-05T19:16:25Z

Debug and calibrate the remaining biomedical and environmental sensors on the SmartBAN multi-sensor shield: resolve the Bosch BME680 barometric pressure compensation formula (currently reporting ~3352 hPa), correct the Melexis MLX90632 FIR sensor die temperature calculation (currently overflowing), and bring up the Maxim MAX30102 / MAX32664 optical pulse oximeter front-end and physical LED.

Working directory: `c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\01_hardware_diagnostic`
Integrity mode: development

## Verification Resources
- Build & Flash Tool: `python build_and_flash.py` (executes SysConfig, `tiarmclang`, `tiarmobjcopy`, and `srfprog` for cJTAG flashing)
- Hardware Schematics & Netlist: `BMEshield.kicad_sch` and `kicad/BMEshield/BMEshield.kicad_pcb` in project root
- Technical Reference: `smartban_sensor_node_journey_log.md` detailing hardware pin mappings, power rails (DIO 21 1.8V, DIO 30 I2C Level Shifter, DIO 18 IMU Switch), and register layouts
- Serial Telemetry: USB UART on COM3 @ 115200 baud, 8-N-1

## Requirements

### R1. Bosch BME680 Barometric Pressure Compensation
- Identify and resolve the calculation discrepancy in the BME680 pressure compensation pipeline so that atmospheric pressure is reported accurately within realistic ambient ranges (~980 - 1030 hPa at normal sea-level / room elevation).
- Preserve the verified room temperature compensation (currently reading ~21.5 °C), relative humidity, and MOX gas hotplate resistance.

### R2. Melexis MLX90632 FIR Skin & Die Temperature Compensation
- Correct the thermopile raw RAM conversion and temperature calculation so that both Sensor Die Temperature and Target/Skin Temperature report rational physical values (~20 - 37 °C) without numerical overflow.

### R3. Maxim MAX32664 & MAX30102 Optical PPG Subsystem
- Investigate the hardware and firmware initialization chain between CC2652R1 host, PCA9306 level shifter, MAX32664 Biometric Hub, and MAX30102 optical sensor.
- Ensure the optical front-end is commanded into an active sampling mode with physical LED illumination (Red/IR) and verify live biometric sample collection (heart rate, SpO2, raw reflection counts) upon finger placement.

## Acceptance Criteria

### BME680 Barometric Pressure
- [ ] Atmospheric pressure reads between 980.0 hPa and 1030.0 hPa under ambient indoor conditions.
- [ ] Temperature reading remains accurate at ambient room temperature (~20 - 25 °C).
- [ ] Firmware builds cleanly with `python build_and_flash.py` and flashes to target without errors.

### MLX90632 FIR Temperature
- [ ] Sensor die temperature reports rational ambient temperature (~20 - 30 °C) with zero arithmetic overflow.
- [ ] Target temperature responds dynamically to hand proximity (~30 - 35 °C).

### MAX30102 / MAX32664 Optical Pulse Oximeter
- [ ] MAX30102 optical LED emits visible light when active.
- [ ] Serial telemetry streams non-zero raw optical counts (IR and Red) and outputs heart rate / SpO2 when a finger is placed on the sensor.
