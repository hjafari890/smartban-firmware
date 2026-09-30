import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Update DISPLAY_MODE_PPG to DISPLAY_MODE_ATT
content = content.replace("#define DISPLAY_MODE_PPG    7", "#define DISPLAY_MODE_ATT    7")
content = content.replace("{'P', 'P', 'G'},  /* PPG = Photoplethysmogram */", "{'A', 't', 't'},  /* Att = Attitude / Angle */")

# 2. Update the 25 Hz IMU telemetry in the main loop to calculate roll/pitch and output them
old_imu_block = """            int16_t ax = 0, ay = 0, az = 0;
            if (adxl362_read_accel(&ax, &ay, &az)) {
                int16_t cal_ax = ax - (imu_tare_valid ? imu_tare_x : 0);
                int16_t cal_ay = ay - (imu_tare_valid ? imu_tare_y : 0);
                int16_t cal_az = az - (imu_tare_valid ? imu_tare_z : 0);
                float gx = (float)cal_ax / 1000.0f;
                float gy = (float)cal_ay / 1000.0f;
                float gz = (float)cal_az / 1000.0f;

                uint8_t stat = adxl362_read_reg(0x0B);
                uint8_t is_awake = (stat & (1 << 6)) ? 1 : 0; /* Bit 6 = AWAKE */

                uartPrint("{\\"type\\":\\"imu\\",\\"ax\\":"); printFloat3(gx);
                uartPrint(",\\"ay\\":"); printFloat3(gy);
                uartPrint(",\\"az\\":"); printFloat3(gz);
                uartPrint(",\\"act\\":"); printDec(is_awake);
                uartPrint("}\\r\\n");
            }"""

new_imu_block = """            int16_t ax = 0, ay = 0, az = 0;
            if (adxl362_read_accel(&ax, &ay, &az)) {
                int16_t cal_ax = ax - (imu_tare_valid ? imu_tare_x : 0);
                int16_t cal_ay = ay - (imu_tare_valid ? imu_tare_y : 0);
                int16_t cal_az = az - (imu_tare_valid ? imu_tare_z : 0);
                float gx = (float)cal_ax / 1000.0f;
                float gy = (float)cal_ay / 1000.0f;
                float gz = (float)cal_az / 1000.0f;

                /* Airplane Attitude Angles (Pitch & Roll in degrees) */
                float pitch_deg = atan2f(-gx, sqrtf(gy*gy + gz*gz)) * 57.29578f;
                float roll_deg  = atan2f(gy, gz) * 57.29578f;

                uint8_t stat = adxl362_read_reg(0x0B);
                uint8_t is_awake = (stat & (1 << 6)) ? 1 : 0; /* Bit 6 = AWAKE / ATTACK */

                uartPrint("{\\"type\\":\\"imu\\",\\"ax\\":"); printFloat3(gx);
                uartPrint(",\\"ay\\":"); printFloat3(gy);
                uartPrint(",\\"az\\":"); printFloat3(gz);
                uartPrint(",\\"pitch\\":"); printFloat1(pitch_deg);
                uartPrint(",\\"roll\\":"); printFloat1(roll_deg);
                uartPrint(",\\"act\\":"); printDec(is_awake);
                uartPrint("}\\r\\n");

                /* If 7-segment display is in Attitude mode, show pitch angle */
                if (!display_show_label && display_mode == DISPLAY_MODE_ATT) {
                    int16_t p_int = (int16_t)(pitch_deg >= 0 ? (pitch_deg + 0.5f) : (pitch_deg - 0.5f));
                    if (p_int < 0) p_int = -p_int;
                    if (p_int > 999) p_int = 999;
                    ch455_display_number((uint16_t)p_int);
                }
            }"""

if old_imu_block in content:
    content = content.replace(old_imu_block, new_imu_block)
    print("IMU block replaced successfully!")
else:
    print("Warning: old_imu_block not found exactly, doing regex search...")
    content = re.sub(r'int16_t ax = 0, ay = 0, az = 0;\s+if \(adxl362_read_accel.*?\n\s+uartPrint\("\\\\r\\\\n"\);\s+\}', new_imu_block, content, flags=re.DOTALL)

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("main.c updated with Attitude Pitch/Roll!")
