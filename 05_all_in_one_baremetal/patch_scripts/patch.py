import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

start_marker = r"        /\* --- Structured Telemetry Block --- \*/"
end_marker = r"        /\* Responsive delay with button polling \(10 x 100ms\) \*/"

# The new telemetry block
new_block = """        /* --- Structured JSON Telemetry Block --- */
        uartPrint("{");
        uartPrint("\\"iter\\":"); printDec(iteration++);
        
        uint8_t curr_btn = pcal6408_read_buttons();
        uartPrint(",\\"btn\\":"); printDec(curr_btn);
        
        if (bmeOk) {
            uartPrint(",\\"T\\":"); printFloat2(bmeTemp);
            uartPrint(",\\"H\\":"); printFloat2(bmeHum);
            uartPrint(",\\"P\\":"); printFloat2(bmePress);
            uartPrint(",\\"gas\\":"); printFloat1(gasRes);
            uartPrint(",\\"iaq\\":"); printFloat1(iaq);
            uartPrint(",\\"co2\\":"); printFloat1(co2_eq);
            uartPrint(",\\"bvoc\\":"); printFloat2(bvoc);
        }

        if (mlxOk && mlx_ready) {
            uartPrint(",\\"mlx_obj\\":"); printFloat2(mlx_obj);
            uartPrint(",\\"mlx_amb\\":"); printFloat2(mlx_amb);
        }

        if (optOk) {
            uartPrint(",\\"lux\\":"); printFloat2(lux);
        }

        if (vcnlOk) {
            uartPrint(",\\"prox\\":"); printDec(prox);
            uartPrint(",\\"als\\":"); printDec(vcnlAls);
        }

        if (hub_ready && bioOk) {
            uartPrint(",\\"hr\\":"); printFloat1(bio.heartRate);
            uartPrint(",\\"spo2\\":"); printFloat1(bio.oxygen);
            uartPrint(",\\"hub_stat\\":"); printDec(bio.status);
        }

        if (imuOk) {
            int16_t cal_ax = ax - (imu_tare_valid ? imu_tare_x : 0);
            int16_t cal_ay = ay - (imu_tare_valid ? imu_tare_y : 0);
            int16_t cal_az = az - (imu_tare_valid ? imu_tare_z : 0);
            float gx = (float)cal_ax / 1000.0f;
            float gy = (float)cal_ay / 1000.0f;
            float gz = (float)cal_az / 1000.0f;
            
            uartPrint(",\\"ax\\":"); printFloat3(gx);
            uartPrint(",\\"ay\\":"); printFloat3(gy);
            uartPrint(",\\"az\\":"); printFloat3(gz);
        }

        uartPrint(",\\"ecg1\\":"); printFloat1(ads_ch1_uv);
        uartPrint(",\\"ecg2\\":"); printFloat1(ads_ch2_uv);

        uartPrint("}\\r\\n");

"""

# find start and end
try:
    start_idx = re.search(start_marker, content).start()
    end_idx = re.search(end_marker, content).start()

    new_content = content[:start_idx] + new_block + content[end_idx:]

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(new_content)

    print("Patch applied!")
except Exception as e:
    print(f"Error: {e}")
