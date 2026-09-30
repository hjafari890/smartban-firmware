import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Insert ISRs before int main(void)
isr_code = """
/* =========================================================================
 * Phase 2 High-Speed ISRs
 * ========================================================================= */
volatile bool flag_ecg_drdy = false;
volatile bool flag_imu_int = false;

void ecg_drdy_callback(uint_least8_t index) {
    flag_ecg_drdy = true;
}

void imu_int_callback(uint_least8_t index) {
    flag_imu_int = true;
}

/* =========================================================================
 * Main Function
"""
content = content.replace("/* =========================================================================\n * Main Function", isr_code)

# 2. Replace everything from uint32_t iteration = 1; down to the end
loop_start_marker = r"    uint32_t iteration = 1;"
loop_match = re.search(loop_start_marker, content)
if not loop_match:
    print("Could not find loop start marker")
    exit(1)

start_idx = loop_match.start()

new_loop_code = """    uint32_t iteration = 1;
    uint32_t ms_tick = 0;
    
    /* Configure High-Speed Sensor Interrupts */
    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, ecg_drdy_callback);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);

    GPIO_setConfig(CONFIG_GPIO_IMU_INT1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_IMU_INT1, imu_int_callback);
    GPIO_enableInt(CONFIG_GPIO_IMU_INT1);

    while (1)
    {
        /* --- HIGH SPEED SENSORS (Evaluated Every 1ms) --- */
        if (flag_ecg_drdy) {
            flag_ecg_drdy = false;
            int32_t ecg1 = 0, ecg2 = 0;
            if (ads1292_read_sample(&ecg1, &ecg2)) {
                uartPrint("{\\"type\\":\\"ecg\\",\\"e1\\":"); printInt(ecg1);
                uartPrint(",\\"e2\\":"); printInt(ecg2);
                uartPrint("}\\r\\n");
            }
        }

        if (flag_imu_int) {
            flag_imu_int = false;
            int16_t ax = 0, ay = 0, az = 0;
            if (adxl362_read_accel(&ax, &ay, &az)) {
                int16_t cal_ax = ax - (imu_tare_valid ? imu_tare_x : 0);
                int16_t cal_ay = ay - (imu_tare_valid ? imu_tare_y : 0);
                int16_t cal_az = az - (imu_tare_valid ? imu_tare_z : 0);
                float gx = (float)cal_ax / 1000.0f;
                float gy = (float)cal_ay / 1000.0f;
                float gz = (float)cal_az / 1000.0f;
                uartPrint("{\\"type\\":\\"imu\\",\\"ax\\":"); printFloat3(gx);
                uartPrint(",\\"ay\\":"); printFloat3(gy);
                uartPrint(",\\"az\\":"); printFloat3(gz);
                uartPrint("}\\r\\n");
            }
        }

        /* --- TIME SLICED TASKS --- */
        ms_tick++;
        
        /* UI 10Hz Tasks */
        if (ms_tick % 100 == 0) {
            uint8_t b = pcal6408_read_buttons();
            if (b != 0 && b != pcal_prev_buttons) {
                uint8_t edge = b & (~pcal_prev_buttons);
                pcal_prev_buttons = b;

                if (edge & BTN_SW6_MODE) {
                    display_mode = (display_mode + 1) % DISPLAY_MODE_COUNT;
                    ch455_display_text(mode_labels[display_mode][0],
                                       mode_labels[display_mode][1],
                                       mode_labels[display_mode][2]);
                    ch455_set_leds(mode_leds[display_mode]);
                    display_show_label = true;
                    display_label_countdown = 15; /* 1.5 seconds */
                }
                if (edge & BTN_SW4_BT) {
                    adxl362_calibrate_tare();
                    ch455_display_text('C', 'A', 'L');
                    display_show_label = true;
                    display_label_countdown = 15;
                }
            } else {
                pcal_prev_buttons = b;
            }

            if (display_show_label) {
                if (display_label_countdown > 0) {
                    display_label_countdown--;
                } else {
                    display_show_label = false;
                }
            }
        }

        /* Environmental 1Hz Task */
        if (ms_tick >= 1000) {
            ms_tick = 0;
            float bmeTemp = 0, bmePress = 0, bmeHum = 0, gasRes = 0, iaq = 0, co2_eq = 0, bvoc = 0;
            bool bmeOk = bme680_read_all(&bmeTemp, &bmePress, &bmeHum, &gasRes, &iaq, &co2_eq, &bvoc);

            float lux = 0; uint8_t optExp = 0; uint32_t optMant = 0;
            bool optOk = opt4041_read_lux(&lux, &optExp, &optMant);

            uint16_t prox = 0, vcnlAls = 0, vcnlWhite = 0;
            bool vcnlOk = vcnl4040_read_data(&prox, &vcnlAls, &vcnlWhite);

            float mlx_amb = 0, mlx_obj = 0;
            bool mlxOk = mlx90632_read_temp(&mlx_amb, &mlx_obj);

            BioData_t bio; memset(&bio, 0, sizeof(bio));
            bool bioOk = max32664_read_biometrics(&bio);

            uartPrint("{\\"type\\":\\"env\\",\\"iter\\":"); printDec(iteration++);
            if (bmeOk) {
                uartPrint(",\\"T\\":"); printFloat2(bmeTemp);
                uartPrint(",\\"H\\":"); printFloat2(bmeHum);
                uartPrint(",\\"P\\":"); printFloat2(bmePress);
            }
            if (optOk) { uartPrint(",\\"lux\\":"); printFloat2(lux); }
            if (vcnlOk) { uartPrint(",\\"prox\\":"); printDec(prox); }
            if (mlxOk && mlx_ready) {
                uartPrint(",\\"mlx_obj\\":"); printFloat2(mlx_obj);
            }
            if (hub_ready && bioOk) {
                uartPrint(",\\"hr\\":"); printFloat1(bio.heartRate);
                uartPrint(",\\"spo2\\":"); printFloat1(bio.oxygen);
            }
            uartPrint("}\\r\\n");

            if (!display_show_label) {
                switch (display_mode) {
                    case DISPLAY_MODE_TEMP: if(bmeOk) ch455_display_float1(bmeTemp); else ch455_display_dash(); break;
                    case DISPLAY_MODE_PRESS: if(bmeOk) ch455_display_number((uint16_t)bmePress); else ch455_display_dash(); break;
                    case DISPLAY_MODE_HUM: if(bmeOk) ch455_display_float1(bmeHum); else ch455_display_dash(); break;
                    case DISPLAY_MODE_LUX: if(optOk) ch455_display_number((uint16_t)(lux>999?999:lux)); else ch455_display_dash(); break;
                    case DISPLAY_MODE_PROX: if(vcnlOk) ch455_display_number(prox>999?999:prox); else ch455_display_dash(); break;
                    case DISPLAY_MODE_IRTEMP: if(mlxOk) ch455_display_float1(mlx_obj); else ch455_display_dash(); break;
                    case DISPLAY_MODE_PPG: if(bioOk) ch455_display_number((uint16_t)bio.heartRate); else ch455_display_dash(); break;
                    default: ch455_display_dash(); break;
                }
            }
        }

        usleep(1000); /* 1 millisecond hardware non-blocking tick */
    }
    return 0;
}
"""

content = content[:start_idx] + new_loop_code

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("Phase 2 Patch applied successfully!")
