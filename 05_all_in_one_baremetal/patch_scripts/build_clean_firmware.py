import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# Locate where ADS1292 helper functions end (around line 2120)
target_marker = r"static bool ads1292_read_sample\(int32_t \*ch1_out, int32_t \*ch2_out\)"
match = re.search(target_marker, content)
if not match:
    print("Error: Could not find ads1292_read_sample")
    exit(1)

# Find the end of ads1292_read_sample function
func_end = content.find("return false;\n}", match.start())
if func_end == -1:
    func_end = content.find("return false;\r\n}", match.start())
if func_end == -1:
    print("Error: Could not find end of ads1292_read_sample")
    exit(1)

cut_point = func_end + len("return false;\n}")

new_tail = """

/* =========================================================================
 * ADS1292R 4 Self-Test Modes
 * ========================================================================= */
typedef enum {
    ECG_MODE_SQUARE_WAVE = 1,   /* 1 Hz, +/-1 mV internal test generator */
    ECG_MODE_INPUT_SHORT = 2,   /* Inputs shorted (noise floor & offset) */
    ECG_MODE_TEMPERATURE = 3,   /* Internal die temperature diode */
    ECG_MODE_LIVE_ELECTRODE = 4 /* External electrode pins (live ECG) */
} EcgTestMode_t;

static EcgTestMode_t ecg_test_mode = ECG_MODE_LIVE_ELECTRODE;

static bool ads1292_set_test_mode(EcgTestMode_t mode)
{
    if (!ads_ready && ads_id == 0) return false;

    /* 1. Stop continuous conversion mode to write registers */
    ads1292_send_cmd(ADS1292_CMD_SDATAC); /* 0x11 */
    usleep(100);

    /* 2. Common configuration: 500 SPS, 2.42V Ref enabled, RLD sense enabled */
    ads1292_write_reg(ADS1292_REG_CONFIG1, 0x02);
    ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
    ads1292_write_reg(ADS1292_REG_RLD_SENS,0x2C);

    switch (mode) {
        case ECG_MODE_SQUARE_WAVE:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA3); /* Internal 1Hz, +/-1mV test signal */
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x05); /* Test signal */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x05); /* Test signal */
            break;

        case ECG_MODE_INPUT_SHORT:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0); /* Test signal OFF, Ref ON */
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x01); /* Input shorted */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x01); /* Input shorted */
            break;

        case ECG_MODE_TEMPERATURE:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x04); /* Internal Temp Sensor */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x04);
            break;

        case ECG_MODE_LIVE_ELECTRODE:
        default:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x00); /* Normal Electrode Input */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x00);
            break;
    }

    /* 3. Settle reference */
    usleep(50000);

    /* 4. Start conversions */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(10000);

    /* 5. Re-enter continuous data mode */
    ads1292_send_cmd(ADS1292_CMD_RDATAC); /* 0x10 */
    usleep(10000);

    ecg_test_mode = mode;
    return true;
}

/* Hardware Interrupt ISR for ECG DRDY */
volatile bool flag_ecg_drdy = false;
void ecg_drdy_callback(uint_least8_t index) {
    flag_ecg_drdy = true;
}

/* =========================================================================
 * Main Function - High-Performance Balanced Telemetry
 * ========================================================================= */
int main(void)
{
    Board_init();
    NoRTOS_start();
    GPIO_init();
    I2C_init();
    SPI_init();

    /* Assert power rails & enable peripherals */
    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    GPIO_write(CONFIG_GPIO_HUB_MFIO, 1);
    GPIO_write(CONFIG_GPIO_IMU_SW, 1);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    usleep(100000);

    /* Open UART2 in NONBLOCKING read mode so GUI commands are instantly received */
    UART2_Params uartParams;
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readMode = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    spi_set_mode(SPI_POL0_PHA0);

    /* Initialize Peripherals */
    ch455_init();
    pcal6408_init();
    adxl362_init();

    /* Configure ADXL362 Autonomous Activity / Inactivity Detection for Attack Indicator */
    adxl362_write_reg(0x2D, 0x00); /* Standby */
    adxl362_write_reg(0x20, 0xFA); /* THRESH_ACT_L = 250mg */
    adxl362_write_reg(0x21, 0x00); /* THRESH_ACT_H */
    adxl362_write_reg(0x22, 4);    /* TIME_ACT = 40ms */
    adxl362_write_reg(0x23, 0x96); /* THRESH_INACT_L = 150mg */
    adxl362_write_reg(0x24, 0x00); /* THRESH_INACT_H */
    adxl362_write_reg(0x25, 50);   /* TIME_INACT_L = 500ms */
    adxl362_write_reg(0x26, 0x00); /* TIME_INACT_H */
    adxl362_write_reg(0x27, 0x3F); /* ACT_INACT_CTL: Loop mode + Referenced */
    adxl362_write_reg(0x2C, 0x13); /* 100 Hz ODR, 25 Hz filter, +/-2g */
    adxl362_write_reg(0x2D, 0x22); /* Measurement Mode + Ultra-Low Noise */

    ads1292_init();
    bme680_load_calibration();
    opt4041_init();
    vcnl4040_init();
    mlx90632_init();

    /* Default to live electrode mode */
    ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);

    /* Enable Falling-Edge Interrupt on ECG DRDY pin */
    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, ecg_drdy_callback);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);

    uint32_t ms_tick = 0;
    uint32_t ecg_sample_div = 0;
    uint8_t rx_cmd = 0;

    while (1)
    {
        /* 1. Receive UART Commands from GUI for ECG Test Modes */
        size_t rx_bytes = 0;
        int_fast16_t rx_stat = UART2_read(uart, &rx_cmd, 1, &rx_bytes);
        if (rx_stat == UART2_STATUS_SUCCESS && rx_bytes > 0) {
            if (rx_cmd == '1') ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);
            else if (rx_cmd == '2') ads1292_set_test_mode(ECG_MODE_INPUT_SHORT);
            else if (rx_cmd == '3') ads1292_set_test_mode(ECG_MODE_TEMPERATURE);
            else if (rx_cmd == '4') ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);
        }

        /* 2. High-Speed ECG Acquisition (500 SPS DRDY Interrupt)
         * Decimate 4:1 to 125 Hz for UART transmission.
         * 125 Hz * 38 bytes = ~4750 B/s (well within 115200 baud capacity!) */
        if (flag_ecg_drdy) {
            flag_ecg_drdy = false;
            int32_t ecg1 = 0, ecg2 = 0;
            if (ads1292_read_sample(&ecg1, &ecg2)) {
                ecg_sample_div++;
                if ((ecg_sample_div & 0x03) == 0) {
                    uartPrint("{\\"type\\":\\"ecg\\",\\"e1\\":"); printInt(ecg1);
                    uartPrint(",\\"e2\\":"); printInt(ecg2);
                    uartPrint("}\\r\\n");
                }
            }
        }

        ms_tick++;

        /* 3. IMU Polling at 25 Hz (every 40ms) */
        if ((ms_tick % 40) == 0) {
            int16_t ax = 0, ay = 0, az = 0;
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
            }
        }

        /* 4. UI 10Hz Buttons & 7-Segment Refresh */
        if ((ms_tick % 100) == 0) {
            uint8_t b = pcal6408_read_buttons();
            if (b != 0 && b != pcal_prev_buttons) {
                uint8_t edge = b & (~pcal_prev_buttons);
                pcal_prev_buttons = b;

                /* SW6: Toggle 7-Segment Display Sensor Mode */
                if (edge & BTN_SW6_MODE) {
                    display_mode = (display_mode + 1) % DISPLAY_MODE_COUNT;
                    ch455_display_text(mode_labels[display_mode][0],
                                       mode_labels[display_mode][1],
                                       mode_labels[display_mode][2]);
                    ch455_set_leds(mode_leds[display_mode]);
                    display_show_label = true;
                    display_label_countdown = 15;
                }
                /* SW1: Toggle ECG Test Mode from Board */
                if (edge & BTN_SW1_ECG) {
                    uint32_t nm = (uint32_t)ecg_test_mode + 1;
                    if (nm > 4) nm = 1;
                    ads1292_set_test_mode((EcgTestMode_t)nm);
                    ch455_display_text('E', 'C', 'G');
                    display_show_label = true;
                    display_label_countdown = 15;
                }
                /* SW4: IMU Tare Zero-G */
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
                if (display_label_countdown > 0) display_label_countdown--;
                else display_show_label = false;
            }
        }

        /* 5. Environmental & Climate Sensors at 1 Hz (1000ms) */
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

            uartPrint("{\\"type\\":\\"env\\"");
            if (bmeOk) {
                uartPrint(",\\"T\\":"); printFloat2(bmeTemp);
                uartPrint(",\\"H\\":"); printFloat2(bmeHum);
                uartPrint(",\\"P\\":"); printFloat2(bmePress);
            }
            if (optOk) { uartPrint(",\\"lux\\":"); printFloat2(lux); }
            if (vcnlOk) { uartPrint(",\\"prox\\":"); printDec(prox); }
            if (mlxOk && mlx_ready) { uartPrint(",\\"mlx\\":"); printFloat2(mlx_obj); }
            uartPrint(",\\"mode\\":"); printDec((uint32_t)ecg_test_mode);
            uartPrint("}\\r\\n");

            /* Update 7-segment display */
            if (!display_show_label) {
                switch (display_mode) {
                    case DISPLAY_MODE_TEMP: if(bmeOk) ch455_display_float1(bmeTemp); else ch455_display_dash(); break;
                    case DISPLAY_MODE_PRESS: if(bmeOk) ch455_display_number((uint16_t)bmePress); else ch455_display_dash(); break;
                    case DISPLAY_MODE_HUM: if(bmeOk) ch455_display_float1(bmeHum); else ch455_display_dash(); break;
                    case DISPLAY_MODE_LUX: if(optOk) ch455_display_number((uint16_t)(lux>999?999:lux)); else ch455_display_dash(); break;
                    case DISPLAY_MODE_PROX: if(vcnlOk) ch455_display_number(prox>999?999:prox); else ch455_display_dash(); break;
                    case DISPLAY_MODE_IRTEMP: if(mlxOk) ch455_display_float1(mlx_obj); else ch455_display_dash(); break;
                    default: ch455_display_dash(); break;
                }
            }
        }

        /* 1ms tick delay */
        usleep(1000);
    }
    return 0;
}
"""

clean_content = content[:cut_point] + new_tail
with open(file_path, "w", encoding="utf-8") as f:
    f.write(clean_content)

print("Clean firmware patched successfully!")
