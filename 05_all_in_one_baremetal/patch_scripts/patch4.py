import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

new_main = """
/* =========================================================================
 * Phase 4: Full JSON Telemetry for GUI + UART Control
 * ========================================================================= */
int main(void)
{
    Board_init();
    NoRTOS_start();
    GPIO_init();
    I2C_init();
    SPI_init();

    GPIO_write(CONFIG_GPIO_1V8_EN, 1);
    GPIO_write(CONFIG_GPIO_I2C_EN, 1);
    GPIO_write(CONFIG_GPIO_HUB_MFIO, 1);
    GPIO_write(CONFIG_GPIO_IMU_SW, 1);
    GPIO_write(CONFIG_GPIO_ECG_PWDN, 1);
    GPIO_write(CONFIG_GPIO_IMU_CS, 1);
    GPIO_write(CONFIG_GPIO_ECG_CS, 1);
    usleep(100000);

    UART2_Params uartParams; UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readMode = UART2_Mode_NONBLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    I2C_Params i2cParams; I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    spi_set_mode(SPI_POL0_PHA0);

    ch455_init();
    pcal6408_init();
    adxl362_init();
    
    /* ADXL362 Activity/Inactivity Setup (for Attack Indicator) */
    adxl362_write_reg(0x20, 0xFA); /* THRESH_ACT_L (250mg) */
    adxl362_write_reg(0x21, 0x00); /* THRESH_ACT_H */
    adxl362_write_reg(0x22, 4);    /* TIME_ACT */
    adxl362_write_reg(0x23, 0x96); /* THRESH_INACT_L (150mg) */
    adxl362_write_reg(0x24, 0x00); /* THRESH_INACT_H */
    adxl362_write_reg(0x25, 50);   /* TIME_INACT_L */
    adxl362_write_reg(0x26, 0x00); /* TIME_INACT_H */
    adxl362_write_reg(0x27, 0x3F); /* ACT_INACT_CTL: Loop mode */

    ads1292_init();
    bme680_load_calibration();
    opt4041_init();
    vcnl4040_init();
    mlx90632_init();

    ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);

    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, ecg_drdy_callback);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);

    GPIO_setConfig(CONFIG_GPIO_IMU_INT1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_IMU_INT1, imu_int_callback);
    GPIO_enableInt(CONFIG_GPIO_IMU_INT1);

    uint32_t ms_tick = 0;
    uint8_t rx_byte = 0;

    while (1) {
        /* Read UART for ECG Test Mode Commands from GUI */
        size_t bytesRead = 0;
        int status = UART2_read(uart, &rx_byte, 1, &bytesRead);
        if (status == UART2_STATUS_SUCCESS && bytesRead == 1) {
            if (rx_byte == '1') ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);
            else if (rx_byte == '2') ads1292_set_test_mode(ECG_MODE_INPUT_SHORT);
            else if (rx_byte == '3') ads1292_set_test_mode(ECG_MODE_TEMPERATURE);
            else if (rx_byte == '4') ads1292_set_test_mode(ECG_MODE_LIVE_ELECTRODE);
        }

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
                float gx = (float)ax / 1000.0f;
                float gy = (float)ay / 1000.0f;
                float gz = (float)az / 1000.0f;
                uint8_t stat = adxl362_read_reg(0x0B);
                uint8_t is_awake = (stat & (1 << 6)) ? 1 : 0;
                
                uartPrint("{\\"type\\":\\"imu\\",\\"ax\\":"); printFloat3(gx);
                uartPrint(",\\"ay\\":"); printFloat3(gy);
                uartPrint(",\\"az\\":"); printFloat3(gz);
                uartPrint(",\\"act\\":"); printDec(is_awake);
                uartPrint("}\\r\\n");
            }
        }

        ms_tick++;
        
        if (ms_tick % 100 == 0) {
            uint8_t b = pcal6408_read_buttons();
            if (b != 0 && b != pcal_prev_buttons) {
                uint8_t edge = b & (~pcal_prev_buttons);
                pcal_prev_buttons = b;
                
                if (edge & BTN_SW1_ECG) {
                    uint32_t nm = (uint32_t)ecg_test_mode + 1;
                    if (nm > 4) nm = 1;
                    ads1292_set_test_mode((EcgTestMode_t)nm);
                }
            } else {
                pcal_prev_buttons = b;
            }
        }

        if (ms_tick >= 1000) {
            ms_tick = 0;
            float bmeTemp = 0, bmePress = 0, bmeHum = 0, gasRes = 0, iaq = 0, co2_eq = 0, bvoc = 0;
            bme680_read_all(&bmeTemp, &bmePress, &bmeHum, &gasRes, &iaq, &co2_eq, &bvoc);

            float lux = 0; uint8_t optExp = 0; uint32_t optMant = 0;
            opt4041_read_lux(&lux, &optExp, &optMant);

            uint16_t prox = 0, vcnlAls = 0, vcnlWhite = 0;
            vcnl4040_read_data(&prox, &vcnlAls, &vcnlWhite);

            float mlx_amb = 0, mlx_obj = 0;
            mlx90632_read_temp(&mlx_amb, &mlx_obj);

            uartPrint("{\\"type\\":\\"env\\"");
            uartPrint(",\\"T\\":"); printFloat2(bmeTemp);
            uartPrint(",\\"H\\":"); printFloat2(bmeHum);
            uartPrint(",\\"P\\":"); printFloat2(bmePress);
            uartPrint(",\\"lux\\":"); printFloat2(lux);
            uartPrint(",\\"prox\\":"); printDec(prox);
            uartPrint(",\\"mlx\\":"); printFloat2(mlx_obj);
            uartPrint(",\\"mode\\":"); printDec((uint32_t)ecg_test_mode);
            uartPrint("}\\r\\n");
        }
        usleep(1000); 
    }
    return 0;
}
"""

start_marker = r"int main\(void\)"
match = re.search(start_marker, content)
if match:
    start_idx = match.start()
    new_content = content[:start_idx] + new_main
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(new_content)
    print("Firmware patched for JSON GUI!")
else:
    print("Could not find int main(void)")
