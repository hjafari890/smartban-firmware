import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# The new code to insert right before int main(void)
# It includes the ECG modes, the test mode function, and a completely rewritten int main(void)

new_code = """
/* =========================================================================
 * Phase 3: Readable Telemetry & ECG Test Modes
 * ========================================================================= */
typedef enum {
    ECG_MODE_SQUARE_WAVE = 1,
    ECG_MODE_INPUT_SHORT = 2,
    ECG_MODE_TEMPERATURE = 3,
    ECG_MODE_LIVE_ELECTRODE = 4
} EcgTestMode_t;
static const char *ECG_MODE_NAMES[] = {
    "NONE", "1 Hz Square Wave (+/-1 mV)", "Input Short (Noise Floor)", "Die Temperature", "Live Electrodes"
};
static EcgTestMode_t ecg_test_mode = ECG_MODE_SQUARE_WAVE;

static float ecg_ch1_min = 100000.0f;
static float ecg_ch1_max = -100000.0f;
static float ecg_ch1_sum = 0.0f;
static uint32_t ecg_stat_count = 0;

static bool ads1292_set_test_mode(EcgTestMode_t mode)
{
    if (!ads_ready) return false;
    ads1292_send_cmd(0x0A); /* SDATAC */
    usleep(100);
    ads1292_write_reg(0x01, 0x02); /* CONFIG1 */
    ads1292_write_reg(0x0A, 0x83); /* RESP2 */
    ads1292_write_reg(0x0D, 0x2C); /* RLD_SENS */

    switch (mode) {
        case ECG_MODE_SQUARE_WAVE:
            ads1292_write_reg(0x02, 0xA3); /* CONFIG2: Int Test Sig, 1 Hz */
            ads1292_write_reg(0x04, 0x05); /* CH1SET: Test Signal */
            ads1292_write_reg(0x05, 0x05); /* CH2SET: Test Signal */
            break;
        case ECG_MODE_INPUT_SHORT:
            ads1292_write_reg(0x02, 0xA0); 
            ads1292_write_reg(0x04, 0x01); /* CH1SET: Input Short */
            ads1292_write_reg(0x05, 0x01); 
            break;
        case ECG_MODE_TEMPERATURE:
            ads1292_write_reg(0x02, 0xA0); 
            ads1292_write_reg(0x04, 0x04); /* CH1SET: Temp Sensor */
            ads1292_write_reg(0x05, 0x04); 
            break;
        case ECG_MODE_LIVE_ELECTRODE:
        default:
            ads1292_write_reg(0x02, 0xA0); 
            ads1292_write_reg(0x04, 0x00); /* CH1SET: Normal */
            ads1292_write_reg(0x05, 0x00); 
            break;
    }
    usleep(100000);
    ads1292_read_regs(0x00, 12, ads_regs);
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(10000);
    ads1292_send_cmd(0x10); /* RDATAC */
    usleep(10000);
    ecg_test_mode = mode;
    return true;
}

volatile bool flag_ecg_drdy = false;
volatile bool flag_imu_int = false;
void ecg_drdy_callback(uint_least8_t index) { flag_ecg_drdy = true; }
void imu_int_callback(uint_least8_t index) { flag_imu_int = true; }

/* =========================================================================
 * Main Function
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
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    I2C_Params i2cParams; I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    spi_set_mode(SPI_POL0_PHA0);

    ch455_init();
    pcal6408_init();
    adxl362_init();
    ads1292_init();
    bme680_load_calibration();
    opt4041_init();
    vcnl4040_init();
    mlx90632_init();
    max32664_init();

    ads1292_set_test_mode(ECG_MODE_SQUARE_WAVE);

    GPIO_setConfig(CONFIG_GPIO_ECG_DRDY, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(CONFIG_GPIO_ECG_DRDY, ecg_drdy_callback);
    GPIO_enableInt(CONFIG_GPIO_ECG_DRDY);

    GPIO_setConfig(CONFIG_GPIO_IMU_INT1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING);
    GPIO_setCallback(CONFIG_GPIO_IMU_INT1, imu_int_callback);
    GPIO_enableInt(CONFIG_GPIO_IMU_INT1);

    uartPrint("\\r\\n===============================================================\\r\\n");
    uartPrint("  SmartBAN Multi-Modal Sensor Array - Readable Serial Mode\\r\\n");
    uartPrint("===============================================================\\r\\n");

    uint32_t iteration = 1;
    uint32_t ms_tick = 0;

    while (1) {
        if (flag_ecg_drdy) {
            flag_ecg_drdy = false;
            int32_t ecg1 = 0, ecg2 = 0;
            if (ads1292_read_sample(&ecg1, &ecg2)) {
                float uv = (float)ecg1 * (2420000.0f / 8388607.0f);
                if (uv < ecg_ch1_min) ecg_ch1_min = uv;
                if (uv > ecg_ch1_max) ecg_ch1_max = uv;
                ecg_ch1_sum += uv;
                ecg_stat_count++;
            }
        }
        if (flag_imu_int) {
            flag_imu_int = false;
            int16_t ax = 0, ay = 0, az = 0;
            adxl362_read_accel(&ax, &ay, &az);
        }

        ms_tick++;
        
        /* UI 10Hz Tasks */
        if (ms_tick % 100 == 0) {
            uint8_t b = pcal6408_read_buttons();
            if (b != 0 && b != pcal_prev_buttons) {
                uint8_t edge = b & (~pcal_prev_buttons);
                pcal_prev_buttons = b;

                if (edge & BTN_SW6_MODE) {
                    display_mode = (display_mode + 1) % DISPLAY_MODE_COUNT;
                    ch455_display_text(mode_labels[display_mode][0], mode_labels[display_mode][1], mode_labels[display_mode][2]);
                    ch455_set_leds(mode_leds[display_mode]);
                    display_show_label = true; display_label_countdown = 15;
                }
                if (edge & BTN_SW1_ECG) {
                    uint32_t nm = (uint32_t)ecg_test_mode + 1;
                    if (nm > 4) nm = 1;
                    ads1292_set_test_mode((EcgTestMode_t)nm);
                    ch455_display_text('E', 'C', 'G');
                    display_show_label = true; display_label_countdown = 15;
                }
                if (edge & BTN_SW4_BT) {
                    adxl362_calibrate_tare();
                    ch455_display_text('C', 'A', 'L');
                    display_show_label = true; display_label_countdown = 15;
                }
            } else {
                pcal_prev_buttons = b;
            }

            if (display_show_label) {
                if (display_label_countdown > 0) display_label_countdown--;
                else display_show_label = false;
            }
        }

        /* 1Hz Telemetry Output */
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

            BioData_t bio; memset(&bio, 0, sizeof(bio));
            max32664_read_biometrics(&bio);

            uartPrint("\\r\\n---------------------------------------------------------------\\r\\n");
            uartPrint(">> Sample #"); printDec(iteration++); uartPrint(" | Readable Diagnostic Stream\\r\\n");
            
            // ECG Stats
            uartPrint("  * [ADS1292R ECG]    : Mode: "); uartPrint(ECG_MODE_NAMES[ecg_test_mode]);
            float ch1_avg = (ecg_stat_count > 0) ? (ecg_ch1_sum / ecg_stat_count) : 0;
            float ch1_vpp = ecg_ch1_max - ecg_ch1_min;
            if (ch1_vpp < 0.0f) ch1_vpp = 0.0f;
            uartPrint("\\r\\n      --> Stats       : Vpp = "); printFloat1(ch1_vpp); uartPrint(" uV | Offset = "); printFloat1(ch1_avg);
            uartPrint(" uV | Samples = "); printDec(ecg_stat_count); uartPrint("\\r\\n");
            
            if (ecg_test_mode == ECG_MODE_SQUARE_WAVE) {
                if (ch1_vpp > 8000.0f && ch1_vpp < 15000.0f) uartPrint("      --> Verdict     : [PASS] Clean +/-1mV square wave detected!\\r\\n");
                else uartPrint("      --> Verdict     : [WAIT] Tracking test signal...\\r\\n");
            } else if (ecg_test_mode == ECG_MODE_INPUT_SHORT) {
                if (ch1_vpp < 500.0f) uartPrint("      --> Verdict     : [PASS] Low noise floor & minimal offset!\\r\\n");
            } else if (ecg_test_mode == ECG_MODE_TEMPERATURE) {
                float temp_c = ((ch1_avg / 1000.0f) - 110.0f) / 0.145f + 25.0f;
                uartPrint("      --> Verdict     : Die Temperature = "); printFloat1(temp_c); uartPrint(" deg C\\r\\n");
            }
            
            // Env Stats
            uartPrint("  * [BME680 Climate]  : T="); printFloat2(bmeTemp); uartPrint(" C | H="); printFloat2(bmeHum); uartPrint(" % | P="); printFloat2(bmePress); uartPrint(" hPa\\r\\n");
            uartPrint("  * [OPT4041 Light]   : "); printFloat2(lux); uartPrint(" Lux\\r\\n");
            uartPrint("  * [VCNL4040 Prox]   : "); printDec(prox); uartPrint(" counts\\r\\n");
            uartPrint("  * [MLX90632 IR]     : Target = "); printFloat2(mlx_obj); uartPrint(" C | Ambient = "); printFloat2(mlx_amb); uartPrint(" C\\r\\n");
            
            // Reset ECG Stats
            ecg_stat_count = 0;
            ecg_ch1_min = 100000.0f; ecg_ch1_max = -100000.0f;
            ecg_ch1_sum = 0.0f;
        }

        usleep(1000); 
    }
    return 0;
}
"""

start_marker = r"/\* =========================================================================\n \* Phase 2 High-Speed ISRs"
match = re.search(start_marker, content)
if match:
    start_idx = match.start()
else:
    # fallback to Main Function
    start_marker = r"/\* =========================================================================\n \* Main Function"
    start_idx = re.search(start_marker, content).start()

new_content = content[:start_idx] + new_code

with open(file_path, "w", encoding="utf-8") as f:
    f.write(new_content)

print("Readable monitor & ECG test modes patched!")
