import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Update ads1292_read_sample signature to output status byte as well
old_read_sample = """static bool ads1292_read_sample(int32_t *ch1_out, int32_t *ch2_out)"""
new_read_sample = """static bool ads1292_read_sample(uint8_t *stat_out, int32_t *ch1_out, int32_t *ch2_out)"""

if old_read_sample in content:
    content = content.replace(old_read_sample, new_read_sample)

# Update the assign in ads1292_read_sample
old_assign = """            if (ch1_out) *ch1_out = c1;
            if (ch2_out) *ch2_out = c2;
            return true;"""
new_assign = """            if (stat_out) *stat_out = rx[0];
            if (ch1_out) *ch1_out = c1;
            if (ch2_out) *ch2_out = c2;
            return true;"""

if old_assign in content:
    content = content.replace(old_assign, new_assign)

# 2. Update ads1292_set_test_mode with full clinical settings and OFFSETCAL
old_set_test_mode = """static bool ads1292_set_test_mode(EcgTestMode_t mode)
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
}"""

new_set_test_mode = """static bool ads1292_set_test_mode(EcgTestMode_t mode)
{
    if (!ads_ready && ads_id == 0) return false;

    /* 1. Stop continuous conversion mode to write registers */
    ads1292_send_cmd(ADS1292_CMD_SDATAC); /* 0x11 */
    usleep(1000);

    /* 2. Common configuration: 500 SPS */
    ads1292_write_reg(ADS1292_REG_CONFIG1, 0x02);

    switch (mode) {
        case ECG_MODE_SQUARE_WAVE:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA3); /* Internal 1Hz, +/-1mV test signal, Ref ON */
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x05); /* CH1 Test signal */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x05); /* CH2 Test signal */
            ads1292_write_reg(ADS1292_REG_RLD_SENS,0x00);
            ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x00);
            ads1292_write_reg(ADS1292_REG_RESP1,   0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
            break;

        case ECG_MODE_INPUT_SHORT:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0); /* Test signal OFF, Ref ON */
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x01); /* CH1 Input shorted */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x01); /* CH2 Input shorted */
            ads1292_write_reg(ADS1292_REG_RLD_SENS,0x00);
            ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x00);
            ads1292_write_reg(ADS1292_REG_RESP1,   0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
            break;

        case ECG_MODE_TEMPERATURE:
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xA0);
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x04); /* Internal Temp Sensor */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x04);
            ads1292_write_reg(ADS1292_REG_RLD_SENS,0x00);
            ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x00);
            ads1292_write_reg(ADS1292_REG_RESP1,   0x00);
            ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
            break;

        case ECG_MODE_LIVE_ELECTRODE:
        default:
            /* CONFIG2: Reference ON (bit 5), Lead-off comparator ON (bit 6), Bit 7=1 */
            ads1292_write_reg(ADS1292_REG_CONFIG2, 0xE0);
            /* LOFF: 95%/5% threshold, 6nA DC lead-off current */
            ads1292_write_reg(ADS1292_REG_LOFF,    0x10);
            /* CH1SET: Gain=1 (0x10), Normal input (Respiration) */
            ads1292_write_reg(ADS1292_REG_CH1SET,  0x10);
            /* CH2SET: Gain=6 (0x00), Normal input (ECG Lead I) */
            ads1292_write_reg(ADS1292_REG_CH2SET,  0x00);
            /* RLD_SENS: 0x2C (RLD buffer ON, PGA chop fmod/16, route CH2 IN2P+IN2N to RLD) */
            ads1292_write_reg(ADS1292_REG_RLD_SENS,0x2C);
            /* LOFF_SENS: 0x03 (Enable lead-off sensing on CH2 IN2P and IN2N) */
            ads1292_write_reg(ADS1292_REG_LOFF_SENS, 0x03);
            /* RESP1: 0xEA (Demod ON, Mod ON, 112.5 deg phase, internal 32kHz clock) */
            ads1292_write_reg(ADS1292_REG_RESP1,   0xEA);
            /* RESP2: 0x83 (Calib ON, 32kHz resp freq, internal RLDREF) */
            ads1292_write_reg(ADS1292_REG_RESP2,   0x83);
            break;
    }

    /* 3. Run Internal Offset Calibration (ADS1292R OFFSETCAL 0x1A) */
    GPIO_write(CONFIG_GPIO_ECG_START, 1);
    usleep(20000);
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);
    ads1292_send_cmd(ADS1292_CMD_OFFSETCAL);
    usleep(250000); /* Wait for offset calibration cycle to complete */
    ads1292_send_cmd(ADS1292_CMD_SDATAC);
    usleep(1000);

    /* 4. Re-enter continuous data mode (RDATAC 0x10) */
    ads1292_send_cmd(ADS1292_CMD_RDATAC);
    usleep(10000);

    ecg_test_mode = mode;
    return true;
}"""

if old_set_test_mode in content:
    content = content.replace(old_set_test_mode, new_set_test_mode)
    print("Updated ads1292_set_test_mode successfully!")
else:
    print("Warning: old_set_test_mode not found exactly, using regex...")
    content = re.sub(r'static bool ads1292_set_test_mode\(EcgTestMode_t mode\)\s*\{.*?return true;\s*\}', new_set_test_mode, content, flags=re.DOTALL)

# 3. Update ECG streaming in main loop: 250 Hz with compact JSON format {"e":[stat,c1,c2]}
old_ecg_stream = """        if (flag_ecg_drdy) {
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
        }"""

new_ecg_stream = """        if (flag_ecg_drdy) {
            flag_ecg_drdy = false;
            uint8_t stat = 0;
            int32_t ecg1 = 0, ecg2 = 0;
            if (ads1292_read_sample(&stat, &ecg1, &ecg2)) {
                ecg_sample_div++;
                /* Send every 2nd sample = 250 Hz clinical stream */
                if (ecg_sample_div & 1) {
                    uartPrint("{\\"e\\":[");
                    printDec((uint32_t)stat);
                    uartPrint(",");
                    printInt(ecg1);
                    uartPrint(",");
                    printInt(ecg2);
                    uartPrint("]}\\r\\n");
                }
            }
        }"""

if old_ecg_stream in content:
    content = content.replace(old_ecg_stream, new_ecg_stream)
    print("Updated ECG streaming loop to 250 Hz compact format successfully!")
else:
    print("Warning: old_ecg_stream not found exactly, using regex...")
    content = re.sub(r'if \(flag_ecg_drdy\) \{.*?uartPrint\("\\"\}\\\\r\\\\n"\);\s*\}\s*\}', new_ecg_stream, content, flags=re.DOTALL)

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("main.c patched with 250 Hz clinical ECG and lead-off!")
