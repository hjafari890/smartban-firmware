import re

with open('main.c', 'r') as f:
    text = f.read()

new_loop = """while (1) {
        BioData_t bio;
        memset(&bio, 0, sizeof(bio));
        bool bioOk = false;
        
        if (maxm86161_direct_mode) {
            bioOk = maxm86161_direct_read_biometrics(&bio);
        } else {
            bioOk = max32664_read_biometrics(&bio);
        }

        if (bioOk) {
            uartPrint("PPG Data - Green: "); printDec(bio.greenRaw);
            uartPrint(", IR: "); printDec(bio.irRaw);
            uartPrint(", Red: "); printDec(bio.redRaw);
            uartPrint("\\r\\n");
        } else {
            // uartPrint("PPG Read failed.\\r\\n");
        }
        usleep(100000);
    }
}
"""

text = re.sub(r'while\s*\(1\)\s*\{.*$', new_loop, text, flags=re.DOTALL)

inits = [
    'ch455_init();',
    'pcal6408_init();',
    'adxl362_init();',
    'ads1292_init();',
    'bme680_load_calibration();',
    'opt4041_init();',
    'vcnl4040_init();',
    'mlx90632_init();'
]
for init in inits:
    text = text.replace(init, '// ' + init)

with open('main.c', 'w') as f:
    f.write(text)

print("Patch applied.")
