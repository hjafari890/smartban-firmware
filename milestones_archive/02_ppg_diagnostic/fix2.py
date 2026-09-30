import re

with open('main.c', 'r') as f:
    text = f.read()

# I want to find: uartPrint("PPG Data - Green: "); printDec(bio.greenRaw);
# And replace the whole block carefully.
text = re.sub(r'while\s*\(1\)\s*\{.*$', '', text, flags=re.DOTALL)

loop = """while (1) {
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

text += loop

with open('main.c', 'w') as f:
    f.write(text)
