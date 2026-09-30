import re

with open('main.c', 'r') as f:
    text = f.read()

text = re.sub(r'while\s*\(1\)\s*\{.*$', '', text, flags=re.DOTALL)

loop = """
    uartPrint("\\r\\nScanning I2C Bus...\\r\\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        I2C_Transaction trans;
        memset(&trans, 0, sizeof(trans));
        uint8_t tx = 0;
        trans.targetAddress = addr;
        trans.writeBuf = &tx;
        trans.writeCount = 1;
        trans.readBuf = NULL;
        trans.readCount = 0;
        if (I2C_transfer(i2c, &trans)) {
            uartPrint("Found I2C device at 0x");
            printHex8(addr);
            uartPrint("\\r\\n");
        }
    }
    uartPrint("Scan complete.\\r\\n");

    while (1) {
        usleep(1000000);
    }
}
"""

with open('main.c', 'w') as f:
    f.write(text + loop)
