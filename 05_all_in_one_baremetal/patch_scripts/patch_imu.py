import re

file_path = r"c:\Users\hjafa\OneDrive\Desktop\shield cc2650\firmware\07_all_in_one_sensor\main.c"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

target = r"    adxl362_write_reg\(0x2D, 0x22\);"
replacement = """    /* Enable DATA_READY interrupt on INT1 pin (Register 0x2A, bit 0) */
    adxl362_write_reg(0x2A, 0x01);

    adxl362_write_reg(0x2D, 0x22);"""

new_content = re.sub(target, replacement, content)

with open(file_path, "w", encoding="utf-8") as f:
    f.write(new_content)

print("IMU interrupt patched!")
