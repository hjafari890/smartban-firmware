with open('main.c', 'r') as f:
    lines = f.readlines()

new_lines = []
for l in lines:
    if 'uartPrint("\n' in l:
        new_lines.append('            uartPrint("\\r\\n");\n')
    elif '// uartPrint("PPG Read failed.\n' in l:
        new_lines.append('            // uartPrint("PPG Read failed.\\r\\n");\n')
    elif '");' in l and new_lines and 'uartPrint("\n' in new_lines[-1]:
        pass # skip
    else:
        new_lines.append(l)

with open('main.c', 'w') as f:
    f.writelines(new_lines)
