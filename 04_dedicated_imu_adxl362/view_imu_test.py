import serial
import serial.tools.list_ports
import time
import sys
import os
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

import msvcrt
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.layout import Layout
from rich.text import Text

console = Console(force_terminal=True)

def find_xds110_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "XDS110" in p.description and "Application/User" in p.description:
            return p.device
        if "XDS110" in p.description:
            return p.device
    for p in ports:
        if "COM3" == p.device:
            return "COM3"
    return None

def make_bar(val, min_val=-2000, max_val=2000, width=24, color="cyan"):
    val = max(min_val, min(max_val, val))
    center = width // 2
    fraction = (val - min_val) / (max_val - min_val)
    pos = int(fraction * width)
    pos = max(0, min(width - 1, pos))

    chars = [" "] * width
    chars[center] = "|"

    if pos < center:
        for i in range(pos, center):
            chars[i] = "="
    elif pos > center:
        for i in range(center + 1, pos + 1):
            chars[i] = "="
    chars[pos] = "O"

    bar_str = "".join(chars)
    return f"[{color}][{bar_str}][/{color}]"

def make_horizon_indicator(pitch, roll, width=20, height=5):
    p_clamped = max(-90.0, min(90.0, pitch))
    r_clamped = max(-90.0, min(90.0, roll))

    x_offset = int((r_clamped / 90.0) * (width // 2))
    y_offset = int((-p_clamped / 90.0) * (height // 2))

    bubble_x = (width // 2) + x_offset
    bubble_y = (height // 2) + y_offset
    bubble_x = max(1, min(width - 2, bubble_x))
    bubble_y = max(0, min(height - 1, bubble_y))

    lines = []
    for y in range(height):
        row = ["."] * width
        if y == height // 2:
            row[width // 2] = "+"
        if y == bubble_y:
            row[bubble_x] = "@"
        lines.append("".join(row))
    return "\n".join(lines)

def build_dashboard(data, status_msg="Streaming"):
    layout = Layout()
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="main", size=14),
        Layout(name="footer", size=5)
    )
    layout["main"].split_row(
        Layout(name="left", ratio=1),
        Layout(name="right", ratio=1)
    )

    title = Text(" SmartBAN ADXL362 MEMS Accelerometer Telemetry Dashboard ", style="bold white on blue")
    subtitle = Text(f" Target: CC2652R1 / BAN Shield V3.5  |  Status: {status_msg}  |  Port: COM3 @ 115200 baud ", style="dim")
    layout["header"].update(Panel(Text.assemble(title, "\n", subtitle), border_style="blue"))

    accel_table = Table.grid(padding=(0, 1))
    accel_table.add_column("Axis", justify="right", style="bold")
    accel_table.add_column("Value", justify="right", style="bold white")
    accel_table.add_column("Bar", justify="left")

    ax = data.get('ax', 0)
    ay = data.get('ay', 0)
    az = data.get('az', 0)
    mag = data.get('mag', 0)

    accel_table.add_row("[red]X-Axis[/red]", f"{ax:+5d} mg", make_bar(ax, color="red"))
    accel_table.add_row("[green]Y-Axis[/green]", f"{ay:+5d} mg", make_bar(ay, color="green"))
    accel_table.add_row("[blue]Z-Axis[/blue]", f"{az:+5d} mg", make_bar(az, color="blue"))
    accel_table.add_row("", "", "")

    mag_bar = make_bar(mag, min_val=0, max_val=2500, width=24, color="bright_yellow")
    accel_table.add_row("[yellow]|a| Mag[/yellow]", f"{mag:5d} mg", mag_bar)

    left_panel = Panel(accel_table, title="[bold]3-Axis Dynamic Acceleration[/bold]", border_style="cyan")
    layout["left"].update(left_panel)

    pitch = data.get('pitch', 0.0)
    roll = data.get('roll', 0.0)
    temp = data.get('temp', 24.5)
    temp_f = temp * 1.8 + 32.0

    tilt_table = Table.grid(padding=(0, 1))
    tilt_table.add_column("Param", justify="right", style="bold")
    tilt_table.add_column("Val", justify="left")

    tilt_table.add_row("Pitch (deg):", f"[bold yellow]{pitch:+6.1f} deg[/bold yellow]  {make_bar(int(pitch), -90, 90, 16, 'yellow')}")
    tilt_table.add_row("Roll  (deg):", f"[bold magenta]{roll:+6.1f} deg[/bold magenta]  {make_bar(int(roll), -90, 90, 16, 'magenta')}")
    tilt_table.add_row("", "")
    tilt_table.add_row("Horizon    :", f"[green]{make_horizon_indicator(pitch, roll)}[/green]")
    tilt_table.add_row("Die Temp   :", f"[bold red]{temp:4.1f} C[/bold red] ({temp_f:4.1f} F)")

    right_panel = Panel(tilt_table, title="[bold]Spatial Attitude & Temperature[/bold]", border_style="green")
    layout["right"].update(right_panel)

    int1 = data.get('int1', 0)
    int2 = data.get('int2', 1)
    status_hex = data.get('status_hex', "0x41")
    flags = data.get('flags', "RDY AWK")

    footer_table = Table.grid(padding=(0, 2))
    footer_table.add_column(justify="left")
    footer_table.add_column(justify="left")

    int1_badge = "[bold green][ HIGH ][/bold green]" if int1 else "[dim][ LOW  ][/dim]"
    int2_badge = "[bold green][ HIGH ][/bold green]" if int2 else "[dim][ LOW  ][/dim]"

    hardware_line = f"Hardware Pins: INT1 (DIO 26): {int1_badge}  |  INT2 (DIO 27): {int2_badge}  |  STATUS: [bold cyan]{status_hex}[/bold cyan] [{flags}]"
    controls_line = "[bold white]Controls:[/bold white] [1] Stream  [2] Self-Test  [3] FIFO  [4] Motion Detect  [5] Range  [8] Tare  [9] Regs  [R] Reset  [Q] Exit"

    footer_table.add_row(hardware_line)
    footer_table.add_row(controls_line)

    layout["footer"].update(Panel(footer_table, border_style="magenta"))
    return layout

def main():
    port = find_xds110_port()
    if len(sys.argv) > 1:
        port = sys.argv[1]

    if not port:
        console.print("[red][ERROR] Could not automatically find XDS110 COM port.[/red]")
        sys.exit(1)

    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except Exception as e:
        console.print(f"[red][ERROR] Could not open {port}: {e}[/red]")
        sys.exit(1)

    console.clear()
    ser.reset_input_buffer()

    current_data = {
        'ax': -985, 'ay': 152, 'az': 616, 'mag': 1171,
        'pitch': -57.2, 'roll': 7.5, 'temp': 24.5,
        'int1': 0, 'int2': 1, 'status_hex': "0x41", 'flags': "RDY AWK"
    }

    pattern = re.compile(
        r'X:([-\d]+)mg\s+Y:([-\d]+)mg\s+Z:([-\d]+)mg\s+\|\s+\|a\|:(\d+)mg\s+\|\s+Pitch:([-\d\.]+)\s+deg\s+Roll:([-\d\.]+)\s+deg\s+\|\s+Temp:([-\d\.]+)\s+C.*INT1:(\d+)\s+INT2:(\d+)\s+\|\s+ST:(0x[0-9A-Fa-f]+)\s*\[([^\]]*)\]'
    )

    with Live(build_dashboard(current_data), refresh_per_second=15, console=console, screen=True) as live:
        try:
            while True:
                if msvcrt.kbhit():
                    ch = msvcrt.getch().decode('ascii', errors='ignore').lower()
                    if ch == 'q':
                        break
                    elif ch in ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'r', 'm']:
                        ser.write(ch.encode('ascii'))

                line = ser.readline()
                if line:
                    text = line.decode('ascii', errors='replace').rstrip()
                    m = pattern.search(text)
                    if m:
                        current_data['ax'] = int(m.group(1))
                        current_data['ay'] = int(m.group(2))
                        current_data['az'] = int(m.group(3))
                        current_data['mag'] = int(m.group(4))
                        current_data['pitch'] = float(m.group(5))
                        current_data['roll'] = float(m.group(6))
                        current_data['temp'] = float(m.group(7))
                        current_data['int1'] = int(m.group(8))
                        current_data['int2'] = int(m.group(9))
                        current_data['status_hex'] = m.group(10)
                        current_data['flags'] = m.group(11).strip()
                        live.update(build_dashboard(current_data, status_msg="Live 10Hz Telemetry"))
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()

if __name__ == '__main__':
    main()
