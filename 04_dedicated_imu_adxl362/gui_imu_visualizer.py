import sys
import time
import threading
import collections
import re
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

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

class ImuVisualizerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SmartBAN ADXL362 Dedicated IMU Real-Time Graphical Dashboard")
        self.geometry("1100x780")
        self.configure(bg="#1e1e2e")

        # Serial state
        self.ser = None
        self.running = True
        self.port = find_xds110_port() or "COM3"

        # Data history buffers (100 samples)
        self.history_len = 100
        self.time_buffer = collections.deque(maxlen=self.history_len)
        self.x_buffer    = collections.deque(maxlen=self.history_len)
        self.y_buffer    = collections.deque(maxlen=self.history_len)
        self.z_buffer    = collections.deque(maxlen=self.history_len)
        self.mag_buffer  = collections.deque(maxlen=self.history_len)

        for i in range(self.history_len):
            self.time_buffer.append(i)
            self.x_buffer.append(0)
            self.y_buffer.append(0)
            self.z_buffer.append(1000)
            self.mag_buffer.append(1000)

        # Current telemetry metrics
        self.cur_x = 0
        self.cur_y = 0
        self.cur_z = 1000
        self.cur_mag = 1000
        self.cur_pitch = 0.0
        self.cur_roll = 0.0
        self.cur_temp = 24.5
        self.cur_int1 = 0
        self.cur_int2 = 1
        self.cur_status = "0x41"
        self.cur_flags = "RDY AWK"

        self.setup_ui()
        self.start_serial_thread()
        self.update_gui_loop()

    def setup_ui(self):
        # Top banner
        header_frame = tk.Frame(self, bg="#181825", pady=8, padx=12)
        header_frame.pack(fill=tk.X)

        title_lbl = tk.Label(header_frame, text="SmartBAN ADXL362 3-Axis IMU Real-Time Visualizer",
                             font=("Segoe UI", 16, "bold"), fg="#cdd6f4", bg="#181825")
        title_lbl.pack(side=tk.LEFT)

        self.conn_lbl = tk.Label(header_frame, text=f"Port: {self.port} (115200 baud) │ Connected",
                                 font=("Segoe UI", 11), fg="#a6e3a1", bg="#181825")
        self.conn_lbl.pack(side=tk.RIGHT)

        # Main horizontal split
        main_content = tk.Frame(self, bg="#1e1e2e", padx=10, pady=10)
        main_content.pack(fill=tk.BOTH, expand=True)

        # Left Column: Matplotlib Strip Chart
        left_col = tk.Frame(main_content, bg="#1e1e2e")
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6.5, 5), dpi=100, facecolor="#1e1e2e")
        self.ax_plot = self.fig.add_subplot(211)
        self.ax_mag  = self.fig.add_subplot(212)

        for ax in [self.ax_plot, self.ax_mag]:
            ax.set_facecolor("#181825")
            ax.tick_params(colors="#cdd6f4", labelsize=8)
            for spine in ax.spines.values():
                spine.set_color("#45475a")
            ax.grid(True, color="#313244", linestyle="--", linewidth=0.5)

        self.ax_plot.set_title("3-Axis Acceleration (mg)", color="#cdd6f4", fontsize=10, pad=6)
        self.ax_plot.set_ylim(-2000, 2000)
        self.line_x, = self.ax_plot.plot([], [], label="X-Axis", color="#f38ba8", linewidth=1.5)
        self.line_y, = self.ax_plot.plot([], [], label="Y-Axis", color="#a6e3a1", linewidth=1.5)
        self.line_z, = self.ax_plot.plot([], [], label="Z-Axis", color="#89b4fa", linewidth=1.5)
        self.ax_plot.legend(loc="upper right", facecolor="#181825", edgecolor="#45475a", labelcolor="#cdd6f4", fontsize=8)

        self.ax_mag.set_title("Vector Magnitude |a| (mg)", color="#cdd6f4", fontsize=10, pad=6)
        self.ax_mag.set_ylim(0, 2500)
        self.line_mag, = self.ax_mag.plot([], [], label="|a| Mag", color="#f9e2af", linewidth=1.5)
        self.ax_mag.axhline(1000, color="#fab387", linestyle=":", linewidth=1, label="1.0g Reference")
        self.ax_mag.legend(loc="upper right", facecolor="#181825", edgecolor="#45475a", labelcolor="#cdd6f4", fontsize=8)

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=left_col)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right Column: Attitude Horizon + Digital Readouts + Controls
        right_col = tk.Frame(main_content, bg="#1e1e2e", width=340, padx=10)
        right_col.pack(side=tk.RIGHT, fill=tk.Y)

        # Artificial Horizon Canvas (160x160)
        horizon_box = tk.LabelFrame(right_col, text="Artificial Horizon / Tilt", font=("Segoe UI", 10, "bold"),
                                   fg="#cdd6f4", bg="#181825", bd=1, relief="solid", padx=8, pady=8)
        horizon_box.pack(fill=tk.X, pady=(0, 10))

        self.horizon_canvas = tk.Canvas(horizon_box, width=180, height=140, bg="#11111b", highlightthickness=0)
        self.horizon_canvas.pack(pady=4)

        # Readout cards
        readout_box = tk.LabelFrame(right_col, text="Live Telemetry Metrics", font=("Segoe UI", 10, "bold"),
                                    fg="#cdd6f4", bg="#181825", bd=1, relief="solid", padx=8, pady=8)
        readout_box.pack(fill=tk.X, pady=(0, 10))

        self.lbl_accel = tk.Label(readout_box, text="X: 0 mg   Y: 0 mg   Z: 1000 mg",
                                  font=("Consolas", 10, "bold"), fg="#89dceb", bg="#181825")
        self.lbl_accel.pack(anchor="w", pady=2)

        self.lbl_attitude = tk.Label(readout_box, text="Pitch: 0.0°   Roll: 0.0°",
                                     font=("Consolas", 10, "bold"), fg="#f9e2af", bg="#181825")
        self.lbl_attitude.pack(anchor="w", pady=2)

        self.lbl_temp = tk.Label(readout_box, text="Die Temp: 24.5 °C (76.1 °F)",
                                 font=("Consolas", 10), fg="#eba0ac", bg="#181825")
        self.lbl_temp.pack(anchor="w", pady=2)

        self.lbl_pins = tk.Label(readout_box, text="INT1 (DIO 26): LOW   INT2 (DIO 27): HIGH",
                                 font=("Consolas", 9), fg="#a6adc8", bg="#181825")
        self.lbl_pins.pack(anchor="w", pady=2)

        self.lbl_status = tk.Label(readout_box, text="Status: 0x41 [RDY AWK]",
                                   font=("Consolas", 9), fg="#fab387", bg="#181825")
        self.lbl_status.pack(anchor="w", pady=2)

        # Interactive Control Buttons
        btn_box = tk.LabelFrame(right_col, text="Sensor Control Deck", font=("Segoe UI", 10, "bold"),
                                fg="#cdd6f4", bg="#181825", bd=1, relief="solid", padx=8, pady=8)
        btn_box.pack(fill=tk.X)

        b_tare = tk.Button(btn_box, text="Zero-G Tare Calibrate [8]", font=("Segoe UI", 9, "bold"),
                           bg="#89b4fa", fg="#11111b", command=lambda: self.send_cmd('8'))
        b_tare.pack(fill=tk.X, pady=3)

        b_test = tk.Button(btn_box, text="Run MEMS Self-Test [2]", font=("Segoe UI", 9, "bold"),
                           bg="#a6e3a1", fg="#11111b", command=lambda: self.send_cmd('2'))
        b_test.pack(fill=tk.X, pady=3)

        b_fifo = tk.Button(btn_box, text="Read FIFO Burst Stream [3]", font=("Segoe UI", 9),
                           bg="#f9e2af", fg="#11111b", command=lambda: self.send_cmd('3'))
        b_fifo.pack(fill=tk.X, pady=3)

        b_motion = tk.Button(btn_box, text="Autonomous Motion Mode [4]", font=("Segoe UI", 9),
                             bg="#cba6f7", fg="#11111b", command=lambda: self.send_cmd('4'))
        b_motion.pack(fill=tk.X, pady=3)

        b_range = tk.Button(btn_box, text="Toggle Range (+/-2g,4g,8g) [5]", font=("Segoe UI", 9),
                            bg="#313244", fg="#cdd6f4", command=lambda: self.send_cmd('5'))
        b_range.pack(fill=tk.X, pady=3)

        b_reset = tk.Button(btn_box, text="Soft Reset Sensor [R]", font=("Segoe UI", 9),
                            bg="#f38ba8", fg="#11111b", command=lambda: self.send_cmd('r'))
        b_reset.pack(fill=tk.X, pady=3)

    def send_cmd(self, char):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(char.encode('ascii'))
            except Exception as e:
                print("Error sending cmd:", e)

    def start_serial_thread(self):
        def reader():
            pattern = re.compile(
                r'X:([-\d]+)mg\s+Y:([-\d]+)mg\s+Z:([-\d]+)mg\s+\|\s+\|a\|:(\d+)mg\s+\|\s+Pitch:([-\d\.]+)\s+deg\s+Roll:([-\d\.]+)\s+deg\s+\|\s+Temp:([-\d\.]+)\s+C.*INT1:(\d+)\s+INT2:(\d+)\s+\|\s+ST:(0x[0-9A-Fa-f]+)\s*\[([^\]]*)\]'
            )
            while self.running:
                if not self.ser or not self.ser.is_open:
                    try:
                        self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                        self.conn_lbl.config(text=f"Port: {self.port} │ Connected", fg="#a6e3a1")
                    except Exception:
                        self.conn_lbl.config(text=f"Port: {self.port} │ Waiting...", fg="#f38ba8")
                        time.sleep(1.0)
                        continue

                try:
                    line = self.ser.readline()
                    if line:
                        text = line.decode('ascii', errors='replace').rstrip()
                        m = pattern.search(text)
                        if m:
                            self.cur_x = int(m.group(1))
                            self.cur_y = int(m.group(2))
                            self.cur_z = int(m.group(3))
                            self.cur_mag = int(m.group(4))
                            self.cur_pitch = float(m.group(5))
                            self.cur_roll = float(m.group(6))
                            self.cur_temp = float(m.group(7))
                            self.cur_int1 = int(m.group(8))
                            self.cur_int2 = int(m.group(9))
                            self.cur_status = m.group(10)
                            self.cur_flags = m.group(11).strip()

                            self.x_buffer.append(self.cur_x)
                            self.y_buffer.append(self.cur_y)
                            self.z_buffer.append(self.cur_z)
                            self.mag_buffer.append(self.cur_mag)
                except Exception:
                    pass

        t = threading.Thread(target=reader, daemon=True)
        t.start()

    def update_gui_loop(self):
        # Update waveforms
        x_data = list(range(len(self.x_buffer)))
        self.line_x.set_data(x_data, list(self.x_buffer))
        self.line_y.set_data(x_data, list(self.y_buffer))
        self.line_z.set_data(x_data, list(self.z_buffer))
        self.line_mag.set_data(x_data, list(self.mag_buffer))
        self.ax_plot.set_xlim(0, self.history_len)
        self.ax_mag.set_xlim(0, self.history_len)
        self.canvas.draw_idle()

        # Update labels
        self.lbl_accel.config(text=f"X: {self.cur_x:+5d} mg   Y: {self.cur_y:+5d} mg   Z: {self.cur_z:+5d} mg")
        self.lbl_attitude.config(text=f"Pitch: {self.cur_pitch:+5.1f}°   Roll: {self.cur_roll:+5.1f}°   |a|: {self.cur_mag} mg")
        tf = self.cur_temp * 1.8 + 32.0
        self.lbl_temp.config(text=f"Die Temp: {self.cur_temp:4.1f} °C ({tf:4.1f} °F)")
        int1_str = "HIGH" if self.cur_int1 else "LOW"
        int2_str = "HIGH" if self.cur_int2 else "LOW"
        self.lbl_pins.config(text=f"INT1 (DIO 26): {int1_str}    INT2 (DIO 27): {int2_str}")
        self.lbl_status.config(text=f"Status: {self.cur_status} [{self.cur_flags}]")

        # Update artificial horizon
        self.draw_horizon(self.cur_pitch, self.cur_roll)

        self.after(50, self.update_gui_loop) # 20 Hz GUI refresh

    def draw_horizon(self, pitch, roll):
        c = self.horizon_canvas
        c.delete("all")
        w, h = 180, 140
        cx, cy = w // 2, h // 2

        # Roll rotation angle (rad)
        rad = np.radians(roll)
        # Pitch vertical displacement
        dy = (pitch / 90.0) * (h / 2.0)

        # Horizon line
        cos_r = np.cos(rad)
        sin_r = np.sin(rad)
        p1 = (cx - 70 * cos_r - dy * sin_r, cy - 70 * sin_r + dy * cos_r)
        p2 = (cx + 70 * cos_r - dy * sin_r, cy + 70 * sin_r + dy * cos_r)

        # Draw sky/ground separation
        c.create_rectangle(0, 0, w, h, fill="#1e1e2e", outline="")
        c.create_line(p1[0], p1[1], p2[0], p2[1], fill="#89b4fa", width=3)

        # Crosshairs / aircraft reference
        c.create_line(cx - 30, cy, cx - 10, cy, fill="#f9e2af", width=2)
        c.create_line(cx + 10, cy, cx + 30, cy, fill="#f9e2af", width=2)
        c.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, outline="#f9e2af", width=2)

        # Pitch scale markings
        c.create_line(cx - 15, cy - 25, cx + 15, cy - 25, fill="#45475a", width=1)
        c.create_line(cx - 15, cy + 25, cx + 15, cy + 25, fill="#45475a", width=1)

if __name__ == '__main__':
    app = ImuVisualizerApp()
    app.mainloop()
