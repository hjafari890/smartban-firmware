# -*- coding: utf-8 -*-
"""
================================================================================
SmartBAN ADXL362 Dedicated 3D Real-Time IMU Indicator & Telemetry Dashboard
Platform: TI CC2652R1 LaunchPad + BAN Shield V3.5 + ADXL362 Accelerometer
================================================================================
"""

import os
import sys
import math
import time
import threading
import re
import collections

try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except Exception:
    pass

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

# ------------------------------------------------------------------------------
# 3D Math Engine (Vectors, Matrices, Projections, Lighting)
# ------------------------------------------------------------------------------

def mat_mult_vec(M, v):
    """Multiply 3x3 matrix M by 3D vector v."""
    return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]
    ]

def mat_mult_mat(A, B):
    """Multiply 3x3 matrix A by 3x3 matrix B."""
    C = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
    return C

def euler_to_matrix(pitch_deg, roll_deg, yaw_deg=0.0):
    """
    Rotation matrix for board spatial attitude.
    Pitch: rotation about Y-axis (nose up/down)
    Roll: rotation about X-axis (bank left/right)
    Yaw: rotation about Z-axis (heading)
    """
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    y = math.radians(yaw_deg)

    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    cy, sy = math.cos(y), math.sin(y)

    # R = Rz(y) * Ry(p) * Rx(r)
    return [
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ]

def camera_matrix(azimuth_deg, elevation_deg):
    """
    View matrix from camera spherical angles.
    Azimuth: angle around vertical axis (deg).
    Elevation: angle above ground plane (deg).
    """
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)

    c_az, s_az = math.cos(az), math.sin(az)
    c_el, s_el = math.cos(el), math.sin(el)

    # Transform world coords into camera view space
    R_az = [
        [ c_az,  s_az, 0.0],
        [-s_az,  c_az, 0.0],
        [  0.0,   0.0, 1.0]
    ]
    R_el = [
        [1.0,   0.0,   0.0],
        [0.0,  c_el,  s_el],
        [0.0, -s_el,  c_el]
    ]
    return mat_mult_mat(R_el, R_az)

def hex_to_rgb(hex_str):
    hex_str = hex_str.lstrip('#')
    return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))

def rgb_to_hex(rgb):
    r, g, b = [max(0, min(255, int(c))) for c in rgb]
    return f"#{r:02x}{g:02x}{b:02x}"

def shade_color(base_hex, factor):
    r, g, b = hex_to_rgb(base_hex)
    factor = max(0.20, min(1.45, factor))
    return rgb_to_hex((r * factor, g * factor, b * factor))

def vector_cross(u, v):
    return [
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0]
    ]

def vector_dot(u, v):
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]

def vector_norm(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

def vector_normalize(v):
    n = vector_norm(v)
    if n < 1e-9:
        return [0.0, 0.0, 1.0]
    return [v[0]/n, v[1]/n, v[2]/n]

# ------------------------------------------------------------------------------
# Serial Port Detection
# ------------------------------------------------------------------------------

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
    if ports:
        return ports[0].device
    return "COM3"

def list_all_ports():
    ports = serial.tools.list_ports.comports()
    names = [p.device for p in ports]
    if "COM3" not in names:
        names.append("COM3")
    return names

# ------------------------------------------------------------------------------
# 3D Scene Geometry (Board Model, Chips, Pins, Ground Horizon)
# ------------------------------------------------------------------------------

class Model3D:
    """Pre-computes the 3D polygon faces and lines for the PCB assembly."""
    def __init__(self):
        self.faces = []      # list of (vertices_local, base_color, outline_color, label)
        self.lines = []      # list of (p1_local, p2_local, color, width)
        self.build_model()

    def add_box(self, center, size, col_top, col_side, col_bot, outline="#334155"):
        cx, cy, cz = center
        dx, dy, dz = size[0]/2.0, size[1]/2.0, size[2]/2.0

        v0 = [cx - dx, cy - dy, cz - dz]
        v1 = [cx + dx, cy - dy, cz - dz]
        v2 = [cx + dx, cy + dy, cz - dz]
        v3 = [cx - dx, cy + dy, cz - dz]
        v4 = [cx - dx, cy - dy, cz + dz]
        v5 = [cx + dx, cy - dy, cz + dz]
        v6 = [cx + dx, cy + dy, cz + dz]
        v7 = [cx - dx, cy + dy, cz + dz]

        # Top face (+Z)
        self.faces.append(([v4, v5, v6, v7], col_top, outline, None))
        # Bottom face (-Z)
        self.faces.append(([v3, v2, v1, v0], col_bot, outline, None))
        # Front face (+X)
        self.faces.append(([v1, v5, v6, v2], col_side, outline, None))
        # Back face (-X)
        self.faces.append(([v0, v3, v7, v4], col_side, outline, None))
        # Right face (+Y)
        self.faces.append(([v2, v6, v7, v3], col_side, outline, None))
        # Left face (-Y)
        self.faces.append(([v0, v4, v5, v1], col_side, outline, None))

    def build_model(self):
        # 1. Main PCB Board: 154mm x 96mm x 6mm
        # Color: Deep Emerald Green Solder Mask
        self.add_box([0, 0, 0], [154, 96, 6], "#0f5132", "#0a3622", "#051b11", outline="#198754")

        # 2. ADXL362 MEMS Accelerometer IC Package: 26mm x 26mm x 4mm in center
        self.add_box([0, 0, 5], [26, 26, 4], "#212529", "#15171a", "#15171a", outline="#495057")

        # Pin 1 orientation indicator dot on ADXL362 top face
        self.faces.append(([
            [7, 7, 7.1], [10, 7, 7.1], [10, 10, 7.1], [7, 10, 7.1]
        ], "#f59e0b", "#d97706", None))

        # 3. CC2652R Microcontroller Chip: 28mm x 28mm x 3.5mm
        self.add_box([-44, 0, 4.75], [28, 28, 3.5], "#1e293b", "#0f172a", "#0f172a", outline="#334155")

        # 4. Micro-USB Jack: 10mm x 16mm x 6mm at board edge
        self.add_box([-80, 0, 2], [8, 16, 6], "#cbd5e1", "#94a3b8", "#64748b", outline="#64748b")

        # 5. Dual BoosterPack Header Rows (along top and bottom edges)
        # Top Row (+Y)
        self.add_box([0, 41, 6], [134, 8, 6], "#1c1917", "#0c0a09", "#0c0a09", outline="#44403c")
        for x in range(-55, 65, 11):
            self.faces.append(([
                [x - 1.5, 41 - 1.5, 9.1],
                [x + 1.5, 41 - 1.5, 9.1],
                [x + 1.5, 41 + 1.5, 9.1],
                [x - 1.5, 41 + 1.5, 9.1]
            ], "#eab308", "#ca8a04", None))

        # Bottom Row (-Y)
        self.add_box([0, -41, 6], [134, 8, 6], "#1c1917", "#0c0a09", "#0c0a09", outline="#44403c")
        for x in range(-55, 65, 11):
            self.faces.append(([
                [x - 1.5, -41 - 1.5, 9.1],
                [x + 1.5, -41 - 1.5, 9.1],
                [x + 1.5, -41 + 1.5, 9.1],
                [x - 1.5, -41 + 1.5, 9.1]
            ], "#eab308", "#ca8a04", None))

        # 6. Corner Mounting Holes
        for hx, hy in [(-70, -40), (-70, 40), (70, -40), (70, 40)]:
            self.faces.append(([
                [hx-3, hy-3, 3.1], [hx+3, hy-3, 3.1],
                [hx+3, hy+3, 3.1], [hx-3, hy+3, 3.1]
            ], "#334155", "#1e293b", None))

        # 7. Surface Mount LEDs (Green Power, Blue RF)
        self.faces.append(([
            [56, 26, 3.2], [60, 26, 3.2], [60, 30, 3.2], [56, 30, 3.2]
        ], "#22c55e", "#16a34a", None))
        self.faces.append(([
            [56, 18, 3.2], [60, 18, 3.2], [60, 22, 3.2], [56, 22, 3.2]
        ], "#38bdf8", "#0284c7", None))

        # 8. Silkscreen Border Lines
        self.lines.append(([-72, -44, 3.1], [72, -44, 3.1], "#86efac", 1))
        self.lines.append(([-72,  44, 3.1], [72,  44, 3.1], "#86efac", 1))
        self.lines.append(([-72, -44, 3.1], [-72,  44, 3.1], "#86efac", 1))
        self.lines.append(([ 72, -44, 3.1], [ 72,  44, 3.1], "#86efac", 1))

# ------------------------------------------------------------------------------
# Main Application Class
# ------------------------------------------------------------------------------

class Imu3dApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SmartBAN ADXL362 3D IMU Indicator & Telemetry Dashboard")
        self.geometry("1200x820")
        self.minsize(1050, 720)
        self.configure(bg="#0f172a")

        # 3D Camera State
        self.cam_azimuth   = -38.0   # Horizontal orbit angle
        self.cam_elevation = 26.0    # Vertical elevation angle
        self.cam_distance  = 420.0   # Zoom distance
        self.cam_pan_x     = 0.0     # Horizontal pan
        self.cam_pan_y     = 0.0     # Vertical pan

        # Mouse Tracking
        self.last_mouse_x  = 0
        self.last_mouse_y  = 0
        self.mouse_btn_down = None

        # Live Physical IMU Telemetry (Smoothed Target)
        self.target_pitch = 0.0
        self.target_roll  = 0.0
        self.target_ax    = 0.0
        self.target_ay    = 0.0
        self.target_az    = 1000.0
        self.target_mag   = 1000.0
        self.target_temp  = 24.5
        self.cur_int1     = 0
        self.cur_int2     = 1
        self.cur_status   = "0x41"
        self.cur_flags    = "RDY AWK"

        # Current Rendered Values (Interpolated for 60 FPS silky smooth motion)
        self.disp_pitch = 0.0
        self.disp_roll  = 0.0
        self.disp_ax    = 0.0
        self.disp_ay    = 0.0
        self.disp_az    = 1000.0
        self.disp_mag   = 1000.0

        # Operational Modes
        self.demo_mode    = False
        self.demo_t       = 0.0
        self.running      = True
        self.connected    = False
        self.ser          = None
        self.port         = find_xds110_port()

        # Build 3D PCB Geometry Model
        self.model = Model3D()

        # UI Setup
        self.setup_ui()

        # Start Background Serial Thread
        self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.serial_thread.start()

        # Start 60 FPS Render Loop
        self.last_render_time = time.time()
        self.render_loop()

    # --------------------------------------------------------------------------
    # UI Layout & Widget Creation
    # --------------------------------------------------------------------------
    def setup_ui(self):
        # 1. Top Header Banner
        header = tk.Frame(self, bg="#020617", pady=10, padx=16, bd=0)
        header.pack(fill=tk.X)

        title_frame = tk.Frame(header, bg="#020617")
        title_frame.pack(side=tk.LEFT)

        title_main = tk.Label(title_frame, text="SmartBAN ADXL362",
                              font=("Segoe UI", 16, "bold"), fg="#38bdf8", bg="#020617")
        title_main.pack(side=tk.LEFT)

        title_sub = tk.Label(title_frame, text="  3D Spatial Attitude & Telemetry Indicator",
                             font=("Segoe UI", 14), fg="#e2e8f0", bg="#020617")
        title_sub.pack(side=tk.LEFT)

        info_frame = tk.Frame(header, bg="#020617")
        info_frame.pack(side=tk.RIGHT)

        self.lbl_target = tk.Label(info_frame, text="Target: CC2652R1 / BAN V3.5 | 115200 Baud",
                                   font=("Segoe UI", 10), fg="#94a3b8", bg="#020617")
        self.lbl_target.pack(side=tk.RIGHT, padx=10)

        self.lbl_conn_status = tk.Label(info_frame, text="[ CONNECTED ]",
                                        font=("Segoe UI", 10, "bold"), fg="#4ade80", bg="#020617")
        self.lbl_conn_status.pack(side=tk.RIGHT)

        # 2. Main Body (Horizontal Split)
        main_body = tk.Frame(self, bg="#0f172a", padx=10, pady=10)
        main_body.pack(fill=tk.BOTH, expand=True)

        # Left / Center Area: 3D Viewport Frame
        viewport_frame = tk.Frame(main_body, bg="#1e293b", bd=1, relief="solid")
        viewport_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Viewport Header Toolbar
        vp_toolbar = tk.Frame(viewport_frame, bg="#0f172a", padx=8, pady=6)
        vp_toolbar.pack(fill=tk.X)

        vp_title = tk.Label(vp_toolbar, text="3D REAL-TIME ORIENTATION & GRAVITY VECTOR",
                            font=("Segoe UI", 10, "bold"), fg="#38bdf8", bg="#0f172a")
        vp_title.pack(side=tk.LEFT)

        # Quick Camera Angle Preset Buttons
        btn_iso = tk.Button(vp_toolbar, text="Isometric", font=("Segoe UI", 8), bg="#334155", fg="#f8fafc",
                            relief="flat", padx=6, command=lambda: self.set_camera(-38, 26, 420))
        btn_iso.pack(side=tk.RIGHT, padx=2)

        btn_top = tk.Button(vp_toolbar, text="Top (XY)", font=("Segoe UI", 8), bg="#334155", fg="#f8fafc",
                            relief="flat", padx=6, command=lambda: self.set_camera(0, 89, 420))
        btn_top.pack(side=tk.RIGHT, padx=2)

        btn_front = tk.Button(vp_toolbar, text="Front (YZ)", font=("Segoe UI", 8), bg="#334155", fg="#f8fafc",
                              relief="flat", padx=6, command=lambda: self.set_camera(0, 0, 420))
        btn_front.pack(side=tk.RIGHT, padx=2)

        btn_side = tk.Button(vp_toolbar, text="Side (XZ)", font=("Segoe UI", 8), bg="#334155", fg="#f8fafc",
                             relief="flat", padx=6, command=lambda: self.set_camera(90, 0, 420))
        btn_side.pack(side=tk.RIGHT, padx=2)

        btn_reset = tk.Button(vp_toolbar, text="Reset Cam", font=("Segoe UI", 8, "bold"), bg="#0284c7", fg="#ffffff",
                              relief="flat", padx=6, command=self.reset_camera)
        btn_reset.pack(side=tk.RIGHT, padx=4)

        # 3D Rendering Canvas
        self.canvas = tk.Canvas(viewport_frame, bg="#020617", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Bind Mouse Interactions to Canvas
        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down_1)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag_1)
        self.canvas.bind("<ButtonPress-3>", self.on_mouse_down_3)
        self.canvas.bind("<B3-Motion>", self.on_mouse_drag_3)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

        # Right Column Area: Digital Cockpit Instruments & Control Deck
        sidebar = tk.Frame(main_body, bg="#0f172a", width=380, padx=6)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)
        sidebar.pack_propagate(False)

        # Card 1: Connection & Demo Controls
        box_conn = tk.LabelFrame(sidebar, text="Connection & Test Controls", font=("Segoe UI", 9, "bold"),
                                 fg="#94a3b8", bg="#1e293b", bd=1, relief="solid", padx=8, pady=8)
        box_conn.pack(fill=tk.X, pady=(0, 8))

        row_c1 = tk.Frame(box_conn, bg="#1e293b")
        row_c1.pack(fill=tk.X)

        tk.Label(row_c1, text="Port:", font=("Segoe UI", 9), fg="#cbd5e1", bg="#1e293b").pack(side=tk.LEFT)

        self.cb_ports = ttk.Combobox(row_c1, values=list_all_ports(), width=9, state="readonly")
        self.cb_ports.set(self.port)
        self.cb_ports.pack(side=tk.LEFT, padx=6)
        self.cb_ports.bind("<<ComboboxSelected>>", self.on_port_changed)

        self.btn_conn = tk.Button(row_c1, text="Reconnect", font=("Segoe UI", 8, "bold"),
                                  bg="#0284c7", fg="#ffffff", relief="flat", padx=8, pady=2,
                                  command=self.reconnect_serial)
        self.btn_conn.pack(side=tk.LEFT, padx=2)

        self.btn_demo = tk.Button(row_c1, text="Demo Mode: OFF", font=("Segoe UI", 8, "bold"),
                                  bg="#475569", fg="#f8fafc", relief="flat", padx=6, pady=2,
                                  command=self.toggle_demo_mode)
        self.btn_demo.pack(side=tk.RIGHT)

        # Card 2: Primary Flight Attitude Instruments (Pitch, Roll, |a| Mag)
        box_att = tk.LabelFrame(sidebar, text="Spatial Attitude & Vector Magnitude", font=("Segoe UI", 9, "bold"),
                                fg="#38bdf8", bg="#1e293b", bd=1, relief="solid", padx=10, pady=8)
        box_att.pack(fill=tk.X, pady=(0, 8))

        # Pitch Readout & Meter
        p_frame = tk.Frame(box_att, bg="#1e293b")
        p_frame.pack(fill=tk.X, pady=2)
        tk.Label(p_frame, text="PITCH (θ):", font=("Segoe UI", 10, "bold"), fg="#e2e8f0", bg="#1e293b").pack(side=tk.LEFT)
        self.lbl_pitch_val = tk.Label(p_frame, text="+0.0°", font=("Consolas", 14, "bold"), fg="#38bdf8", bg="#1e293b")
        self.lbl_pitch_val.pack(side=tk.RIGHT)

        self.pitch_bar = tk.Canvas(box_att, height=12, bg="#0f172a", highlightthickness=1, highlightbackground="#334155")
        self.pitch_bar.pack(fill=tk.X, pady=(0, 6))

        # Roll Readout & Meter
        r_frame = tk.Frame(box_att, bg="#1e293b")
        r_frame.pack(fill=tk.X, pady=2)
        tk.Label(r_frame, text="ROLL  (φ):", font=("Segoe UI", 10, "bold"), fg="#e2e8f0", bg="#1e293b").pack(side=tk.LEFT)
        self.lbl_roll_val = tk.Label(r_frame, text="+0.0°", font=("Consolas", 14, "bold"), fg="#ec4899", bg="#1e293b")
        self.lbl_roll_val.pack(side=tk.RIGHT)

        self.roll_bar = tk.Canvas(box_att, height=12, bg="#0f172a", highlightthickness=1, highlightbackground="#334155")
        self.roll_bar.pack(fill=tk.X, pady=(0, 6))

        # Vector Magnitude Readout
        m_frame = tk.Frame(box_att, bg="#1e293b")
        m_frame.pack(fill=tk.X, pady=2)
        tk.Label(m_frame, text="|a| TOTAL:", font=("Segoe UI", 10, "bold"), fg="#e2e8f0", bg="#1e293b").pack(side=tk.LEFT)
        self.lbl_mag_val = tk.Label(m_frame, text="1.00 g (1000 mg)", font=("Consolas", 12, "bold"), fg="#facc15", bg="#1e293b")
        self.lbl_mag_val.pack(side=tk.RIGHT)

        self.lbl_mag_state = tk.Label(box_att, text="* 1.0g Standard Earth Gravity", font=("Segoe UI", 8),
                                      fg="#4ade80", bg="#1e293b", anchor="w")
        self.lbl_mag_state.pack(fill=tk.X)

        # Card 3: 3-Axis Dynamic Acceleration Breakdown (mg)
        box_acc = tk.LabelFrame(sidebar, text="3-Axis Dynamic Acceleration (mg)", font=("Segoe UI", 9, "bold"),
                                fg="#94a3b8", bg="#1e293b", bd=1, relief="solid", padx=10, pady=8)
        box_acc.pack(fill=tk.X, pady=(0, 8))

        # X Axis
        row_x = tk.Frame(box_acc, bg="#1e293b")
        row_x.pack(fill=tk.X, pady=1)
        tk.Label(row_x, text="X-Axis (+X):", font=("Segoe UI", 9, "bold"), fg="#ef4444", bg="#1e293b", width=11, anchor="w").pack(side=tk.LEFT)
        self.lbl_x_val = tk.Label(row_x, text="    0 mg", font=("Consolas", 10, "bold"), fg="#ef4444", bg="#1e293b", width=10, anchor="e")
        self.lbl_x_val.pack(side=tk.RIGHT)
        self.bar_x = tk.Canvas(box_acc, height=8, bg="#0f172a", highlightthickness=0)
        self.bar_x.pack(fill=tk.X, pady=(0, 4))

        # Y Axis
        row_y = tk.Frame(box_acc, bg="#1e293b")
        row_y.pack(fill=tk.X, pady=1)
        tk.Label(row_y, text="Y-Axis (+Y):", font=("Segoe UI", 9, "bold"), fg="#22c55e", bg="#1e293b", width=11, anchor="w").pack(side=tk.LEFT)
        self.lbl_y_val = tk.Label(row_y, text="    0 mg", font=("Consolas", 10, "bold"), fg="#22c55e", bg="#1e293b", width=10, anchor="e")
        self.lbl_y_val.pack(side=tk.RIGHT)
        self.bar_y = tk.Canvas(box_acc, height=8, bg="#0f172a", highlightthickness=0)
        self.bar_y.pack(fill=tk.X, pady=(0, 4))

        # Z Axis
        row_z = tk.Frame(box_acc, bg="#1e293b")
        row_z.pack(fill=tk.X, pady=1)
        tk.Label(row_z, text="Z-Axis (+Z):", font=("Segoe UI", 9, "bold"), fg="#38bdf8", bg="#1e293b", width=11, anchor="w").pack(side=tk.LEFT)
        self.lbl_z_val = tk.Label(row_z, text="+1000 mg", font=("Consolas", 10, "bold"), fg="#38bdf8", bg="#1e293b", width=10, anchor="e")
        self.lbl_z_val.pack(side=tk.RIGHT)
        self.bar_z = tk.Canvas(box_acc, height=8, bg="#0f172a", highlightthickness=0)
        self.bar_z.pack(fill=tk.X, pady=(0, 4))

        # Card 4: Environmental & Interrupt Diagnostics
        box_diag = tk.LabelFrame(sidebar, text="Hardware Pins & Environment", font=("Segoe UI", 9, "bold"),
                                 fg="#94a3b8", bg="#1e293b", bd=1, relief="solid", padx=10, pady=8)
        box_diag.pack(fill=tk.X, pady=(0, 8))

        row_t = tk.Frame(box_diag, bg="#1e293b")
        row_t.pack(fill=tk.X, pady=1)
        tk.Label(row_t, text="Die Temp:", font=("Segoe UI", 9, "bold"), fg="#cbd5e1", bg="#1e293b").pack(side=tk.LEFT)
        self.lbl_temp = tk.Label(row_t, text="24.5 °C  (76.1 °F)", font=("Consolas", 10, "bold"), fg="#fb923c", bg="#1e293b")
        self.lbl_temp.pack(side=tk.RIGHT)

        row_p = tk.Frame(box_diag, bg="#1e293b")
        row_p.pack(fill=tk.X, pady=3)
        tk.Label(row_p, text="Interrupts:", font=("Segoe UI", 9, "bold"), fg="#cbd5e1", bg="#1e293b").pack(side=tk.LEFT)

        self.lbl_int2 = tk.Label(row_p, text="INT2: HIGH", font=("Consolas", 8, "bold"),
                                 bg="#166534", fg="#86efac", padx=6, pady=1)
        self.lbl_int2.pack(side=tk.RIGHT, padx=2)

        self.lbl_int1 = tk.Label(row_p, text="INT1: LOW", font=("Consolas", 8, "bold"),
                                 bg="#334155", fg="#94a3b8", padx=6, pady=1)
        self.lbl_int1.pack(side=tk.RIGHT, padx=2)

        row_st = tk.Frame(box_diag, bg="#1e293b")
        row_st.pack(fill=tk.X, pady=2)
        tk.Label(row_st, text="Status:", font=("Segoe UI", 9, "bold"), fg="#cbd5e1", bg="#1e293b").pack(side=tk.LEFT)
        self.lbl_status = tk.Label(row_st, text="0x41 [RDY AWK]", font=("Consolas", 9, "bold"), fg="#facc15", bg="#1e293b")
        self.lbl_status.pack(side=tk.RIGHT)

        # Card 5: Interactive Sensor Control Deck
        box_cmd = tk.LabelFrame(sidebar, text="Interactive Firmware Commands", font=("Segoe UI", 9, "bold"),
                                fg="#94a3b8", bg="#1e293b", bd=1, relief="solid", padx=8, pady=8)
        box_cmd.pack(fill=tk.X)

        r_cmd1 = tk.Frame(box_cmd, bg="#1e293b")
        r_cmd1.pack(fill=tk.X, pady=2)
        b_tare = tk.Button(r_cmd1, text="Tare Level [8]", font=("Segoe UI", 8, "bold"),
                           bg="#0284c7", fg="#ffffff", relief="flat", command=lambda: self.send_cmd('8'))
        b_tare.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        b_range = tk.Button(r_cmd1, text="Range ±2/4/8g [5]", font=("Segoe UI", 8),
                            bg="#334155", fg="#f8fafc", relief="flat", command=lambda: self.send_cmd('5'))
        b_range.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)

        r_cmd2 = tk.Frame(box_cmd, bg="#1e293b")
        r_cmd2.pack(fill=tk.X, pady=2)
        b_test = tk.Button(r_cmd2, text="Self-Test [2]", font=("Segoe UI", 8),
                           bg="#334155", fg="#f8fafc", relief="flat", command=lambda: self.send_cmd('2'))
        b_test.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        b_fifo = tk.Button(r_cmd2, text="FIFO Read [3]", font=("Segoe UI", 8),
                           bg="#334155", fg="#f8fafc", relief="flat", command=lambda: self.send_cmd('3'))
        b_fifo.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)

        r_cmd3 = tk.Frame(box_cmd, bg="#1e293b")
        r_cmd3.pack(fill=tk.X, pady=2)
        b_mot = tk.Button(r_cmd3, text="Motion Mode [4]", font=("Segoe UI", 8),
                          bg="#334155", fg="#f8fafc", relief="flat", command=lambda: self.send_cmd('4'))
        b_mot.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        b_reset = tk.Button(r_cmd3, text="Soft Reset [R]", font=("Segoe UI", 8),
                            bg="#b91c1c", fg="#ffffff", relief="flat", command=lambda: self.send_cmd('R'))
        b_reset.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)

    # --------------------------------------------------------------------------
    # Mouse & Camera Interactions
    # --------------------------------------------------------------------------
    def on_mouse_down_1(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        self.mouse_btn_down = 1

    def on_mouse_drag_1(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

        # Orbit camera: horizontal mouse controls azimuth, vertical controls elevation
        self.cam_azimuth = (self.cam_azimuth + dx * 0.5) % 360.0
        self.cam_elevation = max(-89.0, min(89.0, self.cam_elevation + dy * 0.5))

    def on_mouse_down_3(self, event):
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y
        self.mouse_btn_down = 3

    def on_mouse_drag_3(self, event):
        dx = event.x - self.last_mouse_x
        dy = event.y - self.last_mouse_y
        self.last_mouse_x = event.x
        self.last_mouse_y = event.y

        # Pan camera
        self.cam_pan_x += dx
        self.cam_pan_y += dy

    def on_mouse_wheel(self, event):
        # Zoom camera
        if event.delta > 0:
            self.cam_distance = max(160.0, self.cam_distance - 25.0)
        else:
            self.cam_distance = min(900.0, self.cam_distance + 25.0)

    def set_camera(self, az, el, dist):
        self.cam_azimuth = az
        self.cam_elevation = el
        self.cam_distance = dist
        self.cam_pan_x = 0.0
        self.cam_pan_y = 0.0

    def reset_camera(self):
        self.set_camera(-38.0, 26.0, 420.0)

    def toggle_demo_mode(self):
        self.demo_mode = not self.demo_mode
        if self.demo_mode:
            self.btn_demo.config(text="Demo Mode: ON", bg="#16a34a", fg="#ffffff")
        else:
            self.btn_demo.config(text="Demo Mode: OFF", bg="#475569", fg="#f8fafc")

    def on_port_changed(self, event):
        new_port = self.cb_ports.get()
        if new_port != self.port:
            self.port = new_port
            self.reconnect_serial()

    def reconnect_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.connected = False
        self.lbl_conn_status.config(text="[ CONNECTING... ]", fg="#facc15")

    def send_cmd(self, char_cmd):
        if self.ser and self.connected:
            try:
                self.ser.write(char_cmd.encode('ascii'))
            except Exception:
                pass

    # --------------------------------------------------------------------------
    # Serial Telemetry Worker Thread
    # --------------------------------------------------------------------------
    def serial_worker(self):
        pattern = re.compile(
            r'X:([-\d]+)mg\s+Y:([-\d]+)mg\s+Z:([-\d]+)mg\s+\|\s+\|a\|:(\d+)mg\s+\|\s+Pitch:([-\d\.]+)\s+deg\s+Roll:([-\d\.]+)\s+deg\s+\|\s+Temp:([-\d\.]+)\s+C.*INT1:(\d+)\s+INT2:(\d+)\s+\|\s+ST:(0x[0-9A-Fa-f]+)\s*\[([^\]]*)\]'
        )

        while self.running:
            if not self.ser:
                try:
                    self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                    self.ser.reset_input_buffer()
                    self.connected = True
                    self.after(0, lambda: self.lbl_conn_status.config(text=f"[ {self.port} CONNECTED ]", fg="#4ade80"))
                except Exception:
                    self.connected = False
                    self.after(0, lambda: self.lbl_conn_status.config(text="[ NO HARDWARE ]", fg="#ef4444"))
                    time.sleep(1.0)
                    continue

            try:
                line = self.ser.readline()
                if line:
                    text = line.decode('ascii', errors='replace').rstrip()
                    m = pattern.search(text)
                    if m:
                        ax = int(m.group(1))
                        ay = int(m.group(2))
                        az = int(m.group(3))
                        mag = int(m.group(4))
                        pitch = float(m.group(5))
                        roll  = float(m.group(6))
                        temp  = float(m.group(7))
                        int1  = int(m.group(8))
                        int2  = int(m.group(9))
                        st_hex = m.group(10)
                        flags  = m.group(11).strip()

                        if not self.demo_mode:
                            self.target_ax    = float(ax)
                            self.target_ay    = float(ay)
                            self.target_az    = float(az)
                            self.target_mag   = float(mag)
                            self.target_pitch = pitch
                            self.target_roll  = roll
                            self.target_temp  = temp
                            self.cur_int1     = int1
                            self.cur_int2     = int2
                            self.cur_status   = st_hex
                            self.cur_flags    = flags
            except Exception:
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                self.connected = False
                time.sleep(0.5)

    # --------------------------------------------------------------------------
    # 60 FPS Render & Animation Loop
    # --------------------------------------------------------------------------
    def render_loop(self):
        now = time.time()
        dt = now - self.last_render_time
        self.last_render_time = now

        # 1. Update Simulation in Demo Mode
        if self.demo_mode:
            self.demo_t += 0.03
            self.target_pitch = 32.0 * math.sin(self.demo_t * 1.2)
            self.target_roll  = 48.0 * math.cos(self.demo_t * 0.85)

            p_rad = math.radians(self.target_pitch)
            r_rad = math.radians(self.target_roll)
            self.target_ax  = -1000.0 * math.sin(p_rad)
            self.target_ay  =  1000.0 * math.sin(r_rad) * math.cos(p_rad)
            self.target_az  =  1000.0 * math.cos(r_rad) * math.cos(p_rad)
            self.target_mag = math.sqrt(self.target_ax**2 + self.target_ay**2 + self.target_az**2)
            self.target_temp = 25.2 + 0.4 * math.sin(self.demo_t * 0.2)
            self.cur_int1 = 1 if int(self.demo_t * 2) % 2 == 0 else 0
            self.cur_int2 = 1
            self.cur_status = "0x41"
            self.cur_flags = "RDY AWK ACT"

        # 2. Smooth Interpolation for 60 FPS Fluid Motion (Exponential Decay)
        alpha = 0.25
        self.disp_pitch += (self.target_pitch - self.disp_pitch) * alpha
        self.disp_roll  += (self.target_roll  - self.disp_roll)  * alpha
        self.disp_ax    += (self.target_ax    - self.disp_ax)    * alpha
        self.disp_ay    += (self.target_ay    - self.disp_ay)    * alpha
        self.disp_az    += (self.target_az    - self.disp_az)    * alpha
        self.disp_mag   += (self.target_mag   - self.disp_mag)   * alpha

        # 3. Draw 3D Scene to Canvas
        self.draw_3d_scene()

        # 4. Update Digital Instruments & Meters
        self.update_telemetry_ui()

        # Schedule Next Frame (16ms -> ~60 FPS)
        self.after(16, self.render_loop)

    # --------------------------------------------------------------------------
    # 3D Scene Projection & Rasterization
    # --------------------------------------------------------------------------
    def draw_3d_scene(self):
        c = self.canvas
        c.delete("all")

        cw = c.winfo_width()
        ch = c.winfo_height()
        if cw < 50 or ch < 50:
            return

        cx = cw / 2.0 + self.cam_pan_x
        cy = ch / 2.0 + self.cam_pan_y
        fov_factor = self.cam_distance * 1.5

        # Precompute Rotation Matrices
        # Board Spatial Rotation (from IMU Pitch & Roll)
        R_board = euler_to_matrix(self.disp_pitch, self.disp_roll, 0.0)

        # Camera View Space Rotation
        R_cam = camera_matrix(self.cam_azimuth, self.cam_elevation)

        # Combined Rotation for Board Model: R_total = R_cam * R_board
        R_total = mat_mult_mat(R_cam, R_board)

        # Directional Light Vector in Camera Space (slight top-right-front light)
        light_dir = vector_normalize([0.35, 0.55, 0.75])

        # ----------------------------------------------------------------------
        # A. Draw Static World Ground Grid & Horizon Circle
        # ----------------------------------------------------------------------
        ground_z = -75.0
        # Draw concentric compass horizon circles
        for radius in [120, 200]:
            poly_pts = []
            for deg in range(0, 360, 15):
                rad = math.radians(deg)
                pt_w = [radius * math.cos(rad), radius * math.sin(rad), ground_z]
                pt_c = mat_mult_vec(R_cam, pt_w)
                zc = pt_c[2] + self.cam_distance
                if zc > 10:
                    sx = cx + (pt_c[0] * fov_factor) / zc
                    sy = cy - (pt_c[1] * fov_factor) / zc
                    poly_pts.extend([sx, sy])
            if len(poly_pts) >= 4:
                c.create_polygon(poly_pts, fill="", outline="#1e293b", width=1, dash=(3, 3))

        # Ground Grid Lines (X & Y axis lines at ground level)
        for gx in range(-180, 190, 60):
            p1_c = mat_mult_vec(R_cam, [gx, -180, ground_z])
            p2_c = mat_mult_vec(R_cam, [gx,  180, ground_z])
            z1, z2 = p1_c[2] + self.cam_distance, p2_c[2] + self.cam_distance
            if z1 > 10 and z2 > 10:
                s1 = (cx + (p1_c[0] * fov_factor)/z1, cy - (p1_c[1] * fov_factor)/z1)
                s2 = (cx + (p2_c[0] * fov_factor)/z2, cy - (p2_c[1] * fov_factor)/z2)
                c.create_line(s1[0], s1[1], s2[0], s2[1], fill="#0f172a", width=1)

        for gy in range(-180, 190, 60):
            p1_c = mat_mult_vec(R_cam, [-180, gy, ground_z])
            p2_c = mat_mult_vec(R_cam, [ 180, gy, ground_z])
            z1, z2 = p1_c[2] + self.cam_distance, p2_c[2] + self.cam_distance
            if z1 > 10 and z2 > 10:
                s1 = (cx + (p1_c[0] * fov_factor)/z1, cy - (p1_c[1] * fov_factor)/z1)
                s2 = (cx + (p2_c[0] * fov_factor)/z2, cy - (p2_c[1] * fov_factor)/z2)
                c.create_line(s1[0], s1[1], s2[0], s2[1], fill="#0f172a", width=1)

        # ----------------------------------------------------------------------
        # B. Transform & Depth-Sort Board Faces (Painter's Algorithm)
        # ----------------------------------------------------------------------
        render_queue = []

        for verts_local, base_col, outline_col, label in self.model.faces:
            # Transform vertices to camera view space
            verts_cam = [mat_mult_vec(R_total, v) for v in verts_local]

            # Calculate average depth Z
            avg_z = sum(v[2] for v in verts_cam) / len(verts_cam)
            depth = avg_z + self.cam_distance

            if depth <= 10:
                continue

            # Calculate Face Normal Vector in Camera Space for Flat Shading
            e1 = [verts_cam[1][0] - verts_cam[0][0], verts_cam[1][1] - verts_cam[0][1], verts_cam[1][2] - verts_cam[0][2]]
            e2 = [verts_cam[2][0] - verts_cam[0][0], verts_cam[2][1] - verts_cam[0][1], verts_cam[2][2] - verts_cam[0][2]]
            norm = vector_normalize(vector_cross(e1, e2))

            # Backface culling: if normal points away from camera, dot(norm, [0,0,-1]) > 0
            if norm[2] <= 0:
                continue

            # Directional Lighting Factor
            intensity = 0.40 + 0.65 * max(0.0, vector_dot(norm, light_dir))
            shaded_color = shade_color(base_col, intensity)

            # Project to screen coordinates
            screen_coords = []
            for v in verts_cam:
                zc = v[2] + self.cam_distance
                sx = cx + (v[0] * fov_factor) / zc
                sy = cy - (v[1] * fov_factor) / zc
                screen_coords.extend([sx, sy])

            render_queue.append((depth, "polygon", screen_coords, shaded_color, outline_col))

        # Sort all polygonal faces back-to-front (largest depth first)
        render_queue.sort(key=lambda item: item[0], reverse=True)

        for _, item_type, coords, fill_c, out_c in render_queue:
            if item_type == "polygon":
                c.create_polygon(coords, fill=fill_c, outline=out_c, width=1)

        # ----------------------------------------------------------------------
        # C. Draw Silkscreen Lines on Top of Board
        # ----------------------------------------------------------------------
        for p1_local, p2_local, line_col, lw in self.model.lines:
            v1_c = mat_mult_vec(R_total, p1_local)
            v2_c = mat_mult_vec(R_total, p2_local)
            z1 = v1_c[2] + self.cam_distance
            z2 = v2_c[2] + self.cam_distance
            if z1 > 10 and z2 > 10:
                s1 = (cx + (v1_c[0] * fov_factor)/z1, cy - (v1_c[1] * fov_factor)/z1)
                s2 = (cx + (v2_c[0] * fov_factor)/z2, cy - (v2_c[1] * fov_factor)/z2)
                c.create_line(s1[0], s1[1], s2[0], s2[1], fill=line_col, width=lw)

        # ----------------------------------------------------------------------
        # D. Draw Sensor Chip Label ("ADXL362" & "CC2652R")
        # ----------------------------------------------------------------------
        chip_center_c = mat_mult_vec(R_total, [0, 0, 7.5])
        zc = chip_center_c[2] + self.cam_distance
        if zc > 10:
            sx = cx + (chip_center_c[0] * fov_factor) / zc
            sy = cy - (chip_center_c[1] * fov_factor) / zc
            c.create_text(sx, sy, text="ADXL362", fill="#ffffff", font=("Segoe UI", 7, "bold"))

        mcu_center_c = mat_mult_vec(R_total, [-44, 0, 7.0])
        zc_m = mcu_center_c[2] + self.cam_distance
        if zc_m > 10:
            sx_m = cx + (mcu_center_c[0] * fov_factor) / zc_m
            sy_m = cy - (mcu_center_c[1] * fov_factor) / zc_m
            c.create_text(sx_m, sy_m, text="CC2652R", fill="#94a3b8", font=("Segoe UI", 7, "bold"))

        # ----------------------------------------------------------------------
        # E. Draw 3-Axis Coordinate Frame Emanating from ADXL362 Center
        # ----------------------------------------------------------------------
        origin_local = [0, 0, 7.5]
        axis_len = 65.0

        axes_defs = [
            ([axis_len, 0, 0], "#ef4444", "+X (Roll)"),
            ([0, axis_len, 0], "#22c55e", "+Y (Pitch)"),
            ([0, 0, axis_len], "#38bdf8", "+Z (Normal)")
        ]

        orig_c = mat_mult_vec(R_total, origin_local)
        z_orig = orig_c[2] + self.cam_distance
        if z_orig > 10:
            s_orig = (cx + (orig_c[0] * fov_factor) / z_orig, cy - (orig_c[1] * fov_factor) / z_orig)

            for vec_loc, ax_col, ax_label in axes_defs:
                tip_loc = [origin_local[0] + vec_loc[0], origin_local[1] + vec_loc[1], origin_local[2] + vec_loc[2]]
                tip_c = mat_mult_vec(R_total, tip_loc)
                z_tip = tip_c[2] + self.cam_distance
                if z_tip > 10:
                    s_tip = (cx + (tip_c[0] * fov_factor) / z_tip, cy - (tip_c[1] * fov_factor) / z_tip)

                    # Draw Axis Shaft with Arrowhead
                    c.create_line(s_orig[0], s_orig[1], s_tip[0], s_tip[1], fill=ax_col, width=3, arrow=tk.LAST, arrowshape=(10, 12, 4))
                    # Draw Axis Label
                    c.create_text(s_tip[0] + 6, s_tip[1] - 4, text=ax_label, fill=ax_col, font=("Segoe UI", 8, "bold"), anchor="w")

        # ----------------------------------------------------------------------
        # F. Draw Dynamic Acceleration / Gravity Vector in 3D
        # ----------------------------------------------------------------------
        # Vector points in direction (ax, ay, az). Scale: 1000 mg = 70 units
        scale_g = 70.0 / 1000.0
        g_vec_loc = [
            origin_local[0] + (self.disp_ax * scale_g),
            origin_local[1] + (self.disp_ay * scale_g),
            origin_local[2] + (self.disp_az * scale_g)
        ]

        g_tip_c = mat_mult_vec(R_total, g_vec_loc)
        z_gtip = g_tip_c[2] + self.cam_distance
        if z_orig > 10 and z_gtip > 10:
            s_gtip = (cx + (g_tip_c[0] * fov_factor) / z_gtip, cy - (g_tip_c[1] * fov_factor) / z_gtip)

            # Bold glowing golden arrow for acceleration vector
            c.create_line(s_orig[0], s_orig[1], s_gtip[0], s_gtip[1], fill="#f59e0b", width=4,
                          arrow=tk.LAST, arrowshape=(12, 14, 5))
            mag_g = self.disp_mag / 1000.0
            c.create_text(s_gtip[0] + 8, s_gtip[1] + 6, text=f"g: {mag_g:.2f}g",
                          fill="#facc15", font=("Consolas", 9, "bold"), anchor="w")

        # ----------------------------------------------------------------------
        # G. On-Canvas HUD Badges (Camera Info & Controls Hint)
        # ----------------------------------------------------------------------
        hud_cam = f"Camera Orbit: Azimuth {self.cam_azimuth:+.0f}° | Elevation {self.cam_elevation:+.0f}° | Zoom {self.cam_distance:.0f}"
        c.create_text(12, 14, text=hud_cam, fill="#64748b", font=("Segoe UI", 9), anchor="w")

        hud_hint = "[Left Drag] Orbit  |  [Right Drag] Pan  |  [Scroll] Zoom"
        c.create_text(12, ch - 14, text=hud_hint, fill="#475569", font=("Segoe UI", 8), anchor="w")

        # FPS & Mode Badge
        fps_text = "60 FPS | LIVE TELEMETRY" if not self.demo_mode else "60 FPS | DEMO SIMULATION"
        c.create_text(cw - 12, ch - 14, text=fps_text, fill="#38bdf8" if not self.demo_mode else "#a855f7",
                      font=("Consolas", 8, "bold"), anchor="e")

    # --------------------------------------------------------------------------
    # Telemetry Panel & Instrument Updates
    # --------------------------------------------------------------------------
    def update_telemetry_ui(self):
        # 1. Primary Attitudes
        p = self.disp_pitch
        r = self.disp_roll
        mag = self.disp_mag

        self.lbl_pitch_val.config(text=f"{p:+6.1f}°")
        self.lbl_roll_val.config(text=f"{r:+6.1f}°")

        mag_g = mag / 1000.0
        self.lbl_mag_val.config(text=f"{mag_g:4.2f} g  ({int(mag):4d} mg)")

        if mag_g < 0.35:
            self.lbl_mag_state.config(text="* Free-Fall / Weightless Condition", fg="#38bdf8")
        elif mag_g > 2.00:
            self.lbl_mag_state.config(text="* High-G Dynamic Shock Detected!", fg="#ef4444")
        elif 0.85 <= mag_g <= 1.15:
            self.lbl_mag_state.config(text="* Standard 1.0g Static Rest State", fg="#4ade80")
        else:
            self.lbl_mag_state.config(text="* Active Dynamic Motion", fg="#facc15")

        # Pitch Bar (-90 to +90 deg)
        self.draw_bipolar_bar(self.pitch_bar, p, -90.0, 90.0, "#38bdf8")
        # Roll Bar (-180 to +180 deg)
        self.draw_bipolar_bar(self.roll_bar, r, -180.0, 180.0, "#ec4899")

        # 2. 3-Axis Components
        ax = int(self.disp_ax)
        ay = int(self.disp_ay)
        az = int(self.disp_az)

        self.lbl_x_val.config(text=f"{ax:+5d} mg")
        self.lbl_y_val.config(text=f"{ay:+5d} mg")
        self.lbl_z_val.config(text=f"{az:+5d} mg")

        self.draw_bipolar_bar(self.bar_x, ax, -2000, 2000, "#ef4444")
        self.draw_bipolar_bar(self.bar_y, ay, -2000, 2000, "#22c55e")
        self.draw_bipolar_bar(self.bar_z, az, -2000, 2000, "#38bdf8")

        # 3. Environment & Hardware Pins
        tc = self.target_temp
        tf = tc * 1.8 + 32.0
        self.lbl_temp.config(text=f"{tc:4.1f} °C  ({tf:4.1f} °F)")

        if self.cur_int1:
            self.lbl_int1.config(text="INT1: HIGH", bg="#166534", fg="#86efac")
        else:
            self.lbl_int1.config(text="INT1: LOW", bg="#334155", fg="#94a3b8")

        if self.cur_int2:
            self.lbl_int2.config(text="INT2: HIGH", bg="#166534", fg="#86efac")
        else:
            self.lbl_int2.config(text="INT2: LOW", bg="#334155", fg="#94a3b8")

        self.lbl_status.config(text=f"{self.cur_status} [{self.cur_flags}]")

    def draw_bipolar_bar(self, canvas, val, min_val, max_val, color):
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10 or h < 4:
            return

        mid_x = w / 2.0
        val_clamped = max(min_val, min(max_val, val))
        span = (max_val - min_val) / 2.0
        frac = (val_clamped) / span

        target_x = mid_x + frac * (w / 2.0)

        # Center indicator mark
        canvas.create_line(mid_x, 0, mid_x, h, fill="#475569", width=1)

        # Bar fill
        if target_x >= mid_x:
            canvas.create_rectangle(mid_x, 1, target_x, h-1, fill=color, outline="")
        else:
            canvas.create_rectangle(target_x, 1, mid_x, h-1, fill=color, outline="")

# ------------------------------------------------------------------------------
# Application Entry Point
# ------------------------------------------------------------------------------
if __name__ == "__main__":
    app = Imu3dApp()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.running = False
