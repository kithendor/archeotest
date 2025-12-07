import serial, time, math, numpy as np, pandas as pd
import pigpio
import plotly.graph_objects as go
import plotly.express as px

# ---------------------------------------------------------
# HARDWARE SETTINGS
# ---------------------------------------------------------
UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200

PAN_PIN = 13     # <-- Pan servo here
TILT_PIN = 18    # <-- Tilt servo here

HEIGHT_CM = 70   # Height of LiDAR from table

PAN_MIN, PAN_MAX, PAN_STEP = -35, 35, 1
TILT_MIN, TILT_MAX, TILT_STEP = -15, 15, 1

GRID_SIZE = 2.0         # Size of grid cells in cm
BUILDING_THRESHOLD = 5  # cm above ground to consider “walls”
STL_NAME = "scan_mesh.stl"

# ---------------------------------------------------------
# STATUS FLAGS FOR WEB APP
# ---------------------------------------------------------
is_scanning = False
scan_progress = 0

# ---------------------------------------------------------
# START pigpio + UART
# ---------------------------------------------------------
pi = pigpio.pi()
pi.set_mode(PAN_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)

ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.2)

# ---------------------------------------------------------
# SERVO HELPERS
# ---------------------------------------------------------
def pulse(angle):
    """Convert angle (-90..90) to pulse width."""
    return int(500 + (angle + 90) * 2000 / 180)

def move(pin, angle, smooth=True):
    target = pulse(angle)
    current = pi.get_servo_pulsewidth(pin)

    # If servo is uninitialized
    if current < 500 or current > 2500:
        current = target

    if smooth:
        step = 8
        while abs(current - target) > step:
            current += step if target > current else -step
            pi.set_servo_pulsewidth(pin, current)
            time.sleep(0.003)
    pi.set_servo_pulsewidth(pin, target)
    time.sleep(0.04)

# ---------------------------------------------------------
# LIDAR READER (with buffer flush)
# ---------------------------------------------------------
def read_lidar():
    ser.reset_input_buffer()
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            if len(data) == 7:
                dist = data[0] + data[1]*256
                return dist

# ---------------------------------------------------------
# MAIN SCAN ROUTINE
# ---------------------------------------------------------
def run_scan():
    global is_scanning, scan_progress
    is_scanning = True
    scan_progress = 0

    xs, ys, zs = [], [], []

    total_moves = ((abs(TILT_MAX - TILT_MIN)//TILT_STEP)+1) * ((abs(PAN_MAX - PAN_MIN)//PAN_STEP)+1)
    done = 0

    for tilt in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
        move(TILT_PIN, tilt)

        sweep = range(PAN_MIN, PAN_MAX + 1, PAN_STEP)
        if tilt % 2 == 0:
            sweep = reversed(list(sweep))

        for pan in sweep:
            move(PAN_PIN, pan)
            dist = read_lidar()

            a = math.radians(pan)
            b = math.radians(tilt)

            x = dist * math.cos(b) * math.sin(a)
            y = dist * math.sin(b)
            z = HEIGHT_CM - dist * math.cos(a) * math.cos(b)

            xs.append(x)
            ys.append(y)
            zs.append(z)

            done += 1
            scan_progress = int((done / total_moves) * 100)

    # Stop servos
    pi.set_servo_pulsewidth(PAN_PIN, 0)
    pi.set_servo_pulsewidth(TILT_PIN, 0)

    # Save CSV
    df = pd.DataFrame({"x": xs, "y": ys, "z": zs})
    df.to_csv("scan_points.csv", index=False)

    # Make visualization helpers
    prepare_3d_plot(xs, ys, zs)
    prepare_2d_map(xs, ys, zs)
    save_stl(xs, ys, zs)

    is_scanning = False
    scan_progress = 100


# ---------------------------------------------------------
# 3D PLOT GENERATOR
# ---------------------------------------------------------
last_3d_html = "<h2>No scan yet</h2>"

def prepare_3d_plot(xs, ys, zs):
    global last_3d_html
    fig = go.Figure(data=[go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode='markers',
        marker=dict(size=3, color=zs, colorscale="Viridis")
    )])
    fig.update_layout(width=900, height=700, title="3D LiDAR Scan")
    last_3d_html = fig.to_html(full_html=False)

def get_3d_html():
    return last_3d_html


# ---------------------------------------------------------
# 2D HEATMAP
# ---------------------------------------------------------
last_2d_html = "<h2>No scan yet</h2>"

def prepare_2d_map(xs, ys, zs):
    global last_2d_html

    xs_arr = np.array(xs)
    ys_arr = np.array(ys)
    zs_arr = np.array(zs)

    # Remove ground
    A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
    coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
    plane = A @ coeffs
    zf = zs_arr - plane

    # Grid
    x_min, x_max = xs_arr.min(), xs_arr.max()
    y_min, y_max = ys_arr.min(), ys_arr.max()

    nx = int((x_max - x_min) / GRID_SIZE) + 1
    ny = int((y_max - y_min) / GRID_SIZE) + 1

    grid = np.full((ny, nx), np.nan)

    ix = ((xs_arr - x_min) / GRID_SIZE).astype(int)
    iy = ((ys_arr - y_min) / GRID_SIZE).astype(int)

    for k in range(len(zf)):
        i = ix[k]
        j = iy[k]
        val = zf[k]
        cur = grid[j, i]
        if np.isnan(cur) or val > cur:
            grid[j, i] = val

    fig = px.imshow(
        grid,
        origin="lower",
        color_continuous_scale="Viridis",
        title="Top-Down Heightmap"
    )
    last_2d_html = fig.to_html(full_html=False)

def get_2d_html():
    return last_2d_html


# ---------------------------------------------------------
# STL EXPORT
# ---------------------------------------------------------
def save_stl(xs, ys, zs):
    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)

    # Flatten ground
    A = np.c_[xs, ys, np.ones_like(xs)]
    coeffs, _, _, _ = np.linalg.lstsq(A, zs, rcond=None)
    plane = A @ coeffs
    zf = zs - plane

    # Grid
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    nx = int((x_max - x_min) / GRID_SIZE) + 1
    ny = int((y_max - y_min) / GRID_SIZE) + 1

    grid = np.full((ny, nx), np.nan)

    ix = ((xs - x_min) / GRID_SIZE).astype(int)
    iy = ((ys - y_min) / GRID_SIZE).astype(int)

    for k in range(len(zf)):
        i = ix[k]
        j = iy[k]
        val = zf[k]
        cur = grid[j, i]
        if np.isnan(cur) or val > cur:
            grid[j, i] = val

    # Save STL
    with open(STL_NAME, "w") as f:
        f.write("solid scan\n")

        def tri(a, b, c):
            f.write(" facet normal 0 0 1\n")
            f.write("  outer loop\n")
            f.write(f"   vertex {a[0]} {a[1]} {a[2]}\n")
            f.write(f"   vertex {b[0]} {b[1]} {b[2]}\n")
            f.write(f"   vertex {c[0]} {c[1]} {c[2]}\n")
            f.write("  endloop\n")
            f.write(" endfacet\n")

        # Top surface
        for j in range(ny - 1):
            for i in range(nx - 1):
                h00 = grid[j, i]
                h10 = grid[j, i+1]
                h01 = grid[j+1, i]
                h11 = grid[j+1, i+1]
                if np.isnan(h00) or np.isnan(h10) or np.isnan(h01) or np.isnan(h11):
                    continue

                x0 = x_min + i * GRID_SIZE
                x1 = x_min + (i+1) * GRID_SIZE
                y0 = y_min + j * GRID_SIZE
                y1 = y_min + (j+1) * GRID_SIZE

                v1 = (x0, y0, h00)
                v2 = (x1, y0, h10)
                v3 = (x0, y1, h01)
                v4 = (x1, y1, h11)

                tri(v1, v2, v3)
                tri(v2, v4, v3)

        f.write("endsolid scan\n")
