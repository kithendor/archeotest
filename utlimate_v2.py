import serial, math, time
import pigpio
import pandas as pd
import numpy as np
import plotly.graph_objects as go
import plotly.express as px

# ---------------------------------------
# USER SETTINGS
# ---------------------------------------
PORT = "/dev/ttyS0"
BAUD = 115200

HEIGHT_CM = 70  # ύψος lidar από μακέτα (προφίλ)

PAN_PIN = 13
TILT_PIN = 18

PAN_MIN, PAN_MAX, PAN_STEP = -35, 35, 2   # βάλε 1 για full ανάλυση
TILT_MIN, TILT_MAX, TILT_STEP = -15, 15, 2

GRID_CELL_CM = 2.0          # μέγεθος κελιού για heatmap/mesh
BUILDING_THRESHOLD = 5.0    # cm πάνω από έδαφος για να το θεωρεί "κτήριο"
STL_FILENAME = "scan_mesh.stl"

# ---------------------------------------
# INIT pigpio + UART
# ---------------------------------------
pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpio failed to connect")
    exit()

pi.set_mode(PAN_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)
pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)

ser = serial.Serial(PORT, BAUD, timeout=0.2)


# ---------------------------------------
# SERVO HELPERS
# ---------------------------------------
def pulse(angle):
    return int(500 + (angle + 90) * 2000 / 180)

def move(pin, angle, smooth=True):
    target = pulse(angle)
    current = pi.get_servo_pulsewidth(pin)

    if current < 500 or current > 2500:
        current = target

    if smooth:
        step = 8
        while abs(current - target) > step:
            current += step if target > current else -step
            pi.set_servo_pulsewidth(pin, current)
            time.sleep(0.004)

    pi.set_servo_pulsewidth(pin, target)
    time.sleep(0.08)


# ---------------------------------------
# LIDAR READER (με flush!)
# ---------------------------------------
def read_lidar():
    ser.reset_input_buffer()
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            if len(data) == 7:
                dist = data[0] + data[1]*256
                return dist


# ---------------------------------------
# SCAN
# ---------------------------------------
xs, ys, zs = [], [], []

print("\n📡 STARTING FULL 3D SCAN...\n")

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

        print(f"Tilt {tilt:>3} | Pan {pan:>3} → {dist} cm")

# stop servos
pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)
pi.stop()

print("\n✔ SCAN COMPLETE!")


# ---------------------------------------
# SAVE POINT CLOUD
# ---------------------------------------
df = pd.DataFrame({"x": xs, "y": ys, "z": zs})
df.to_csv("scan_points.csv", index=False)
print("📁 Saved: scan_points.csv")


# ---------------------------------------
# 3D PLOT (raw z)
# ---------------------------------------
fig3d = go.Figure(data=[go.Scatter3d(
    x=xs, y=ys, z=zs,
    mode='markers',
    marker=dict(size=3, color=zs, colorscale='Viridis')
)])
fig3d.update_layout(title="3D LiDAR Scan", width=900, height=700)
fig3d.show()


# ---------------------------------------
# GROUND FLATTENING
# ---------------------------------------
xs_arr = np.array(xs); ys_arr = np.array(ys); zs_arr = np.array(zs)

A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
a, b, c = coeffs
plane = a * xs_arr + b * ys_arr + c

z_flat = zs_arr - plane
print(f"Ground removed. Flat Z range: {z_flat.min():.2f}..{z_flat.max():.2f} cm")


# ---------------------------------------
# 2D HEIGHTMAP GRID (για heatmap + mesh)
# ---------------------------------------
x_min, x_max = xs_arr.min(), xs_arr.max()
y_min, y_max = ys_arr.min(), ys_arr.max()

nx = int(np.ceil((x_max - x_min) / GRID_CELL_CM)) + 1
ny = int(np.ceil((y_max - y_min) / GRID_CELL_CM)) + 1

height_grid = np.full((ny, nx), np.nan)

# map κάθε σημείο σε κελί και κρατάμε ΜΕΓΙΣΤΟ ύψος (ώστε η στέγη να είναι συμπαγής)
ix = ((xs_arr - x_min) / GRID_CELL_CM).astype(int)
iy = ((ys_arr - y_min) / GRID_CELL_CM).astype(int)

for k in range(len(z_flat)):
    i = ix[k]
    j = iy[k]
    if 0 <= i < nx and 0 <= j < ny:
        h = z_flat[k]
        cur = height_grid[j, i]
        if np.isnan(cur) or h > cur:
            height_grid[j, i] = h

print("✅ Height grid created.")


# ---------------------------------------
# 2D HEATMAP (top-down)
# ---------------------------------------
x_coords = x_min + np.arange(nx) * GRID_CELL_CM
y_coords = y_min + np.arange(ny) * GRID_CELL_CM

fig2d = px.imshow(
    height_grid,
    origin="lower",
    x=x_coords,
    y=y_coords,
    color_continuous_scale="Viridis",
    labels={"x": "X (cm)", "y": "Y (cm)", "color": "Height (cm)"},
    title="Top-Down 2D Heightmap"
)

fig2d.update_layout(
    width=800,
    height=800,
    yaxis=dict(scaleanchor="x", scaleratio=1)
)

fig2d.show()


# ---------------------------------------
# STL EXPORT (mesh + walls)
# ---------------------------------------
def write_triangle(f, v1, v2, v3):
    # πολύ απλοποιημένο normal (=0,0,1), STL δεν νοιάζεται ιδιαίτερα
    f.write("  facet normal 0 0 1\n")
    f.write("    outer loop\n")
    f.write(f"      vertex {v1[0]} {v1[1]} {v1[2]}\n")
    f.write(f"      vertex {v2[0]} {v2[1]} {v2[2]}\n")
    f.write(f"      vertex {v3[0]} {v3[1]} {v3[2]}\n")
    f.write("    endloop\n")
    f.write("  endfacet\n")

def save_heightmap_to_stl(grid, x0, y0, cell, filename, thresh):
    ny, nx = grid.shape
    with open(filename, "w") as f:
        f.write("solid scan\n")

        # TOP SURFACE
        for j in range(ny - 1):
            for i in range(nx - 1):
                h00 = grid[j, i]
                h10 = grid[j, i+1]
                h01 = grid[j+1, i]
                h11 = grid[j+1, i+1]
                if np.isnan(h00) or np.isnan(h10) or np.isnan(h01) or np.isnan(h11):
                    continue

                x00 = x0 + i * cell
                x10 = x0 + (i+1) * cell
                y00 = y0 + j * cell
                y01 = y0 + (j+1) * cell

                v1 = (x00, y00, h00)
                v2 = (x10, y00, h10)
                v3 = (x00, y01, h01)
                v4 = (x10, y01, h11)

                # δύο τρίγωνα για κάθε κελί
                write_triangle(f, v1, v2, v3)
                write_triangle(f, v2, v4, v3)

        # WALLS γύρω από "κτήρια"
        mask = grid > thresh
        z0 = 0.0

        for j in range(ny):
            for i in range(nx):
                if not mask[j, i] or np.isnan(grid[j, i]):
                    continue
                h = grid[j, i]

                x_left   = x0 + i * cell
                x_right  = x0 + (i+1) * cell
                y_bottom = y0 + j * cell
                y_top    = y0 + (j+1) * cell

                # για κάθε πλευρά αν ο γείτονας ΔΕΝ είναι building → χτίζουμε τοίχο
                def neighbor_is_empty(j2, i2):
                    if j2 < 0 or j2 >= ny or i2 < 0 or i2 >= nx:
                        return True
                    return (not mask[j2, i2]) or np.isnan(grid[j2, i2])

                # αριστερός τοίχος
                if neighbor_is_empty(j, i-1):
                    v1 = (x_left, y_bottom, z0)
                    v2 = (x_left, y_top, z0)
                    v3 = (x_left, y_bottom, h)
                    v4 = (x_left, y_top, h)
                    write_triangle(f, v1, v3, v2)
                    write_triangle(f, v3, v4, v2)

                # δεξιός τοίχος
                if neighbor_is_empty(j, i+1):
                    v1 = (x_right, y_bottom, z0)
                    v2 = (x_right, y_top, z0)
                    v3 = (x_right, y_bottom, h)
                    v4 = (x_right, y_top, h)
                    write_triangle(f, v2, v3, v1)
                    write_triangle(f, v2, v4, v3)

                # κάτω τοίχος
                if neighbor_is_empty(j-1, i):
                    v1 = (x_left,  y_bottom, z0)
                    v2 = (x_right, y_bottom, z0)
                    v3 = (x_left,  y_bottom, h)
                    v4 = (x_right, y_bottom, h)
                    write_triangle(f, v1, v3, v2)
                    write_triangle(f, v3, v4, v2)

                # πάνω τοίχος
                if neighbor_is_empty(j+1, i):
                    v1 = (x_left,  y_top, z0)
                    v2 = (x_right, y_top, z0)
                    v3 = (x_left,  y_top, h)
                    v4 = (x_right, y_top, h)
                    write_triangle(f, v2, v3, v1)
                    write_triangle(f, v2, v4, v3)

        f.write("endsolid scan\n")

save_heightmap_to_stl(height_grid, x_min, y_min, GRID_CELL_CM,
                      STL_FILENAME, BUILDING_THRESHOLD)

print(f"🧱 STL mesh saved to {STL_FILENAME}")
