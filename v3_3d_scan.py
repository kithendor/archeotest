import serial, math
from time import sleep
import pigpio
import plotly.graph_objects as go
import numpy as np   # <<< ΝΕΟ

# --- USER SETTINGS ---
PORT = "/dev/ttyS0"
BAUD = 115200
HEIGHT_CM = 70  # ύψος του LiDAR από το έδαφος

PAN_PIN = 18
TILT_PIN = 19

# Sweep ranges (δοκίμασε τώρα με step=1 για καλύτερη ανάλυση)
PAN_MIN, PAN_MAX, PAN_STEP = -40, 40, 1
TILT_MIN, TILT_MAX, TILT_STEP = -20, 20, 1

# --- INIT ---
ser = serial.Serial(PORT, BAUD, timeout=1)
pi = pigpio.pi()
if not pi.connected:
    print("❌ Pigpio not connected!")
    exit()

def pulse(angle):
    return int(500 + (angle + 90) * 2000 / 180)

def move(pin, angle, smooth=True):
    target = pulse(angle)
    current = pi.get_servo_pulsewidth(pin)

    if current < 500 or current > 2500:
        current = target

    if smooth:
        step = 10
        while abs(current - target) > step:
            current += step if target > current else -step
            pi.set_servo_pulsewidth(pin, current)
            sleep(0.005)

    pi.set_servo_pulsewidth(pin, target)
    sleep(0.12)  # μικρή παύση για σταθεροποίηση

def read_lidar():
    """Reads TFmini-S packet with timeout."""
    for _ in range(200):
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            return data[0] + data[1] * 256
    return None

xs, ys, zs = [], [], []

print("\n📡 STARTING 3D SCAN...\n")

# --- MAIN SCAN LOOP ---
for tilt in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    move(TILT_PIN, tilt)

    pan_range = range(PAN_MIN, PAN_MAX + 1, PAN_STEP)
    if tilt % 2 == 0:
        pan_range = reversed(list(pan_range))

    for pan in pan_range:
        move(PAN_PIN, pan)

        dist = read_lidar()
        if dist is None:
            print("⚠ Skipped (no data)")
            continue

        a = math.radians(pan)
        b = math.radians(tilt)

        # convert to Cartesian
        x = dist * math.cos(b) * math.sin(a)
        y = dist * math.sin(b)
        z = HEIGHT_CM - dist * math.cos(a) * math.cos(b)

        xs.append(x)
        ys.append(y)
        zs.append(z)

        print(f"Tilt {tilt:>3}° | Pan {pan:>3}° → {dist} cm")

# stop servo PWM
pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)

print("\n✔ SCAN COMPLETE")
print("🧮 Fitting ground plane...")

# --- GROUND FLATTENING ---
xs_arr = np.array(xs)
ys_arr = np.array(ys)
zs_arr = np.array(zs)

# matrix for plane z = a*x + b*y + c
A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
a_plane, b_plane, c_plane = coeffs
print(f"Plane coefficients: a={a_plane:.4f}, b={b_plane:.4f}, c={c_plane:.4f}")

plane_z = a_plane * xs_arr + b_plane * ys_arr + c_plane
z_flat = zs_arr - plane_z   # <<< εδώ γίνεται το "ίσιωμα"

print(f"Flattened Z range: {z_flat.min():.2f} .. {z_flat.max():.2f} cm")
print("📦 Generating 3D plot...")

# --- 3D VISUALIZATION (με ισιωμένο έδαφος) ---
fig = go.Figure(data=[go.Scatter3d(
    x=xs_arr,
    y=ys_arr,
    z=z_flat,              # <<< χρησιμοποιούμε το z_flat
    mode='markers',
    marker=dict(size=3, color=z_flat, colorscale='Viridis')
)])

fig.update_layout(
    title="3D LiDAR Scan (Ground Flattened)",
    scene=dict(
        xaxis_title="X (cm)",
        yaxis_title="Y (cm)",
        zaxis_title="Height above ground (cm)"
    ),
    width=900,
    height=700
)

fig.show()
