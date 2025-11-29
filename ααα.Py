import serial, math, time
from gpiozero import Servo
import plotly.graph_objects as go

# === CONFIG ===
PORT = "/dev/ttyS0"
BAUD = 115200
HEIGHT_CM = 50   # distance from table to LiDAR

PAN_PIN = 18
TILT_PIN = 13

# Servo sweep ranges
PAN_MIN, PAN_MAX, PAN_STEP = -30, 30, 2
TILT_MIN, TILT_MAX, TILT_STEP = -10, 10, 2

# === Init hardware ===
ser = serial.Serial(PORT, BAUD, timeout=1)
pan = Servo(PAN_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(TILT_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

# Storage
xs, ys, zs = [], [], []

def read_lidar():
    """Reads one TFmini-S packet"""
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            return data[0] + data[1] * 256

print("\n📡 Starting 3D scan...\n")

# === SCANNING ===
for tilt_angle in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    tilt.value = tilt_angle / 90
    time.sleep(0.2)

    sweep = range(PAN_MIN, PAN_MAX + 1, PAN_STEP)
    sweep = reversed(list(sweep)) if tilt_angle % 2 == 0 else sweep

    for pan_angle in sweep:
        pan.value = pan_angle / 90
        time.sleep(0.1)

        dist = read_lidar()

        # Convert angles to radians
        a = math.radians(pan_angle)
        b = math.radians(tilt_angle)

        # 3D spherical → Cartesian with height reference
        x = dist * math.cos(b) * math.sin(a)
        y = dist * math.sin(b)
        z = HEIGHT_CM - dist * math.cos(a) * math.cos(b)

        xs.append(x)
        ys.append(y)
        zs.append(z)

        print(f"PAN {pan_angle:>3}°, TILT {tilt_angle:>3}° → {dist:.1f} cm")

print("\n✔ Scan complete!")
print("📦 Showing 3D model...")

# === 3D Vis ===
fig = go.Figure(data=[go.Scatter3d(
    x=xs, y=ys, z=zs,
    mode='markers',
    marker=dict(size=3, color=zs, colorscale='Viridis')
)])

fig.update_layout(
    title="3D LiDAR Scan",
    scene=dict(
        xaxis_title="X",
        yaxis_title="Y",
        zaxis_title="Height"
    ),
    width=900,
    height=700
)

fig.show()
