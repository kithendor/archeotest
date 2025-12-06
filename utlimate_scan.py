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

HEIGHT_CM = 70  # ύψος lidar από μακέτα

PAN_PIN = 18
TILT_PIN = 19

PAN_MIN, PAN_MAX, PAN_STEP = -40, 40, 1
TILT_MIN, TILT_MAX, TILT_STEP = -20, 20, 1

# ---------------------------------------
# INIT pigpio + UART
# ---------------------------------------

pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpio failed to connect")
    exit()

pi.set_mode(PAN_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)

pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)

ser = serial.Serial(PORT, BAUD, timeout=0.2)


# ---------------------------------------
# SERVO MOVEMENT HELPERS
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
# LIDAR READER WITH FLUSH (IMPORTANT!)
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

        # spherical → cartesian
        x = dist * math.cos(b) * math.sin(a)
        y = dist * math.sin(b)
        z = HEIGHT_CM - dist * math.cos(a) * math.cos(b)

        xs.append(x)
        ys.append(y)
        zs.append(z)

        print(f"Tilt {tilt:>3} | Pan {pan:>3} → {dist} cm")

# Stop servo output
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
# SHOW 3D MODEL
# ---------------------------------------
fig = go.Figure(data=[go.Scatter3d(
    x=xs, y=ys, z=zs,
    mode='markers',
    marker=dict(size=3, color=zs, colorscale='Viridis')
)])
fig.update_layout(title="3D LiDAR Scan", width=900, height=700)
fig.show()


# ---------------------------------------
# GROUND FLATTENING + TOP VIEW
# ---------------------------------------
xs_arr = np.array(xs); ys_arr = np.array(ys); zs_arr = np.array(zs)

# Plane fitting
A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
a, b, c = coeffs
plane = a * xs_arr + b * ys_arr + c

# Flatten
z_flat = zs_arr - plane

print(f"Ground removed. Flat Z range: {z_flat.min():.2f}..{z_flat.max():.2f}")


# ---------------------------------------
# TOP-DOWN SCATTER MAP
# ---------------------------------------
fig2 = px.scatter(
    x=xs_arr,
    y=ys_arr,
    color=z_flat,
    color_continuous_scale="Viridis",
    title="Top-Down 2D Map (Flattened)",
    width=800,
    height=800
)

fig2.update_traces(marker=dict(size=5))
fig2.update_layout(
    xaxis_title="X (cm)",
    yaxis_title="Y (cm)",
    yaxis=dict(scaleanchor="x", scaleratio=1)
)

fig2.show()

