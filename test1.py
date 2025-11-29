import serial
from gpiozero import Servo
from time import sleep
import math
import matplotlib.pyplot as plt

# ---------------- CONFIG ----------------

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

PAN_PIN = 18     # Horizontal servo
TILT_PIN = 13    # Vertical servo

PAN_STEP = 3     # smaller = better detail
TILT_STEP = 4

PAN_MIN = -30    # degrees
PAN_MAX = 30

TILT_MIN = -10
TILT_MAX = 10

HEIGHT_CM = 100  # distance from LiDAR to model surface

# ---------------- INIT ----------------

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

pan = Servo(PAN_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(TILT_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

points_3d = []
points_2d = []  # projected floor map

plt.ion()
fig = plt.figure(figsize=(8,6))
ax3d = fig.add_subplot(121, projection='3d')
ax2d = fig.add_subplot(122)

print("\n📡 STARTING OPTIMIZED PAN–TILT SCAN...\n")

# ---------------- SCAN LOOP ----------------

for tilt_angle in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    tilt.value = tilt_angle / 90
    sleep(0.3)

    scan_angles = list(range(PAN_MIN, PAN_MAX + 1, PAN_STEP))

    # reverse direction each pass for faster scanning
    if tilt_angle % 2 == 0:
        scan_angles.reverse()

    for pan_angle in scan_angles:
        pan.value = pan_angle / 90
        sleep(0.07)

        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59:
            dist = data[2] + data[3] * 256  # cm

            # ---------- SPHERICAL → CARTESIAN ----------
            a = math.radians(pan_angle)
            b = math.radians(tilt_angle)

            # Real-world coordinates relative to LiDAR origin
            x = dist * math.cos(b) * math.sin(a)
            y = dist * math.sin(b)
            z = HEIGHT_CM - dist * math.cos(b) * math.cos(a)

            points_3d.append((x,y,z))

            # ---------- 2D PROJECTION (FLOOR PLAN) ----------
            # X-Y projection on the model surface (ignore height)
            points_2d.append((x, z))

            print(f"Tilt:{tilt_angle:>3}°  Pan:{pan_angle:>3}° → {dist}cm")

            # ---------- LIVE VISUALIZATION ----------
            ax3d.clear()
            ax2d.clear()

            # 3D scatter
            xs, ys, zs = zip(*points_3d)
            ax3d.scatter(xs, ys, zs, s=8)
            ax3d.set_title("3D Scan View")
            ax3d.set_xlim(-60, 60)
            ax3d.set_ylim(-40, 40)
            ax3d.set_zlim(0, HEIGHT_CM)

            # 2D floor map projection
            px, pz = zip(*points_2d)
            ax2d.scatter(px, pz, s=6)
            ax2d.set_xlim(-60, 60)
            ax2d.set_ylim(0, 100)
            ax2d.set_title("Top-Down Floor Projection (2D Map)")

            plt.pause(0.001)

print("\n✅ SCAN COMPLETE.\n")
