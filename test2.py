import serial
from gpiozero import Servo
from time import sleep
import math
import matplotlib.pyplot as plt

# -------- CONFIG --------
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

PAN_PIN = 18
TILT_PIN = 13

PAN_STEP = 3
TILT_STEP = 4

PAN_MIN = -30
PAN_MAX = 30
TILT_MIN = -10
TILT_MAX = 10

HEIGHT_CM = 100  # περίπου 1m πάνω από τη μακέτα

# -------- INIT --------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

pan = Servo(PAN_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(TILT_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

points_3d = []
points_2d = []

print("\n📡 Starting pan–tilt scan (hexbin map)...\n")

# -------- SCAN LOOP --------
for tilt_angle in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    tilt.value = tilt_angle / 90
    sleep(0.3)

    scan_angles = list(range(PAN_MIN, PAN_MAX + 1, PAN_STEP))
    if tilt_angle % 2 == 0:
        scan_angles.reverse()

    for pan_angle in scan_angles:
        pan.value = pan_angle / 90
        sleep(0.07)

        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59:
            dist = data[2] + data[3] * 256

            # spherical -> cartesian
            a = math.radians(pan_angle)
            b = math.radians(tilt_angle)

            x = dist * math.cos(b) * math.sin(a)
            y = dist * math.sin(b)
            z = HEIGHT_CM - dist * math.cos(b) * math.cos(a)

            points_3d.append((x, y, z))
            points_2d.append((x, z))  # κάτοψη: X vs Z

            print(f"Tilt:{tilt_angle:>3}°  Pan:{pan_angle:>3}° → {dist}cm")

print("\n✅ Scan complete. Drawing maps...\n")

# -------- PLOT --------
fig = plt.figure(figsize=(10,5))

# 3D view
ax3d = fig.add_subplot(121, projection='3d')
xs, ys, zs = zip(*points_3d)
ax3d.scatter(xs, ys, zs, s=6)
ax3d.set_title("3D Scan View")
ax3d.set_xlim(-60, 60)
ax3d.set_ylim(-40, 40)
ax3d.set_zlim(0, HEIGHT_CM)

# 2D floor map με hexbin
ax2d = fig.add_subplot(122)
px, pz = zip(*points_2d)
hb = ax2d.hexbin(px, pz, gridsize=30, cmap="gray_r")  # γεμάτος χάρτης
ax2d.set_xlim(-60, 60)
ax2d.set_ylim(0, 100)
ax2d.set_title("Top-Down Floor Map (Hexbin Density)")

plt.tight_layout()
plt.show()
