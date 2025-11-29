import serial
from gpiozero import Servo
from time import sleep
import math
import numpy as np
import matplotlib.pyplot as plt
from statistics import median

# ---------------- CONFIG ----------------
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

HEIGHT_CM = 100

# Filtering settings
MEDIAN_WINDOW = 5         # how many readings to smooth per point
DIST_MAX_CHANGE = 30      # max jump allowed between points (cm) before considered noise

# ---------------- INIT ----------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

pan = Servo(PAN_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(TILT_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

raw_points = []
filtered_2d = []

plt.ion()
fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111)
ax.set_title("2D Smoothed Floor Map")

print("\n📡 Starting Smoothed Scan...\n")

# ---------------- SCAN ----------------
for tilt_angle in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    tilt.value = tilt_angle / 90
    sleep(0.3)

    scan_angles = list(range(PAN_MIN, PAN_MAX + 1, PAN_STEP))
    if tilt_angle % 2 == 0:
        scan_angles.reverse()

    dist_buffer = []

    for pan_angle in scan_angles:
        pan.value = pan_angle / 90
        sleep(0.06)

        # Read LiDAR frame
        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59:
            dist = data[2] + data[3] * 256

            # 1) Καταγράφουμε τη μέτρηση για filtering
            dist_buffer.append(dist)

            # 2) Μόνο όταν έχουμε αρκετές μετρήσεις εφαρμόζουμε median filter
            if len(dist_buffer) >= MEDIAN_WINDOW:
                smooth_dist = median(dist_buffer[-MEDIAN_WINDOW:])

                # 3) Αφαιρούμε outlier: αν η μέτρηση διαφέρει ΠΟΛΥ από την προηγούμενη, την αγνοούμε
                if filtered_2d:
                    last_x, last_y = filtered_2d[-1]
                    expected_dist = math.sqrt(last_x**2 + last_y**2)
                    if abs(smooth_dist - expected_dist) > DIST_MAX_CHANGE:
                        continue

                # ---- Convert to XY and apply smoothing ----
                a = math.radians(pan_angle)
                x = smooth_dist * math.sin(a)
                y = HEIGHT_CM - smooth_dist * math.cos(a)

                filtered_2d.append((x, y))

                # ---- LIVE UPDATE ----
                ax.clear()
                px, py = zip(*filtered_2d)
                ax.scatter(px, py, s=20, marker="s", c='black')
                ax.set_xlim(-60, 60)
                ax.set_ylim(0, 100)
                ax.set_title("2D Floor Map (Filtered + Smoothed)")
                plt.pause(0.001)

print("\n✅ Smoothed Mapping Complete.\n")
