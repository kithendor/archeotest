import serial
from gpiozero import Servo
from time import sleep
import math
import matplotlib.pyplot as plt

# ----- CONFIG -----
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200
SERVO_PIN = 18
STEP_DEGREES = 5
MOVEMENT_STEP = 5  # cm forward per full scan cycle (simulate drone movement)

# ----- INIT -----
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
servo = Servo(SERVO_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

scan_points = []
forward_offset = 0

plt.ion()
fig, ax = plt.subplots()
ax.set_title("Live 2D LiDAR Mapping")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.set_aspect("equal")

print("\n📍 Starting LIVE mapping...  (CTRL+C to stop)\n")

while True:
    scan_line = []
    direction = 1
    angles = list(range(-90, 91, STEP_DEGREES))

    # If going back and forth, reverse scanning direction
    if len(scan_points) % 2 == 1:
        angles.reverse()

    for angle in angles:
        servo.value = angle / 90
        sleep(0.1)

        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
            dist = data[2] + data[3] * 256

            # Convert polar --> cartesian
            angle_rad = math.radians(angle)
            x = dist * math.cos(angle_rad)
            y = dist * math.sin(angle_rad) + forward_offset

            scan_line.append((x, y))
            scan_points.append((x, y))

            print(f"Angle {angle:>4}° | Dist {dist:>4} cm | Point ({int(x)}, {int(y)})")

    # Move the virtual drone forward for next scan
    forward_offset += MOVEMENT_STEP

    # ----- LIVE PLOT -----
    ax.clear()
    ax.set_title("Live 2D LiDAR Mapping")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_aspect("equal")

    xs = [p[0] for p in scan_points]
    ys = [p[1] for p in scan_points]

    ax.scatter(xs, ys, s=10, c='blue')
    ax.scatter([0], [forward_offset], s=60, c='red', label="Scanner Position")
    ax.legend()

    plt.pause(0.01)
