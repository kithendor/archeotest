import serial
from gpiozero import Servo
from time import sleep
import math
import numpy as np
import matplotlib.pyplot as plt

# ---- CONFIG ----
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200
SERVO_PIN = 18
STEP_DEGREES = 5
MOVEMENT_STEP = 5  # cm movement forward per scan

GRID_RESOLUTION = 5  # cm per pixel
GRID_SIZE = 200  # pixels (map 10m x 10m at 5cm resolution)

# ---- INIT ----
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
servo = Servo(SERVO_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

grid = np.zeros((GRID_SIZE, GRID_SIZE))
forward_offset = GRID_SIZE // 2  # scanner starts in middle

plt.ion()
fig, ax = plt.subplots()
ax.set_title("LIVE 2D MAP (Occupancy Grid)")
print("\n📡 Mapping started...\n")

while True:
    angles = list(range(-90, 91, STEP_DEGREES))
    if forward_offset % 2 == 1:
        angles.reverse()

    for angle in angles:
        servo.value = angle / 90
        sleep(0.06)

        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59:
            dist = data[2] + data[3] * 256

            # Convert to XY real world coords
            angle_rad = math.radians(angle)
            x = dist * math.cos(angle_rad)
            y = dist * math.sin(angle_rad) + forward_offset * GRID_RESOLUTION

            # Convert to grid coords
            gx = int(x / GRID_RESOLUTION + GRID_SIZE // 2)
            gy = int(y / GRID_RESOLUTION)

            # Check bounds
            if 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE:
                grid[GRID_SIZE - gy - 1, gx] = 1  # mark cell as occupied

    # Move forward as if the drone moves
    forward_offset += MOVEMENT_STEP / GRID_RESOLUTION

    # ----- DRAW MAP -----
    ax.clear()
    ax.set_title("LIVE 2D MAP (Occupancy Grid)")
    ax.imshow(grid, cmap="gray")
    plt.pause(0.01)
