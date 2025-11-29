import serial
from gpiozero import Servo
from time import sleep
import math
import numpy as np
import matplotlib.pyplot as plt

SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200
SERVO_PIN = 18
STEP_DEGREES = 5
FORWARD_STEP_CM = 10

GRID_RES = 2  # cm per pixel
GRID_W = GRID_H = 400  # grid resolution

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
servo = Servo(SERVO_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

grid = np.zeros((GRID_H, GRID_W))
forward_offset_cm = 0

plt.ion()
fig, ax = plt.subplots()
ax.set_title("LIVE 2D MAP")

while True:
    angles = range(-90, 91, STEP_DEGREES)

    for angle in angles:
        servo.value = angle / 90
        sleep(0.04)

        data = ser.read(9)
        if len(data) == 9 and data[0] == 0x59:
            dist = data[2] + data[3] * 256

            # Convert to cartesian coordinates
            a = math.radians(angle)
            x_cm = dist * math.sin(a)
            y_cm = dist * math.cos(a) + forward_offset_cm

            gx = int(x_cm / GRID_RES + GRID_W / 2)
            gy = int(y_cm / GRID_RES)

            if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                grid[GRID_H - gy - 1, gx] = 1

    # move forward for next scan
    forward_offset_cm += FORWARD_STEP_CM

    ax.clear()
    ax.imshow(grid, cmap='gray')
    ax.set_title("LIVE 2D MAP")
    plt.pause(0.01)
