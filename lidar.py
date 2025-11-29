import serial, math, time
from gpiozero import Servo

ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)

pan = Servo(18, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(13, min_pulse_width=0.0008, max_pulse_width=0.0022)

def read_lidar():
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            return data[0] + data[1] * 256

def scan(callback):
    for tilt_angle in range(-10, 11, 2):
        tilt.value = tilt_angle / 90
        time.sleep(0.2)

        sweep = list(range(-30, 31, 2))
        if tilt_angle % 2 == 0:
            sweep.reverse()  # snake pattern

        for pan_angle in sweep:
            pan.value = pan_angle / 90
            time.sleep(0.1)

            dist = read_lidar()

            a, b = math.radians(pan_angle), math.radians(tilt_angle)
            x = dist * math.cos(b) * math.sin(a)
            y = dist * math.sin(b)
            z = 100 - dist * math.cos(b) * math.cos(a)

            callback({"x": x, "y": y, "z": z})
