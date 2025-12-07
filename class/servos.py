import pigpio
import time

PAN_PIN = 13
TILT_PIN = 18

pi = pigpio.pi()

def angle_to_pulse(angle):
    return int(500 + (angle + 90) * 2000 / 180)

for a in range(-45, 46, 5):
    pi.set_servo_pulsewidth(PAN_PIN, angle_to_pulse(a))
    time.sleep(0.05)
