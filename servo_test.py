import pigpio
from time import sleep

PAN_PIN = 18    # βάλε τους σωστούς σου GPIO
TILT_PIN = 19

pi = pigpio.pi()
if not pi.connected:
    print("Pigpio not connected!")
    exit()

def set_angle(pin, angle):
    # MG90S uses ~500–2500us pulses
    pulse = int(500 + (angle + 90) * 2000 / 180)
    pi.set_servo_pulsewidth(pin, pulse)
    print(f"{pin} -> {angle}° ({pulse}us)")

def center(pin):
    # 0° αντιστοιχεί περίπου σε 1500 μs
    pi.set_servo_pulsewidth(pin, 1500)
    print(f"Servo on pin {pin} → CENTER (1500 µs)")
    sleep(2)

print("\n=== SERVO TEST WITH PIGPIO ===")

for angle in [-90, -45, 0, 45, 90]:
    set_angle(PAN_PIN, angle)
    sleep(1)


center(SERVO_PIN)
center(SERVO_PIN2)

pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)

print("✔ Done")
