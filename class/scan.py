import serial
import pigpio
import time

# ==============================
# ΡΥΘΜΙΣΕΙΣ ΥΛΙΚΟΥ
# ==============================

PORT = "/dev/ttyS0"
BAUD = 115200

PAN_PIN = 13
TILT_PIN = 18


# ==============================
# ΑΡΧΙΚΟΠΟΙΗΣΕΙΣ
# ==============================

ser = serial.Serial(PORT, BAUD, timeout=0.5)
pi = pigpio.pi()

# Κεντράρισμα servo στην αρχή (1500 μs = 0°)
pi.set_servo_pulsewidth(PAN_PIN, 1500)
pi.set_servo_pulsewidth(TILT_PIN, 1500)
time.sleep(1)


# ==============================
# ΣΥΝΑΡΤΗΣΗ ΓΙΑ ΤΟ LiDAR
# ==============================

def read_lidar():
    """
    Διαβάζει μία μέτρηση από τον LiDAR (TFmini-S)
    """
    ser.reset_input_buffer()

    while True:
        # Header του TFmini-S: 0x59 0x59 = 'Y' 'Y'
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            distance = data[0] + data[1] * 256
            return distance


# ==============================
# ΣΥΝΑΡΤΗΣΗ ΓΙΑ ΤΑ SERVO
# ==============================

def angle_to_pulse(angle):
    """
    Μετατρέπει μοίρες (-90 έως 90) σε pulse (500 έως 2500)
    """
    return int(500 + (angle + 90) * 2000 / 180)


def move_servo(pin, angle):
    """
    Κινεί ένα servo σε συγκεκριμένη γωνία
    """
    pulse = angle_to_pulse(angle)
    pi.set_servo_pulsewidth(pin, pulse)
    time.sleep(0.05)


# ==============================
# ΚΥΡΙΟ ΠΡΟΓΡΑΜΜΑ ΣΑΡΩΣΗΣ
# ==============================

print("Ξεκινάει η σάρωση...")

for tilt in range(-10, 11, 5):
    move_servo(TILT_PIN, tilt)

    for pan in range(-30, 31, 5):
        move_servo(PAN_PIN, pan)

        distance = read_lidar()

        print("PAN:", pan, "| TILT:", tilt, "| Distance:", distance, "cm")

print("Τέλος σάρωσης!")

# Σταμάτημα servo στο τέλος
pi.set_servo_pulsewidth(PAN_PIN, 0)
pi.set_servo_pulsewidth(TILT_PIN, 0)
pi.stop()
