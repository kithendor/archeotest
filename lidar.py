import serial
import time

PORT = "/dev/ttyS0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("📡 TFmini-S RAW Distance Test (cm)")
print("Press CTRL+C to stop.\n")

while True:
    if ser.read() == b'Y' and ser.read() == b'Y':
        data = ser.read(7)
        distance = data[0] + data[1] * 256    # cm
        strength = data[2] + data[3] * 256    # optional info

        print(f"Distance: {distance:4} cm   | Strength: {strength}")
        time.sleep(0.02)
