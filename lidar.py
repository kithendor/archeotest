import serial
import time

ser = serial.Serial("/dev/ttyS0", 115200, timeout=0.2)

def read_lidar():
    if ser.read() != b'Y':
        return None
    if ser.read() != b'Y':
        return None
    data = ser.read(7)
    if len(data) != 7:
        return None
    dist = data[0] + data[1]*256
    strength = data[2] + data[3]*256
    return dist, strength

print("TFmini-S test. Move object to 10–150 cm. CTRL+C to stop.\n")

while True:
    result = read_lidar()
    if result:
        dist, strength = result
        print(f"Distance: {dist:4} cm | Strength: {strength}")
    time.sleep(0.05)
