import serial, time

ser = serial.Serial("/dev/ttyS0", 115200, timeout=0.2)

def read_lidar():
    # καθαρίζουμε ΟΛΑ τα παλιά bytes
    ser.reset_input_buffer()

    # ψάχνουμε για header YY
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            if len(data) == 7:
                dist = data[0] + data[1] * 256
                strength = data[2] + data[3] * 256
                return dist, strength

print("TFmini-S FLUSH test... move the object around\n")

while True:
    dist, strength = read_lidar()
    print(f"Distance: {dist} cm | Strength: {strength}")
    time.sleep(0.05)
