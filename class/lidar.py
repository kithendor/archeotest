import serial

PORT = "/dev/ttyS0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)

def read_lidar():
    ser.reset_input_buffer()
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            return data[0] + data[1] * 256

while True:
    print("Distance:", read_lidar(), "cm")
