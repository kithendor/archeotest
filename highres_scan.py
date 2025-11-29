import serial
import math
import csv
from time import sleep
from gpiozero import Servo
import numpy as np
import open3d as o3d

# ===== CONFIGURATION =====
SERIAL_PORT = "/dev/ttyS0"
BAUD_RATE = 115200

PAN_PIN = 18
TILT_PIN = 13

# Higher resolution scanning
PAN_MIN, PAN_MAX = -30, 30
TILT_MIN, TILT_MAX = -10, 10

PAN_STEP = 1   # smaller step = higher resolution
TILT_STEP = 1

HEIGHT_CM = 100   # distance from LiDAR to ground

# ===== INIT =====
print("\n🔌 Connecting to LiDAR...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

pan = Servo(PAN_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)
tilt = Servo(TILT_PIN, min_pulse_width=0.0008, max_pulse_width=0.0022)

points = []


def read_lidar():
    """Reads one valid TFmini-S packet."""
    while True:
        if ser.read() == b'Y':
            if ser.read() == b'Y':
                data = ser.read(7)
                return data[0] + data[1] * 256


def read_avg(samples=5):
    """Returns average reading for stability."""
    values = []
    for _ in range(samples):
        values.append(read_lidar())
        sleep(0.01)
    return sum(values) / len(values)


print("\n📡 Starting High-Resolution Scan...\n")

for tilt_angle in range(TILT_MIN, TILT_MAX + 1, TILT_STEP):
    tilt.value = tilt_angle / 90
    sleep(0.25)

    sweep = list(range(PAN_MIN, PAN_MAX + 1, PAN_STEP))
    
    # Snake pattern to reduce unnecessary travel time
    if tilt_angle % 2 == 0:
        sweep.reverse()

    for pan_angle in sweep:
        pan.value = pan_angle / 90
        sleep(0.15)

        dist = read_avg(5)

        a = math.radians(pan_angle)
        b = math.radians(tilt_angle)

        x = dist * math.cos(b) * math.sin(a)
        y = dist * math.sin(b)
        z = HEIGHT_CM - dist * math.cos(b) * math.cos(a)

        points.append((x, y, z))
        print(f"Tilt:{tilt_angle:>3}°, Pan:{pan_angle:>3}° → {dist:.1f} cm")

print("\n💾 Saving scan_points.csv ...")

with open("scan_points.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y", "z"])
    writer.writerows(points)

print("✔️ DONE: scan_points.csv created.")
print("📦 Generating 3D visualization...\n")

# ===== OPEN3D VISUALIZATION =====

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))

print("📍 Opening 3D Viewer... rotate with mouse, scroll to zoom.")
o3d.visualization.draw_geometries([pcd])


# ===== OPTIONAL MESH GENERATION =====

mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd,
    o3d.utility.DoubleVector([5, 10, 15])
)

mesh.paint_uniform_color([0.6, 0.6, 0.6])
mesh.compute_vertex_normals()

print("\n🧱 Showing reconstructed mesh...")
o3d.visualization.draw_geometries([mesh])

print("\n🎉 Scan complete!")
