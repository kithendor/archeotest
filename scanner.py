import serial
import time
import math
import numpy as np
import pandas as pd
import pigpio
import plotly.graph_objects as go
import plotly.express as px
import plotly.io as pio
import json
import os
import threading


live_lock = threading.Lock()
live_points = []   # list of (x,y,z)
live_scan_id = 0   # αλλάζει σε κάθε scan

# ---------------------------------------------------------
# HARDWARE SETTINGS
# ---------------------------------------------------------
UART_PORT = "/dev/ttyS0"
UART_BAUD = 115200

PAN_PIN = 13     # <-- Pan servo here
TILT_PIN = 18    # <-- Tilt servo here

HEIGHT_CM = 70   # Height of LiDAR from table

PAN_MIN, PAN_MAX, PAN_STEP = -25, 25, 5
TILT_MIN, TILT_MAX, TILT_STEP = -25, 25, 5

GRID_SIZE = 2.0         # Size of grid cells in cm
BUILDING_THRESHOLD = 5  # cm above ground to consider “walls”
STL_NAME = "scan_mesh.stl"

# ---------------------------------------------------------
# STATUS FLAGS FOR WEB APP
# ---------------------------------------------------------
is_scanning = False
scan_progress = 0

# ---------------------------------------------------------
# START pigpio + UART
# ---------------------------------------------------------
pi = pigpio.pi()
pi.set_mode(PAN_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)

ser = serial.Serial(UART_PORT, UART_BAUD, timeout=0.2)

# ---------------------------------------------------------
# SERVO HELPERS
# ---------------------------------------------------------
def pulse(angle):
    """Convert angle (-90..90) to pulse width."""
    return int(500 + (angle + 90) * 2000 / 180)

def move(pin, angle, smooth=True):
    target = pulse(angle)
    current = pi.get_servo_pulsewidth(pin)

    # If servo is uninitialized
    if current < 500 or current > 2500:
        current = target

    if smooth:
        step = 8
        while abs(current - target) > step:
            current += step if target > current else -step
            pi.set_servo_pulsewidth(pin, current)
            time.sleep(0.003)
    pi.set_servo_pulsewidth(pin, target)
    time.sleep(0.04)

# ---------------------------------------------------------
# LIDAR READER (with buffer flush)
# ---------------------------------------------------------
def read_lidar():
    ser.reset_input_buffer()
    while True:
        if ser.read() == b'Y' and ser.read() == b'Y':
            data = ser.read(7)
            if len(data) == 7:
                dist = data[0] + data[1]*256
                return dist

# ---------------------------------------------------------
# MAIN SCAN ROUTINE
# ---------------------------------------------------------
def run_scan(step=2):
    global is_scanning, scan_progress, live_points, live_scan_id
    is_scanning = True
    scan_progress = 0

    with live_lock:
        live_points = []
        live_scan_id += 1
    
    pi.set_servo_pulsewidth(PAN_PIN, 1500)
    pi.set_servo_pulsewidth(TILT_PIN, 1500)
    time.sleep(0.3)

    xs, ys, zs = [], [], []

    total_moves = ((abs(TILT_MAX - TILT_MIN)//step)+1) * ((abs(PAN_MAX - PAN_MIN)//step)+1)
    done = 0

    for tilt in range(TILT_MIN, TILT_MAX + 1, step):
        move(TILT_PIN, tilt)

        sweep = range(PAN_MIN, PAN_MAX + 1, step)
        if tilt % 2 == 0:
            sweep = reversed(list(sweep))

        for pan in sweep:
            move(PAN_PIN, pan)
            dist = read_lidar()

            a = math.radians(pan)
            b = math.radians(tilt)

            x = dist * math.cos(b) * math.sin(a)
            y = dist * math.sin(b)
            z = HEIGHT_CM - dist * math.cos(a) * math.cos(b)

            xs.append(x)
            ys.append(y)
            zs.append(z)
            
            with live_lock:
                live_points.append((x, y, z))

            done += 1
            scan_progress = int((done / total_moves) * 100)

    # Stop servos
    pi.set_servo_pulsewidth(PAN_PIN, 0)
    pi.set_servo_pulsewidth(TILT_PIN, 0)

    # Save CSV
    df = pd.DataFrame({"x": xs, "y": ys, "z": zs})
    df.to_csv("scan_points.csv", index=False)

    # Make visualization helpers
    prepare_3d_plot(xs, ys, zs)
    prepare_2d_map(xs, ys, zs)
    save_stl(xs, ys, zs)

    is_scanning = False
    scan_progress = 100


# ---------------------------------------------------------
# 3D PLOT GENERATOR
# ---------------------------------------------------------

def load_markers_file():
    try:
        base_dir = os.path.dirname(__file__)
        path = os.path.join(base_dir, "markers.json")
        if not os.path.exists(path):
            return []
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []

def snap_markers_to_cloud(markers, xs, ys, zs):
    # βρίσκουμε z από το κοντινότερο σημείο στο point cloud (με βάση x,y)
    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)

    mx, my, mz, mid, mnote = [], [], [], [], []
    for m in markers:
        x = float(m.get("x", 0))
        y = float(m.get("y", 0))
        # απόσταση μόνο σε x-y (για top-down mapping)
        d2 = (xs - x)**2 + (ys - y)**2
        k = int(np.argmin(d2))
        mx.append(x)
        my.append(y)
        mz.append(float(zs[k]) + 1.0)  # +1cm για να "κάθεται" από πάνω
        mid.append(m.get("id", "F-??"))
        mnote.append(m.get("note", ""))
    return mx, my, mz, mid, mnote




last_3d_html = "<h2>No scan yet</h2>"

def prepare_3d_plot(xs, ys, zs):
    global last_3d_html

    fig = go.Figure()

    # main point cloud
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode='markers',
        name='Scan',
        marker=dict(size=3, color=zs, colorscale="Viridis"),
        hovertemplate="x=%{x:.1f} cm<br>y=%{y:.1f} cm<br>z=%{z:.1f} cm<extra></extra>"
    ))

    # markers snapped to cloud
    markers = load_markers_file()
    if markers:
        mx, my, mz, mid, mnote = snap_markers_to_cloud(markers, xs, ys, zs)

        fig.add_trace(go.Scatter3d(
            x=mx, y=my, z=mz,
            mode='markers+text',
            name='Markers',
            text=mid,
            textposition='top center',
            customdata=mnote,
            marker=dict(size=6, color='red'),
            hovertemplate="<b>%{text}</b><br>%{customdata}<extra></extra>"
        ))

    fig.update_layout(
        width=900,
        height=700,
        title="3D LiDAR Scan (markers snapped)",
        margin=dict(l=0, r=0, t=40, b=0),
        scene=dict(
            xaxis_title="X (cm)",
            yaxis_title="Y (cm)",
            zaxis_title="Z (cm)"
        )
    )

    # offline safe
    last_3d_html = pio.to_html(fig, full_html=False, include_plotlyjs=True)


def get_3d_html():
    # κάθε φορά, ξαναχτίσε 3D ώστε να "τραβάει" τα νεότερα markers.json
    refresh_3d_from_csv()
    return last_3d_html


def refresh_3d_from_csv():
    if not os.path.exists("scan_points.csv"):
        return False
    df = pd.read_csv("scan_points.csv")
    prepare_3d_plot(df["x"].tolist(), df["y"].tolist(), df["z"].tolist())
    return True

# ---------------------------------------------------------
# 2D HEATMAP
# ---------------------------------------------------------
last_2d_html = "<h2>No scan yet</h2>"

def prepare_2d_map(xs, ys, zs):
    global last_2d_html

    xs_arr = np.array(xs)
    ys_arr = np.array(ys)
    zs_arr = np.array(zs)

    # Remove ground plane (flatten)
    A = np.c_[xs_arr, ys_arr, np.ones_like(xs_arr)]
    coeffs, _, _, _ = np.linalg.lstsq(A, zs_arr, rcond=None)
    plane = A @ coeffs
    zf = zs_arr - plane

    # Grid bounds
    x_min, x_max = xs_arr.min(), xs_arr.max()
    y_min, y_max = ys_arr.min(), ys_arr.max()

    nx = int((x_max - x_min) / GRID_SIZE) + 1
    ny = int((y_max - y_min) / GRID_SIZE) + 1

    grid = np.full((ny, nx), np.nan)

    ix = ((xs_arr - x_min) / GRID_SIZE).astype(int)
    iy = ((ys_arr - y_min) / GRID_SIZE).astype(int)

    for k in range(len(zf)):
        i = ix[k]
        j = iy[k]
        val = zf[k]
        cur = grid[j, i]
        if np.isnan(cur) or val > cur:
            grid[j, i] = val

    # IMPORTANT: give real-world axis coordinates (cm), so click returns cm x/y
    x_coords = x_min + np.arange(nx) * GRID_SIZE
    y_coords = y_min + np.arange(ny) * GRID_SIZE

    fig = px.imshow(
        grid,
        x=x_coords,
        y=y_coords,
        origin="lower",
        color_continuous_scale="Viridis",
        title="Top-Down Heightmap (click to add marker)"
    )

    fig.update_layout(
        width=900,
        height=700,
        margin=dict(l=30, r=30, t=50, b=30),
    )

    # Make a stable div id so we can attach JS
    plot_html = pio.to_html(fig, full_html=False, include_plotlyjs=True)

    # JS: load markers, draw them, click-to-add, click-to-delete
    js = """
<script>
const plotDiv = document.querySelector('.plotly-graph-div');

async function loadMarkers() {
  const res = await fetch('/get_markers');
  return await res.json();
}

function markerTrace(markers) {
  return {
    x: markers.map(m => m.x),
    y: markers.map(m => m.y),
    mode: 'markers+text',
    name: 'Markers',
    text: markers.map(m => m.id),
    textposition: 'top center',
    customdata: markers.map(m => m.id),
    marker: { size: 10, symbol: 'circle' },
    hovertemplate: '<b>%{text}</b><br>x=%{x:.1f} cm<br>y=%{y:.1f} cm<extra></extra>'
  };
}

async function refreshMarkers() {
  const markers = await loadMarkers();
  const idx = plotDiv.data.findIndex(tr => tr.name === 'Markers');
  if (idx !== -1) await Plotly.deleteTraces(plotDiv, idx);
  if (markers.length > 0) await Plotly.addTraces(plotDiv, markerTrace(markers));
}

plotDiv.on('plotly_click', async function(evt) {
  const pt = evt.points[0];

  if (pt.data && pt.data.name === 'Markers') {
    const id = pt.customdata;
    if (confirm('Delete marker ' + id + '?')) {
      await fetch('/delete_marker', {
        method: 'POST',
        headers: {'Content-Type':'application/json'},
        body: JSON.stringify({id: id})
      });
      await refreshMarkers();
    }
    return;
  }

  const x = pt.x;
  const y = pt.y;

  const note = prompt('Σημείωση για νέο εύρημα:', '');
  if (note === null) return;

  let photo = "";
  if (confirm("Να συνδεθεί η τελευταία φωτογραφία με αυτό το εύρημα;")) {
    const r = await fetch("/last_photo");
    const j = await r.json();
    if (j.status === "ok") photo = j.file;
  }


  await refreshMarkers();
});

refreshMarkers();
</script>
"""

    last_2d_html = plot_html + js

def get_2d_html():
    return last_2d_html


# ---------------------------------------------------------
# STL EXPORT
# ---------------------------------------------------------
def save_stl(xs, ys, zs):
    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)

    # Flatten ground
    A = np.c_[xs, ys, np.ones_like(xs)]
    coeffs, _, _, _ = np.linalg.lstsq(A, zs, rcond=None)
    plane = A @ coeffs
    zf = zs - plane

    # Grid
    x_min, x_max = xs.min(), xs.max()
    y_min, y_max = ys.min(), ys.max()

    nx = int((x_max - x_min) / GRID_SIZE) + 1
    ny = int((y_max - y_min) / GRID_SIZE) + 1

    grid = np.full((ny, nx), np.nan)

    ix = ((xs - x_min) / GRID_SIZE).astype(int)
    iy = ((ys - y_min) / GRID_SIZE).astype(int)

    for k in range(len(zf)):
        i = ix[k]
        j = iy[k]
        val = zf[k]
        cur = grid[j, i]
        if np.isnan(cur) or val > cur:
            grid[j, i] = val

    # Save STL
    with open(STL_NAME, "w") as f:
        f.write("solid scan\n")

        def tri(a, b, c):
            f.write(" facet normal 0 0 1\n")
            f.write("  outer loop\n")
            f.write(f"   vertex {a[0]} {a[1]} {a[2]}\n")
            f.write(f"   vertex {b[0]} {b[1]} {b[2]}\n")
            f.write(f"   vertex {c[0]} {c[1]} {c[2]}\n")
            f.write("  endloop\n")
            f.write(" endfacet\n")

        # Top surface
        for j in range(ny - 1):
            for i in range(nx - 1):
                h00 = grid[j, i]
                h10 = grid[j, i+1]
                h01 = grid[j+1, i]
                h11 = grid[j+1, i+1]
                if np.isnan(h00) or np.isnan(h10) or np.isnan(h01) or np.isnan(h11):
                    continue

                x0 = x_min + i * GRID_SIZE
                x1 = x_min + (i+1) * GRID_SIZE
                y0 = y_min + j * GRID_SIZE
                y1 = y_min + (j+1) * GRID_SIZE

                v1 = (x0, y0, h00)
                v2 = (x1, y0, h10)
                v3 = (x0, y1, h01)
                v4 = (x1, y1, h11)

                tri(v1, v2, v3)
                tri(v2, v4, v3)

        f.write("endsolid scan\n")
        
        
def get_live_chunk(after=0, max_points=1500):
    # επιστρέφει (scan_id, next_index, points_list)
    with live_lock:
        sid = live_scan_id
        n = len(live_points)
        if after < 0: after = 0
        pts = live_points[after: min(n, after + max_points)]
        return sid, min(n, after + max_points), pts

