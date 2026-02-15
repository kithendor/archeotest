from flask import Flask, jsonify, send_file, render_template
from flask_cors import CORS
import threading
import scanner
import os, glob, subprocess
from datetime import datetime
import os, glob, subprocess, time   # πρόσθεσε time αν δεν υπάρχει
import json
from flask import request
import zipfile
from io import BytesIO
import json
import requests


app = Flask(__name__)
CORS(app)

scan_thread = None

BASE_DIR = os.path.dirname(__file__)
PHOTO_DIR = os.path.join(BASE_DIR, "static", "photos")
VIDEO_DIR = os.path.join(BASE_DIR, "static", "videos")
os.makedirs(PHOTO_DIR, exist_ok=True)
os.makedirs(VIDEO_DIR, exist_ok=True)

MARKERS_FILE = os.path.join(BASE_DIR, "markers.json")


def load_markers():
    if not os.path.exists(MARKERS_FILE):
        return []
    try:
        with open(MARKERS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []

def save_markers(markers):
    with open(MARKERS_FILE, "w", encoding="utf-8") as f:
        json.dump(markers, f, ensure_ascii=False, indent=2)

def next_marker_id(markers):
    # IDs like F-01, F-02 ...
    nums = []
    for m in markers:
        mid = str(m.get("id", ""))
        if mid.startswith("F-"):
            try:
                nums.append(int(mid.split("-")[1]))
            except:
                pass
    n = max(nums) + 1 if nums else 1
    return f"F-{n:02d}"

# ---------------- LiDAR ROUTES ----------------

@app.route("/")
def home():
    return render_template("index2.html")

@app.route("/scan")
def start_scan():
    global scan_thread

    if scanner.is_scanning:
        return jsonify({"status": "busy"})

    step = request.args.get("step", default=2, type=int)

    scan_thread = threading.Thread(target=scanner.run_scan, args=(step,))
    scan_thread.start()

    return jsonify({"status": "started"})


@app.route("/status")
def status():
    return jsonify({
        "scanning": scanner.is_scanning,
        "progress": scanner.scan_progress
    })

@app.route("/view3d")
def three_d():
    return scanner.get_3d_html()

@app.route("/view2d")
def two_d():
    return scanner.get_2d_html()

@app.route("/live3d")
def live3d():
    return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Live 3D Scan</title>
  <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
</head>
<body style="margin:0;background:#000;color:#fff;font-family:Arial;">
  <div style="padding:10px;background:#111;border-bottom:1px solid #333;">
    <b>Live 3D Scan</b> • updates every 1.5s
    <span id="info" style="color:#aaa;margin-left:10px;"></span>
  </div>
  <div id="plot" style="width:100vw;height:calc(100vh - 44px);"></div>

<script>
let after = 0;
let scanId = null;

const layout = {
  paper_bgcolor: "#999",
  plot_bgcolor: "#000",
  scene: {
    xaxis: {title:"X", gridcolor:"#222", zerolinecolor:"#222"},
    yaxis: {title:"Y", gridcolor:"#222", zerolinecolor:"#222"},
    zaxis: {title:"Z", gridcolor:"#222", zerolinecolor:"#222"},
  },
  margin: {l:0, r:0, t:0, b:0},
};

Plotly.newPlot('plot', [{
  type:'scatter3d',
  mode:'markers',
  x:[], y:[], z:[],
  marker:{size:3, color:[], colorscale:'Viridis'}
}], layout);

async function tick(){
  try{
    const r = await fetch(`/live_points?after=${after}`);
    const j = await r.json();

    if(scanId === null) scanId = j.scan_id;
    // αν ξεκίνησε νέο scan, κάνε reset
    if(j.scan_id !== scanId){
      scanId = j.scan_id;
      after = 0;
      Plotly.restyle('plot', {x:[[]], y:[[]], z:[[]], 'marker.color':[[]]});
    }

    if(j.points && j.points.length){
      const xs = j.points.map(p=>p[0]);
      const ys = j.points.map(p=>p[1]);
      const zs = j.points.map(p=>p[2]);

      Plotly.extendTraces('plot', {
        x:[xs], y:[ys], z:[zs],
        'marker.color':[zs]
      }, [0], 20000); // κρατάμε max 20k σημεία στο plot

      after = j.next;
    }

    document.getElementById('info').textContent =
      `points: ${j.total} • scanning: ${j.scanning} • progress: ${j.progress}%`;

  }catch(e){
    document.getElementById('info').textContent = "live error: " + e;
  }
}

setInterval(tick, 1500);
tick();
</script>
</body>
</html>
"""




# ---------------- Camera helpers ----------------

def get_latest_file(folder, pattern):
    files = glob.glob(os.path.join(folder, pattern))
    if not files:
        return None
    return max(files, key=os.path.getctime)


# ---------------- Photo routes ----------------

@app.route("/capture_photo")
def capture_photo():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"photo_{ts}.jpg"
    filepath = os.path.join(PHOTO_DIR, filename)

    cmd = [
        "rpicam-still",
        "-n",               # no preview window
        "-o", filepath,
        "--width", "1280",
        "--height", "720"
    ]

    try:
        subprocess.run(cmd, check=True)
        return jsonify({"status": "ok", "file": f"/static/photos/{filename}"})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500


@app.route("/show_photo")
def show_photo():
    latest = get_latest_file(PHOTO_DIR, "*.jpg")
    if not latest:
        return "<h3 style='color:white;background:black;'>No photos yet</h3>"

    rel = os.path.relpath(latest, BASE_DIR)

    return f"""
    <html><body style='background:#000; text-align:center;'>
    <h2 style='color:white;'>Last Photo</h2>
    <img src='/{rel}' style='max-width:100%; height:auto;'/>
    </body></html>
    """


# ---------------- Video routes ----------------

@app.route("/record_video")
def record_video():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_name = f"video_{ts}.h264"
    mp4_name = f"video_{ts}.mp4"

    raw_path = os.path.join(VIDEO_DIR, raw_name)
    mp4_path = os.path.join(VIDEO_DIR, mp4_name)

    # 1) Γυρνάμε 5s βίντεο σε H.264
    cmd_rec = [
        "rpicam-vid",
        "-n",
        "-t", "5000",      # 5000ms = 5 δευτερόλεπτα
        "-o", raw_path
    ]

    try:
        subprocess.run(cmd_rec, check=True)

        # 2) Το μετατρέπουμε σε MP4 με ffmpeg (χωρίς επανακωδικοποίηση)
        cmd_conv = [
            "ffmpeg",
            "-y",              # overwrite χωρίς ερώτηση
            "-framerate", "30",
            "-i", raw_path,
            "-c", "copy",
            mp4_path
        ]
        subprocess.run(cmd_conv, check=True)

        # (προαιρετικό) σβήνουμε το .h264 για να μην γεμίζει ο δίσκος
        try:
            os.remove(raw_path)
        except FileNotFoundError:
            pass

        return jsonify({"status": "ok", "file": f"/static/videos/{mp4_name}"})

    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500
    




@app.route("/show_video")
def show_video():
    latest = get_latest_file(VIDEO_DIR, "*.mp4")
    if not latest:
        return "<h3 style='color:white;background:black;'>No videos yet</h3>"

    rel = os.path.relpath(latest, BASE_DIR)

    return f"""
    <html><body style='background:#000; text-align:center;'>
    <h2 style='color:white;'>Last Video</h2>
    <video controls autoplay style='max-width:100%; height:auto;'>
      <source src='/{rel}' type='video/mp4'>
      Your browser does not support the video tag.
    </video>
    </body></html>
    """

@app.route("/shutdown", methods=["POST"])
def shutdown_pi():
    # Στέλνουμε αμέσως απάντηση στο browser
    def do_shutdown():
        time.sleep(1)
        subprocess.run(["sudo", "/usr/sbin/shutdown", "-h", "now"])

    threading.Thread(target=do_shutdown).start()

    return jsonify({"status": "ok", "message": "Shutting down..."})




@app.route("/last_photo")
def last_photo():
    latest = get_latest_file(PHOTO_DIR, "*.jpg")
    if not latest:
        return jsonify({"status": "none"})
    rel = "/" + os.path.relpath(latest, BASE_DIR)
    return jsonify({"status": "ok", "file": rel})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)

