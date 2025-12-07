from flask import Flask, jsonify, send_file, render_template
from flask_cors import CORS
import threading
import scanner

app = Flask(__name__)
CORS(app)

scan_thread = None

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/scan")
def start_scan():
    global scan_thread

    if scanner.is_scanning:
        return jsonify({"status": "busy"})

    scan_thread = threading.Thread(target=scanner.run_scan)
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

@app.route("/download_stl")
def download_stl():
    return send_file("scan_mesh.stl", as_attachment=True)

@app.route("/download_csv")
def download_csv():
    return send_file("scan_points.csv", as_attachment=True)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
