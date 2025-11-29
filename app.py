from flask import Flask, render_template
from flask_socketio import SocketIO
from lidar import scan

app = Flask(__name__)
socketio = SocketIO(app)

@app.route("/")
def index():
    return render_template("viewer.html")

@socketio.on("start_scan")
def start_scan_event():
    scan(lambda point: socketio.emit("point", point))

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
