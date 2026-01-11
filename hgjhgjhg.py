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
