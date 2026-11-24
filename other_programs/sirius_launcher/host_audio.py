"""Read and adjust the Launcher's default PipeWire audio output."""

import os
import re
import subprocess
from threading import Lock

from flask import Blueprint, jsonify, request


DEFAULT_SINK = "@DEFAULT_AUDIO_SINK@"
VOLUME_LINE = re.compile(r"Volume:\s+(\d+(?:\.\d+)?)\s*(\[MUTED\])?\s*")
OUTPUT_LABEL = re.compile(r'\bnode\.description\s*=\s*"([^"\n]+)"')


def wpctl(*args):
    env = {**os.environ, "LC_ALL": "C"}
    if not env.get("XDG_RUNTIME_DIR"):
        env["XDG_RUNTIME_DIR"] = f"/run/user/{os.getuid()}"
    return subprocess.run(
        ["wpctl", *args], env=env, check=True, capture_output=True, text=True, timeout=2,
    ).stdout


def read_audio():
    result = VOLUME_LINE.fullmatch(wpctl("get-volume", DEFAULT_SINK).strip())
    if result is None:
        raise ValueError("Unexpected wpctl volume response")
    try:
        label = OUTPUT_LABEL.search(wpctl("inspect", DEFAULT_SINK))
    except (OSError, subprocess.SubprocessError):
        label = None
    return {
        "volume": round(float(result[1]) * 100),
        "muted": result[2] is not None,
        "output": label[1] if label else None,
    }


def register_host_audio(app, auth_ok):
    routes = Blueprint("host_audio", __name__)
    lock = Lock()

    @routes.route("/api/host-audio", methods=["GET", "POST"])
    def host_audio():
        if not auth_ok():
            return jsonify(error="auth"), 401
        if request.method == "POST":
            origin = request.headers.get("Origin")
            valid_origins = (f"http://{request.host}", f"https://{request.host}")
            if (request.headers.get("Sec-Fetch-Site") == "cross-site"
                    or (origin and origin not in valid_origins)):
                return jsonify(error="forbidden"), 403
            body = request.get_json(silent=True)
            if not isinstance(body, dict) or len(body) != 1:
                return jsonify(error="invalid_audio_setting"), 400
            volume, muted = body.get("volume"), body.get("muted")
            if type(volume) is int and 0 <= volume <= 100:
                command = ("set-volume", "-l", "1.0", DEFAULT_SINK, f"{volume}%")
            elif type(muted) is bool:
                command = ("set-mute", DEFAULT_SINK, str(int(muted)))
            else:
                return jsonify(error="invalid_audio_setting"), 400
        try:
            with lock:
                if request.method == "POST":
                    wpctl(*command)
                result = jsonify(read_audio())
        except (OSError, subprocess.SubprocessError, ValueError):
            return jsonify(error="audio_unavailable"), 503
        result.headers["Cache-Control"] = "no-store"
        return result

    app.register_blueprint(routes)
