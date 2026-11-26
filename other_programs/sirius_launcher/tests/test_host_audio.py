from pathlib import Path
import subprocess
import sys

from flask import Flask

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import host_audio


def test_host_audio_auth_validation_and_readback(monkeypatch):
    app = Flask(__name__)
    host_audio.register_host_audio(app, lambda: host_audio.request.headers.get("X-Test-Auth") == "true")
    state = {"volume": "0.99", "mute": True}
    calls = []

    def wpctl(*args):
        calls.append(args)
        if args[0] == "get-volume":
            return "Volume: " + state["volume"] + (" [MUTED]" if state["mute"] else "")
        if args[0] == "inspect":
            return '* node.description = "Loudness Maximizer (HDMI)"'
        if args[0] == "set-volume":
            assert args[1:4] == ("-l", "1.5", host_audio.DEFAULT_SINK)
            state["volume"] = str(int(args[-1][:-1]) / 100)
        if args[0] == "set-mute":
            assert args[1] == host_audio.DEFAULT_SINK
            state["mute"] = args[-1] == "1"

    monkeypatch.setattr(host_audio, "wpctl", wpctl)
    client = app.test_client()
    url = "/api/host-audio"
    headers = {"X-Test-Auth": "true"}
    assert client.get(url).status_code == 401
    assert client.post(url, json={"volume": 0}).status_code == 401
    for body in ({"volume": -1}, {"volume": 151}, {"volume": True}, {"muted": 0}, {"volume": 0, "muted": True}):
        assert client.post(url, json=body, headers=headers).status_code == 400
    assert client.post(url, json={"muted": False}, headers={**headers, "Origin": "https://other.example"}).status_code == 403
    assert calls == []
    assert client.get(url, headers=headers).json == {"volume": 99, "muted": True, "output": "Loudness Maximizer (HDMI)"}
    assert client.post(url, json={"volume": 150}, headers=headers).json["volume"] == 150
    assert client.post(url, json={"volume": 0}, headers=headers).json["muted"] is True
    assert client.post(url, json={"muted": False}, headers=headers).json["muted"] is False

    def unavailable(*args):
        raise subprocess.CalledProcessError(2, args)

    monkeypatch.setattr(host_audio, "wpctl", unavailable)
    assert client.get(url, headers=headers).status_code == 503
