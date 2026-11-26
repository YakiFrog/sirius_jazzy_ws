import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from flask import Flask

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import host_profile


URL = "/api/host-profile"


def test_reads_only_profiles_announced_by_the_os(monkeypatch):
    output = "  performance:\n    Degraded: no\n\n* balanced:\n    CpuDriver: intel_pstate\n"
    monkeypatch.setattr(host_profile.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(stdout=output))
    assert host_profile.read_profiles() == dict(active="balanced", available=["performance", "balanced"])


def test_write_requires_same_origin_and_reads_the_applied_mode_back(monkeypatch):
    app = Flask(__name__)
    host_profile.register_host_profile(app, lambda: host_profile.request.headers.get("X-Test-Auth") == "1")
    state = dict(active="balanced", available=["balanced", "power-saver"])
    calls = []
    monkeypatch.setattr(host_profile, "read_profiles", lambda: state.copy())

    def apply(mode):
        calls.append(mode)
        state["active"] = mode

    monkeypatch.setattr(host_profile, "apply_profile", apply)
    client = app.test_client()
    headers = {"X-Test-Auth": "1", "Origin": "http://localhost"}
    assert client.get(URL).status_code == 401
    assert client.get(URL, headers=headers).json == {**state, "can_change": True}
    assert client.post(URL, headers={**headers, "Origin": "https://remote.example"},
                       json={"profile": "power-saver"}).status_code == 403
    assert client.post(URL, headers={**headers, "Sec-Fetch-Site": "cross-site"},
                       json={"profile": "power-saver"}).status_code == 403
    assert client.post(URL, headers=headers, json={"profile": "performance"}).status_code == 400
    assert client.post(URL, headers=headers, json={"profile": ["balanced"]}).status_code == 400
    assert calls == []
    assert client.post(URL, headers=headers, json={"profile": "power-saver"}).json["active"] == "power-saver"
    assert calls == ["power-saver"]
    monkeypatch.setattr(host_profile, "apply_profile", lambda _: None)
    unchanged = client.post(URL, headers=headers, json={"profile": "balanced"})
    assert unchanged.status_code == 409
    assert unchanged.json["state"]["active"] == "power-saver"

    def locked(_):
        raise subprocess.CalledProcessError(1, "powerprofilesctl", stderr="Code26: energy_performance_preference")

    monkeypatch.setattr(host_profile, "apply_profile", locked)
    failure = client.post(URL, headers=headers, json={"profile": "balanced"})
    assert failure.status_code == 409
    assert failure.json["error"] == "profile_cpu_locked"


def test_serve_write_capability_requires_loopback_and_the_device_owner(monkeypatch):
    app = Flask(__name__)
    base = "https://launcher.example.ts.net:8444"
    identity = {"Tailscale-User-Login": "owner@example.test"}
    result = {
        "Self": {"DNSName": "launcher.example.ts.net.", "UserID": 1},
        "User": {"1": {"LoginName": "owner@example.test"}},
    }
    monkeypatch.setattr(host_profile.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(stdout=json.dumps(result)))
    with app.test_request_context("/", base_url=base, headers=identity, environ_base={"REMOTE_ADDR": "127.0.0.1"}):
        assert host_profile.can_change_profile()
    with app.test_request_context("/", base_url=base, headers=identity, environ_base={"REMOTE_ADDR": "192.0.2.4"}):
        assert not host_profile.can_change_profile()
    with app.test_request_context("/", base_url=base, headers={"Tailscale-User-Login": "other@example.test"},
                                  environ_base={"REMOTE_ADDR": "127.0.0.1"}):
        assert not host_profile.can_change_profile()
