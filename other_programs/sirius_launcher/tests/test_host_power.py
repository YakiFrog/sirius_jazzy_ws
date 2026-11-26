from pathlib import Path
import sys
from threading import Lock

from flask import Flask

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import host_power


def test_shutdown_requires_local_or_authenticated_origin_and_no_recording(monkeypatch):
    calls = []
    recording = True

    def login1(method, *args):
        calls.append((method, *args))
        return 's "yes"'

    monkeypatch.setattr(host_power, "login1", login1)
    monkeypatch.setattr(host_power, "recording_bag", lambda: recording)
    monkeypatch.setattr(host_power.socket, "gethostname", lambda: "sirius-nuc")
    app = Flask(__name__)
    register_password = "configured"
    host_power.register_host_power(app, lambda: register_password, Lock())
    client = app.test_client()
    url = "/api/host-power"
    body = {"host": "sirius-nuc", "action": "poweroff"}
    origin = {"Origin": "http://localhost"}

    assert client.get(url).json["recording"]
    assert client.post(url, json=body, headers=origin).status_code == 409
    recording = False
    assert client.post(url, json=body).status_code == 403
    assert client.post(url, json={**body, "host": "another-host"}, headers=origin).status_code == 400

    remote = "https://robot.example"
    remote_origin = {"Origin": remote}
    assert client.post(url, base_url=remote, json=body, headers=remote_origin).status_code == 403
    assert all(call[0] != "PowerOff" for call in calls)

    client.set_cookie("pw", register_password, domain="robot.example")
    response = client.post(url, base_url=remote, json=body, headers=remote_origin)
    assert response.json == {"accepted": True}
    assert calls[-1] == ("PowerOff", "b", "false")


def test_tailnet_access_checks_identity_for_all_routes(monkeypatch):
    from launcher_access import enforce_tailnet_access

    host = "robot.example.ts.net"
    origin = f"https://{host}"
    login = "operator@example.test"
    app = Flask(__name__)
    app.before_request(lambda: enforce_tailnet_access((origin, login)))

    @app.route("/api/term/open", methods=["GET", "POST"])
    def terminal():
        return "ok"

    monkeypatch.setattr(host_power, "login1", lambda *args: 's "yes"')
    monkeypatch.setattr(host_power, "recording_bag", lambda: False)
    host_power.register_host_power(app, lambda: "", Lock())
    client = app.test_client()
    url = "/api/term/open"
    identity = {"Tailscale-User-Login": login}
    assert client.get(url).status_code == 200
    assert client.get(url, base_url=origin).status_code == 403
    assert client.get(url, base_url=origin,
                      headers={"Tailscale-User-Login": "guest@example.test"}).status_code == 403
    assert client.get(url, base_url=origin, headers=identity,
                      environ_overrides={"REMOTE_ADDR": "192.0.2.3"}).status_code == 403
    assert client.get(url, base_url=origin, headers=identity, environ_overrides={
        "REMOTE_ADDR": "192.0.2.3", "werkzeug.proxy_fix.orig": {"REMOTE_ADDR": "127.0.0.1"},
    }).status_code == 200
    assert client.get(url, base_url=origin, headers=identity).status_code == 200
    assert client.post(url, base_url=origin, headers=identity).status_code == 403
    assert client.post(url, base_url=origin, headers={**identity, "Origin": origin}).status_code == 200

    assert client.get("/api/host-power", base_url=origin, headers=identity).json["available"]["reboot"] == "yes"


def test_power_button_is_added_directly_to_the_header():
    root = Path(__file__).resolve().parents[1]
    script = (root / 'static' / 'host_power.js').read_text(encoding='utf-8')
    style = (root / 'static' / 'host_power.css').read_text(encoding='utf-8')
    page = (root / 'launcher.html').read_text(encoding='utf-8')
    assert "id = 'header-power-button'" in script
    assert "className = 'host-power-trigger'" in script
    assert "moreWrap.before(powerTrigger)" in script
    assert "id = 'header-power-popover'" in script
    assert "class = 'host-power-trigger'" not in script
    assert "class = 'appbar-icon" not in script
    assert "void refresh()" in script
    assert "popover.hidden = false" in script
    assert '.host-power-popover' in style and '.host-power-trigger' in style
    assert 'host_power.css?v=4' in page and 'host_power.js?v=6' in page
    assert '電源専用画面' not in script
    assert not (root / 'power.html').exists()
