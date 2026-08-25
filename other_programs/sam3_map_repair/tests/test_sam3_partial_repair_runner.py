import importlib.util
from pathlib import Path
import sys


MODULE_PATH = Path(__file__).parents[1] / "sam3_partial_repair_runner.py"
SPEC = importlib.util.spec_from_file_location("sam3_partial_repair_runner", MODULE_PATH)
RUNNER = importlib.util.module_from_spec(SPEC)
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC.loader.exec_module(RUNNER)


def test_tactile_prompt_is_registered_as_semantic_id_four(monkeypatch):
    requests = []

    def fake_http_json(endpoint, payload=None, timeout=5.0):
        requests.append((endpoint, payload))
        return {"ok": True}

    monkeypatch.setattr(RUNNER, "http_json", fake_http_json)
    RUNNER.configure_server(
        {"tactile paving": 0.4},
        {"tactile paving": "line type tactile paving"},
    )

    payloads = dict(requests)
    assert payloads["/prompt"] == {"prompt": "line type tactile paving"}
    assert payloads["/class_thresholds"] == {
        "classes": {"line type tactile paving": 0.4}
    }
    assert payloads["/class_registry"] == {
        "classes": {
            "line type tactile paving": {"id": 4, "color": [255, 255, 0]}
        }
    }
