#!/usr/bin/env python3
"""Apply a named Sirius Nav2 mode from the command line."""

import json
import os
import sys
from contextlib import redirect_stdout
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from sirius_navigation.navigation_modes import (
        NAVIGATION_MODE_INFO,
        NAVIGATION_MODE_PRESENTATION,
        navigation_mode_controller,
        navigation_mode_speech,
        normalize_navigation_mode,
        resolve_navigation_mode_config,
    )
except ImportError:
    workspace = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(workspace / "src/sirius/sirius_navigation"))
    from sirius_navigation.navigation_modes import (  # noqa: E402
        NAVIGATION_MODE_INFO,
        NAVIGATION_MODE_PRESENTATION,
        navigation_mode_controller,
        navigation_mode_speech,
        normalize_navigation_mode,
        resolve_navigation_mode_config,
    )


def log(message):
    print(message, file=sys.stderr if JSON_OUTPUT else sys.stdout)


JSON_OUTPUT = False


class ParameterSetter(Node):
    """Small synchronous client used by the shell mode-switch command."""

    def __init__(self):
        super().__init__("parameter_setter_node")
        selector_qos = QoSProfile(depth=1)
        selector_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        selector_qos.reliability = ReliabilityPolicy.RELIABLE
        self.controller_selector_pub = self.create_publisher(
            String, "/controller_selector", selector_qos
        )
        self.navigation_mode_pub = self.create_publisher(
            String, "/sirius/navigation_mode", selector_qos
        )
        self.current_mode = None
        self.errors = []
        self.current_subscription = self.create_subscription(
            String, "/sirius/navigation_mode", self.on_current_mode, selector_qos
        )

    def on_current_mode(self, message):
        self.current_mode = str(message.data or "").strip() or None

    def read_current_mode(self, timeout_sec=0.5):
        from time import monotonic

        domain = str(os.environ.get("ROS_DOMAIN_ID", "53"))
        state_path = Path("/tmp") / f"sirius_nav_mode_state_{domain}.json"
        try:
            state = json.loads(state_path.read_text(encoding="utf-8"))
            pid = state.get("pid")
            alive = not pid or Path(f"/proc/{pid}").exists()
            if alive and state.get("status") == "ok" and state.get("mode"):
                self.current_mode = str(state["mode"])
                return self.current_mode
        except (OSError, ValueError):
            pass

        deadline = monotonic() + timeout_sec
        while monotonic() < deadline and self.current_mode is None:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.current_mode

    def publish_mode(self, mode):
        selector = String(data=navigation_mode_controller(mode))
        status = String(data=mode)
        self.controller_selector_pub.publish(selector)
        self.navigation_mode_pub.publish(status)
        rclpy.spin_once(self, timeout_sec=0.2)

    def set_node_parameters(self, node_name, params_dict):
        srv_name = f"{node_name}/set_parameters"
        client = self.create_client(SetParameters, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.errors.append(f"{node_name}: サービスが見つかりません")
            log(f"[Warning] Node '{node_name}' is not running or service not available.")
            return False

        request = SetParameters.Request()
        for name, value in params_dict.items():
            parameter = Parameter(name=name)
            parameter_value = ParameterValue()
            if isinstance(value, bool):
                parameter_value.type = ParameterType.PARAMETER_BOOL
                parameter_value.bool_value = value
            elif isinstance(value, int):
                parameter_value.type = ParameterType.PARAMETER_INTEGER
                parameter_value.integer_value = value
            elif isinstance(value, float):
                parameter_value.type = ParameterType.PARAMETER_DOUBLE
                parameter_value.double_value = value
            elif isinstance(value, str):
                parameter_value.type = ParameterType.PARAMETER_STRING
                parameter_value.string_value = value
            elif isinstance(value, list):
                if value and isinstance(value[0], float):
                    parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                    parameter_value.double_array_value = value
                elif value and isinstance(value[0], int):
                    parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                    parameter_value.integer_array_value = value
                else:
                    parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
                    parameter_value.string_array_value = value
            parameter.value = parameter_value
            request.parameters.append(parameter)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.errors.append(f"{node_name}: パラメータ設定がタイムアウトしました")
            log(f"Timeout setting parameters on {node_name}")
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.errors.append(f"{node_name}: {exc}")
            log(f"Parameter request failed on {node_name}: {exc}")
            return False

        success = True
        names = list(params_dict)
        for index, result in enumerate(response.results):
            if not result.successful:
                name = names[index] if index < len(names) else "unknown"
                message = f"{node_name}.{name}: {result.reason}"
                self.errors.append(message)
                log(f"Failed to set parameter '{name}' on {node_name}: {result.reason}")
                success = False
        return success


def speak_mode(mode):
    text = navigation_mode_speech(mode)
    try:
        from sirius_navigation.dialogue.face_client import FaceClient

        output = sys.stderr if JSON_OUTPUT else sys.stdout
        with redirect_stdout(output):
            ok = bool(FaceClient().send_speak(text))
        return {
            "ok": ok,
            "text": text,
            "error": None if ok else "SiriusFaceの音声APIに接続できません",
        }
    except Exception as error:  # noqa: BLE001
        return {"ok": False, "text": text, "error": str(error)}


def emit_error(message):
    if JSON_OUTPUT:
        print(json.dumps({"ok": False, "error": message}, ensure_ascii=False))
    else:
        print(f"Error: {message}", file=sys.stderr)


def main():
    global JSON_OUTPUT

    raw_args = sys.argv[1:]
    JSON_OUTPUT = "--json" in raw_args
    speech_enabled = (
        "--no-speech" not in raw_args
        and os.environ.get("SIRIUS_NAV_MODE_SPEECH", "1").lower()
        not in {"0", "false", "no", "off"}
    )
    mode_args = [arg for arg in raw_args if not arg.startswith("--")]
    if len(mode_args) != 1:
        emit_error("Usage: change_nav_mode_fast.py <mode> [--json] [--no-speech]")
        return 1

    mode = normalize_navigation_mode(mode_args[0])
    if mode is None:
        emit_error(f"Mode '{mode_args[0]}' is not defined.")
        return 1

    info = NAVIGATION_MODE_INFO[mode]
    presentation = NAVIGATION_MODE_PRESENTATION[mode]
    rclpy.init()
    node = ParameterSetter()
    success = True
    audio = None
    previous_mode = None
    try:
        for node_name, params in resolve_navigation_mode_config(mode).items():
            log(f"Setting parameters on '{node_name}'...")
            if not node.set_node_parameters(node_name, params):
                success = False
        if success:
            previous_mode = node.read_current_mode()
            node.publish_mode(mode)
            if not speech_enabled:
                audio = {"ok": True, "skipped": True, "reason": "disabled"}
            elif normalize_navigation_mode(previous_mode) == mode:
                audio = {"ok": True, "skipped": True, "reason": "same_mode"}
            else:
                audio = speak_mode(mode)
                if not audio["ok"]:
                    log(f"[Warning] 音声通知に失敗しました: {audio['error']}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    result = {
        "ok": success,
        "mode": mode,
        "label": presentation["label"],
        "speed": float(info["speed"]),
        "previous_mode": previous_mode,
        "audio": audio,
        "errors": node.errors if not success else [],
    }
    if JSON_OUTPUT:
        print(json.dumps(result, ensure_ascii=False))
    elif success:
        print(f"✓ Successfully applied all navigation parameters for '{mode}'!")
    else:
        print(f"⚠ Some parameters failed while applying '{mode}'.")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
