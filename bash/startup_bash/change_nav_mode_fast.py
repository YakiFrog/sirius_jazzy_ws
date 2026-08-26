#!/usr/bin/env python3
"""Apply a named Sirius Nav2 mode from the command line."""

from pathlib import Path
import sys

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

try:
    from sirius_navigation.navigation_modes import (
        NAVIGATION_MODE_CONFIGS,
        normalize_navigation_mode,
    )
except ImportError:
    workspace = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(workspace / "src/sirius/sirius_navigation"))
    from sirius_navigation.navigation_modes import (  # noqa: E402
        NAVIGATION_MODE_CONFIGS,
        normalize_navigation_mode,
    )


class ParameterSetter(Node):
    """Small synchronous client used by the shell mode-switch command."""

    def __init__(self):
        super().__init__("parameter_setter_node")

    def set_node_parameters(self, node_name, params_dict):
        srv_name = f"{node_name}/set_parameters"
        client = self.create_client(SetParameters, srv_name)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"[Warning] Node '{node_name}' is not running or service not available.")
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
            print(f"Timeout setting parameters on {node_name}")
            return False

        try:
            response = future.result()
        except Exception as exc:
            print(f"Parameter request failed on {node_name}: {exc}")
            return False

        success = True
        names = list(params_dict)
        for index, result in enumerate(response.results):
            if not result.successful:
                name = names[index] if index < len(names) else "unknown"
                print(f"Failed to set parameter '{name}' on {node_name}: {result.reason}")
                success = False
        return success


def main():
    if len(sys.argv) < 2:
        print("Usage: change_nav_mode_fast.py <mode>")
        return 1

    mode = normalize_navigation_mode(sys.argv[1])
    if mode is None:
        print(f"Error: Mode '{sys.argv[1]}' is not defined.")
        return 1

    rclpy.init()
    node = ParameterSetter()
    try:
        success = True
        for node_name, params in NAVIGATION_MODE_CONFIGS[mode].items():
            print(f"Setting parameters on '{node_name}'...")
            if not node.set_node_parameters(node_name, params):
                success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if success:
        print(f"✓ Successfully applied all navigation parameters for '{mode}'!")
        return 0
    print(f"⚠ Some parameters failed while applying '{mode}'.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
