"""Tests for direct remote-controller Nav2 mode switching."""

import threading
from unittest.mock import Mock

import pytest

from sirius_navigation.dialogue.llm_dynamic_goal import LlmDynamicGoal
from sirius_navigation.dialogue.modules.nav_controller import NavController
from sirius_navigation.navigation_modes import (
    NAVIGATION_MODE_CONFIGS,
    build_speed_parameters,
    navigation_mode_controller,
    navigation_mode_confirmation,
    parse_navigation_mode_command,
    speed_setting_to_mode,
)


def test_strict_normal_command_keeps_mode_information():
    command = '{"type":"nav_mode","mode":"strict_normal"}'

    assert parse_navigation_mode_command(command) == "strict_normal"
    assert "パス厳守・通常" in navigation_mode_confirmation("strict_normal")
    assert "0.9" in navigation_mode_confirmation("strict_normal")

    config = NAVIGATION_MODE_CONFIGS["strict_normal"]
    assert config["/controller_server"]["FollowPath.PathAlignCritic.cost_weight"] == 12.0
    assert config["/controller_server"]["FollowPath.CostCritic.cost_weight"] == 10.0
    assert config["/global_costmap/global_costmap"]["obstacle_layer.enabled"] is True


def test_strict_modes_keep_avoidance_and_path_emphasis():
    for mode in ("strict_normal", "strict_safe", "strict_slow"):
        config = NAVIGATION_MODE_CONFIGS[mode]
        assert config["/global_costmap/global_costmap"]["obstacle_layer.enabled"] is True
        controller = config["/controller_server"]
        assert controller["FollowPath.CostCritic.cost_weight"] == 10.0
        assert controller["FollowPath.PathAlignCritic.cost_weight"] == 12.0


def test_unknown_navigation_mode_is_rejected_before_llm_fallback():
    with pytest.raises(ValueError):
        parse_navigation_mode_command('{"type":"nav_mode","mode":"warp"}')


def test_wait_normal_uses_normal_speed_and_wait_controller():
    command = '{"type":"nav_mode","mode":"wait_normal"}'

    assert parse_navigation_mode_command(command) == "wait_normal"
    assert navigation_mode_controller("wait_normal") == "WaitPath"
    assert navigation_mode_controller("normal") == "FollowPath"

    config = NAVIGATION_MODE_CONFIGS["wait_normal"]
    assert config["/controller_server"]["WaitPath.desired_linear_vel"] == 0.60
    assert config["/velocity_smoother"]["max_velocity"][0] == 0.60
    assert config["/velocity_smoother"]["max_velocity"][2] == 0.60
    assert config["/controller_server"]["WaitPath.min_lookahead_dist"] == 1.00
    assert config["/controller_server"]["WaitPath.regulated_linear_scaling_min_radius"] == 1.50
    assert config["/controller_server"]["WaitPath.cost_scaling_dist"] == 1.00
    assert config["/controller_server"]["WaitPath.max_allowed_time_to_collision_up_to_carrot"] == 3.0
    assert config["/controller_server"]["WaitPath.use_rotate_to_heading"] is True
    assert config["/controller_server"]["WaitPath.rotate_to_heading_angular_vel"] == 0.60
    assert config["/global_costmap/global_costmap"]["obstacle_layer.enabled"] is False


def test_wait_active_uses_wait_controller_at_higher_speed():
    assert navigation_mode_controller("wait_active") == "WaitPath"

    config = NAVIGATION_MODE_CONFIGS["wait_active"]
    assert config["/controller_server"]["WaitPath.desired_linear_vel"] == 0.72
    assert config["/velocity_smoother"]["max_velocity"][0] == 0.72
    assert config["/velocity_smoother"]["max_velocity"][2] == 0.60
    assert config["/controller_server"]["WaitPath.min_lookahead_dist"] == 1.00
    assert config["/controller_server"]["WaitPath.regulated_linear_scaling_min_radius"] == 1.50
    assert config["/controller_server"]["WaitPath.cost_scaling_dist"] == 1.00
    assert config["/controller_server"]["WaitPath.max_allowed_time_to_collision_up_to_carrot"] == 3.0
    assert config["/controller_server"]["WaitPath.use_rotate_to_heading"] is True
    assert config["/controller_server"]["WaitPath.rotate_to_heading_angular_vel"] == 0.60
    assert config["/global_costmap/global_costmap"]["obstacle_layer.enabled"] is False
    assert "0.7" in navigation_mode_confirmation("wait_active")


def test_complete_mode_application_updates_state_only_after_all_services_succeed():
    node = Mock()
    node.lock = threading.Lock()
    controller = NavController(node)
    controller.set_node_parameters = Mock(return_value=True)

    assert controller.set_navigation_mode("strict_normal") is True
    assert node.current_navigation_mode == "strict_normal"
    assert node.current_speed_setting == 0.90
    assert controller.set_node_parameters.call_count == 3
    for call in controller.set_node_parameters.call_args_list:
        assert call.kwargs["wait_for_result"] is True


def test_remote_mode_handler_publishes_confirmed_mode_and_explicit_response():
    node = LlmDynamicGoal.__new__(LlmDynamicGoal)
    node.nav_ctrl = Mock()
    node.nav_ctrl.set_navigation_mode.return_value = True
    node.controller_selector_pub = Mock()
    node.navigation_mode_pub = Mock()
    node.send_sirius_speak = Mock()
    node.get_logger = Mock(return_value=Mock())

    handled = node.handle_navigation_mode_instruction(
        '{"type":"nav_mode","mode":"strict_normal"}'
    )

    assert handled is True
    node.nav_ctrl.set_navigation_mode.assert_called_once_with("strict_normal")
    controller = node.controller_selector_pub.publish.call_args.args[0]
    assert controller.data == "FollowPath"
    status = node.navigation_mode_pub.publish.call_args.args[0]
    assert status.data == "strict_normal"
    spoken = node.send_sirius_speak.call_args.args[0]
    assert "パス厳守・通常" in spoken
    assert "0.9" in spoken


def test_remote_wait_mode_selects_wait_path_controller():
    node = LlmDynamicGoal.__new__(LlmDynamicGoal)
    node.nav_ctrl = Mock()
    node.nav_ctrl.set_navigation_mode.return_value = True
    node.controller_selector_pub = Mock()
    node.navigation_mode_pub = Mock()
    node.send_sirius_speak = Mock()
    node.get_logger = Mock(return_value=Mock())

    handled = node.handle_navigation_mode_instruction(
        '{"type":"nav_mode","mode":"wait_normal"}'
    )

    assert handled is True
    selector = node.controller_selector_pub.publish.call_args.args[0]
    assert selector.data == "WaitPath"
    status = node.navigation_mode_pub.publish.call_args.args[0]
    assert status.data == "wait_normal"


def test_all_modes_keep_prediction_distance_near_normal():
    for mode, config in NAVIGATION_MODE_CONFIGS.items():
        controller = config["/controller_server"]
        lookahead = controller["FollowPath.vx_max"] * controller["FollowPath.time_steps"] * 0.10
        assert 3.9 <= lookahead <= 5.5, f"{mode} lookahead={lookahead:.2f}m"


def test_speed_setting_maps_to_named_modes():
    assert speed_setting_to_mode(0.20) == "slow"
    assert speed_setting_to_mode(0.40) == "safe"
    assert speed_setting_to_mode(0.90) == "normal"
    assert speed_setting_to_mode(1.00) == "normal_active"
    assert speed_setting_to_mode("safe") == "safe"
    assert speed_setting_to_mode("fast") == "normal_active"
    assert speed_setting_to_mode("unknown-mode") == "normal"
    assert speed_setting_to_mode("not-a-number") == "normal"


def test_build_speed_parameters_excludes_critics_and_costmap():
    params = build_speed_parameters("slow")
    assert set(params) == {"/controller_server", "/velocity_smoother"}

    controller = params["/controller_server"]
    assert controller["FollowPath.vx_max"] == 0.20
    assert controller["FollowPath.time_steps"] == 200
    assert all("Critic" not in key for key in controller)

    named = NAVIGATION_MODE_CONFIGS["slow"]["/controller_server"]
    for key, value in controller.items():
        assert named[key] == value

    assert params["/velocity_smoother"] == NAVIGATION_MODE_CONFIGS["slow"]["/velocity_smoother"]
