"""
A small Python model that reproduces the core calculations in the
roboteq ROS2 driver: twist -> wheel speeds -> power/rpm, parsing
encoder streaming lines and converting them into odometry updates.

This is intentionally independent of ROS for simple verification.
"""

from dataclasses import dataclass
import math
from typing import Tuple


@dataclass
class Params:
    wheel_circumference: float = 0.877
    track_width: float = 0.40
    pulse: int = 325
    gear_ratio: float = 50.0
    max_rpm: int = 50
    max_speed: float = 0.5
    odom_scale: float = 1.0


@dataclass
class OdomState:
    encoder_right_old: int = 0
    encoder_left_old: int = 0
    odom_roll_right: float = 0.0
    odom_roll_left: float = 0.0
    first_time: bool = True
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


# Utility: normalize angle
def normalize(z: float) -> float:
    return math.atan2(math.sin(z), math.cos(z))


def twist_to_wheel_speeds(linear_x: float, angular_z: float, track_width: float) -> Tuple[float, float]:
    """
    Compute left and right wheel linear speeds (m/s) from twist message.

    This reproduces the original driver's sign convention where the wheels
    receive a negative sign compared to the forward linear velocity.

    Returns: (right_speed, left_speed)
    """
    right_speed = (linear_x + (track_width * angular_z) / 2.0) * -1
    left_speed = (linear_x - (track_width * angular_z) / 2.0) * -1
    return right_speed, left_speed


def apply_minimum_command(right_speed: float, left_speed: float, linear_x: float, angular_z: float) -> Tuple[float,float]:
    """
    Replicate the driver's minimum command logic. If both wheel speeds are
    small (abs < 0.08) but a non-zero twist was commanded, bump to
    small forward / rotate commands.

    This matches the C++ logic.
    """
    if (abs(right_speed) < 0.08) and (abs(left_speed) < 0.08) and ((linear_x != 0.0) or (angular_z != 0.0)):
        # slow straight
        if abs(angular_z) == 0.0:
            if linear_x > 0:
                right_speed, left_speed = -0.08, -0.08
            else:
                right_speed, left_speed = 0.08, 0.08
        # spin-in-place
        elif abs(abs(right_speed) - abs(left_speed)) < 0.01:
            if right_speed > 0:
                right_speed, left_speed = 0.1, -0.1
            else:
                right_speed, left_speed = -0.1, 0.1
        # turn
        else:
            if abs(right_speed) > abs(left_speed):
                if right_speed > 0:
                    right_speed, left_speed = 0.1, 0.0
                else:
                    right_speed, left_speed = -0.1, 0.0
            else:
                if left_speed > 0.0:
                    right_speed, left_speed = 0.0, 0.1
                else:
                    right_speed, left_speed = 0.0, -0.1
    return right_speed, left_speed


def clamp_to_max(right_speed: float, left_speed: float, max_speed: float) -> Tuple[float,float]:
    """
    The C++ code only clamps positive speeds (only checks > max_speed),
    which we replicate here exactly to avoid changing behavior.
    """
    if right_speed > max_speed:
        right_speed = max_speed
    if left_speed > max_speed:
        left_speed = max_speed
    return right_speed, left_speed


def compute_power_or_rpm(open_loop: bool, right_speed: float, left_speed: float, params: Params) -> Tuple[int,int]:
    """
    Compute the wheel power (open-loop) or wheel speed RPM (closed-loop).

    For open loop: power is scaled 0-1000 from speed using max_rpm and wheel circumference.
    For closed loop: rpm = speed / circumference * 60

    The C++ code uses int32_t truncation; we replicate by using int() which
    truncates toward 0.
    """
    if open_loop:
        # motor power scaled to 0..1000. Maintain sign.
        right_power = int((right_speed / params.wheel_circumference) * 60.0 / params.max_rpm * 1000.0)
        left_power = int((left_speed / params.wheel_circumference) * 60.0 / params.max_rpm * 1000.0)
        return right_power, left_power
    else:
        right_rpm = int((right_speed / params.wheel_circumference) * 60.0)
        left_rpm = int((left_speed / params.wheel_circumference) * 60.0)
        return right_rpm, left_rpm


def compute_cmdvel_results(linear_x: float, angular_z: float, params: Params, open_loop: bool = True) -> dict:
    """
    Compute full result object for a cmd_vel input to help in CLI/visualization.
    Returns a dictionary with intermediate and final values.
    """
    # compute raw wheel speeds
    right_speed, left_speed = twist_to_wheel_speeds(linear_x, angular_z, params.track_width)

    # apply minimum command (same logic the driver uses)
    right_speed, left_speed = apply_minimum_command(right_speed, left_speed, linear_x, angular_z)

    # clamp
    right_speed, left_speed = clamp_to_max(right_speed, left_speed, params.max_speed)

    # compute final power or rpm
    v_right_val, v_left_val = compute_power_or_rpm(open_loop, right_speed, left_speed, params)

    # create command strings as in the C++ code
    if open_loop:
        right_cmd = f"!G 1 {v_right_val}\r"
        left_cmd = f"!G 2 {v_left_val}\r"
    else:
        right_cmd = f"!S 1 {v_right_val}\r"
        left_cmd = f"!S 2 {v_left_val}\r"

    return {
        "linear_x": linear_x,
        "angular_z": angular_z,
        "right_speed": right_speed,
        "left_speed": left_speed,
        "right_value": v_right_val,
        "left_value": v_left_val,
        "right_cmd": right_cmd,
        "left_cmd": left_cmd,
    }


def parse_encoder_line(line: str) -> Tuple[int, int]:
    """
    Parse a line matching 'CB=<right>:<left>' and return two ints.
    The C++ search starts at character index 3 (after 'CB='). We'll mimic that.

    Raises ValueError if the format is invalid.
    """
    # trim whitespace
    line = line.strip()
    # We expect the line to start with CB= (or optionally leading characters)
    if not (len(line) >= 4 and line[0] == 'C' and line[1] == 'B' and line[2] == '='):
        raise ValueError("Line does not look like encoder stream (CB=...)")
    # find ':' after index 3
    delim = line.find(':', 3)
    if delim == -1:
        raise ValueError("Missing delimiter ':' in encoder line")
    right = int(line[3:delim])
    left = int(line[delim+1:])
    return right, left


def update_odometry_from_counts(state: OdomState, right_count: int, left_count: int, params: Params):
    """
    Updates the odom state in-place following the C++ driver's logic.

    The C++ code computes counts_per_wheel_rev = pulse * gear_ratio, then
    finds the difference relative to last counts and converts to wheel revolutions.
    Then calculates linear and angular increments and updates x, y, yaw.
    """
    # Compute counts per wheel revolution respecting gear ratio
    gr = 1.0 if params.gear_ratio <= 0.0 else params.gear_ratio
    counts_per_wheel_rev = float(params.pulse) * float(gr)

    right_diff = 0.0
    left_diff = 0.0
    if counts_per_wheel_rev > 0.0:
        right_diff = (float(right_count) - float(state.encoder_right_old)) / counts_per_wheel_rev
        left_diff = (float(left_count) - float(state.encoder_left_old)) / counts_per_wheel_rev

    # Skip if diffs are absurdly large (error prevention in C++ code)
    if abs(right_diff) > 100.0 or abs(left_diff) > 100.0:
        # Update old counters and continue without movement
        state.encoder_right_old = float(right_count)
        state.encoder_left_old = float(left_count)
        return

    # Store roll result with sign inverted as in C++
    state.odom_roll_right = -1.0 * right_diff
    state.odom_roll_left = -1.0 * left_diff

    if state.first_time:
        # On the very first reading, do not move; C++ sets rolls to zero and clears first_time
        state.odom_roll_right = 0.0
        state.odom_roll_left = 0.0
        state.first_time = False

    # Now update old counts
    state.encoder_right_old = float(right_count)
    state.encoder_left_old = float(left_count)

    # Apply odom scale
    linear = (state.odom_roll_right + state.odom_roll_left) * params.wheel_circumference / 2.0
    angular = (state.odom_roll_right - state.odom_roll_left) * params.wheel_circumference / params.track_width

    if abs(params.odom_scale - 1.0) > 1e-9:
        linear *= params.odom_scale
        angular *= params.odom_scale

    # Update pose
    state.x += linear * math.cos(state.yaw)
    state.y += linear * math.sin(state.yaw)
    state.yaw = normalize(state.yaw + angular)


if __name__ == '__main__':
    # tiny demo
    params = Params()
    state = OdomState()

    print("Demo: twist 0.5m/s forward -> wheel speeds")
    r, l = twist_to_wheel_speeds(0.5, 0, params.track_width)
    print("right, left", r, l)

    print("Compute open-loop power:")
    rp, lp = compute_power_or_rpm(True, r, l, params)
    print(rp, lp)

    print("Simulate encoder lines")
    print("initial state:", state)
    try:
        rr, lr = parse_encoder_line('CB=0:0')
        update_odometry_from_counts(state, rr, lr, params)
        print("first update - no movement due to first_time:", state.x, state.y, state.yaw)
    except ValueError as e:
        print(e)
    rr, lr = parse_encoder_line('CB=100:100')
    update_odometry_from_counts(state, rr, lr, params)
    print("after CB=100:100:", state.x, state.y, state.yaw)
