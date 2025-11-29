import math
from roboteq_model.roboteq_model import (
    Params,
    OdomState,
    twist_to_wheel_speeds,
    apply_minimum_command,
    clamp_to_max,
    compute_power_or_rpm,
    compute_cmdvel_results,
    compute_displacement_from_twist,
    predict_pose_from_twist,
    compute_quantized_displacement_from_twist,
    counts_for_distance,
    distance_for_counts,
    parse_encoder_line,
    update_odometry_from_counts,
)


def approx_eq(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_twist_forward():
    params = Params()
    r, l = twist_to_wheel_speeds(0.5, 0.0, params.track_width)
    assert approx_eq(r, -0.5)
    assert approx_eq(l, -0.5)


def test_minimum_command_straight():
    # small linear velocity should be bumped to 0.08
    params = Params()
    r, l = twist_to_wheel_speeds(0.01, 0.0, params.track_width)
    r2, l2 = apply_minimum_command(r, l, 0.01, 0.0)
    assert approx_eq(r2, -0.08)
    assert approx_eq(l2, -0.08)


def test_clamping_positive():
    params = Params()
    r, l = 1.0, 1.0
    r2, l2 = clamp_to_max(r, l, params.max_speed)
    # since clamp_to_max only clamps values greater than positive max_speed
    assert approx_eq(r2, params.max_speed)
    assert approx_eq(l2, params.max_speed)


def test_open_loop_power_and_closed_loop_rpm():
    params = Params()
    # forward 0.5 -> negative because of sign convention
    r, l = twist_to_wheel_speeds(0.5, 0.0, params.track_width)

    rp, lp = compute_power_or_rpm(True, r, l, params)
    # compute expected as C++ does
    expected_rp = int((r / params.wheel_circumference) * 60.0 / params.max_rpm * 1000.0)
    expected_lp = int((l / params.wheel_circumference) * 60.0 / params.max_rpm * 1000.0)
    assert rp == expected_rp
    assert lp == expected_lp

    rr, lr = compute_power_or_rpm(False, r, l, params)
    expected_rr = int((r / params.wheel_circumference) * 60.0)
    expected_lr = int((l / params.wheel_circumference) * 60.0)
    assert rr == expected_rr
    assert lr == expected_lr


def test_encode_parse_and_odometry_forward():
    params = Params()
    state = OdomState()

    # first reading is used only to set old counts (first_time True means zero motion expected)
    right, left = parse_encoder_line('CB=0:0')
    update_odometry_from_counts(state, right, left, params)
    assert approx_eq(state.x, 0.0)
    assert approx_eq(state.y, 0.0)

    # second reading: symmetrical right/left difference should produce forward motion
    r2, l2 = parse_encoder_line('CB=100:100')
    update_odometry_from_counts(state, r2, l2, params)

    # counts per revolution
    counts_per_rev = params.pulse * (params.gear_ratio if params.gear_ratio > 0 else 1.0)
    rev = (100 - 0) / counts_per_rev
    expected_linear = -rev * params.wheel_circumference  # sign inverted by odom_roll

    # x updated by linear*cos(yaw) with yaw==0
    assert math.isclose(state.x, expected_linear, abs_tol=1e-6)


def test_odom_rotation():
    params = Params()
    state = OdomState()

    # first reading to initialize
    update_odometry_from_counts(state, 0, 0, params)

    # right increases by +100, left stays 0, so rotation should occur
    update_odometry_from_counts(state, 100, 0, params)

    counts_per_rev = params.pulse * params.gear_ratio
    right_rev = (100 - 0) / counts_per_rev
    left_rev = (0 - 0) / counts_per_rev
    expected_angular = (-right_rev - -left_rev) * params.wheel_circumference / params.track_width
    # yaw grows by expected_angular
    assert math.isclose(state.yaw, expected_angular, abs_tol=1e-7)


def test_displacement_from_twist_forward_and_rotation():
    params = Params()
    # forward 1 second
    dx, dy, dtheta, new_yaw, vlin, vang = compute_displacement_from_twist(0.5, 0.0, 1.0, 0.0, params)
    assert math.isclose(dx, -0.5, abs_tol=1e-7)
    assert math.isclose(dy, 0.0, abs_tol=1e-7)
    assert math.isclose(dtheta, 0.0, abs_tol=1e-7)

    # rotation 1 second (angular = 1.0), track_width=0.4 => expected dtheta = -1.0 rad
    dx, dy, dtheta, new_yaw, vlin, vang = compute_displacement_from_twist(0.0, 1.0, 1.0, 0.0, params)
    assert math.isclose(dx, 0.0, abs_tol=1e-7)
    assert math.isclose(dy, 0.0, abs_tol=1e-7)
    assert math.isclose(dtheta, -1.0, abs_tol=1e-7)


def test_predict_pose_from_twist():
    params = Params()
    # initial pose 0; forward 1 second
    x1, y1, yaw1, vlin, vang = predict_pose_from_twist(0.0, 0.0, 0.0, 0.5, 0.0, 1.0, params)
    assert math.isclose(x1, -0.5, abs_tol=1e-7)
    assert math.isclose(y1, 0.0, abs_tol=1e-7)
    assert math.isclose(yaw1, 0.0, abs_tol=1e-7)

    # initial pose non-zero, forward 1s
    x1, y1, yaw1, vlin, vang = predict_pose_from_twist(1.0, 2.0, 0.0, 0.5, 0.0, 1.0, params)
    assert math.isclose(x1, 0.5, abs_tol=1e-7)
    assert math.isclose(y1, 2.0, abs_tol=1e-7)


def test_compute_cmdvel_results_values():
    params = Params()
    # test forward 0.5 m/s open-loop
    res = compute_cmdvel_results(0.5, 0.0, params, open_loop=True)
    # command strings should match the integer scaling
    expected_rval = int((res['right_speed'] / params.wheel_circumference) * 60.0 / params.max_rpm * 1000.0)
    assert res['right_value'] == expected_rval
    assert res['right_cmd'].startswith('!G 1 ')
    assert res['left_cmd'].startswith('!G 2 ')

    # closed-loop
    res2 = compute_cmdvel_results(0.5, 0.0, params, open_loop=False)
    expected_rpm = int((res2['right_speed'] / params.wheel_circumference) * 60.0)
    assert res2['right_value'] == expected_rpm
    assert res2['right_cmd'].startswith('!S 1 ')


def test_quantized_displacement_depends_on_pulse():
    params_low = Params(pulse=10, gear_ratio=1.0)
    params_high = Params(pulse=1000, gear_ratio=1.0)
    lin = 0.05
    ang = 0.0
    dt = 1.0
    dx_low, dy_low, dtheta_low, new_yaw_low, cr, cl = compute_quantized_displacement_from_twist(lin, ang, dt, 0.0, params_low)
    dx_high, dy_high, dtheta_high, new_yaw_high, cr2, cl2 = compute_quantized_displacement_from_twist(lin, ang, dt, 0.0, params_high)
    # At least counts or dx should differ when pulse is changed
    assert (dx_low != dx_high) or (cr != cr2) or (cl != cl2)


def test_counts_for_distance_and_back():
    params = Params()
    d = 0.5
    counts = counts_for_distance(d, params)
    expected = int(round((d / params.wheel_circumference) * params.pulse * (params.gear_ratio if params.gear_ratio > 0 else 1.0)))
    assert counts == expected
    d_back = distance_for_counts(counts, params)
    assert math.isclose(d_back, (counts / (params.pulse * (params.gear_ratio if params.gear_ratio > 0 else 1.0))) * params.wheel_circumference, rel_tol=1e-6)

