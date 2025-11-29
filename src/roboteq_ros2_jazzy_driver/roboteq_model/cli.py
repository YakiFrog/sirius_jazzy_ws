#!/usr/bin/env python3
"""
Simple interactive CLI: type a cmd_vel (linear angular) and get the computed outputs.
"""
import argparse
import sys
import os

# Ensure the module path works when running cli.py directly from the repo
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from roboteq_model.roboteq_model import Params, compute_cmdvel_results, compute_displacement_from_twist, predict_pose_from_twist, compute_quantized_displacement_from_twist, counts_for_distance, distance_for_counts


def repl(params: Params, open_loop: bool):
    print("Roboteq cmdvel CLI (enter 'q' to quit)")
    print("Format: <linear_x> <angular_z>")
    # print parameter values on startup for clarity
    print("Parameters:")
    print(f"  wheel_circumference: {params.wheel_circumference}")
    print(f"  track_width: {params.track_width}")
    print(f"  pulse: {params.pulse}")
    print(f"  gear_ratio: {params.gear_ratio}")
    print(f"  max_rpm: {params.max_rpm}")
    print(f"  max_speed: {params.max_speed}")
    print(f"  odom_scale: {params.odom_scale}")
    print(f"  mode: {'open-loop' if open_loop else 'closed-loop'}")
    while True:
        try:
            s = input('> ').strip()
        except (KeyboardInterrupt, EOFError):
            print('\nExiting')
            return
        if s.lower() in ('q', 'quit', 'exit'):
            print('Exiting')
            return
        if not s:
            continue
        parts = s.split()
        if len(parts) < 2:
            print('Please supply linear and angular values')
            continue
        # special command: 'counts <distance>' to compute encoder counts for a distance
        if parts[0] in ('counts', 'count'):
            if len(parts) < 2:
                print('Please supply distance in meters, e.g. counts 0.5')
                continue
            try:
                dist = float(parts[1])
                c = counts_for_distance(dist, params)
                print(f"Counts per wheel for distance {dist} m: {c} (pulse={params.pulse}, gear_ratio={params.gear_ratio})")
                print(f"Distance from counts {c}: {distance_for_counts(c, params):.6f} m")
                continue
            except ValueError:
                print('Invalid distance')
                continue
        try:
            linear = float(parts[0])
            angular = float(parts[1])
        except ValueError:
            print('Invalid floats. Example: 0.5 0.0')
            continue
        result = compute_cmdvel_results(linear, angular, params, open_loop=open_loop)
        # Display results succinctly
        print(f"right_speed={result['right_speed']:.6f} m/s, left_speed={result['left_speed']:.6f} m/s")
        print(f"right_value={result['right_value']}, left_value={result['left_value']}")
        print(f"cmds: {result['right_cmd'].strip()}  {result['left_cmd'].strip()}")
        # optional 3rd parameter: time duration (seconds) to compute displacement
        if len(parts) >= 3:
            try:
                dt = float(parts[2])
                dx, dy, dtheta, new_yaw, linear_speed, angular_speed = compute_displacement_from_twist(linear, angular, dt, 0.0, params)
                print(f"dx={dx:.6f} m, dy={dy:.6f} m, dtheta={dtheta:.6f} rad, new_yaw={new_yaw:.6f} rad")
                print(f"linear_speed={linear_speed:.6f} m/s, angular_speed={angular_speed:.6f} rad/s")
                # quantized encoder-based odometry for comparison
                dx_q, dy_q, dtheta_q, new_yaw_q, counts_r, counts_l = compute_quantized_displacement_from_twist(linear, angular, dt, 0.0, params)
                print(f"(quantized) dx={dx_q:.6f} m, dy={dy_q:.6f} m, dtheta={dtheta_q:.6f} rad")
                print(f"(quantized) counts: right={counts_r}, left={counts_l}")
                # Optional: if user provided an initial pose (x y yaw), compute the final pose
                if len(parts) >= 6:
                    try:
                        x0 = float(parts[3])
                        y0 = float(parts[4])
                        yaw0 = float(parts[5])
                        x1, y1, yaw1, vlin, vang = predict_pose_from_twist(x0, y0, yaw0, linear, angular, dt, params)
                        print(f"predicted pose: x={x1:.6f} m, y={y1:.6f} m, yaw={yaw1:.6f} rad")
                    except ValueError:
                        print('Invalid initial pose values; should be floats (e.g. 0.0 0.0 0.0)')
            except ValueError:
                print('Invalid time (dt); should be a float (e.g. 0.5)')


def main(argv=None):
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--open-loop', dest='open_loop', action='store_true', help='Use open-loop power (default)')
    group.add_argument('--closed-loop', dest='open_loop', action='store_false', help='Use closed-loop speed (RPM)')
    parser.set_defaults(open_loop=True)
    parser.add_argument('--pulse', type=int, default=325, help='Encoder pulse per rev')
    parser.add_argument('--wheel-circumference', type=float, default=0.877, help='Wheel circumference (m)')
    parser.add_argument('--track-width', type=float, default=0.40, help='Track width (m)')
    parser.add_argument('--gear-ratio', type=float, default=50.0, help='gear ratio')
    parser.add_argument('--max-rpm', type=int, default=50, help='max rpm')
    parser.add_argument('--max-speed', type=float, default=0.5, help='max speed (m/s)')
    args = parser.parse_args(argv)

    params = Params(
        wheel_circumference=args.wheel_circumference,
        track_width=args.track_width,
        pulse=args.pulse,
        gear_ratio=args.gear_ratio,
        max_rpm=args.max_rpm,
        max_speed=args.max_speed,
    )
    repl(params, args.open_loop)


if __name__ == '__main__':
    main()
