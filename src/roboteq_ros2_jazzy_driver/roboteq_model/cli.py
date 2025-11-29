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

from roboteq_model.roboteq_model import Params, compute_cmdvel_results


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


def main(argv=None):
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--open-loop', dest='open_loop', action='store_true', help='Use open-loop power (default)')
    group.add_argument('--closed-loop', dest='open_loop', action='store_false', help='Use closed-loop speed (RPM)')
    parser.set_defaults(open_loop=True)
    # parser.add_argument('--pulse', type=int, default=325, help='Encoder pulse per rev (not used directly in cmdvel calc)')
    # parser.add_argument('--wheel-circumference', type=float, default=0.877)
    # parser.add_argument('--track-width', type=float, default=0.40)
    # parser.add_argument('--max-rpm', type=int, default=100)
    # parser.add_argument('--max-speed', type=float, default=0.5)
    args = parser.parse_args(argv)

    params = Params()
    repl(params, args.open_loop)


if __name__ == '__main__':
    main()
