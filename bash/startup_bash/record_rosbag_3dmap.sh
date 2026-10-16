#!/bin/bash
# Raw LiDAR/RGB + IMU + wheel odometry, with calibration and safe bag finalization.
# Usage: record_rosbag_3dmap.sh [sim|real] [name] [--duration seconds]
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
python3 "$WS/other_programs/theta_3d_map/record_3d_bag.py" "$@"
