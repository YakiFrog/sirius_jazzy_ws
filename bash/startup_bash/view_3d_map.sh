#!/bin/bash
# Usage: view_3d_map.sh [bag_directory|output_directory|file.ply]
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
python3 "$WS/other_programs/theta_3d_map/view_3d_map.py" "$@"
