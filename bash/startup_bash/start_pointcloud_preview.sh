#!/bin/bash
# Display-only latest-frame relay; no effect on FAST-LIO or BAG source topics.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/sirius_jazzy_ws/install/setup.bash"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sirius_pointcloud_preview_${UID}_${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
    echo "表示用点群は既に起動しています (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
    exit 1
fi
exec python3 "$HOME/sirius_jazzy_ws/other_programs/theta_3d_map/live_pointcloud_preview.py" "$@"
