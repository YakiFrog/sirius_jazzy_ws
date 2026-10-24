#!/bin/bash
set -eo pipefail
set +u
WS="${SIRIUS_WORKSPACE:-$HOME/sirius_jazzy_ws}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
source "$WS/install/setup.bash"
set -u
exec python3 "$WS/other_programs/sirius_launcher/nav_mode_state.py" "$@"
