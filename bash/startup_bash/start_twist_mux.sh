#!/bin/bash
# Start twist_mux; a user-local ROS package override can repair a partial apt ABI update.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/sirius_jazzy_ws/install/setup.bash"
MUX_OVERRIDE="$HOME/.local/share/sirius/ros-overrides/twist_mux/opt/ros/jazzy"
if [[ -x "$MUX_OVERRIDE/lib/twist_mux/twist_mux" && -f "$MUX_OVERRIDE/share/ament_index/resource_index/packages/twist_mux" ]]; then
    export AMENT_PREFIX_PATH="$MUX_OVERRIDE:${AMENT_PREFIX_PATH:-}"
    echo "twist_mux: ユーザー領域の互換パッケージを使用します ($MUX_OVERRIDE)"
fi
exec ros2 launch sirius_navigation twist_mux.launch.py
