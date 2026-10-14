#!/bin/bash
# 3D地図への局所自己位置推定（small_gicp）を起動する。
# 前提: FAST-LIO2等が odom->base_footprint を配信していること。
set -u
WS="$HOME/sirius_jazzy_ws"
source "$WS/bash/startup_bash/sirius_3d_maps.sh"

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

entry=$(sirius3d_pick) || exit 1
map3d="${entry%%|*}"
echo "3D地図: $map3d"
echo "初期姿勢は RViz の「2D Pose Estimate」で与えられます（map_localizer が /initialpose を購読）。"
exec ros2 launch sirius_navigation theta_3d_localization.launch.py map_path:="$map3d"
