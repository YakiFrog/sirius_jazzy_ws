#!/bin/bash
# 3D地図への局所自己位置推定（small_gicp）を起動する。
# 使い方: start_3d_localization.sh [sim|real]   (既定: real)
# 前提: FAST-LIO2等が odom->base_footprint を配信していること。
set -u
WS="$HOME/sirius_jazzy_ws"
MODE="${1:-real}"

if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real]"
    exit 1
fi

source "$WS/bash/startup_bash/sirius_3d_maps.sh"

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

if [ "$MODE" = "sim" ]; then
    USE_SIM_TIME=true
else
    USE_SIM_TIME=false
fi

entry=$(sirius3d_pick) || exit 1
map3d="${entry%%|*}"
echo "3D地図: $map3d"
echo "モード: $MODE (use_sim_time=$USE_SIM_TIME)"
echo "初期姿勢は RViz の「2D Pose Estimate」で与えられます（map_localizer が /initialpose を購読）。"
exec ros2 launch sirius_navigation theta_3d_localization.launch.py \
    map_path:="$map3d" \
    use_sim_time:="$USE_SIM_TIME"
