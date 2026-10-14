#!/bin/bash
# 3D地図への局所自己位置推定（small_gicp）を起動する。
# 前提: FAST-LIO2等が odom->base_footprint を配信していること。
set -u
WS="$HOME/sirius_jazzy_ws"
source "$WS/bash/startup_bash/sirius_3d_maps.sh"

cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

map_dir=$(sirius3d_pick_dir) || exit 1
map3d=$(sirius3d_find_file "$map_dir" npy ply pcd) || {
    echo "3D地図ファイルが見つかりません: $map_dir"
    exit 1
}
echo "3D地図: $map3d"
echo "初期姿勢は RViz の「2D Pose Estimate」で与えられます（map_localizer が /initialpose を購読）。"
exec ros2 launch sirius_navigation theta_3d_localization.launch.py map_path:="$map3d"
