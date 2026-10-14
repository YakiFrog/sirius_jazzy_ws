#!/bin/bash
# 3D地図ベースの自己位置推定 + Nav2 自律移動セットを起動する。
# AMCLは起動しない（map->odom は map_localizer が配信）。
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
map2d=$(sirius3d_find_file "$map_dir" yaml) || {
    echo "2D投影地図(.yaml)が見つかりません: $map_dir"
    exit 1
}

PARAMS="${HOME}/sirius_jazzy_ws/params/nav2_params.yaml"
read -p "セマンティックコストレイヤー(semantic_layer)を有効にしますか？ (y/N): " use_semantic
if [[ "$use_semantic" =~ ^[Yy] ]]; then
    PARAMS="${HOME}/sirius_jazzy_ws/params/nav2_params_semantic.yaml"
fi

echo "2D地図: $map2d"
echo "3D地図: $map3d"
echo "Nav2パラメータ: $PARAMS"
echo "初期姿勢は RViz の「2D Pose Estimate」で与えてください。"
exec ros2 launch sirius_navigation theta_3d_nav2.launch.py \
    map2d:="$map2d" map3d:="$map3d" nav2_params:="$PARAMS"
