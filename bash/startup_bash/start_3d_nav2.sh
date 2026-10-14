#!/bin/bash
# 3D地図ベースの自己位置推定 + Nav2 自律移動セットを起動する。
# AMCLは起動しない（map->odom は map_localizer が配信）。
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
map2d="${entry##*|}"
if [ -z "$map2d" ]; then
    echo "この3D地図には2D投影地図(.yaml)がありません: $map3d"
    echo "  Nav2用に2D地図が必要です。save3d で生成するか、同名の .yaml を配置してください。"
    exit 1
fi

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
