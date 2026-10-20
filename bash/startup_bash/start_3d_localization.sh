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

exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sirius_loc3d_${UID}_${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
    echo "3D自己位置推定は既に起動しています (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
    exit 0
fi
# Also recognize a localizer started directly, before this wrapper owned the lock.
if timeout 8 ros2 node list --no-daemon --spin-time 2 2>/dev/null | grep -Fxq '/map_localizer'; then
    echo "map_localizer は既に起動しています。現在の位置推定を継続します。"
    exit 0
fi

if [ "$MODE" = "sim" ]; then
    USE_SIM_TIME=true
    USE_3DOF=true
else
    USE_SIM_TIME=false
    USE_3DOF=false
fi

entry=$(sirius3d_pick) || exit 1
map3d="${entry%%|*}"
echo "3D地図: $map3d"
echo "モード: $MODE (use_sim_time=$USE_SIM_TIME)"
PUBLISH_MAP=false
case "$map3d" in
    *.ply) PUBLISH_MAP=true ;;
esac
echo "Foxglove: 表示フレーム map / 地図 /localization/map（PLY地図）"
echo "初期位置・向きが異なる場合は Foxglove/RViz から /initialpose を与えてください。"
echo "検証用の録画は record_loc3d で別途開始します。"
INITIAL_POSE_ARGS=()
if [ -n "${SIRIUS_LOCALIZATION_INITIAL_POSE_FILE:-}" ]; then
    INITIAL_POSE_ARGS+=("initial_pose_file:=$SIRIUS_LOCALIZATION_INITIAL_POSE_FILE")
fi
exec ros2 launch sirius_navigation theta_3d_localization.launch.py \
    map_path:="$map3d" \
    use_sim_time:="$USE_SIM_TIME" \
    use_3dof:="${SIRIUS_LOCALIZATION_3DOF:-$USE_3DOF}" \
    publish_map:="$PUBLISH_MAP" \
    "${INITIAL_POSE_ARGS[@]}"
