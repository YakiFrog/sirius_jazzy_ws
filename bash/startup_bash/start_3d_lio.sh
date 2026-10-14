#!/bin/bash
# FAST-LIO2（LiDAR-IMUオドメトリ）を起動する。odom->base_footprint を配信する。
set -u
WS="$HOME/sirius_jazzy_ws"

cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

CONFIG_DIR="${SIRIUS_FASTLIO_CONFIG_DIR:-$WS/install/sirius_navigation/share/sirius_navigation/config}"
CONFIG_FILE="${SIRIUS_FASTLIO_CONFIG_FILE:-fast_lio_sirius.yaml}"

if ! ros2 pkg prefix fast_lio >/dev/null 2>&1; then
    echo "FAST-LIO2 (fast_lio) が未導入です。"
    echo "  導入: bash $WS/bash/startup_bash/install_3d_stack.sh"
    exit 1
fi

echo "FAST-LIO2 起動: config=$CONFIG_DIR/$CONFIG_FILE"
echo "注意: 3Dモードでは EKF(sf_real) の odom->base と競合します。EKFは起動しないでください。"
echo "IMU: /imu, LiDAR: /velodyne_points"
exec ros2 launch sirius_navigation theta_3d_lio.launch.py \
    config_path:="$CONFIG_DIR" \
    config_file:="$CONFIG_FILE" \
    use_sim_time:=false
