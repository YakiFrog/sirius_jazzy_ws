#!/bin/bash
# FAST-LIO2（LiDAR-IMUオドメトリ）を起動する。odom->base_footprint を配信する。
# 使い方: start_3d_lio.sh [sim|real]   (既定: real)
set -u
WS="$HOME/sirius_jazzy_ws"
MODE="${1:-real}"

if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real]"
    exit 1
fi

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

if [ "$MODE" = "sim" ]; then
    # Webシミュレータ: base_footprint->imu_link=0.3m, ->lidar_link=0.85m（既定のセンサー高さ）
    USE_SIM_TIME=true
    IMU_HEIGHT="${SIRIUS_LIO_IMU_HEIGHT:-0.3}"
    CONFIG_DIR="${SIRIUS_FASTLIO_CONFIG_DIR:-$WS/params}"
    CONFIG_FILE="${SIRIUS_FASTLIO_CONFIG_FILE:-fast_lio_sirius_sim.yaml}"
else
    USE_SIM_TIME=false
    IMU_HEIGHT="${SIRIUS_LIO_IMU_HEIGHT:-0.65}"
    CONFIG_DIR="${SIRIUS_FASTLIO_CONFIG_DIR:-$WS/install/sirius_navigation/share/sirius_navigation/config}"
    CONFIG_FILE="${SIRIUS_FASTLIO_CONFIG_FILE:-fast_lio_sirius.yaml}"
fi

if ! ros2 pkg prefix fast_lio >/dev/null 2>&1; then
    echo "FAST-LIO2 (fast_lio) が未導入です。"
    echo "  導入: bash $WS/bash/startup_bash/install_3d_stack.sh"
    exit 1
fi

echo "FAST-LIO2 起動 [$MODE]: config=$CONFIG_DIR/$CONFIG_FILE (imu_height=$IMU_HEIGHT)"
if [ "$MODE" = "sim" ]; then
    echo "注意: Webシミュレータでrosbridgeに接続し、/clock と /velodyne_points をON、/tf はOFFにしてください。"
else
    echo "注意: 3Dモードでは EKF(sf_real) の odom->base と競合します。EKFは起動しないでください。"
fi
echo "IMU: /imu, LiDAR: /velodyne_points"
exec ros2 launch sirius_navigation theta_3d_lio.launch.py \
    config_path:="$CONFIG_DIR" \
    config_file:="$CONFIG_FILE" \
    imu_height:="$IMU_HEIGHT" \
    use_sim_time:="$USE_SIM_TIME"
