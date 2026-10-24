#!/bin/bash
# FAST-LIO2（LiDAR-IMUオドメトリ）を起動する。odom->base_footprint を配信する。
# 使い方: start_3d_lio.sh [sim|real] [mapping|localization]
set -u
WS="$HOME/sirius_jazzy_ws"
MODE="${1:-real}"
PROFILE="${2:-mapping}"

if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real]"
    exit 1
fi
if [ "$PROFILE" != "mapping" ] && [ "$PROFILE" != "localization" ]; then
    echo "使い方: $0 [sim|real] [mapping|localization]"
    exit 1
fi
LOCALIZATION_MODE=false
if [ "$PROFILE" = "localization" ]; then LOCALIZATION_MODE=true; fi

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sirius_lio3d_${UID}_${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
    echo "FAST-LIO2は既に起動しています (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
    exit 1
fi

if [ "$MODE" = "sim" ]; then
    # Webシミュレータ: base_footprint->imu_link=0.3m, ->lidar_link=0.85m（既定のセンサー高さ）
    USE_SIM_TIME=true
    IMU_HEIGHT="${SIRIUS_LIO_IMU_HEIGHT:-0.65}"
    CONFIG_DIR="${SIRIUS_FASTLIO_CONFIG_DIR:-$WS/params}"
    CONFIG_FILE="${SIRIUS_FASTLIO_CONFIG_FILE:-fast_lio_sirius_sim.yaml}"
    MOUNT_CONFIG=""
else
    USE_SIM_TIME=false
    IMU_HEIGHT="${SIRIUS_LIO_IMU_HEIGHT:-0.65}"
    CONFIG_DIR="${SIRIUS_FASTLIO_CONFIG_DIR:-$WS/install/sirius_navigation/share/sirius_navigation/config}"
    CONFIG_FILE="${SIRIUS_FASTLIO_CONFIG_FILE:-fast_lio_sirius.yaml}"
    MOUNT_CONFIG="${SIRIUS_SENSOR_MOUNT_CONFIG-$(ros2 pkg prefix sirius_description)/share/sirius_description/config/sensor_mount_real.yaml}"
fi

if ! ros2 pkg prefix fast_lio >/dev/null 2>&1; then
    echo "FAST-LIO2 (fast_lio) が未導入です。"
    echo "  導入: bash $WS/bash/startup_bash/install_3d_stack.sh"
    exit 1
fi

# The Web announces its actual mounts and point-time format. Resolve both together.
if [ "$MODE" = "sim" ] && [ -z "${SIRIUS_FASTLIO_CONFIG_FILE:-}" ]; then
    PROFILE_DIR=$(mktemp -d "${TMPDIR:-/tmp}/sirius_lio_profile.XXXXXX")
    trap 'rm -rf "$PROFILE_DIR"' EXIT
    if ! IMU_HEIGHT=$(python3 "$WS/other_programs/theta_3d_map/web_sensor_profile.py" \
        --template "$CONFIG_DIR/$CONFIG_FILE" --output "$PROFILE_DIR/profile.yaml"); then
        exit 1
    fi
    CONFIG_DIR="$PROFILE_DIR"
    CONFIG_FILE="profile.yaml"
fi

echo "FAST-LIO2 起動 [$MODE/$PROFILE]: config=$CONFIG_DIR/$CONFIG_FILE (imu_height=$IMU_HEIGHT)"
if [ "$MODE" = "sim" ]; then
    echo "注意: Webシミュレータでrosbridgeに接続し、/clock と /velodyne_points をON、/tf はOFFにしてください。"
else
    echo "注意: 3Dモードでは EKF(sf_real) の odom->base と競合します。EKFは起動しないでください。"
fi
echo "IMU: /imu, LiDAR: /velodyne_points"
ros2 launch sirius_navigation theta_3d_lio.launch.py \
    config_path:="$CONFIG_DIR" \
    config_file:="$CONFIG_FILE" \
    imu_height:="$IMU_HEIGHT" \
    mount_config:="${MOUNT_CONFIG:-none}" \
    localization_mode:="$LOCALIZATION_MODE" \
    use_sim_time:="$USE_SIM_TIME"
