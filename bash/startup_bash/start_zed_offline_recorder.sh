#!/usr/bin/env bash
# Start the CUDA-free ZED recorder with an operator-selected rosbag output FPS.

WS_DIR="${HOME}/sirius_jazzy_ws"
CONFIG_FILE="${WS_DIR}/src/sirius/sirius_zed_recorder/config/zed_offline_recorder.yaml"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi

CAPTURE_FPS=$(python3 - "$CONFIG_FILE" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
print(config["zed_stereo_publisher"]["ros__parameters"].get("fps", 15))
PY
)

DEFAULT_RECORD_FPS=8.0
echo "================================================="
echo "  実機ZED オフライン録画用画像Publisher"
echo "================================================="
echo "カメラ取得: ${CAPTURE_FPS} FPS（ZED対応モード）"
read -r -p "Rosbagへ保存する画像FPS (0より大、${CAPTURE_FPS}以下) [${DEFAULT_RECORD_FPS}]: " RECORD_FPS
RECORD_FPS=${RECORD_FPS:-$DEFAULT_RECORD_FPS}

if ! python3 - "$RECORD_FPS" "$CAPTURE_FPS" <<'PY'
import sys

try:
    record_fps = float(sys.argv[1])
    capture_fps = float(sys.argv[2])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if 0.0 < record_fps <= capture_fps else 1)
PY
then
    echo "エラー: 保存FPSは0より大きく${CAPTURE_FPS}以下で指定してください。"
    exit 1
fi

echo "ZEDを${CAPTURE_FPS} FPSで取得し、約${RECORD_FPS} FPSをROS/rosbagへ出力します。"
exec ros2 launch sirius_zed_recorder zed_stereo_publisher.launch.py \
    record_fps:="$RECORD_FPS"
