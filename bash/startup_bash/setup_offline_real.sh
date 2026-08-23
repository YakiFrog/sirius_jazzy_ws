#!/usr/bin/env bash
# One-time setup on the NVIDIA-free real-robot PC.

set -eo pipefail

WS_DIR="${HOME}/sirius_jazzy_ws"

echo "CUDA/ZED SDKはインストールしません。"
echo "ZED Open Captureのビデオ機能とOpenCVだけを使用します。"

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libusb-1.0-0-dev \
    libopencv-dev

if ! id -nG | tr ' ' '\n' | grep -qx video; then
    sudo usermod -aG video "$(id -un)"
    echo "videoグループを追加しました。セットアップ後に一度ログアウトしてください。"
fi

cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
set -u
colcon build --packages-select sirius_zed_recorder sirius_navigation \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "✓ 実機オフライン録画プログラムをビルドしました。"
echo "ZEDをUSB 3へ接続し、Launcherから実機用項目を起動してください。"
echo "videoグループを今回追加した場合だけ、一度ログアウト・ログインしてください。"
