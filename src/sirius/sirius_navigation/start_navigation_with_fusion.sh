#!/bin/bash
# センサフュージョン + Nav2 統合起動スクリプト

echo "==================================="
echo "Sirius センサフュージョン + Nav2"
echo "==================================="
echo ""

# ワークスペースのパス
WS_PATH="$HOME/sirius_jazzy_ws"

# ワークスペースに移動
cd "$WS_PATH" || exit 1

# セットアップスクリプトを読み込み
source install/setup.bash

echo "✓ ワークスペース: $WS_PATH"
echo "✓ センサフュージョン: 有効"
echo ""
echo "起動中..."
echo ""

# センサフュージョン + Nav2を起動
ros2 launch sirius_navigation navigation_with_fusion.launch.py
