#!/bin/bash
# HWT905 IMUセンサーの起動スクリプト
# 使い方: ./start_hwt905.sh [port]
# 例: ./start_hwt905.sh /dev/ttyUSB0

set -e

# デフォルト設定
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"

# 引数からポートを取得（指定がない場合はデフォルト）
PORT=${1:-$DEFAULT_PORT}

echo "======================================"
echo "Witmotion HWT905 IMU 起動スクリプト"
echo "======================================"
echo "ポート: $PORT"
echo "ボーレート: $DEFAULT_BAUD"
echo ""

# デバイスの存在確認
if [ ! -e "$PORT" ]; then
    echo "エラー: デバイス $PORT が見つかりません"
    echo ""
    echo "利用可能なデバイス:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  シリアルデバイスが見つかりません"
    echo ""
    echo "使い方: $0 [port]"
    echo "例: $0 /dev/ttyACM0"
    exit 1
fi

# 権限の確認
if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "警告: デバイス $PORT に読み書き権限がありません"
    echo ""
    echo "権限を設定しますか？ (sudo権限が必要)"
    read -p "y/Nを入力: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo chmod 666 "$PORT"
        echo "権限を設定しました"
    else
        echo "権限エラーで起動に失敗する可能性があります"
        echo "永続的な解決方法:"
        echo "  sudo usermod -a -G dialout \$USER"
        echo "  （設定後、再ログインまたは再起動が必要）"
    fi
    echo ""
fi

# ROS2ワークスペースのソース
if [ -f "$HOME/sirius_jazzy_ws/install/setup.bash" ]; then
    source "$HOME/sirius_jazzy_ws/install/setup.bash"
    echo "ROS2ワークスペースをソースしました"
else
    echo "警告: ROS2ワークスペースが見つかりません"
    echo "パス: $HOME/sirius_jazzy_ws/install/setup.bash"
fi

echo ""
echo "HWT905 IMUを起動しています..."
echo "終了するには Ctrl+C を押してください"
echo ""

# HWT905を起動
ros2 launch sirius_navigation witmotion_hwt905.launch.py \
    port:="$PORT" \
    baud_rate:="$DEFAULT_BAUD" \
    frequency:=100.0 \
    log_level:=info
