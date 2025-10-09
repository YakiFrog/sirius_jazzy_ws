#!/bin/bash

# Sirius3ロボットシミュレーション起動スクリプト（高機能版）
# 使用方法: 
#   ./sim_sirius_advanced.bash           # 通常起動
#   ./sim_sirius_advanced.bash --build   # ビルドしてから起動
#   ./sim_sirius_advanced.bash --clean   # クリーンビルドしてから起動
#   ./sim_sirius_advanced.bash --help    # ヘルプ表示

# 色付きメッセージ用の関数
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

# ヘルプ表示
show_help() {
    echo "=== Sirius3 Robot Simulation Launcher ==="
    echo "使用方法:"
    echo "  $0                # 通常起動"
    echo "  $0 --build        # ビルドしてから起動"
    echo "  $0 --clean        # クリーンビルドしてから起動"
    echo "  $0 --help         # このヘルプを表示"
    echo ""
    echo "オプション:"
    echo "  --build    ワークスペースをビルドしてから起動"
    echo "  --clean    ワークスペースをクリーンビルドしてから起動"
    echo "  --help     ヘルプを表示して終了"
}

# 引数処理
BUILD_WORKSPACE=false
CLEAN_BUILD=false

case "$1" in
    --help|-h)
        show_help
        exit 0
        ;;
    --build)
        BUILD_WORKSPACE=true
        ;;
    --clean)
        BUILD_WORKSPACE=true
        CLEAN_BUILD=true
        ;;
    "")
        # 引数なし（通常起動）
        ;;
    *)
        print_error "不明なオプション: $1"
        show_help
        exit 1
        ;;
esac

echo "=== Sirius3 Robot Simulation Launcher ==="
print_info "ワークスペースをセットアップしています..."

# ワークスペースのパスを設定（スクリプトの場所から自動検出）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_PATH="$(dirname "$SCRIPT_DIR")"

# ワークスペースディレクトリに移動
cd $WORKSPACE_PATH || {
    print_error "ワークスペースディレクトリに移動できませんでした: $WORKSPACE_PATH"
    exit 1
}

# ROS2環境をセットアップ
print_info "ROS2環境をセットアップしています..."
source /opt/ros/jazzy/setup.bash

# ビルド処理
if [ "$BUILD_WORKSPACE" = true ]; then
    if [ "$CLEAN_BUILD" = true ]; then
        print_info "クリーンビルドを実行しています..."
        rm -rf build/ install/ log/
        colcon build --symlink-install
    else
        print_info "ワークスペースをビルドしています..."
        colcon build --symlink-install
    fi
    
    if [ $? -eq 0 ]; then
        print_success "ビルドが完了しました"
    else
        print_error "ビルドに失敗しました"
        exit 1
    fi
elif [ ! -d "$WORKSPACE_PATH/install" ]; then
    print_warning "installディレクトリが見つかりません。ワークスペースをビルドしています..."
    colcon build --symlink-install
    
    if [ $? -eq 0 ]; then
        print_success "ビルドが完了しました"
    else
        print_error "ビルドに失敗しました"
        exit 1
    fi
fi

# ワークスペースの環境をセットアップ
print_info "ワークスペース環境をセットアップしています..."
source $WORKSPACE_PATH/install/setup.bash

# Gazebo環境変数を設定
print_info "Gazebo環境変数を設定しています..."
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_PATH/src/sirius_description:$GZ_SIM_RESOURCE_PATH

# 起動前チェック
print_info "起動前チェックを実行しています..."

# パッケージの存在確認
if ! ros2 pkg prefix sirius_description >/dev/null 2>&1; then
    print_error "sirius_descriptionパッケージが見つかりません"
    exit 1
fi

# launchファイルの存在確認
LAUNCH_FILE="$WORKSPACE_PATH/src/sirius_description/launch/sim.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    print_error "sim.launch.pyが見つかりません: $LAUNCH_FILE"
    exit 1
fi

print_success "すべてのチェックが完了しました"

# シミュレーション起動
echo ""
print_info "=== Sirius3シミュレーションを起動しています ==="
print_info "Ctrl+C で終了できます"
echo ""

# Gazeboが既に起動している場合の警告
if pgrep -f "gz sim" >/dev/null; then
    print_warning "Gazeboが既に起動している可能性があります"
    print_warning "問題が発生した場合は、'pkill -f gz' でGazeboを終了してください"
fi

# sim.launch.pyを起動
ros2 launch sirius_description sim.launch.py

print_info "シミュレーションが終了しました"
