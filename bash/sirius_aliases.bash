# Sirius3ロボットシミュレーション用エイリアス
# このファイルの内容を~/.bashrcに追加してください
# または、以下のコマンドで自動的にロードされるようにできます：
# echo "source /path/to/sirius_jazzy_ws/bash/sirius_aliases.bash" >> ~/.bashrc

# ワークスペースのパスを動的に取得
SIRIUS_WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Sirius3シミュレーション起動
alias sim_sirius="cd $SIRIUS_WS_PATH && ./bash/sim_sirius.bash"
alias sim_sirius_advanced="cd $SIRIUS_WS_PATH && ./bash/sim_sirius_advanced.bash"

# Sirius3ワークスペース移動
alias cd_sirius="cd $SIRIUS_WS_PATH"

# Sirius3関連のよく使うコマンド
alias build_sirius="cd $SIRIUS_WS_PATH && colcon build --symlink-install"
alias source_sirius="cd $SIRIUS_WS_PATH && source install/setup.bash"

# ROS2 + Sirius3環境セットアップ
alias setup_sirius="source /opt/ros/jazzy/setup.bash && cd $SIRIUS_WS_PATH && source install/setup.bash"

echo "Sirius3エイリアスが読み込まれました:"
echo "  sim_sirius         - シミュレーション起動（シンプル版）"
echo "  sim_sirius_advanced - シミュレーション起動（高機能版）"
echo "  cd_sirius         - ワークスペースに移動"
echo "  build_sirius      - ワークスペースをビルド"
echo "  source_sirius     - ワークスペース環境をセットアップ"
echo "  setup_sirius      - ROS2+ワークスペース環境をセットアップ"
