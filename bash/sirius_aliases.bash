# Sirius3ロボットシミュレーション用エイリアス
# このファイルの内容を~/.bashrcに追加してください

# Sirius3シミュレーション起動
alias sim_sirius='cd /home/kotantu-nuc/sirius_jazzy_ws && ./bash/sim_sirius.bash'
alias sim_sirius_advanced='cd /home/kotantu-nuc/sirius_jazzy_ws && ./bash/sim_sirius_advanced.bash'

# Sirius3ワークスペース移動
alias cd_sirius='cd /home/kotantu-nuc/sirius_jazzy_ws'

# Sirius3関連のよく使うコマンド
alias build_sirius='cd /home/kotantu-nuc/sirius_jazzy_ws && colcon build --symlink-install'
alias source_sirius='cd /home/kotantu-nuc/sirius_jazzy_ws && source install/setup.bash'

# ROS2 + Sirius3環境セットアップ
alias setup_sirius='source /opt/ros/jazzy/setup.bash && cd /home/kotantu-nuc/sirius_jazzy_ws && source install/setup.bash'

echo "Sirius3エイリアスが読み込まれました:"
echo "  sim_sirius         - シミュレーション起動（シンプル版）"
echo "  sim_sirius_advanced - シミュレーション起動（高機能版）"
echo "  cd_sirius         - ワークスペースに移動"
echo "  build_sirius      - ワークスペースをビルド"
echo "  source_sirius     - ワークスペース環境をセットアップ"
echo "  setup_sirius      - ROS2+ワークスペース環境をセットアップ"
