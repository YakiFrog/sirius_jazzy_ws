# .bashrcを編集するコマンド
alias ebash='code ~/.bashrc && source ~/.bashrc'

# ROSのインストールディレクトリを開くコマンド
alias rosapt='code /opt/ros/jazzy/share/'

# wsに移動するコマンド
alias ws='cd ~/sirius_jazzy_ws/'

# sourceコマンド
alias src='ws && source install/setup.bash'

# rosdepのコマンド
alias rdep='ws && rosdep install --from-paths src --ignore-src -riy'

# colconのビルドコマンド
alias bd='ws && colcon build --symlink-install --executor sequential --allow-overriding nav2_costmap_2d'

# RQTコンソール起動
alias rqt_console='ros2 run rqt_console rqt_console'

# TFツリー表示
alias tftree='ros2 run rqt_tf_tree rqt_tf_tree'

# すべてのプロセスを終了するコマンド
alias restart='pkill -f "ros2|gz|gazebo" && ros2 daemon stop && ros2 daemon start'