alias ws='cd ~/sirius_jazzy_ws'
alias src='ws && source install/setup.bash'
alias bd='ws && src && colcon build --symlink-install'

# シミュレーション環境起動
alias sim='src && ros2 launch sirius_description sim_with_ui.launch.py'

# SLAMToolBox起動
alias slam='src && ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=~/sirius_jazzy_ws/params/mapper_params_online_async.yaml'

# Nav2起動
alias nav2='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=/home/kotantu-desktop/sirius_jazzy_ws/params/nav2_params.yaml \
use_sim_time:=true \
map:=/home/kotantu-desktop/sirius_jazzy_ws/maps_waypoints/maps/map.yaml
use_composition:=false'

# プロセス終了
alias restart='pkill -f "ros2|gz|gazebo" && ros2 daemon stop && ros2 daemon start'