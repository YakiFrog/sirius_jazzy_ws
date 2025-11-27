# PRESET: フルセンサーセット
# PRESET_ITEMS: roboteq,velodyne,hokuyo,imu,scn

# PRESET: シミュレータセット
# PRESET_ITEMS: sim,sf,nav2

# PRESET: シミュレータセット（SLAM）
# PRESET_ITEMS: sim,sf,nav2slam,slamtoolbox

# PRESET: リアル実験セット（MAPなし）
# PRESET_ITEMS: sf_real,nav2slam_real,slamtoolbox_real,roboteq,velodyne,hokuyo,imu,scn,rviz2desc

# TAB: センサー・ハードウェア
# GROUP: センサー・ハードウェア

# Roboteq起動(udevルール設定済み前提)
alias roboteq='src && ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py pub_odom_tf:=false'

# Velodyne起動
alias velodyne='src && ros2 launch velodyne velodyne-all-nodes-VLP16-composed-launch.py'

# Sirius Controller起動
alias scn='src && ros2 run sirius_keyop sirius_controller_node'

# Hokuyo起動(udevルール設定済み前提)
alias hokuyo='src && ros2 launch urg_node2 urg_node2.launch.py'

# IMU起動(udevルール設定済み前提)
alias imu='src && ros2 launch sirius_navigation witmotion_hwt905.launch.py'

# TAB: シミュレーション
# GROUP: シミュレーション

# Simulation起動
alias sim='src && ros2 launch sirius_description sim_with_ui.launch.py'

# Rviz2起動
alias rviz2desc='src && ros2 launch sirius_description display.launch.py'

# TAB: ユーティリティ
# GROUP: ユーティリティ

# install packages （初回のみ実行）
alias install_packages='sudo apt install ros-jazzy-spatio-temporal-voxel-layer -y  && \
sudo apt install ros-jazzy-rqt-tf-tree -y && \
sudo apt-get install libqt5serialport5-dev'

# TAB: ナビゲーション
# GROUP: ナビゲーション

# Nav2起動(既存MAP)
alias nav2='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params_sim.yaml \
map:=${HOME}/sirius_jazzy_ws/maps_waypoints/maps/map.yaml \
use_composition:=False \
use_sim_time:=True'

# Nav2起動（MAPなし）
alias nav2slam='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params_sim.yaml \
slam:=True \
use_composition:=False \
use_sim_time:=true'

# SLAMToolbox起動
alias slamtoolbox='src && ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=${HOME}/sirius_jazzy_ws/params/mapper_params_online_async_sim.yaml \
use_sim_time:=true'

# Sensor Fusion起動
alias sf='src && ros2 launch sirius_navigation sensor_fusion.launch.py'

# Sensor Fusion + IMU起動
alias sfimu='src && ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true'

# TAB: Pythonスクリプト
# GROUP: Pythonスクリプト
# Siriusランチャー起動
alias sirius_launcher='cd ${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher && python3 sirius_launcher.py'

# TAB: Sirius Ear関連
# GROUP: Sirius Ear関連
alias src2='cd ${HOME}/Sirius_ear && source install/setup.bash'
alias blue='src2 && ros2 run bluetooth bluetooth_node'
alias path='src2 && ros2 run path_listener path_listener_node'

# TAB: リアル実験
# GROUP: リアル実験

# Nav2起動(既存MAP、実時間)
alias nav2_real='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params.yaml \
map:=${HOME}/sirius_jazzy_ws/maps_waypoints/maps/map.yaml \
use_composition:=False \
use_sim_time:=false'

# Nav2起動(MAPなし、実時間)
alias nav2slam_real='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params.yaml \
slam:=True \
use_composition:=False \
use_sim_time:=false'

# SLAMToolbox起動(実時間)
alias slamtoolbox_real='src && ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=${HOME}/sirius_jazzy_ws/params/mapper_params_online_async.yaml \
use_sim_time:=false'

# Sensor Fusion起動(実時間)
alias sf_real='src && ros2 launch sirius_navigation sensor_fusion.launch.py use_sim_time:=false'

# ウェイポイントナビゲーション起動
alias mv_goal='bash ~/sirius_jazzy_ws/bash/startup_bash/move_goal.sh'

# ウェイポイント保存（距離）
alias get_pos_dis='bash ~/sirius_jazzy_ws/bash/startup_bash/get_pos_dis.sh'

# ウェイポイント保存（手動）
alias get_pos_ent='bash ~/sirius_jazzy_ws/bash/startup_bash/get_pos_ent.sh'