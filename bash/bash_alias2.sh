# PRESET: フルセンサーセット
# PRESET_ITEMS: roboteq,velodyne,hokuyo,imu,sf_real,odom_path_real

# PRESET: シミュレータセット
# PRESET_ITEMS: rte,rviz2sim,sf_sim,odom_path

# PRESET: 実機オフライン録画準備 (録画は別ボタン)
# PRESET_ITEMS: roboteq,velodyne,imu,rviz2real,sf_real,slamtoolbox_real,zed_offline_recorder

# PRESET: ZED, THETA起動_real
# PRESET_ITEMS: theta_capture,zed_offline_recorder

# PRESET: 現実自律移動セット
# PRESET_ITEMS: rviz2real,twist_mux,llm_goal,sirius_ble_gateway_ui,pos,nav2_real,foxglove

# PRESET: THETA色付き3D地図（実機）
# PRESET_ITEMS: rviz2real,theta_capture,theta_colorize

# PRESET: 3D地図生成セット
# PRESET_ITEMS: roboteq,velodyne,imu,tf_real,lio3d,rtab3d

# PRESET: 3D自律移動セット
# PRESET_ITEMS: roboteq,velodyne,imu,tf_real,lio3d,twist_mux,nav2_3d

# PRESET: 3D地図生成セット(Webシミュ)
# PRESET_ITEMS: rosbridge,tf_sim,lio3d_sim

# PRESET: 3D自律移動セット(Webシミュ)
# PRESET_ITEMS: rosbridge,tf_sim,lio3d_sim,twist_mux,nav2_3d_sim

# PRESET: PCデバッグ
# PRESET_ITEMS: foxglove

# TAB: センサー・ハードウェア
# GROUP: センサー・ハードウェア
# GROUP_DESC: 実機のモーター、LiDAR、IMUなどを個別に起動します。デバイス接続とudev設定を確認してから使用してください。

# Roboteq起動(udevルール設定済み前提)
alias roboteq='bash ~/sirius_jazzy_ws/bash/startup_bash/start_roboteq.sh false'

alias roboteq_no_sf='bash ~/sirius_jazzy_ws/bash/startup_bash/start_roboteq.sh true'

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
# GROUP_DESC: Unity/GazeboとROS 2の通信、シミュレーション用RViz、SAM3/RTAB-Mapを起動する項目です。

# Simulation起動
alias sim='src && ros2 launch sirius_description sim_with_ui.launch.py'

# Unity ROS-TCP Endpoint起動
alias rte='bash ~/sirius_jazzy_ws/bash/startup_bash/start_ros_tcp.sh'

# Rviz2起動(シミュレーション)
alias rviz2sim='src && ros2 launch sirius_description display.launch.py use_sim_time:=true'

# Rviz2起動(互換用)
alias rviz2desc='rviz2sim'

# Unity同期用Rviz2起動
# alias unity_viz='src && ros2 launch sirius_description unity_sim.launch.py'

# Rosbridge Web Socket (WSS/ngrok対応) 起動
alias rosbridge='bash ~/sirius_jazzy_ws/bash/startup_bash/start_rosbridge.sh'

# Web ZED画像をRViz用Imageに変換。Web側でROS接続・ZED配信をON。rviz2simではImageに /camera/stereo_sbs/image_raw を指定。
alias web_zed_image='src && /usr/bin/python3 "$HOME/sirius-mujoco-sim/scripts/zed_image_republisher.py"'

# ZED画像専用RViz。web_zed_imageと併用（通常のrviz2simとは別ウィンドウ）。
alias rviz2zed='src && rviz2 -d "$HOME/sirius-mujoco-sim/config/zed_image.rviz"'

# Web THETA Dual Fisheye(JPEG) をBEV平面画像に変換。Web側でROS接続＋「THETA映像をROS2へ送信」をON。
# 出力は /theta/bev/image_raw（bgr8）。rviz2ではImageにこのトピックを指定する。
alias web_theta_bev='src && ros2 run sirius_navigation theta_bev_node --ros-args -p use_tf:=true -p publish_raw:=true'

# THETA表示専用RViz。web_theta_bevと併用（Dual Fisheye生ImageとBEVの2画面）。
alias rviz2theta='src && rviz2 -d "$HOME/sirius_jazzy_ws/install/sirius_navigation/share/sirius_navigation/rviz/theta_bev.rviz"'

# Foxglove Bridge 起動
# ポートは8766固定（8765はDartsRobot monitor_dashboardが使用中のため）。接続URLを表示する
alias foxglove='bash ~/sirius_jazzy_ws/bash/startup_bash/start_foxglove.sh'

# Unity上のZEDカメラのトピックをROS2に流す
alias sam3_bridge='src && ros2 launch sirius_navigation sam3_bridge.launch.py use_sim_time:=true'

# RTAB-MAP, sam3_bridgeも起動する
alias rtab_bridge='bash ~/sirius_jazzy_ws/bash/startup_bash/rtab_bridge.sh'

alias sam3_map_load='bash ~/sirius_jazzy_ws/bash/startup_bash/sam3_colored_map_select.sh'

# TAB: ユーティリティ
# GROUP: ユーティリティ
# GROUP_DESC: Dockerサーバー、可視化、物体検出などの補助ツールです。必要な項目だけ起動してください。

alias install_packages='sudo apt update && sudo apt install xterm -y && \
sudo apt install ros-jazzy-spatio-temporal-voxel-layer -y  && \
sudo apt install ros-jazzy-rqt-tf-tree -y && \
sudo apt install ros-jazzy-foxglove-bridge -y && \
sudo apt-get install libqt5serialport5-dev'

# Behavior Tree 可視化ツール Groot2 起動
alias groot2='$HOME/Groot2/groot2.sh'

# Behavior Tree Docker 操作
alias bt_start='xhost +local:docker > /dev/null 2>&1 && cd ~/sirius_jazzy_ws/bt_jazzy_docker && docker compose run --rm --name bt_dev_container bt_dev'
alias bt_enter='docker exec -it bt_dev_container bash'

# SAM3 Dockerサーバー起動（ZEDなし）ポート8000
alias sam3_docker_start='cd ~/DA3_SAM3_Project/sam3_da3_3d/sam3_server_export && docker compose up sam3-server'

# SAM3 + ZED Dockerサーバー起動 ポート8080
# オンライン・オフラインで同じ実装、モデル、既定推論設定を使用
alias sam3_zed_docker_start='cd ~/sam3_zed_server && docker compose up sam3-zed-merged'

# PCL 3D点群物体検出ノード起動 (Velodyne等の点群をリアルタイムクラスタリング・バウンディングボックス化します)
alias pcl_detect='bash ~/sirius_jazzy_ws/bash/startup_bash/pcl_detect.sh'

# 走行軌跡可視化パスパブリッシャー
alias odom_path='src && ros2 run sirius_navigation odom_path_publisher --ros-args -p use_sim_time:=true'
alias odom_path_real='src && ros2 run sirius_navigation odom_path_publisher --ros-args -p use_sim_time:=false'


# TAB: ナビゲーション
# GROUP: ナビゲーション
# GROUP_DESC: Nav2、SLAM Toolbox、Sensor Fusion、手動操作などを起動します。シミュレーションでは *_sim、実機では *_real を使います。

# Nav2起動(既存MAP)
alias nav2='bash ~/sirius_jazzy_ws/bash/startup_bash/nav2_bringup_sim.sh'

# Nav2のBT XMLのみ再読込（nav2スタック再起動なし）
alias nav_bt_reload='bash ~/sirius_jazzy_ws/bash/startup_bash/reload_bt_navigator.sh'

# Nav2起動（MAPなし）
alias nav2slam='src && ros2 launch nav2_bringup bringup_launch.py \
params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params_sim_slam.yaml \
slam:=True \
use_composition:=False \
use_sim_time:=true'

# SLAMToolbox起動
alias slamtoolbox='src && ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=${HOME}/sirius_jazzy_ws/params/mapper_params_online_async_sim.yaml \
use_sim_time:=true'

# Sensor Fusion起動(シミュレーション)
alias sf_sim='src && ros2 launch sirius_navigation sensor_fusion.launch.py use_sim_time:=true'
alias sf='sf_sim'

# 優先順位制御（twist_mux）起動
alias twist_mux='src && ros2 launch sirius_navigation twist_mux.launch.py'

# 手動操作 V2（優先順位対応版）
alias keyop2='src && ros2 run sirius_keyop sirius_keyop_v2'

# ダイナミックゴール手動操作起動
alias keyop_goal='src && xterm -hold -T "Sirius 360°ダイナミックゴール・キーオペ" -geometry 95x25 -e ros2 run sirius_navigation keyboard_dynamic_goal'
# リモコンからの移動指示を処理する必須ノード（リモコンを使う前に起動）
alias llm_goal='src && ros2 run sirius_navigation llm_dynamic_goal'
alias status_monitor='src && ros2 run sirius_navigation status_monitor'
alias status_monitor_win='src && xterm -hold -T "Sirius Status Monitor" -geometry 95x28 -e ros2 run sirius_navigation status_monitor'

# BLE管理ノード起動（リモコン移動には LLM Dynamic Goal の起動が必要）
alias sirius_ble_gateway='bash ~/sirius_jazzy_ws/bash/startup_bash/sirius_ble_gateway.sh'
# リモコン接続画面（移動操作には先に LLM Dynamic Goal / llm_goal の起動が必要）
alias sirius_ble_gateway_ui='bash ~/sirius_jazzy_ws/bash/startup_bash/sirius_ble_gateway_ui.sh'

# アシスト付き手動操作（Assisted Teleop）起動
alias assisted_teleop='src && ros2 launch sirius_navigation assisted_teleop.launch.py'
alias assisted_teleop_nav2='src && ros2 launch sirius_navigation assisted_teleop_with_nav2.launch.py'

# Sensor Fusion + IMU起動(シミュレーション)
alias sfimu_sim='src && ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true use_sim_time:=true'
alias sfimu='sfimu_sim'

# TAB: Pythonスクリプト
# GROUP: Pythonスクリプト
# GROUP_DESC: Siriusの運用・開発用Pythonツールを起動します。
# Siriusランチャー起動
alias sirius_launcher='cd ${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher && python3 sirius_launcher.py'

# SiriusLauncher ブラウザ版（遠隔PCからTailscale越しに操作。既定 http://<IP>:8090）
# パスワード保護: sirius_launcher_web --password 任意  → 起動後にURLで ?pw=任意 を一度開く
alias sirius_launcher_web='cd ${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher && python3 web_server.py'

# TAB: Sirius Ear関連
# GROUP: Sirius Ear関連
# GROUP_DESC: Sirius EarのBluetooth接続、経路、曲率、位置関連ノードを個別に起動します。
alias src2='cd ${HOME}/miura_ws && source install/setup.bash'
alias blue='src2 && ros2 run bluetooth bluetooth_node'
alias path='src2 && ros2 run path_listener path_listener_node'
alias curve='src2 && ros2 run curvature curvature_node'
alias curve2='src2 && ros2 run curvature_new curvature_new_node'
alias pos='src2 && ros2 run position position_node'

# TAB: リアル実験
# GROUP: リアル実験
# GROUP_DESC: 実機のNav2、SLAM、Sensor Fusion、マップ保存などを実時間（use_sim_time=false）で起動します。

# Nav2起動(任意MAP、実時間)
alias nav2_real='bash ~/sirius_jazzy_ws/bash/startup_bash/nav2_bringup_real.sh'

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

# SAM3 ZEDカメラのトピックをROS2に流す(実時間)
alias sam3_bridge_real='src && ros2 launch sirius_navigation sam3_bridge.launch.py use_sim_time:=false'

# ウェイポイントナビゲーション起動
alias mv_goal='bash ~/sirius_jazzy_ws/bash/startup_bash/move_goal.sh'

# ターゲット検出・追従制御（対話メニュー）
alias target_detect_and_follow='bash ~/sirius_jazzy_ws/bash/startup_bash/target_follow.sh'

# ターゲット検出・追従ノード単体起動（デバッグ用）
# alias target_detect='src && ros2 run sirius_navigation target_detector'
# alias target_follower='src && ros2 run sirius_navigation target_follower'

# ウェイポイント保存（距離）
alias get_pos_dis='bash ~/sirius_jazzy_ws/bash/startup_bash/get_pos_dis.sh'

# ウェイポイント保存（手動）
alias get_pos_ent='bash ~/sirius_jazzy_ws/bash/startup_bash/get_pos_ent.sh'

# マップ保存起動
alias map_save='bash ~/sirius_jazzy_ws/bash/startup_bash/map_save.sh'

alias rtabmap_save='bash ~/sirius_jazzy_ws/bash/startup_bash/rtabmap_save.sh'

# Rviz2起動(実時間)
alias rviz2real='src && ros2 launch sirius_description display.launch.py use_sim_time:=false'

# マップ切り替え（Nav2実行中に地図を変更）
alias change_map='bash ~/sirius_jazzy_ws/bash/startup_bash/change_map.sh'

# 走行モード切り替え（安全歩行 / 通常 / パス追従）
alias nav_safe='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh safe'
alias nav_slow='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh slow'
alias nav_normal='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh normal'
alias nav_normal_active='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh normal_active'
alias nav_wait='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh wait_normal'
alias nav_wait_active='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh wait_active'
alias nav_strict_normal='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh strict_normal'
alias nav_strict_safe='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh strict_safe'
alias nav_strict_slow='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh strict_slow'
alias nav_strict='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh strict_safe'
alias nav_mode='bash ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode.sh'

# ROS2 bag記録起動
alias record_rosbag='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag.sh'

# 好きなタイミングで実験メモを送信するエイリアス
# 使い方: pub_memo "送信したい内容"
alias pub_memo='bash ~/sirius_jazzy_ws/bash/startup_bash/pub_memo.sh'

# # マップ切り替え（プログラム呼び出し用、引数に地図名を指定）
# # 例: change_map_simple 1202-15f
# alias change_map_simple='bash ~/sirius_jazzy_ws/bash/startup_bash/change_map_simple.sh'
alias sam3_map_load='bash ~/sirius_jazzy_ws/bash/startup_bash/sam3_colored_map_select.sh'

# TAB: オフライン・マッピング
# GROUP: オフライン・マッピング
# GROUP_DESC: 【録画時に必要】Unity→rte→rviz2sim→sf_sim→slamtoolbox→record_offline_sim の順で起動。SLAM Toolboxの補正済みTFを /tf としてbagに録画します。
# GROUP_DESC: 【実機録画】実機センサー→rviz2real→sf_real→slamtoolbox_real→zed_offline_recorder→check_offline_real→record_offline_real。ZED取得にNVIDIA/CUDA/ZED SDKは不要です。
# GROUP_DESC: 【地図生成時に必要】sam3_docker_gpu→sam3_settings_ui（設定確認）→run_offline_mapping の順で起動。SLAM Toolboxは起動せず、bag内の補正済みTFを使います。
# GROUP_DESC: record_offline_sim はSAM3 Dockerとの8080競合を自動回避し、画像・LiDAR・補正TFを確認できた場合だけ録画します。終了後にもbagを検証します。
# GROUP_DESC: sam3_map_repair_ui は作成済み地図を残し、rosbagの指定区間・対象クラスだけを再推論して別名の補正版を作ります。

# SUBGROUP: 録画

# Unityステレオ映像・SLAMデータをRosbagに録画（オフラインマッピング用）
alias record_offline_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline.sh'

# 実機ZEDの補正済みステレオ画像をROS 2へ指定FPSで配信（既定8 FPS、CUDA/ZED SDK不要）
alias zed_offline_recorder='bash ~/sirius_jazzy_ws/bash/startup_bash/start_zed_offline_recorder.sh'

# 実機録画の必須トピックと補正TFを検査
alias check_offline_real='bash ~/sirius_jazzy_ws/bash/startup_bash/check_offline_real_ready.sh'

# 実機用Rosbag録画（事前チェック付き、他ノードの自動起動なし）
alias record_offline_real='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_real.sh'

# WebシミュレータTHETAのDual Fisheye・LiDAR・補正TFをRosbagに録画（THETA路面マッピング用 / sim）
alias record_offline_theta_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_theta.sh sim'

# 実機THETAのDual Fisheye・LiDAR・補正TFをRosbagに録画（THETA路面マッピング用 / real）
alias record_offline_theta_real='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_theta.sh real'

# ZED+THETAを1つのbagにまとめて録画（オフライン マッピング用。どちらか無くてもOK / sim|real）
alias record_offline_both='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_both.sh'
alias record_offline_both_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_both.sh sim'
alias record_offline_both_real='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_offline_both.sh real'

# SUBGROUP: THETA キャプチャ・校正

# 実機THETA(HDMI->USBキャプチャ)の映像取得確認（真っ黒ならTHETA側のHDMI出力設定を確認）
alias check_theta_capture='python3 ~/sirius_jazzy_ws/bash/startup_bash/check_theta_capture.py'

# 実機THETAのHDMIキャプチャをROS2(/theta/dual_fisheye/image_raw/compressed)へ配信
# 起動時に対話でfps(既定5)と解像度(既定1280x720)を選択。引数指定も可: theta_capture 5 2
alias theta_capture='bash "$HOME/sirius_jazzy_ws/bash/startup_bash/start_theta_capture.sh"'
alias theta_calib_capture='src && python3 "$HOME/sirius_jazzy_ws/bash/startup_bash/theta_checkerboard_capture.py"'
alias theta_calib_solve='src && python3 "$HOME/sirius_jazzy_ws/bash/startup_bash/theta_checkerboard_solve.py"'
alias theta_calib_check='src && python3 "$HOME/sirius_jazzy_ws/bash/startup_bash/theta_checkerboard_check.py"'

# 実機THETAキャプチャのプレビュー（theta_capture 起動後に実行）
# theta_bev_node(BEV生成) + RViz2 を起動し、raw dual-fisheye と BEV を並べて表示する。
# 実機校正 theta_calibration_real.yaml を使用（TFが無ければYAML姿勢にフォールバック）。
alias rviz2theta_capture='src && ros2 launch sirius_navigation theta_capture_preview.launch.py'

# VLP16点群をTHETAで色付けし /velodyne_points_colored を配信（TFがあれば姿勢はTF優先）
alias theta_colorize='src && ros2 run sirius_navigation theta_colorize_cloud_node'

# SUBGROUP: セットアップ・地図生成

# 実機PCへCUDA不要の録画依存関係を初回セットアップ
alias setup_offline_real='bash ~/sirius_jazzy_ws/bash/startup_bash/setup_offline_real.sh'

# 記録済みRosbagからSAM3推論+RTAB-Mapで2Dカラー地図を生成・保存
alias run_offline_mapping='bash ~/sirius_jazzy_ws/bash/startup_bash/run_offline_mapping.sh'

# 記録済みRosbagからTHETA BEV->地面点群->RTAB-Mapで路面地図を生成・保存（SAM3なし）
alias run_offline_theta_mapping='bash ~/sirius_jazzy_ws/bash/startup_bash/run_offline_theta_mapping.sh'

# SAM3 + SGM GPU推論Dockerサーバーを起動 (port 8080)
alias sam3_docker_gpu='cd ~/sam3_zed_server && docker compose up sam3-zed-merged'

# SAM3のプロンプト、閾値、解像度、深度設定を変更する専用Web UI
alias sam3_settings_ui='bash ~/sirius_jazzy_ws/bash/startup_bash/open_sam3_settings_ui.sh'

# SAM3設定ダイアログ（theta_sam3.yamlを編集・保存）
alias sam3_settings_dialog='bash ~/sirius_jazzy_ws/bash/startup_bash/open_sam3_settings_dialog.sh'

# THETA rosbagの代表フレームでSAM3事前確認（HTMLレポートをブラウザ表示）
alias theta_sam3_preview='bash ~/sirius_jazzy_ws/bash/startup_bash/run_theta_sam3_preview.sh'

# Rosbagの指定区間をクラス別閾値で再推論し、既存地図へ部分マージ
alias sam3_map_repair_ui='bash ~/sirius_jazzy_ws/other_programs/sam3_map_repair/run_sam3_map_repair.sh'

# SUBGROUP: その他

# Unity映像レシーバー単体起動 (HTTP -> ROS2 CompressedImage)
alias unity_stereo_bridge='src && ros2 run sirius_navigation unity_stereo_bridge'

# 2D/3Dカラー地図の保存 (RTAB-Map & SAM3インデックス地図)
alias rtabmap_save='bash ~/sirius_jazzy_ws/bash/startup_bash/rtabmap_save.sh'

# 作成済みセマンティックカラー地図の選択・ロード
alias sam3_colored_map_select='bash ~/sirius_jazzy_ws/bash/startup_bash/sam3_colored_map_select.sh'

# TAB: 3次元地図生成・自律移動
# GROUP: 3次元地図生成・自律移動
# GROUP_DESC: BAGからの地図生成は「3D地図の録画・後処理」の offline_3dmap を使用。3D専用の録画・準備確認・後処理・表示をこのタブに集約しています。THETA路面マッピングの取得・校正は「オフライン・マッピング」タブにあります。以前のTF蓄積・KISS-ICP・RTAB-Map方式は「従来方式の3D地図生成」にまとめています。
# GROUP_DESC: FAST-LIO2で3Dオドメトリを取り、small_gicpで3D地図へ局所自己位置推定し、Nav2で自律移動します。3D地図は2Dへ投影してNav2に渡します。EKF(sf_real)とは排他（odom->baseの二重配信を避ける）です。Webシミュレータで検証する場合は *_sim を使います（/clock・/velodyne_pointsをON、/tfはOFF）。

# SUBGROUP: セットアップ

# 3Dスタック導入 (small_gicp + FAST-LIO2)
alias install3d='bash ~/sirius_jazzy_ws/bash/startup_bash/install_3d_stack.sh'

# ロボットTFのみ起動 (robot_state_publisher。RVizなし＝遠隔/ヘッドレス/XQuartz向け)
alias tf_real='src && ros2 launch sirius_description display.launch.py use_sim_time:=false rviz:=false'

# ロボットTFのみ起動 (Webシミュ用。use_sim_time=true・RViz/関節GUIなし)
alias tf_sim='src && ros2 launch sirius_description display.launch.py use_sim_time:=true rviz:=false use_gui:=false'

# SUBGROUP: 3D地図生成

# 3D LiDAR-IMUオドメトリ (FAST-LIO2) 起動
alias lio3d='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_lio.sh'

# 3D LiDAR-IMUオドメトリ (FAST-LIO2) 起動 (Webシミュ用。use_sim_time=true)
alias lio3d_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_lio.sh sim'

# 現在の点群(/cloud_map等)を3D地図として保存し2Dへ投影 (maps_waypoints/maps3d/<name>)
alias save3d='bash ~/sirius_jazzy_ws/bash/startup_bash/save_3d_map.sh'

# RTAB-Mapでループ閉じ込めしつつライブ3Dマッピング (FAST-LIO2オドメトリ使用, /cloud_mapを配信)
alias rtab3d='src && ros2 launch sirius_navigation theta_3d_mapping.launch.py'

# THETAで色付き3D点群マップを生成 (lio3dと併用。30秒ごと＋停止時にPLY保存)
alias colored3d='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_colored_map.sh'

# THETA色付き3D点群マップ生成 (Webシミュ用。ブラウザの色付き/velodyne_pointsをlio3d_sim姿勢で蓄積)
alias colored3d_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_colored_map.sh sim'

# SUBGROUP: 3D地図の録画・後処理

# 録画前に必須トピック・TF・レートが揃っているか検査（緑=OK）
alias check_3dmap_ready='bash ~/sirius_jazzy_ws/bash/startup_bash/check_theta_3dmap_ready.sh'

# 3D後処理用bag録画（生点群+RGB+IMU+車輪。実機、THETA画像は任意）
alias record_3dmap='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_3dmap.sh real'
# 3D後処理用bag録画（Webシミュ。RGB点群・IMU・/odom・/clockを必須確認）
alias record_3dmap_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_3dmap.sh sim'

# 車輪相対移動+IMU姿勢+縮退方向を抑制したICPで、3D地図と同一形状の色付き3D地図を生成
# BAGから3D地図を生成（床補正・残像除去・色付けに対応。引数なしでBAG選択）
alias offline_3dmap='bash ~/sirius_jazzy_ws/bash/startup_bash/run_offline_3d_mapping.sh'
# FastLIOをBAGから独立再計算。引数なしならBAGを番号選択。
alias recompute_3d_lio='bash ~/sirius_jazzy_ws/bash/startup_bash/recompute_3d_lio.sh'

# 軌跡を変えず残像除去だけ再実行。元地図とframesを照合して誤った組合せを拒否。
alias clean_3dmap='bash ~/sirius_jazzy_ws/bash/startup_bash/clean_3d_map.sh'
# 固定のカメラ/LiDAR校正と元画像から再着色。軌跡・形状・残像除去結果を保持。
alias recolor_3dmap='bash ~/sirius_jazzy_ws/bash/startup_bash/recolor_3d_map.sh'
# 生成済み3D地図と移動軌跡をFoxgloveへ配信（通常・色付き・削除候補・追加仕上げ）
alias view_3dmap='bash ~/sirius_jazzy_ws/bash/startup_bash/view_3d_map.sh'

# SUBGROUP: 従来方式の3D地図生成

# 従来方式：記録済みTFで点群を積み、色付き3D地図(PLY)を生成（bagを対話選択。引数でbagパス指定も可）
alias theta_bag_to_ply='bash ~/sirius_jazzy_ws/bash/startup_bash/run_theta_bag_to_ply.sh'

# 従来方式のcolored_map.plyを /colored_map として配信（RViz2で表示。最新のcolored_map.plyを自動選択）
alias pub_3dmap='src && ros2 run sirius_navigation theta_ply_publisher'

# 従来方式：色付き点群bagをKISS-ICP+ポーズグラフでオフラインSLAM（ファイル直読み・DDS不使用）
# 使い方: run_theta_offline_slam <bag_dir> --start 80 --end 305 [--voxel 0.05]
alias run_theta_offline_slam='python3 ~/sirius_jazzy_ws/other_programs/theta_3d_map/theta_offline_slam.py'

# 従来方式：色付き点群bagをループ閉じ込めSLAM(rtabmap)で処理し、壁2重を解消した色付き3D地図を作成
#（bagを対話選択。引数でbagパス指定も可）
alias run_theta_slam_mapping='bash ~/sirius_jazzy_ws/bash/startup_bash/run_theta_slam_mapping.sh'

# SUBGROUP: 3D自己位置推定

# 3D地図への局所自己位置推定 (small_gicp, map->odom)。初期姿勢はRVizの2D Pose Estimateで指定
alias loc3d='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_localization.sh'

# 3D地図への局所自己位置推定 (Webシミュ用。use_sim_time=true)
alias loc3d_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_localization.sh sim'

# SUBGROUP: 3D自律移動

# 3D地図ベースのNav2自律移動セット (localizer + Nav2。AMCLは使用しない)
alias nav2_3d='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_nav2.sh'

# 3D地図ベースのNav2自律移動セット (Webシミュ用。use_sim_time=true・simパラメータ)
alias nav2_3d_sim='bash ~/sirius_jazzy_ws/bash/startup_bash/start_3d_nav2.sh sim'

# SUBGROUP: 表示

# 3D地図表示用RViz (NUC画面用。XQuartzでは起動不可。遠隔は foxglove を使用)
alias rviz3d='src && rviz2 -d "$HOME/sirius_jazzy_ws/install/sirius_navigation/share/sirius_navigation/rviz/theta_3d_map.rviz"'

# TAB: 実験的
# GROUP: 実験的
# GROUP_DESC: 実験的な機能（KISS-ICPで地面だけXY投影した路面マッピング→Foxglove配信）。

# KISS-ICPで地面XY投影の色付き路面マップ(PNG/PLY)を生成（bagを対話選択。地面z指定可）
alias theta_kiss_ground_map='bash ~/sirius_jazzy_ws/other_programs/theta_3d_map/run_theta_kiss_ground_map.sh'

# 生成した路面マップを /ground_map(点群) と /ground_map_image(画像) で配信（Foxglove用）
alias pub_ground_map='bash ~/sirius_jazzy_ws/other_programs/theta_3d_map/run_pub_ground_map.sh'

# NUCリポジトリの版を監視/同期（originへ）
alias sync_nuc_repo='bash ~/sirius_jazzy_ws/bash/startup_bash/sync_nuc_repo.sh sync'
alias check_nuc_repo='bash ~/sirius_jazzy_ws/bash/startup_bash/sync_nuc_repo.sh check'
