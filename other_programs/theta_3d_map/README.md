# theta_3d_map : 色付き3D地図（VLP-16 × RICOH THETA）

走行中に録画した rosbag から、色付き3D点群地図をオフラインで生成するツールです。
OmniColor (arXiv:2404.04693) の投影＋色付けの発展形として、まず「蓄積して地図化」を
実現します（姿勢最適化や多視点色融合は今後の拡張）。

## 1. 録画

事前に VLP16 と THETAキャプチャ、色付けノードを起動しておく:

```bash
# 実機
theta_capture                         # /theta/dual_fisheye/image_raw/compressed
ros2 run sirius_navigation theta_colorize_cloud_node   # /velodyne_points_colored
# TF を使うため robot_state_publisher (rviz2real) を起動しておくこと

# 録画（real|sim）
bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_3dmap.sh real
```

録画トピック: `/velodyne_points_colored`, `/velodyne_points`(生: deskew用),
`/imu`, `/magnetometer`, THETA画像, `/tf`, `/tf_static`, `/odom`, `/odom/filtered`
（simは `/clock` も）。`/scan3` は含めません。
用途: 色付き地図（`theta_bag_to_ply.py` / `theta_offline_slam.py`）と
LiDAR-IMU SLAM（FAST-LIO2等、生点群+IMUを使用）。

帯域目安: 約6.4MB/s（≒380MB/分）。録画は終わったら必ず Ctrl+C で停止すること
（停止しないとbagが成長し続け、オフライン再生が破綻する）。

## 2. オフラインでPLY生成

```bash
python3 ~/sirius_jazzy_ws/other_programs/theta_3d_map/theta_bag_to_ply.py \
    ~/rosbag2_data/<bag名> -o ~/rosbag2_data/<bag名>/colored_map.ply
```

主なオプション:
- `--frame sirius3/odom` : 出力の固定フレーム（SLAMを使う場合は `map` も可）
- `--voxel 0.05`         : ボクセルサイズ[m]（小さいほど密・重い）
- `--min-range 0.5` / `--max-range 60` : 距離フィルタ[m]
- `--drop-uncolored`     : 色が付かなかった点を除外（既定は黒で保持）
- `--topic /velodyne_points_colored` : 入力点群（`/velodyne_points` も可、無色）

## 3. 表示

```bash
rviz2   # PointCloud2でcolored_map.plyの代わりにトピックを見る場合
python3 -c "import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('colored_map.ply')])"
```

## 制約（初版）
- 姿勢は odom(EKF) 基準。長距離はドリフトで壁が二重になり得る → SLAM Toolbox(`map`)や
  RTAB-Map のポーズで改善する。
- VLP-16は1スキャンが薄い扇。**動きながら**録画することで立体化する。
- 色はカメラに見える面のみ。裏面・遮蔽面は無色（黒）。
