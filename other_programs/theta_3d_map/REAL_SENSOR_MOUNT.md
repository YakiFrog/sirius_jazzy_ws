# 実機のLiDAR・THETA取り付け角度

共通設定: `sirius_description/config/sensor_mount_real.yaml`。
2026-10-20 BAGの静止床面から推定した暫定値は ROS RPY(deg) =
`[-0.4560445695, +1.2777840777, 0]`。正のpitchは前方を下げる向き。
床基準のロール・ピッチであり、LiDAR–IMUの6自由度校正が完了した値ではない。
残差を消すために角度を追加したり、加速度の横成分を一律に引いたりしない。

実機起動の経路:

- `tf_real`: 共通設定からLiDARの実際の取り付けTFを生成。IMUは変更しない。
  THETAは同一の剛体取り付けという申告に基づき、LiDAR中心まわりに同じ回転を適用し、
  LiDAR–THETA間の相対位置・姿勢を保つ。THETA単体での角度校正ではない。
- `lio3d`: 同じ設定から `mapping.extrinsic_R = R_imu_base * R_base_lidar` を生成。
  `/velodyne_points`（生点群）を入力する。補正後点群をここに入れると二重補正になる。
- `velodyne`: 生点群に加えて `/velodyne_points_leveled` を生成。
  `sirius3/lidar_level` はLiDARと同じ原点で、軸は `base_link` と平行。
  XYZだけをTFで変換し、ring・intensity・点時刻・ヘッダ時刻を保持する。
  固定取り付け角度の変換であり、走行中の姿勢補正・deskew・床の平面化ではない。
  `/scan` と `/scan3` もこの補正後点群から生成する。
- `record_loc3d`: 生点群、補正後点群、`/scan3`、TF、実行時FastLIO設定を保存する。

反映には `tf_real`・`velodyne`・`lio3d` の再起動が必要。
自己位置推定中なら `loc3d` も停止し、全体を起動し直して初期位置を再設定する。
設定を揃えず一部だけ再起動しない。既存のBAGや地図は書き換えない。
シミュレータの取り付け設定には実機補正を適用しない。

一時的な無効化は `tf_real` の `mount_config:=none` と
`SIRIUS_SENSOR_MOUNT_CONFIG='' lio3d`（または起動スクリプト）を対にして使う。
生点群だけの従来ドライバは元の `velodyne-all-nodes-VLP16-composed-launch.py`。
