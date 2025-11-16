# Witmotion HWT905 IMU セットアップガイド

## 概要

このガイドでは、Witmotion HWT905 AHRSセンサをSiriusロボットのナビゲーションシステムに統合する方法を説明します。

## HWT905の特徴

- **高精度姿勢推定**: 0.05°の精度（静止時）
- **3軸センサー**: 加速度計、ジャイロスコープ、磁力計
- **内蔵Kalmanフィルタ**: 姿勢計算済みAHRS出力
- **高速データ出力**: 0.2-200Hz（調整可能）
- **広範囲計測**:
  - 加速度: ±16g
  - 角速度: ±2000°/s
  - 磁場: ±4900µT

## 座標系の変換

HWT905とROS2では座標系の定義が異なります：

### HWT905の座標系
- **X軸**: 前方
- **Y軸**: 右方向
- **Z軸**: 下方向（重力が+Z方向）
- 右手座標系

### ROS2標準座標系（REP-103）
- **X軸**: 前方
- **Y軸**: 左方向
- **Z軸**: 上方向（重力が-Z方向）
- 右手座標系

### 自動変換機能

`witmotion_ros`パッケージの`use_native_orientation: false`パラメータにより、**HWT905の座標系からROS2標準座標系への自動変換が行われます**。

このローンチファイルでは、この変換が自動的に適用されるため、**追加の座標変換は不要**です。EKFや他のROS2ノードは、正しいROS2標準座標系でIMUデータを受信できます。

```yaml
# witmotion_rosの内部設定
imu_publisher.use_native_orientation: false  # ROS2座標系へ自動変換
```

変換の詳細：
- Y軸とZ軸が反転されます（Y' = -Y, Z' = -Z）
- クォータニオンも適切に変換されます
- 加速度、角速度も同様に変換されます

## システム構成

```
HWT905 IMU
    ↓ (USB接続)
witmotion_ros_node
    ↓ (トピック: /imu)
robot_localization (EKF)
    ↓ (融合後のodometry)
Nav2 ナビゲーションスタック
```

## セットアップ手順

### 1. ハードウェア接続

1. HWT905をロボットのコンピュータにUSB接続
2. デバイスの確認:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
3. ポート権限の設定（一時的）:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
4. ポート権限の設定（永続的）:
   ```bash
   sudo usermod -a -G dialout $USER
   # 再ログインまたは再起動が必要
   ```

### 2. witmotion_rosパッケージのインストール

```bash
cd ~/sirius_jazzy_ws/src
git clone https://github.com/ElettraSciComp/witmotion_IMU_ros.git
cd ~/sirius_jazzy_ws
colcon build --packages-select witmotion_ros
source install/setup.bash
```

### 3. HWT905の起動

#### 基本的な使用方法

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py
```

#### カスタムポート指定

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py port:=/dev/ttyACM0
```

#### 詳細設定

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py \
  port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  frequency:=100.0 \
  frame_id:=imu_link \
  log_level:=info
```

### 4. EKFとの統合

HWT905とEKFを同時に起動:

```bash
# ターミナル1: HWT905 IMU
ros2 launch sirius_navigation witmotion_hwt905.launch.py

# ターミナル2: EKF Fusion
ros2 launch sirius_navigation sensor_fusion.launch.py
```

### 5. 完全なナビゲーションスタックの起動

```bash
ros2 launch sirius_navigation navigation_with_fusion.launch.py
```

## トピックとメッセージ

### 出力トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/imu` | `sensor_msgs/msg/Imu` | IMUデータ（姿勢、角速度、加速度） |

### メッセージ内容

```python
# sensor_msgs/msg/Imu
orientation:            # 姿勢（クォータニオン）
  x: float
  y: float
  z: float
  w: float
angular_velocity:       # 角速度 [rad/s]
  x: float  # roll rate
  y: float  # pitch rate
  z: float  # yaw rate
linear_acceleration:    # 線形加速度 [m/s²]
  x: float
  y: float
  z: float
```

## EKF設定

`ekf_fusion.yaml`でのHWT905の設定:

```yaml
imu0: /imu
imu0_config: [false, false, false,  # x, y, z（位置情報なし）
              false, false, true,   # roll, pitch, yaw（yawのみ使用）
              false, false, false,  # vx, vy, vz
              false, false, false,  # vroll, vpitch, vyaw
              false, false, false]  # ax, ay, az
imu0_differential: false  # AHRS（絶対姿勢）モード
```

### 設計の理由

1. **yawのみ使用**: 2D平面移動ロボットではroll/pitchは不要
2. **differential: false**: HWT905は内蔵Kalmanフィルタで姿勢計算済み（AHRS）
3. **角速度未使用**: IMU内部で既に姿勢計算に使用済み（二重使用を回避）
4. **加速度未使用**: ノイズが大きく、ホイールオドメトリで十分

## 検証とテスト

### 1. IMUデータの確認

```bash
ros2 topic echo /imu
```

### 2. 更新周波数の確認

```bash
ros2 topic hz /imu
```

期待値: 50-100Hz

### 3. 姿勢の確認

RViz2でIMUの姿勢を可視化:

```bash
rviz2
```

- Add → By topic → `/imu` → Imu

### 4. EKF出力の確認

```bash
ros2 topic echo /odometry/filtered
```

### 5. TFツリーの確認

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

期待されるTF構成:
```
map
  └─ odom (AMCLが配信)
      └─ base_footprint (EKFが配信)
          └─ imu_link
```

## トラブルシューティング

### デバイスが見つからない

```bash
# 接続されているシリアルデバイスを確認
ls -l /dev/ttyUSB* /dev/ttyACM*

# dmesgでUSBデバイスを確認
dmesg | grep tty
```

### 権限エラー

```bash
# 一時的な解決策
sudo chmod 666 /dev/ttyUSB0

# 永続的な解決策
sudo usermod -a -G dialout $USER
# 再ログインまたは再起動
```

### データが取得できない

1. ボーレートを確認（デフォルト: 115200）
2. ポートが正しいか確認
3. HWT905のファームウェアをアップデート
4. USBケーブルを確認（データ通信対応のものを使用）

### 姿勢がドリフトする

1. `imu0_differential: false`が設定されているか確認
2. キャリブレーションを実行（HWT905設定ソフトウェアを使用）
3. 磁気干渉の確認（モーター、バッテリーから離す）
4. `use_native_orientation: false`が設定されているか確認

### 座標系が反転している

HWT905とROS2の座標系の違いに注意してください：

**問題**: 加速度やジャイロのデータが反転している

**解決方法**:
1. `use_native_orientation: false`を確認（デフォルト設定）
2. witmotion_rosパッケージが最新版か確認
```bash
cd ~/sirius_jazzy_ws/src/witmotion_ros
git pull
cd ~/sirius_jazzy_ws
colcon build --packages-select witmotion_ros
```

**手動で確認する方法**:
```bash
# IMUデータを確認
ros2 topic echo /imu

# 正しい座標系の場合:
# - ロボットを前に傾ける → linear_acceleration.x が正（+）
# - ロボットを左に傾ける → linear_acceleration.y が正（+）
# - ロボットを持ち上げる → linear_acceleration.z が正（+）
```

### EKFがIMUデータを使用しない

1. トピック名が一致しているか確認: `/imu`
2. フレームIDがTFツリーに存在するか確認
3. タイムスタンプが正しいか確認
4. EKFの診断情報を確認:
   ```bash
   ros2 topic echo /diagnostics
   ```

## パフォーマンスチューニング

### 更新周波数の最適化

- **50Hz**: 標準的な使用（推奨）
- **100Hz**: 高速移動時の精度向上
- **200Hz**: 最高精度（CPU負荷高）

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py frequency:=100.0
```

### フィルタパラメータの調整

`ekf_fusion.yaml`の調整ポイント:

```yaml
# プロセスノイズ（yaw）
process_noise_covariance[35]: 0.01  # 小さいほどIMUを信頼

# センサータイムアウト
sensor_timeout: 0.2  # 秒
```

## Launch引数一覧

| 引数 | デフォルト値 | 説明 |
|-----|------------|------|
| `port` | `/dev/ttyUSB0` | シリアルポート |
| `baud_rate` | `115200` | ボーレート |
| `frequency` | `100.0` | 更新周波数 [Hz] |
| `frame_id` | `imu_link_raw` | HWT905生データのフレームID |
| `ros_frame_id` | `imu_link` | ROS2標準座標系のフレームID |
| `imu_topic` | `/imu` | 出力トピック名 |
| `use_transform` | `true` | 座標変換の有効化 |
| `log_level` | `info` | ログレベル |

## 関連ドキュメント

- [HWT905_COORDINATE_TRANSFORM.md](./HWT905_COORDINATE_TRANSFORM.md) - 座標系変換の詳細説明
- [EKF_INITIALPOSE_GUIDE.md](./EKF_INITIALPOSE_GUIDE.md) - EKFの初期位置設定
- [SENSOR_FUSION_README.md](./SENSOR_FUSION_README.md) - センサーフュージョンの概要
- [ekf_fusion.yaml](../../params/ekf_fusion.yaml) - EKF設定ファイル

## 参考リンク

- [Witmotion公式サイト](https://www.wit-motion.com/)
- [witmotion_ros GitHubリポジトリ](https://github.com/ElettraSciComp/witmotion_IMU_ros)
- [robot_localization ドキュメント](http://docs.ros.org/en/jazzy/p/robot_localization/)

## ライセンス

このドキュメントは、Siriusロボットプロジェクトの一部として提供されています。
