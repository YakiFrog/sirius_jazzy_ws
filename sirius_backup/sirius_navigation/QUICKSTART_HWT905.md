# HWT905 IMU クイックスタートガイド

## 概要

このガイドでは、Witmotion HWT905 IMUセンサーをSiriusロボットで使用する最速の方法を説明します。

## 前提条件

### 1. ハードウェア
- Witmotion HWT905 IMUセンサー
- USBケーブル（データ通信対応）
- Siriusロボット（またはシミュレーション環境）

### 2. ソフトウェア
```bash
# witmotion_rosパッケージのインストール
cd ~/sirius_jazzy_ws/src
git clone https://github.com/ElettraSciComp/witmotion_IMU_ros.git
cd ~/sirius_jazzy_ws
colcon build --packages-select witmotion_ros
source install/setup.bash
```

### 3. デバイス権限設定
```bash
# 一時的な設定（再起動後に再設定が必要）
sudo chmod 666 /dev/ttyUSB0

# または、永続的な設定（推奨）
sudo usermod -a -G dialout $USER
# 設定後、再ログインまたは再起動
```

## 起動方法

### オプション1: HWT905単体で起動

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py
```

**カスタムポート指定:**
```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py \
  port:=/dev/ttyACM0 \
  baud_rate:=115200
```

### オプション2: EKFと一緒に起動（推奨）

```bash
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true
```

**カスタム設定:**
```bash
ros2 launch sirius_navigation sensor_fusion.launch.py \
  start_hwt905:=true \
  hwt905_port:=/dev/ttyUSB0 \
  hwt905_baud:=115200
```

### オプション3: 完全なナビゲーションスタック

```bash
# シミュレーション環境を起動（別のターミナル）
ros2 launch sirius_description sim.launch.py

# ナビゲーション + HWT905を起動
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true
```

## 動作確認

### 1. IMUデータの確認

```bash
# トピックが配信されているか確認
ros2 topic list | grep imu

# データの内容を確認
ros2 topic echo /imu --once
```

**期待される出力:**
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81  # 重力加速度（静止時）
```

### 1.5. 座標系の確認（重要！）

HWT905とROS2では座標系が異なります。このローンチファイルでは自動変換されます。

**座標系の確認方法:**

```bash
# IMUデータをリアルタイムで確認
ros2 topic echo /imu

# 以下のテストを実行:
# 1. ロボット/センサーを前に傾ける
#    → linear_acceleration.x が正（+）になるべき
# 2. ロボット/センサーを左に傾ける
#    → linear_acceleration.y が正（+）になるべき
# 3. ロボット/センサーを持ち上げる
#    → linear_acceleration.z が正（+）になるべき（重力方向）
```

**正しい座標系（ROS2 REP-103）:**
- X軸: 前方
- Y軸: 左方向
- Z軸: 上方向（重力が-Z、つまり静止時にz=+9.81）

もしデータが反転している場合は、`use_native_orientation`パラメータを確認してください。

### 2. 更新周波数の確認

```bash
ros2 topic hz /imu
```

**期待される結果:** 約50-100Hz

### 3. EKFが動作しているか確認

```bash
ros2 topic echo /odom/filtered --once
```

### 4. TFツリーの確認

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

**期待されるTF構成:**
```
map
  └─ odom
      └─ base_footprint
          └─ imu_link
```

## トラブルシューティング

### デバイスが見つからない

**症状:** `Cannot open port /dev/ttyUSB0`

**解決方法:**
```bash
# デバイスの確認
ls -l /dev/ttyUSB* /dev/ttyACM*

# 見つかったデバイスを指定
ros2 launch sirius_navigation witmotion_hwt905.launch.py port:=/dev/ttyACM0
```

### 権限エラー

**症状:** `Permission denied: '/dev/ttyUSB0'`

**解決方法:**
```bash
# 方法1: 一時的な解決
sudo chmod 666 /dev/ttyUSB0

# 方法2: 永続的な解決（推奨）
sudo usermod -a -G dialout $USER
# 再ログインまたは再起動
```

### データが取得できない

**症状:** トピック `/imu` にデータが流れない

**解決方法:**
```bash
# 1. ボーレートを確認
ros2 launch sirius_navigation witmotion_hwt905.launch.py baud_rate:=9600

# 2. ログレベルをdebugに変更
ros2 launch sirius_navigation witmotion_hwt905.launch.py log_level:=debug

# 3. USBケーブルを確認（データ通信対応のものを使用）

# 4. HWT905の動作を確認（LEDが点灯しているか）
```

### 座標系が反転している

**症状:** IMUデータの加速度や角速度の方向が逆

**解決方法:**
```bash
# witmotion_rosパッケージの設定を確認
ros2 param get /hwt905_imu imu_publisher.use_native_orientation
# Expected: False（ROS2座標系を使用）

# パラメータが正しくない場合、ローンチファイルを確認
cat ~/sirius_jazzy_ws/src/sirius_navigation/launch/witmotion_hwt905.launch.py | grep use_native_orientation
```

**座標系の確認:**
- ロボットを前に傾ける → `linear_acceleration.x` が正（+）
- ロボットを左に傾ける → `linear_acceleration.y` が正（+）
- 静止時の重力 → `linear_acceleration.z` が約+9.81

### EKFがIMUデータを使わない

**症状:** `/odom/filtered` がIMUの姿勢を反映していない

**解決方法:**
```bash
# 1. トピック名を確認
ros2 topic list | grep imu
# /imu が存在することを確認

# 2. EKFの診断情報を確認
ros2 topic echo /diagnostics | grep ekf

# 3. EKFのログを確認
ros2 launch sirius_navigation sensor_fusion.launch.py
# EKFのログで "imu0" が認識されているか確認
```

## Launch引数リファレンス

### witmotion_hwt905.launch.py

| 引数 | デフォルト | 説明 |
|-----|-----------|------|
| `port` | `/dev/ttyUSB0` | シリアルポート |
| `baud_rate` | `115200` | ボーレート |
| `frequency` | `100.0` | 更新周波数 [Hz] |
| `frame_id` | `imu_link_raw` | HWT905生データフレーム |
| `ros_frame_id` | `imu_link` | ROS2標準フレーム |
| `imu_topic` | `/imu` | 出力トピック名 |
| `use_transform` | `true` | 座標変換を有効化 |
| `log_level` | `info` | ログレベル |

### sensor_fusion.launch.py (HWT905オプション付き)

| 引数 | デフォルト | 説明 |
|-----|-----------|------|
| `start_hwt905` | `false` | HWT905を自動起動 |
| `hwt905_port` | `/dev/ttyUSB0` | HWT905のポート |
| `hwt905_baud` | `115200` | HWT905のボーレート |

## 使用例

### 例1: 基本的な起動

```bash
# HWT905とEKFを起動
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# 別のターミナルで、IMUデータを確認
ros2 topic echo /imu
```

### 例2: カスタムポートで起動

```bash
ros2 launch sirius_navigation sensor_fusion.launch.py \
  start_hwt905:=true \
  hwt905_port:=/dev/ttyACM0
```

### 例3: デバッグモード

```bash
ros2 launch sirius_navigation witmotion_hwt905.launch.py \
  port:=/dev/ttyUSB0 \
  log_level:=debug
```

### 例4: シミュレーションと統合

```bash
# ターミナル1: Gazeboシミュレーション
ros2 launch sirius_description sim.launch.py

# ターミナル2: ナビゲーション + HWT905
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# ターミナル3: RViz2で可視化
rviz2
```

## 次のステップ

1. **キャリブレーション**: HWT905付属のソフトウェアでキャリブレーションを実行
2. **パフォーマンス調整**: `ekf_fusion.yaml` のパラメータを最適化
3. **ナビゲーション**: AMCLやNav2と統合してロボットを自律移動

## 関連ドキュメント

- [README_HWT905.md](./README_HWT905.md) - HWT905の詳細設定ガイド
- [HWT905_COORDINATE_TRANSFORM.md](./HWT905_COORDINATE_TRANSFORM.md) - 座標系変換の詳細
- [SENSOR_FUSION_README.md](./SENSOR_FUSION_README.md) - センサーフュージョンの概要
- [EKF_INITIALPOSE_GUIDE.md](./EKF_INITIALPOSE_GUIDE.md) - EKFの初期位置設定

## サポート

問題が発生した場合は、以下を確認してください:
1. デバイスが正しく接続されているか
2. ポート権限が設定されているか
3. ボーレートが正しいか
4. USBケーブルがデータ通信に対応しているか
5. HWT905のLEDが点灯しているか

それでも解決しない場合は、`--log-level debug` でログを確認してください。
