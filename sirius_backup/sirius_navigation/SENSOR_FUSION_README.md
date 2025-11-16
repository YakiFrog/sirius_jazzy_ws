# IMU & オドメトリ センサフュージョン

## 概要
`robot_localization`パッケージのEKF（拡張カルマンフィルタ）を使用して、IMUとホイールオドメトリを融合します。

### 融合の効果
- **より安定した姿勢推定**: IMUの姿勢情報でオドメトリのドリフトを補正
- **高精度な速度推定**: ホイールエンコーダーとIMUの角速度を組み合わせ
- **ロバストな自己位置推定**: 片方のセンサーが不安定でも補完可能

## 対応IMUセンサー

### Witmotion HWT905（実機推奨）
- **高精度AHRS**: 静止時0.05°の姿勢精度
- **内蔵Kalmanフィルタ**: 姿勢計算済みで高品質
- **高速更新**: 最大200Hz
- **詳細情報**: [README_HWT905.md](./README_HWT905.md)
- **クイックスタート**: [QUICKSTART_HWT905.md](./QUICKSTART_HWT905.md)

### シミュレーション用IMU
- Gazeboの仮想IMUセンサー
- シミュレーション環境でのテスト用

## 設定ファイル

### `/params/ekf_fusion.yaml`
EKFの設定ファイル。以下を定義：
- **Odometry**: x, y位置、vx, vy速度を使用
- **IMU**: roll, pitch, yaw姿勢、角速度、加速度を使用
- **更新周波数**: 30Hz
- **プロセスノイズ**: 各状態変数の不確実性

## 起動方法

### 方法1: シミュレーション環境

```bash
# ターミナル1: シミュレーション
cd ~/sirius_jazzy_ws
source install/setup.bash
ros2 launch sirius_description sim.launch.py

# ターミナル2: センサフュージョン（シミュレーションIMU使用）
cd ~/sirius_jazzy_ws
source install/setup.bash
ros2 launch sirius_navigation sensor_fusion.launch.py
```

### 方法2: 実機環境（HWT905使用）

```bash
# HWT905を自動起動してセンサフュージョン
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# カスタムポート指定
ros2 launch sirius_navigation sensor_fusion.launch.py \
  start_hwt905:=true \
  hwt905_port:=/dev/ttyUSB0 \
  hwt905_baud:=115200
```

### 方法3: HWT905単体テスト

```bash
# HWT905のみを起動してデータ確認
ros2 launch sirius_navigation witmotion_hwt905.launch.py

# データを確認
ros2 topic echo /imu
```

### 方法4: 完全なナビゲーションスタック（実機）

```bash
# ターミナル1: センサフュージョン + HWT905
ros2 launch sirius_navigation sensor_fusion.launch.py start_hwt905:=true

# ターミナル2: Nav2
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$HOME/sirius_jazzy_ws/params/nav2_params.yaml

# ターミナル3: Localization
ros2 launch nav2_bringup localization_launch.py \
    map:=$HOME/sirius_jazzy_ws/maps_waypoints/map.yaml \
    params_file:=$HOME/sirius_jazzy_ws/params/nav2_params.yaml
```

## 動作確認

### フュージョン後のオドメトリを確認
```bash
# 元のオドメトリ
ros2 topic echo /odom

# フュージョン後のオドメトリ
ros2 topic echo /odom/filtered

# 両方を比較
ros2 topic hz /odom
ros2 topic hz /odom/filtered
```

### TFツリーを確認
```bash
# TFフレームの関係を可視化
ros2 run tf2_tools view_frames

# 生成されたPDFを開く
evince frames.pdf
```

### EKFの診断情報を確認
```bash
# 診断情報をリアルタイム表示
ros2 topic echo /diagnostics

# EKFノードのログを確認
ros2 node info /ekf_filter_node
```

### RVizで可視化
```bash
# RVizでフュージョン後のオドメトリを表示
ros2 run rviz2 rviz2
```

RVizで以下を追加：
1. **Odometry** → Topic: `/odom` （元のオドメトリ・赤色）
2. **Odometry** → Topic: `/odom/filtered` （フュージョン後・緑色）
3. **TF** → すべてのフレームを表示

## チューニング

### センサーの信頼度調整

`ekf_fusion.yaml`の各センサーの`config`配列で調整：

```yaml
# Odometryの使用データ（trueで使用）
odom0_config: [true,  true,  false,   # x, y, z位置
               false, false, false,   # roll, pitch, yaw
               true,  true,  false,   # vx, vy, vz速度
               ...]

# IMUの使用データ
imu0_config: [false, false, false,    # x, y, z位置（なし）
              true,  true,  true,     # roll, pitch, yaw姿勢
              false, false, false,    # vx, vy, vz速度
              true,  true,  true,     # 角速度
              true,  true,  true]     # 加速度
```

### ノイズ調整

センサーのノイズが大きい場合、`process_noise_covariance`を調整：

```yaml
# 値を大きくする → そのセンサーの信頼度を下げる
# 値を小さくする → そのセンサーの信頼度を上げる
process_noise_covariance: [0.05, 0.0, ...]
```

### 更新周波数調整

```yaml
# 高頻度更新（CPU負荷高・精度高）
frequency: 30.0

# 低頻度更新（CPU負荷低・精度やや低）
frequency: 10.0
```

## トラブルシューティング

### 自己位置がぐちゃぐちゃになる

**症状**: パーティクルが乱れる、ロボットの位置が不安定

**原因と対処**:

1. **EKFの更新レートが高すぎる**
   ```yaml
   # 修正前（高すぎる）
   frequency: 30.0
   
   # 修正後（シミュレータに適切）
   frequency: 10.0
   ```

2. **センサータイムアウトが短すぎる**
   ```yaml
   # 修正前（短すぎる）
   sensor_timeout: 0.1
   
   # 修正後（余裕を持たせる）
   sensor_timeout: 0.5
   ```

3. **IMUの角速度・加速度がノイジー**
   ```yaml
   # yawのみを使用（roll, pitch, 角速度, 加速度をオフ）
   imu0_config: [false, false, false,  # x, y, z
                 false, false, true,   # roll, pitch, yaw（yawのみ）
                 false, false, false,  # vx, vy, vz
                 false, false, false,  # 角速度オフ
                 false, false, false]  # 加速度オフ
   ```

4. **TF配信の競合**
   ```yaml
   # EKFのTF配信をオフにする
   publish_tf: false
   ```

5. **2D/3Dモードの問題**
   ```yaml
   # シミュレータでは2Dモードが安定
   two_d_mode: true
   ```

### EKFが起動しない
```bash
# robot_localizationがインストールされているか確認
ros2 pkg list | grep robot_localization

# インストールされていない場合
sudo apt install ros-jazzy-robot-localization
```

### フュージョン後のオドメトリが配信されない
```bash
# IMUとオドメトリが正常に配信されているか確認
ros2 topic list | grep -E "odom|imu"
ros2 topic hz /odom
ros2 topic hz /imu

# EKFノードが実行中か確認
ros2 node list | grep ekf
```

### TF変換エラー
```bash
# TFが正常に配信されているか確認
ros2 run tf2_ros tf2_echo map sirius3/base_link
ros2 run tf2_ros tf2_echo sirius3/odom sirius3/base_link

# タイムスタンプの問題がある場合
# ekf_fusion.yamlのsensor_timeoutを大きくする
sensor_timeout: 0.5  # 0.1 → 0.5
```

## 参考資料
- [robot_localization公式ドキュメント](http://docs.ros.org/en/jazzy/p/robot_localization/)
- [EKFパラメータ説明](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)
- [Nav2との統合](https://navigation.ros.org/)
- **HWT905セットアップ**: [README_HWT905.md](./README_HWT905.md)
- **HWT905クイックスタート**: [QUICKSTART_HWT905.md](./QUICKSTART_HWT905.md)
