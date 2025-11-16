# HWT905 座標系変換ガイド

## 概要

HWT905 IMUセンサーとROS2では、座標系の定義が異なります。このドキュメントでは、座標系の違いと、`witmotion_ros`パッケージによる自動変換について説明します。

## 座標系の定義

### HWT905の座標系（ネイティブ）

```
         Y (右)
         ^
         |
         |
    _____|_____
   |     |     |
   |  HWT905   |
   |_____|_____|
         |
         |--------> X (前)
        /
       /
      v Z (下)

重力方向: +Z（下向き）
```

- **X軸**: 前方（ロボットの進行方向）
- **Y軸**: 右方向
- **Z軸**: 下方向（重力が+Z方向）
- **座標系**: 右手座標系
- **加速度**: 静止時、Z軸が約+9.81 m/s²

### ROS2標準座標系（REP-103）

```
         Z (上)
         ^
         |
        /|
       / |
      /  |--------> X (前)
     /
    v Y (左)

重力方向: -Z（下向き）
```

- **X軸**: 前方（ロボットの進行方向）
- **Y軸**: 左方向
- **Z軸**: 上方向（重力が-Z方向）
- **座標系**: 右手座標系
- **加速度**: 静止時、Z軸が約+9.81 m/s²（重力加速度は+Z方向を示す）

参考: [ROS REP-103](https://www.ros.org/reps/rep-0103.html)

## 座標変換の詳細

### 変換方法

HWT905からROS2標準座標系への変換は以下のように行われます：

```
ROS2座標系 = 変換 × HWT905座標系

変換行列:
[X']   [ 1  0  0] [X]
[Y'] = [ 0 -1  0] [Y]
[Z']   [ 0  0 -1] [Z]

つまり:
X' = X   (前方向は同じ)
Y' = -Y  (右 → 左に反転)
Z' = -Z  (下 → 上に反転)
```

### クォータニオンの変換

姿勢（orientation）もクォータニオンで適切に変換されます：

```python
# HWT905のクォータニオン (q_hwt905)
q_hwt905 = (x, y, z, w)

# ROS2標準クォータニオン (q_ros2)
# X軸周りに180度回転することで、Y軸とZ軸を反転
rotation_x_180 = (1, 0, 0, 0)  # X軸周りに180度
q_ros2 = q_hwt905 * rotation_x_180
```

## witmotion_rosパッケージの設定

### use_native_orientationパラメータ

`witmotion_ros`パッケージには、座標系を制御するパラメータがあります：

```yaml
imu_publisher:
  use_native_orientation: false  # ROS2標準座標系を使用（推奨）
  # use_native_orientation: true   # HWT905ネイティブ座標系（非推奨）
```

#### false（デフォルト、推奨）
- HWT905のデータをROS2標準座標系に**自動変換**
- EKFやNav2などの標準ROS2パッケージと互換性あり
- **このプロジェクトで使用**

#### true（非推奨）
- HWT905のネイティブ座標系をそのまま使用
- 手動で座標変換が必要
- ROS2標準と非互換

## ローンチファイルでの設定

`witmotion_hwt905.launch.py`では、以下のように設定されています：

```python
witmotion_node = Node(
    package='witmotion_ros',
    executable='witmotion_ros_node',
    name='hwt905_imu',
    parameters=[
        witmotion_config,
        {
            'imu_publisher.use_native_orientation': False,  # 自動変換
            'imu_publisher.frame_id': 'imu_link',
            'imu_publisher.topic_name': '/imu',
        }
    ],
)
```

## 動作確認方法

### 方法1: 加速度データで確認

```bash
# IMUデータを表示
ros2 topic echo /imu

# 以下のテストを実行:
```

**テスト1: 前方に傾ける**
```
センサーを前に傾ける
→ linear_acceleration.x が正（+）になる
→ 正しい！（ROS2標準）
```

**テスト2: 左に傾ける**
```
センサーを左に傾ける
→ linear_acceleration.y が正（+）になる
→ 正しい！（ROS2標準）
```

**テスト3: 静止状態**
```
センサーを水平に置く
→ linear_acceleration.z が約+9.81 m/s²
→ 正しい！（重力加速度）
```

### 方法2: RViz2で視覚的に確認

```bash
# RViz2を起動
rviz2

# 以下を追加:
# 1. Fixed Frame: imu_link
# 2. Add → By topic → /imu → Imu
# 3. IMU表示の矢印方向を確認
```

- **赤矢印（X軸）**: 前方を指すべき
- **緑矢印（Y軸）**: 左方を指すべき
- **青矢印（Z軸）**: 上方を指すべき

### 方法3: TFフレームで確認

```bash
# TFフレームを確認
ros2 run tf2_ros tf2_echo base_footprint imu_link

# 出力例:
# At time 1234567890.123456789
# - Translation: [0.000, 0.000, 0.100]
# - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

## トラブルシューティング

### 座標系が反転している場合

**症状**: 
- ロボットを前に動かすと、オドメトリが後ろに動く
- 回転方向が逆になる
- EKFが不安定

**原因**:
`use_native_orientation`が`true`になっている可能性

**解決方法**:
```bash
# パラメータを確認
ros2 param get /hwt905_imu imu_publisher.use_native_orientation

# Expected: False

# パラメータを変更（一時的）
ros2 param set /hwt905_imu imu_publisher.use_native_orientation false

# または、ローンチファイルを修正して再起動
```

### 手動で座標変換する場合

もし何らかの理由で手動変換が必要な場合：

```python
# Pythonでの変換例
def hwt905_to_ros2(hwt905_data):
    """HWT905座標系からROS2標準座標系へ変換"""
    ros2_data = {
        'x': hwt905_data['x'],      # X軸はそのまま
        'y': -hwt905_data['y'],     # Y軸を反転
        'z': -hwt905_data['z'],     # Z軸を反転
    }
    return ros2_data

# 加速度の変換
accel_ros2 = hwt905_to_ros2(accel_hwt905)

# 角速度の変換
gyro_ros2 = hwt905_to_ros2(gyro_hwt905)
```

## EKFとの統合

EKFは、変換後のROS2標準座標系のデータを期待します：

```yaml
# ekf_fusion.yaml
imu0: /imu
imu0_config: [false, false, false,  # x, y, z（位置情報なし）
              false, false, true,   # roll, pitch, yaw（yawのみ使用）
              false, false, false,  # vx, vy, vz
              false, false, false,  # vroll, vpitch, vyaw
              false, false, false]  # ax, ay, az
imu0_differential: false  # AHRS（絶対姿勢）モード
```

座標系が正しく設定されていれば、EKFは安定して動作します。

## まとめ

| 項目 | HWT905ネイティブ | ROS2標準（変換後） |
|-----|----------------|-------------------|
| X軸 | 前（+X） | 前（+X） |
| Y軸 | 右（+Y） | 左（+Y） |
| Z軸 | 下（+Z） | 上（+Z） |
| 重力方向 | +Z | -Z（データでは+Z） |
| 使用設定 | `use_native_orientation: true` | `use_native_orientation: false` ✓ |
| 推奨 | ❌ | ✅ |

**重要**: `witmotion_ros`パッケージの`use_native_orientation: false`設定により、座標系は**自動的に変換**されます。追加の手動変換は不要です。

## 参考資料

- [ROS REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [Witmotion HWT905 データシート](https://www.wit-motion.com/)
- [witmotion_ros GitHubリポジトリ](https://github.com/ElettraSciComp/witmotion_IMU_ros)
- [robot_localization座標系ガイド](http://docs.ros.org/en/jazzy/p/robot_localization/)
