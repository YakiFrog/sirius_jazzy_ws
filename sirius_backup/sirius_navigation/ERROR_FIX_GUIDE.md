# Assisted Teleop エラー修正ガイド

## 発生していたエラー

```
[ERROR] Extrapolation Error looking up target frame: 
Lookup would require extrapolation into the past.
Requested time 24164.250000 but the earliest data is at time 24209.600000

[ERROR] Current footprint not available.
```

## 原因

1. **TF時間同期の問題**: 
   - `sirius3/base_link` → `sirius3/odom` の変換で時刻のずれ
   - `transform_tolerance`パラメータが小さすぎた

2. **初期ポーズ未設定**:
   - AMCLが初期位置を知らないため、ローカライゼーションができない
   - footprint情報が取得できない

3. **Nav2の初期化タイミング**:
   - Assisted Teleopがbehavior_serverの準備完了前に実行された

## 修正内容

### 1. `assisted_teleop.py`の修正

- **初期ポーズ設定機能を追加**:
  ```python
  def set_initial_pose(self):
      # AMCLに初期位置(0, 0, 0)を通知
      initial_pose = PoseStamped()
      initial_pose.header.frame_id = 'map'
      # ... 位置設定 ...
      self.initial_pose_pub.publish(initial_pose)
  ```

- **タイムアウト処理の改善**:
  - アクションサーバー待機に30秒のタイムアウト追加
  - より詳細なログメッセージ

### 2. Nav2パラメータファイルの作成

`config/nav2_params_assisted_teleop.yaml`:

- **TF許容誤差を増加**:
  ```yaml
  transform_tolerance: 0.5  # デフォルト0.1から増加
  ```

- **フレームIDの明示的な設定**:
  ```yaml
  global_frame: sirius3/odom
  robot_base_frame: sirius3/base_link
  ```

- **Assisted Teleop pluginの設定**:
  ```yaml
  behavior_server:
    ros__parameters:
      behavior_plugins: [..., "assisted_teleop", ...]
      assisted_teleop:
        plugin: "nav2_behaviors/AssistedTeleop"
  ```

### 3. 完全なLaunchファイルの作成

`launch/assisted_teleop_with_nav2.launch.py`:

- Nav2スタック全体を起動
- 適切なタイミング制御（TimerAction使用）
- カスタムパラメータファイルの適用

## 使用方法（修正後）

### 方法1: Nav2込みで起動（最も簡単）

```bash
# ターミナル1: シミュレーション起動
ros2 launch sirius_description sim.launch.py

# ターミナル2: Nav2 + Assisted Teleop起動
source ~/sirius_jazzy_ws/install/setup.bash
ros2 launch sirius_navigation assisted_teleop_with_nav2.launch.py
```

### 方法2: Nav2が既に起動している場合

```bash
# Nav2が既に起動していると仮定

# Assisted Teleopのみ起動
source ~/sirius_jazzy_ws/install/setup.bash
ros2 launch sirius_navigation assisted_teleop.launch.py
```

## 起動タイミングの調整

Launchファイル内のTimerActionで調整可能:

```python
# Nav2起動後、10秒待ってからAssisted Teleopを起動
TimerAction(
    period=10.0,  # この値を調整
    actions=[assisted_teleop_node]
)
```

重いシステムの場合は、この値を15-20秒に増やすことを推奨。

## トラブルシューティング

### まだTFエラーが出る場合

1. **TFツリーを確認**:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   
   確認ポイント:
   - `map` → `sirius3/odom` → `sirius3/base_link` の階層
   - 各変換が正常にブロードキャストされているか

2. **transform_toleranceをさらに増やす**:
   `nav2_params_assisted_teleop.yaml`で:
   ```yaml
   transform_tolerance: 1.0  # さらに増やす
   ```

3. **use_sim_timeを確認**:
   ```bash
   ros2 param get /behavior_server use_sim_time
   # 結果: true であるべき
   ```

### footprintエラーが続く場合

1. **AMCLの状態を確認**:
   ```bash
   ros2 topic echo /amcl_pose
   # ポーズが出力されればOK
   ```

2. **コストマップを確認**:
   ```bash
   ros2 topic echo /local_costmap/published_footprint
   # footprintが出力されればOK
   ```

3. **RVizで可視化**:
   - Local Costmap表示
   - Robot Footprint表示
   - TF表示

### アクションサーバーが見つからない

```bash
# behavior_serverが起動しているか確認
ros2 node list | grep behavior_server

# assisted_teleopアクションがあるか確認
ros2 action list | grep assisted_teleop
```

起動していない場合:
```bash
# Nav2を起動
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    params_file:=/path/to/nav2_params_assisted_teleop.yaml
```

## パラメータのカスタマイズ

### robot_radius（ロボットの半径）

実際のロボットサイズに合わせて調整:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22  # メートル単位
```

### 速度制限

```yaml
assisted_teleop:
  max_rotational_vel: 1.0  # 最大回転速度
  min_rotational_vel: 0.1  # 最小回転速度
```

### シミュレーション時間

```yaml
assisted_teleop:
  simulate_ahead_time: 2.0  # 先読みする時間（秒）
```

## 参考コマンド

```bash
# ワークスペースをソース
source ~/sirius_jazzy_ws/install/setup.bash

# パッケージ再ビルド
cd ~/sirius_jazzy_ws
colcon build --packages-select sirius_navigation --symlink-install

# TFツリーを確認
ros2 run tf2_tools view_frames

# トピック一覧
ros2 topic list

# ノード一覧
ros2 node list

# パラメータ確認
ros2 param list /behavior_server
ros2 param get /behavior_server transform_tolerance

# ログレベル変更（デバッグ用）
ros2 run rqt_console rqt_console
```

## まとめ

主な修正ポイント:
1. ✅ 初期ポーズの自動設定
2. ✅ TF許容誤差の増加
3. ✅ 適切な起動タイミングの制御
4. ✅ フレームIDの明示的な設定
5. ✅ 完全なNav2パラメータファイルの提供

これらの修正により、TFエラーとfootprintエラーが解消されます。
