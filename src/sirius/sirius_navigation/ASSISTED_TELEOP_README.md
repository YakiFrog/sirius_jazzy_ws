# Sirius Navigation - Assisted Teleop

このパッケージには、Siriusロボット用のAssisted Teleop（支援付き手動操縦）機能が含まれています。

## 機能

### Assisted Teleop（支援付き手動操縦）

Assisted Teleopは、Nav2の障害物回避機能を使用して、手動操縦を支援します。

**主な特徴:**
- キーボードでロボットを操縦しながら、自動的に障害物を回避
- コストマップを使用して安全な経路を計算
- 衝突しないように速度コマンドをフィルタリング
- 安全な手動操縦をサポート

## 前提条件

Assisted Teleopを使用する前に、以下が起動している必要があります:

1. **ロボットシミュレーション** (または実機)
2. **Nav2スタック** (コストマップとアクションサーバー)

## 使用方法

### 方法1: Launchファイルを使用（推奨）

一つのコマンドでAssisted TeleopとTeleopキーボードを起動:

```bash
ros2 launch sirius_navigation assisted_teleop.launch.py
```

このコマンドは:
- Assisted Teleopノードを起動
- 別ウィンドウでTeleop Twist Keyboardを起動

### 方法2: 個別に起動

#### ターミナル1: Assisted Teleopノードを起動

```bash
ros2 run sirius_navigation assisted_teleop
```

#### ターミナル2: Teleopキーボードを起動

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 完全な起動手順

### 1. シミュレーション環境を起動

```bash
# シリウスシミュレーターを起動
ros2 launch sirius_description sim.launch.py
```

または、UIで設定を選択して起動:

```bash
ros2 launch sirius_description sim_with_ui.launch.py
```

### 2. Nav2を起動

```bash
# Nav2ブリングアップ
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

または、特定の設定ファイルを使用:

```bash
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true \
    params_file:=/path/to/your/nav2_params.yaml
```

### 3. Assisted Teleopを起動

```bash
ros2 launch sirius_navigation assisted_teleop.launch.py
```

## キーボード操作

Teleop Twist Keyboardウィンドウで以下のキーを使用:

```
   u    i    o
   j    k    l
   m    ,    .
```

- `i`: 前進
- `,`: 後退
- `j`: 左回転
- `l`: 右回転
- `u`: 前進+左回転
- `o`: 前進+右回転
- `m`: 後退+左回転
- `.`: 後退+右回転
- `k`: 停止

速度調整:
- `q`/`z`: 線速度を増加/減少
- `w`/`x`: 角速度を増加/減少
- `space`: 緊急停止

## パラメータ

### Assisted Teleopノード

| パラメータ | デフォルト | 説明 |
|-----------|----------|------|
| `time_allowance` | 600秒 (10分) | Assisted Teleopの最大実行時間 |

パラメータをカスタマイズして起動:

```bash
ros2 run sirius_navigation assisted_teleop --ros-args -p time_allowance:=300
```

## トラブルシューティング

### "assisted_teleop action server not available"

**原因:** Nav2が起動していない、またはAssisted Teleopアクションサーバーが無効

**解決策:**
1. Nav2が起動していることを確認
2. Nav2のパラメータファイルでAssisted Teleopが有効になっていることを確認

### ロボットが動かない

**原因:** 
- cmd_velトピックが正しくリマップされていない
- ロボットコントローラーが起動していない

**解決策:**
1. トピックを確認: `ros2 topic list`
2. cmd_velがパブリッシュされていることを確認: `ros2 topic echo /cmd_vel`

### 障害物回避が機能しない

**原因:** コストマップが正しく設定されていない

**解決策:**
1. コストマップが更新されていることを確認
2. センサーデータ(LiDAR, カメラなど)が正しくパブリッシュされていることを確認
3. RVizでコストマップを可視化して確認

## 技術詳細

### トピック

- **購読:**
  - `/cmd_vel` (geometry_msgs/Twist): キーボードからの速度コマンド
  
- **パブリッシュ:**
  - `/cmd_vel` (geometry_msgs/Twist): フィルタリングされた安全な速度コマンド

### アクション

- **使用するアクション:**
  - `/assisted_teleop` (nav2_msgs/AssistedTeleop): Nav2のAssisted Teleopアクション

## 参考資料

- [Nav2 Assisted Teleop](https://navigation.ros.org/behavior_trees/trees/nav_to_pose_and_pause_near_goal_obstacle.html)
- [Nav2 Simple Commander](https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander)
- [Teleop Twist Keyboard](https://github.com/ros2/teleop_twist_keyboard)

## ライセンス

TODO: ライセンス情報を追加

## 作者

- kotantu-desktop
