# EKF Initial Pose 設定ガイド

## 📋 概要

RViz2の「2D Pose Estimate」でEKFの初期位置を設定できるようになりました！

## 🎯 実装内容

### 1. **EKF Pose Initializer ノード**
- **役割**: RViz2の`/initialpose`をEKFの`set_pose`サービスに転送
- **効果**: AMCLとEKFの初期位置を同期

### 2. **自動起動**
`sensor_fusion.launch.py`で自動的に起動されます

## 🚀 使い方

### ステップ1: センサフュージョン起動
```bash
cd ~/sirius_jazzy_ws
source install/setup.bash
ros2 launch sirius_navigation sensor_fusion.launch.py
```

**起動ログ確認**:
```
[ekf_pose_initializer-2] [INFO] [xxx]: ✅ EKF Pose Initializer 起動完了
[ekf_pose_initializer-2] [INFO] [xxx]: RViz2で "2D Pose Estimate" を設定してください
```

### ステップ2: RViz2で初期位置設定

1. RViz2を起動
2. 上部ツールバーの「**2D Pose Estimate**」をクリック
3. マップ上でロボットの位置をクリック
4. ドラッグして向きを設定

**成功ログ**:
```
[ekf_pose_initializer-2] [INFO] [xxx]: 📍 Initial Pose受信: x=2.50, y=1.20
[ekf_pose_initializer-2] [INFO] [xxx]: ✅ EKFの位置を初期化しました
```

## 🔄 動作フロー

```
┌─────────────┐
│   RViz2     │ 2D Pose Estimate
│             │ (/initialpose)
└──────┬──────┘
       │
       ▼
┌─────────────────────┐
│ EKF Pose Initializer│ ← 新しく追加したノード
│                     │
│ /initialpose 受信   │
│ ↓                   │
│ set_pose 呼び出し   │
└──────┬──────────────┘
       │
       ▼
┌─────────────┐
│  EKF Node   │ 位置をリセット
│             │ (x, y, yaw を設定)
└─────────────┘
```

## 📊 座標系の関係

```
TFツリー:
map → sirius3/odom → sirius3/base_footprint
 ↑        ↑              ↑
AMCL     EKF         Gazebo/ロボット

初期位置設定:
┌─────────┬───────────────┬──────────────┐
│ 対象    │ 入力           │ 出力          │
├─────────┼───────────────┼──────────────┤
│ AMCL    │ /initialpose  │ map→odom変換  │
├─────────┼───────────────┼──────────────┤
│ EKF     │ set_pose      │ odom座標をリセット│
│         │ (自動転送)     │               │
└─────────┴───────────────┴──────────────┘
```

## ⚙️ 技術詳細

### EKF set_pose サービス
```python
# サービス定義: robot_localization/srv/SetPose
PoseWithCovarianceStamped pose  # 設定する位置と共分散
---
# レスポンスなし（成功/失敗のみ）
```

### 座標変換
- **入力**: `map`座標系の位置（RViz2から）
- **EKF内部**: `odom`座標系に変換して設定
- **効果**: EKFの推定位置がジャンプせずに滑らかにリセット

## 🔧 トラブルシューティング

### エラー: "EKF set_pose サービスがまだ利用できません"
**原因**: EKFノードが起動していない  
**解決**: EKFが完全に起動するまで待機（自動的に再試行）

### エラー: "EKF位置初期化失敗"
**原因**: TF変換が取得できない  
**解決**: 
1. Gazeboシミュレータが起動しているか確認
2. TFツリーを確認: `ros2 run tf2_tools view_frames`

### Initial Poseを設定してもロボットが動かない
**原因**: AMCLが起動していない  
**解決**: Navigation2を起動してください

## 📝 ファイル構成

```
sirius_navigation/
├── sirius_navigation/
│   └── ekf_pose_initializer.py  ← 新規追加
├── launch/
│   └── sensor_fusion.launch.py  ← 更新（ノード追加）
└── setup.py                     ← 更新（エントリーポイント追加）
```

## 🎓 利点

### ❌ 従来の問題
- EKFを再起動すると原点(0,0)にリセット
- AMCLとEKFの位置がズレる
- 手動で調整が必要

### ✅ 新しい方式
- RViz2で簡単に初期位置設定
- AMCLとEKFが同期
- ナビゲーション開始前に正確な位置を設定可能

## 🔗 関連コマンド

```bash
# サービスの確認
ros2 service list | grep set_pose

# サービスの呼び出し（テスト）
ros2 service call /ekf_filter_node/set_pose \
  robot_localization/srv/SetPose \
  "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}}}"

# ノードの確認
ros2 node list | grep ekf

# トピックの確認
ros2 topic echo /initialpose
```

## 📚 参考リンク

- [robot_localization Documentation](http://docs.ros.org/en/ros2_packages/rolling/api/robot_localization/index.html)
- [SetPose Service](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/srv/SetPose.srv)
