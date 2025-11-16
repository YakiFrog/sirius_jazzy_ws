# Sirius Assisted Teleop - クイックスタートガイド

## 概要

Assisted Teleopは、Nav2の障害物回避機能を使って、安全な手動操縦を実現します。
キーボードで操縦すると、自動的に障害物を避けるように速度が調整されます。

## 3ステップで起動

### ステップ1: シミュレーション起動

```bash
ros2 launch sirius_description sim.launch.py
```

### ステップ2: Nav2起動

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### ステップ3: Assisted Teleop起動

```bash
ros2 launch sirius_navigation assisted_teleop.launch.py
```

これで、別ウィンドウが開いてキーボード操縦ができます！

## 操作方法

キーボードの以下のキーで操縦:

```
   u    i    o     前進方向
   j    k    l     回転/停止
   m    ,    .     後退方向
```

- **i**: 前進
- **,**: 後退  
- **j**: 左回転
- **l**: 右回転
- **k**: 停止
- **u/o**: 前進しながら回転
- **m/.**: 後退しながら回転

速度調整:
- **q/z**: 前進速度を上げる/下げる
- **w/x**: 回転速度を上げる/下げる
- **Space**: 緊急停止

## Assisted Teleopの効果

通常のTeleopとの違い:

| 通常のTeleop | Assisted Teleop |
|-------------|-----------------|
| 壁に衝突する | 壁の前で自動的に停止 |
| 障害物を避けられない | 障害物を検出して速度調整 |
| 狭い場所で困難 | 安全に通過をサポート |

## トピックの確認

別のターミナルで動作確認:

```bash
# 速度コマンドを確認
ros2 topic echo /cmd_vel

# Assisted Teleopの状態を確認
ros2 action list

# コストマップを確認
ros2 topic echo /local_costmap/costmap
```

## RVizで確認

RVizで以下を表示して動作を確認:

1. **Map**: 環境地図
2. **Local Costmap**: 障害物マップ（青=安全、赤=危険）
3. **Robot Footprint**: ロボットの占有範囲
4. **Laser Scan**: LiDARデータ

Assisted Teleopが動作すると、コストマップの赤い部分（障害物）に
近づくと自動的に速度が落ちるのが分かります。

## トラブルシューティング

### Q: "action server not available" エラー

A: Nav2が起動していません。ステップ2を実行してください。

### Q: ロボットが動かない

A: 以下を確認:
1. シミュレーションが起動しているか
2. `/cmd_vel` トピックがパブリッシュされているか
3. Teleopキーボードウィンドウがアクティブか

### Q: 障害物を避けない

A: コストマップの設定を確認:
1. LiDARデータが正しくパブリッシュされているか
2. コストマップが更新されているか
3. パラメータファイルの設定を確認

## より詳しい情報

完全なドキュメントは `ASSISTED_TELEOP_README.md` を参照してください。
