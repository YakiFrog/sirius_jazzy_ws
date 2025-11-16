# Sirius シミュレーション UI付き起動

## 概要

このディレクトリには、PySide6を使用したGUIから設定を選択してシミュレーションを起動できるlaunchファイルが含まれています。

## ファイル構成

- `sim_with_ui.launch.py` - メインのlaunchファイル（簡潔版）
- `launch_config_ui.py` - UI部分を分離したモジュール

## 使用方法

```bash
# ワークスペースをsource
source /home/kotantu-nuc/sirius_jazzy_ws/install/setup.bash

# UIつきlaunchファイルを起動
ros2 launch sirius_description sim_with_ui.launch.py
```

## UI機能

### ワールド選択
- `worlds` ディレクトリ内のSDFファイルから起動するワールドを選択できます

### コンポーネント選択

**ロボット:**
- ロボットをスポーン
- Robot State Publisher

**ROS-Gazebo ブリッジ:**
- TF Bridge
- Odometry Bridge
- Twist Bridge (cmd_vel)
- Joint State Bridge
- Clock Bridge

**センサー:**
- LiDAR Bridge (/scan)
- LiDAR2 Bridge (/hokuyo_scan)
- IMU Bridge
- Velodyne Bridge (3D LiDAR)

**ツール:**
- Teleopキーボードコントロール
- RViz2

## 依存関係

```bash
# PySide6のインストール（既にインストール済み）
pip3 install PySide6 --break-system-packages

# Qt xcbライブラリ（既にインストール済み）
sudo apt install libxcb-cursor0
```

## カスタマイズ

UIの見た目や機能を変更したい場合は、`launch_config_ui.py` を編集してください。
launchの起動ロジックを変更したい場合は、`sim_with_ui.launch.py` を編集してください。

## トラブルシューティング

### PySide6がインポートできない
```bash
pip3 install PySide6 --break-system-packages
```

### Qtプラットフォームプラグインエラー
```bash
sudo apt install libxcb-cursor0
```
