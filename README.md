# SIRIUS Jazzy Workspace

このワークスペースは、SIRIUSロボットのROS 2 Jazzy環境用の開発環境です。

## 概要

このワークスペースには、SIRIUSロボットの自律ナビゲーション、センサー統合、および制御に必要なパッケージが含まれています。

## srcフォルダ内のパッケージ

```mermaid
graph TD
    A[sirius_jazzy_ws/src] --> B[SIRIUS固有パッケージ]
    A --> C[サードパーティ]
    A --> D[ハードウェアドライバー]
    
    B --> B1[sirius/]
    B1 --> B1A[sirius_description]
    B1 --> B1B[sirius_interfaces]
    B1 --> B1C[sirius_keyop]
    B1 --> B1D[sirius_navigation]
    
    C --> C1[navigation2]
    C --> C2[slam_toolbox]
    C --> C3[urg_node2]
    C --> C4[velodyne]
    
    D --> D1[roboteq/]
    D1 --> D1A[roboteq_ros2_driver]
    D1 --> D1B[serial]
    D1 --> D1C[udev_rule]
    
    style B fill:#e8f5e8
    style C fill:#e1f5fe
    style D fill:#f3e5f5
```

### SIRIUS固有パッケージ (`sirius/`)
- **sirius_description**: SIRIUSロボットのURDF/Gazeboモデル
- **sirius_interfaces**: カスタムメッセージとアクション定義
- **sirius_keyop**: キーボード操作用のテレオペレーションパッケージ
- **sirius_navigation**: SIRIUS固有のナビゲーション設定とランチファイル

### サードパーティパッケージ
- **navigation2**: ROS 2の公式ナビゲーションスタック
- **slam_toolbox**: SLAMとマッピング機能を提供
- **urg_node2**: Hokuyo URGレーザースキャナードライバー
- **velodyne**: Velodyneライダーセンサードライバー

### ハードウェアドライバー (`roboteq/`)
- **roboteq_ros2_driver**: Roboteqモータードライバー本体
- **serial**: シリアル通信ライブラリ
- **udev_rule**: udevルール設定

## ビルド手順

```bash
# ワークスペースのルートディレクトリで実行
colcon build --symlink-install
```

## 使用方法

```bash
# 環境設定
source install/setup.bash

# 基本的なナビゲーション起動
ros2 launch sirius_navigation navigation.launch.py
```

## 注意事項

- `src`フォルダはgitignoreで除外されているため、パッケージの詳細な更新情報は各パッケージのREADMEを参照してください
- このワークスペースはROS 2 Jazzy環境での使用を前提としています