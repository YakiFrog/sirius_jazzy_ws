# Nav2 Behavior Tree Log Viewer

Nav2の動作ツリー(Behavior Tree)のログを見やすく表示するツール

## 概要

`/behavior_tree_log`トピックをサブスクライブし、Nav2の内部処理状態をリアルタイムで視覚的に表示します。

## ファイル

- `bt_log_viewer_gui.py` - **GUI版** (PySide6使用、メインタスク強調表示)
- `bt_log_viewer.py` - 詳細版(状態サマリー付き)
- `bt_log_viewer_simple.py` - シンプル版(変化のみ表示)

## 使い方

### GUI版 (推奨) 🌟

```bash
cd /home/kotantu-nuc/sirius_jazzy_ws/view_nav2_tasks
python3 bt_log_viewer_gui.py
```

**特徴:**
- メインタスクを大きく表示
- カラフルな状態表示
- アクティブノードの一覧表示
- イベントログのスクロール表示

### コンソール版 - 詳細版

```bash
python3 bt_log_viewer.py
```

### コンソール版 - シンプル版

```bash
python3 bt_log_viewer_simple.py
```

## 表示内容

### メインタスク（重要なノード）

以下のノードはメインタスクとして特別に強調表示されます:

- `NavigateRecovery` - リカバリーナビゲーション
- `NavigateWithReplanning` - 再計画ナビゲーション
- `NavigateToPose` - ゴールへ移動
- `ComputePathToPose` - 経路計算
- `FollowPath` - 経路追従
- `Spin` - 回転動作
- `BackUp` - 後退動作
- `Wait` - 待機動作

### ステータス

- ⚪ `待機中` (IDLE) - アイドル状態
- 🔵 `実行中` (RUNNING) - 実行中
- ✅ `成功` (SUCCESS) - 成功
- ❌ `失敗` (FAILURE) - 失敗

### その他のノード

- `RateController` - レート制御
- `PlannerSelector` - プランナー選択
- `ControllerSelector` - コントローラー選択
- `GoalChecker` - ゴール判定

## 必要なパッケージ

### GUI版
```bash
pip install PySide6
```

### コンソール版
- `rclpy`
- `nav2_msgs`

## 参考

- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [BT.CPP](https://www.behaviortree.dev/)
