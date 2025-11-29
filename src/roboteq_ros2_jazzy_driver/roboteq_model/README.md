Roboteq モデル（Python）
======================

この小さな Python モジュールは、C++ で書かれた Roboteq ROS2 ノードの主要な入出力計算を再現します。

- twist（直線速度、回転速度）から左右の車輪速度（m/s）への変換
- 最小コマンドしきい値の処理
- 最大速度へのクランプ（元の C++ 実装の挙動に合わせています）
- オープンループ（パワー）とクローズドループ（RPM）への変換
- エンコーダストリーム行 `CB=<right>:<left>` の解析
- エンコーダ値からのオドメトリ更新計算

ファイル:
- `roboteq_model.py` — モデル本体およびデモスクリプト
- `test_roboteq_model.py` — コアの振る舞いを検証する pytest 単体テスト

使い方（デモとテスト）:

スクリプトを直接実行してデモを確認できます:

```bash
python3 src/roboteq_ros2_jazzy_driver/roboteq_model/roboteq_model.py
```

テストを実行する（推奨）:

```bash
pytest -q src/roboteq_ros2_jazzy_driver/roboteq_model
```

インタラクティブな cmd_vel CLI
-----------------------
簡単な REPL を起動して、cmd_vel を入力するたびに計算結果（車輪の速度やコマンド値）を確認できます:

```bash
python3 src/roboteq_ros2_jazzy_driver/roboteq_model/cli.py
```

入力の例:
- `0.5 0.0` — 前進 0.5 m/s
- `0.0 1.0` — 回転

フラグ:
- `--closed-loop` を付けるとオープンループのパワーではなく RPM を計算します

起動時に CLI は現在のパラメータ値（wheel_circumference, track_width, pulse, gear_ratio, max_rpm, max_speed, odom_scale）と動作モード（open-loop/closed-loop）を表示します。


補足
-----
- このモジュールはあえて ROS2 依存を排しており、ドライバ内部の計算ロジックを簡単に検証できるようにしています。
- Python 実装は C++ のロジック（例: 正の速度のみクランプする挙動など）にできるだけ忠実に従っています。数値や計算過程の検証にお使いください。
