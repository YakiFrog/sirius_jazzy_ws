# 3D地図生成 引き継ぎメモ（2026-10-17）

対象: 実機BAG `slam3d_real_20261016_204610` のオフライン3D地図生成（床二重化・Z軸ブレ問題）で実施した改修と検証の引き継ぎ。
全体方針・仕様の詳細は同ディレクトリの `MAPPING_METHOD.md` が正。本書は「確定した事実・現在の状態・残課題・再現手順」の要約。

## 現在の状態（結論）

- **実機**: `~/rosbag2_data/slam3d_real_20261016_204610/offline_3d_general_v3/`（`VALIDATION.md` 同梱）
  - 再訪床点の高さずれ95%値 **6.7cm**（5cm基準超 → `needs_review` のまま。旧13.6cmから改善）
  - 最終最適化は収束（cost 10.3）、軌跡の異常速度スパイク 0件、点数435,318
  - 共有面補正の候補は床の重なりを 6.7→11.8cm に悪化させたため自動棄却（ガード正常動作）
  - 重力方向の不整合4.65°のため IMU姿勢・重力拘束は未使用（`imu_edges: 0`）
- **シミュ（坂57cm含む全区間）**: `~/rosbag2_data/slam3d_sim_20261016_164430/offline_3d_general_v4_full_validation/`（`VALIDATION.md` 同梱）
  - 位置RMSE 25.5m → **0.63m**、高さRMSE **1.2cm**、床の再訪ずれ 1.5cm
  - 中間区間の弓形（最大1.2m）と速度スパイク2件が残る（残課題2）
- git: 全変更は本メモと同時にコミット・PUSH済み（親: master、サブモジュール FAST_LIO_ROS2: ros2 ブランチ）

## なぜ良くなったか（主要因）

1. **壊れた収録FastLIOを捨て、生データから再計算**（`recompute_fastlio.py`）
   収録時のLIOは約1m四方に潰れていた。生のLiDAR/IMU/車輪から修正版FastLIOを再計算し、初期軌跡・deskew・隣接拘束に使用（実機v3は最適化35評価で収束）。`frontend: auto` は `capture/recording.json` が実機録画を示すBAGで自動的に再計算する。
2. **符号付き点時刻の修正**（C++サブモジュール側）
   Velodyneドライバが最終パケット基準でheaderを打つ場合、点オフセットは負値になる（実機: −49ms〜+1ms）。旧実装は「最後の点>0」でオフセット有無を判定し、負値を丸めていた。`include/point_time.hpp` の `normalize_point_times` がスキャン原点（time_buffer）と点オフセットを同時に移動し、絶対取得時刻を保存する（`preprocess.normalize_point_time`、既定true）。C++単体テスト付き。
3. **車体Z運動拘束＋IMU重力拘束（整合時のみ）**
   「車体に対する上下移動」を車輪・IMUからソフト拘束（`wheel_vertical_sigma`）。坂は許しつつ、幾何で観測できない区間の高さ発散を抑止（シミュの高さ25.5m飛散 → 1.2cm）。IMU重力拘束は `sensor_consistency` が2°以内と確認したときだけ使用。
4. **点群の共有局所平面補正（`mapping_bundle.py`）と自動棄却**
   床・壁の共有面と全姿勢を同時補正。候補が再訪床の重なりを悪化させた場合は診断保存のみで地図に採用しない。
5. **収束とは独立の検証**（`mapping_quality.py` / `sensor_consistency.py`）
   床再訪残差（5cm基準）・軌跡飛び・IMU整合を毎回自動検査し、`validation_status` と完了表示の「要確認」で明示。

## 検証機構（詳細）

- `floor_overlap_quality`: 移動した観測位置の再訪ペアで、床点の高さ差（傾き補正済み）の95%値をペア間95%集計。>5cm で `needs_review`。**評価専用で拘束には使わない**。
- `trajectory_jump_quality`: 連続キーフレーム間速度が物理上限（`--max-trajectory-speed`、既定2.0m/s）またはロバスト統計（中央値2倍・MAD×15σの大きい方、最低0.8m/s）を超えたら検出。
  - 検証実績: 正当な加減速（実測1.25→1.73m/s）を誤検出しない / 破綻ラン（m/s桁の飛び）を10件検出 / 実機0件
- `sensor_consistency`: 最初の静止30秒でIMU quaternion由来の重力方向と加速度を比較。2°超で警告＋重力拘束不使用（実機4.65°）。回転励起の固有値も記録。
- 回帰テスト: `python3 -m pytest other_programs/theta_3d_map/tests/ -q` → 38件（坂を平坦化しない・LIOが床実測を上書きしない・二重床検出等を含む）。FastLIO側はC++ユニットテスト（wheel odometry・点時刻）も成功。

## 既知の残課題（次にやる）

1. **実機IMU取付の校正（4.65°不整合）** — 最重要
   - 原因候補: 取付角度/AHRS基準/加速度バイアス（このBAGの1軸励起では分離不能）
   - 影響: 重力拘束が使えず、絶対的な上下基準が「未検証」。地図の縦精度の上限
   - 手順: ロボットを複数軸で動かす校正録画（静止30秒→傾け・旋回・坂の上り下り等）を1本撮る → 取付回転・バイアス・時刻差を分離して**ロボット設定として固定**（BAGごとに見た目で調整しない）
   - 予備確認: ロボットの平らな面にスマホ水準器を置き 4〜5° の傾きが実在するか確認可（実在すれば校正で定数として吸収可能）
2. **シミュ中間区間の弓形（最大1.2m、KF105-175）**
   - 分析済み: 両端は真値一致（0.0m）。KF170付近の車輪逸脱（真値に対し最大1.7m）と開始↔終了ループ11本の整合を、最適化がチェーン全体へ弾性分配したもの。2000評価でも未収束（optimality 2.95）
   - 次の候補: 車輪逸脱区間の同定と重み低下（motion consistency の拡張）、ループ補正の局所化
3. **実機の床残差5cm未満** — 1の校正確定後が本命。共有面補正の再挑戦は「再訪床を悪化させない」場合のみ採用の条件を維持。

## 再現手順

```bash
source /opt/ros/jazzy/setup.bash
source ~/sirius_jazzy_ws/bash/bash_alias2.sh

# 録画（実機。トピック構成は record_3d_bag.py が定義）
record_3dmap

# 地図生成（実機録画はLIOを自動再計算。既存の再計算BAGを再利用する場合）
offline_3dmap ~/rosbag2_data/<BAG> -o ~/rosbag2_data/<BAG>/offline_3d_general_v4
offline_3dmap ~/rosbag2_data/<BAG> --lio-bag ~/rosbag2_data/<BAG>/recomputed_lio_wheel_on -o ...
#  ※ 今回の実機検証: 後者（recomputed_lio_wheel_on）。シミュBAGはLIOなし（wheel+IMU）

# FastLIO再計算のみ
recompute_3d_lio

# 表示（Foxglove）
view_3dmap

# テスト
OPENBLAS_NUM_THREADS=2 python3 -m pytest other_programs/theta_3d_map/tests/ -q
```

## 主要ファイル

| ファイル | 役割 |
|---|---|
| `offline_3d_mapping.py` | パイプライン本体（8段階）。CLI/レポート/検証統合 |
| `mapping_fusion.py` | 座標系・LIO/車輪/IMU融合（LIOは相対+nullspaceのみ）、visibility filter |
| `mapping_bundle.py` | 共有局所平面と全姿勢の同時補正（候補検証・自動棄却付き） |
| `mapping_quality.py` | 床再訪評価・軌跡飛び検出（評価専用） |
| `sensor_consistency.py` | 静止IMUの重力整合・励起チェック |
| `mapping_geometry.py` / `mapping_progress.py` | 幾何ヘルパ / 進捗表示 |
| `recompute_fastlio.py` + `bash/startup_bash/recompute_3d_lio.sh` | FastLIO再計算 |
| `params/offline_3d_mapping.yaml` | 共通設定（`max_iterations: 2000` 等）。BAGごとに変えない |
| `src/FAST_LIO_ROS2`（サブモジュール） | 修正版FastLIO。`include/point_time.hpp`（符号付き点時刻）等。YakiFrog fork の ros2 ブランチ |
| `MAPPING_METHOD.md` | 共通方針の正（検証基準・禁止事項含む） |

## 運用ルール（必ず維持）

- 元BAG・既存地図を上書きしない。BAG名による分岐・真値の融合・床の水平固定はしない
- 校正値はロボット単位で確定して固定。BAGごとの見た目合わせをしない
- 未収束・要確認は隠さず表示（`validation_status`）。収束は合格基準ではない
- コード変更後は `graphify update .` を実行
- 出力（`~/rosbag2_data/...`）はgit管理外。検証値は各出力の `VALIDATION.md` に記録する
