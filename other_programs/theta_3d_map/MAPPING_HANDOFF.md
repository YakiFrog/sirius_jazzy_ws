# 3D地図生成 引き継ぎメモ（2026-10-17）

対象: 実機BAG `slam3d_real_20261016_204610` のオフライン3D地図生成（床二重化・Z軸ブレ問題）で実施した改修と検証の引き継ぎ。
2026-10-17のユーザー確認: THETA–LiDAR取り付けTF（真上28.5cm・回転0）は概算で未計測。ソフトの色付け改善と、実機外部校正の完了を混同しない。
全体方針・仕様の詳細は同ディレクトリの `MAPPING_METHOD.md` が正。本書は「確定した事実・現在の状態・残課題・再現手順」の要約。

## 現在の状態（結論）

- **旋回信頼度付き再着色v13（2026-10-17）**: `offline_3d_general_v13_rotation_confidence/`。`color_rotation_scale_deg_s: 5.0` により `1/(1+(回転速度/5)^2)` を色信頼度へ乗算。−0.3秒候補の適用や外部校正変更はしていない。618画像のうち156枚が半分未満の重み、運動姿勢欠落0枚。全416,417 XYZ・軌跡・削除マスク等はv12と完全一致、色付き338,650点 / 未確定77,767点。指定エレベータの代表点RGB=(184,188,189)→(1,1,1)。XY=(17.76,8.22)半径15cm・z=0.5〜1.8mの291点では白判定（RGB平均>150）が165→0点。周辺白壁も保持、全体の色正解率は未測定。72テスト成功、全範囲＋指定部位の比較図を目視確認。検査は `rotation_validation.json`、比較は `rotation_confidence_comparison.png`。
- **エレベータXY=(17.76,8.22)の追跡（2026-10-17）**: `elevator_color_trace/`。v12の全RGB再現一致を確認し、白い代表点を着色した上位5枚を特定。開始約175.6〜178.2秒・距離2.70〜2.93m・旋回17.2〜18.2度/秒の画像から白い壁の色を採用。時刻−0.1〜−0.4秒の投影試行では全5枚が暗い画素へ移る。遅延値の確定や校正適用は未実施。停止・逆旋回で外部角度差と時刻差を分離する必要がある。以前の比較図はこの座標が範囲外だったことも `COLOR_CALIBRATION.md` に明記。66テスト成功。
- **距離信頼度付き再着色v12（2026-10-17）**: `offline_3d_general_v12_color_confidence/`。既存の距離逆二乗信頼度は候補選択だけに使われ最終融合が単純中央値だったため、共通設定 `fusion_weighted_median: true` で最終RGBにも反映。`color_distance_power`（既定2、有効0〜8）を追加。618画像から再着色、v11の416,417 XYZ・通常PLY・軌跡・削除マスク・赤橙の対象PLYは完全保持。色付き338,650点 / 未確定77,767点、RGB差45,061点。66テスト成功、比較図を目視確認。投影校正値は不変で、エレベータ境界の改善率は未測定。詳細は `COLOR_CALIBRATION.md`、出力の `confidence_validation.json`。
- **全生スキャン残像確認v11（2026-10-17）**: `offline_3d_general_v11_all_scans/`。`clean_3dmap --all-scans` を追加し、最適化補正を高頻度軌跡へ反映。7,687生スキャン中7,668をdeskewしてDUFOMapへ、範囲外19は除外。v10から1,094点を追加削除、保持416,417点 / 総削除18,986点。旧削除を維持、保持XYZ/RGBと軌跡は完全一致。床診断帯のv10の18,316点は追加削除なし。正解ラベルなし、全残像除去・全静的点保持とはしない。49テスト成功。通常生成は引き続きキーフレーム方式。
- **横方向の色境界診断（2026-10-17）**: 黒いエレベータ壁への白色の混入は境界沿いの細い帯とのユーザー確認。`theta_alignment_diagnostics.py --map MAP` でdeskew・高頻度姿勢を使ったyaw/時刻差の9場面診断を追加。`theta_horizontal_check_wide/` の集計候補はyaw +1.33度 / 時刻差−0.30秒だが場面間で揃わず、適合用画像での評価に過ぎない。校正・RGBは変更せず。詳細は `COLOR_CALIBRATION.md`。
- **色付けv10（2026-10-17）**: `offline_3d_general_v10_color/`。v9の417,511点と形状・軌跡・残像マスクを完全保持し、618画像で337,873点を再着色（79,638点は未確定の灰色）。固定外部変換・遮蔽・撮影姿勢・複数画像中央値をライブ/後処理の共通方針で扱う。未使用7枚の画像との条件付き色差中央値24.33→10.67/255、絶対校正精度ではない。公称TFの実測確認は未完了で、Z/PitchをBAGごとに合わせる補正はしていない。今後の通常生成は元画像があれば自動再着色、既存地図は `recolor_3dmap` で番号選択。共通設定は `src/sirius/sirius_navigation/config/theta_colorization.yaml`。詳細・制限は `COLOR_CALIBRATION.md`。
- **輪郭仕上げv9（2026-10-17）**: `offline_3d_general_v9_fragments/`。v8から287群・1,167点を追加除去し、保持417,511点 / 総削除17,892点。小さい孤立群・動体近傍・弱い再観測の複合条件による独自の後処理で、DUFOMap本体の追加機能とは区別する。47テスト成功、実物v9の4トピックを隔離ROSドメインで受信確認。軌跡CSV・保持XYZ/RGBは不変。`/offline_map/fragment_points` は追加分だけオレンジ、赤い削除対象は合計。床付近の診断帯では18,317点中1点が追加対象で、床全点保持とはしない。点ごとの意味ラベルなしのため除去率・誤除去率は未測定。v8を保存、`--no-dynamic-fragments` / 再清掃の `--no-fragments` で追加処理を無効化できる。詳細は `DYNAMIC_REMOVAL.md`。
- **残像除去v8（2026-10-17）**: `offline_3d_general_v8_dufomap/`。著者公開DUFOMap 1.1.1で同じ435,403点を分類、保持418,678点 / 削除対象16,725点。v7軌跡をそのままコピー、元地図の保持XYZ/RGBは変更しない。v6キャッシュはv7元地図全体の復元一致を検査して使用。正解ラベルなしのため除去率・保持率は未測定。45テスト成功（ネイティブDUFOMap、ROS遅延購読・空点群含む）。`view_3dmap` は保持色付きと赤い削除対象 `/offline_map/removed_points` を分離配信。共通YAMLはDUFOMap、従来v7投票も選択可能。詳細・文献・再処理は `DYNAMIC_REMOVAL.md` / `README.md`。
- **実機**: `~/rosbag2_data/slam3d_real_20261016_204610/offline_3d_general_v3/`（`VALIDATION.md` 同梱）
  - 再訪床点の高さずれ95%値 **6.7cm**（5cm基準超 → `needs_review` のまま。旧13.6cmから改善）
  - 最終最適化は収束（cost 10.3）、軌跡の異常速度スパイク 0件、点数435,318
  - 共有面補正の候補は床の重なりを 6.7→11.8cm に悪化させたため自動棄却（ガード正常動作）
  - 重力方向の不整合4.65°のため IMU姿勢・重力拘束は未使用（`imu_edges: 0`）
- **シミュ（坂57cm含む全区間）**: `~/rosbag2_data/slam3d_sim_20261016_164430/offline_3d_general_v4_full_validation/`（`VALIDATION.md` 同梱）
  - 位置RMSE 25.5m → **0.63m**、高さRMSE **1.2cm**、床の再訪ずれ 1.5cm
  - 中間区間の弓形（最大1.2m）と速度スパイク2件が残る（残課題2）
- **動的点（人物残像）除去の改善**: `~/rosbag2_data/slam3d_real_20261016_204610/offline_3d_general_v7/`（`VALIDATION.md` 同梱。v4〜v7の履歴は各ディレクトリ）
  - v4: 自由空間票の「観測時刻±8秒」制限を撤廃（除去110→3,387点）→ v5: 立ち去り型（変化点）ルール追加 → v7: 閾値を票≥2・比率2・クラスタ≤2へ強化（除去7,315点）。幾何・床・ループは全版で同一
  - 世界の手法整理: 可視性/自由空間系（DUFOMap・Dynablox・R2R/Removert・ERASOR/ERASOR2・FreeDOM・BeautyMap）、学習MOS系（LMNet/4DMOS/InsMOS等）、時間分布系（DynPurge・HMM-MOS）。評価は DynamicMap_Benchmark（KTH-RPL）、DOALS（歩行者が多い）
  - 残る限界: その空間が一度も「空」として観測されなかった残像は幾何では原理的に除去不能（意味情報か別日データが必要）
- git: 過去の基本改修はコミット済みだが、本書のv8/v9・Viewer関連の変更は未コミット・未PUSH。現在の `git status` を確認すること。

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
| `mapping_dynamic.py` / `clean_3d_map.py` | 著者DUFOMap呼出し / 軌跡固定・元地図照合付き再清掃 |
| `mapping_bundle.py` | 共有局所平面と全姿勢の同時補正（候補検証・自動棄却付き） |
| `mapping_quality.py` | 床再訪評価・軌跡飛び検出（評価専用） |
| `sensor_consistency.py` | 静止IMUの重力整合・励起チェック |
| `mapping_geometry.py` / `mapping_progress.py` | 幾何ヘルパ / 進捗表示 |
| `recompute_fastlio.py` + `bash/startup_bash/recompute_3d_lio.sh` | FastLIO再計算 |
| `params/offline_3d_mapping.yaml` | 共通設定（`max_iterations: 2000` 等）。BAGごとに変えない |
| `src/FAST_LIO_ROS2`（サブモジュール） | 修正版FastLIO。`include/point_time.hpp`（符号付き点時刻）等。YakiFrog fork の ros2 ブランチ |
| `MAPPING_METHOD.md` | 共通方針の正（検証基準・禁止事項含む） |

## 運用ルール（必ず維持）

- 元BAG・既存地図を上書きしない。BAG名による分岐・真値の融合・通常生成での床の水平固定はしない
- 校正値はロボット単位で確定して固定。BAGごとの見た目合わせをしない
- 未収束・要確認は隠さず表示（`validation_status`）。収束は合格基準ではない
- コード変更後は `graphify update .` を実行
- 出力（`~/rosbag2_data/...`）はgit管理外。検証値は各出力の `VALIDATION.md` に記録する

## 明示的な単一床補正（2026-10-18）

ユーザーが実環境を平坦な屋内床と確認し、その条件での補正を依頼したため、`mapping_floor.py` / `refine_3d_map.py --known-single-floor` を追加。既定の地図生成とは独立した選択モード。床面から高さ・傾きの姿勢を補正し、点群を再融合、DUFOMapと色投影を再実行する。点の床面投影はしない。参照床をZ=0とする地図座標へ変わるため、旧ウェイポイントの流用に注意。

今回の出力は `~/rosbag2_data/slam3d_real_20261016_204610/offline_3d_general_v14_floor_refined`。未使用52スキャンの参照床面距離p95は26.48→4.49cm、床再訪p95は6.67→5.65cm、壁再訪p95は6.54→6.51cm。絶対精度の実測ではなく、床再訪が5cmを超えるため `needs_review` を維持。原データやv13を上書きしない。

`offline_3dmap --known-single-floor` も接続済み。BAG未指定なら従来どおり番号選択する。元の軌跡推定後・点群融合前に、別コマンドと共通の `refine_known_floor` で補正と採否検査を行う。初期軌跡と車輪軌跡も新しい地図座標へ変換する。フラグ省略時は無効、共通YAMLからの常時有効化は不可。床・再訪壁の根拠不足はエラーで停止する。既存BAGからの入力・出力の接続と拒否動作を含め、関連テスト78件を確認。

## 任意の追加仕上げ（2026-10-19）

`mapping_cleanup.py`：小孤立群と動体近傍の観測を集計し、直接観測より後の複数時刻・複数レーザーの通過証拠を使う。ユーザーの意味のある点の保持指定に従い、削除は小さな孤立群だけに制限。大きな構造と接続部、遮蔽・再観測・細長い静的部材を保護し、座標・高さ・BAG名の条件は使わない。小群の1点でも継続して再観測されれば群全体を保護。`clean_3dmap --polish` で既存の保持点に削除だけを追加。`offline_3dmap --dynamic-polish` で新規生成にも使用可（DUFOMapのみ、既定無効）。新規生成でも `dynamic_mask.npz` を保存する。追加仕上げ点は従来のfragment点と合わせてオレンジ表示する。

実機出力：`offline_3d_general_v15_polished`。最初の試行の167点から構造と接続した90点と平面状の小部品候補13点を戻し、最終追加削除64点、保持419,201点。平面状の小群も保護する。XYZ/RGBは元地図の厳密な部分集合、軌跡はバイト一致。合成試験・別シミュBAGの共通設定試行を含め、関連テスト88件成功。詳細と限界は出力の `VALIDATION.md`、`polish_validation.json`、`DYNAMIC_REMOVAL.md` を参照。v14配信を自動で停止・切替していない。

## 保存済み移動軌跡の表示（2026-10-19）

`view_3dmap` が `/offline_map/trajectory`（nav_msgs/Path）を追加配信する。`theta_ply_publisher` の `trajectory_topic` を指定したとき、PLYと同じディレクトリの `trajectory.csv` を読む。CSVはmap座標のbase姿勢であり、外部変換を再適用しない。保存XYZ・姿勢・ナノ秒時刻を保持。Pathヘッダは点群と同時刻、各PoseStampedは録画時刻。Zカットは軌跡に適用しない。CSVなしは空Pathを配信。既存の配信プロセスは自動再起動していない。

v15では263姿勢・約386.65秒、キーフレーム折れ線長約64.05m。v15出力の `map_with_trajectory.png` と `trajectory_validation.json` を参照。後から接続したsubscriberでPathと地図を受信し、XYZ・姿勢・時刻・地図の非変更を検証。パッケージをsymlink-installでビルドし、関連テスト95件成功。
