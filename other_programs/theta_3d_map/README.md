# bagから3D地図・色付き3D地図を生成する

実機・WebシミュレータのBAGに対応します。既定では実機録画のBAGからFastLIO＋車輪を新規に再計算して後処理します。シミュ等では記録LIOがあれば適応融合し、なければ車輪＋IMU＋ICPを使います。録画時のオンラインFAST-LIO起動は必須ではありません。
共通設定は `params/offline_3d_mapping.yaml`。点時刻の復元、TF校正、動的点の可視性除去、参照論文・限界は [MAPPING_METHOD.md](MAPPING_METHOD.md) を参照してください。

## 1. 録画

新しいターミナルを開くか、エイリアスを再読み込みします。

```bash
source ~/sirius_jazzy_ws/bash/bash_alias2.sh
record_3dmap_sim hallway_001
```

Webシミュでrosbridgeへ接続し、3D点群・IMU・車輪オドメトリ・`/clock`を配信してください。
THETAの映像による色付けをONにすると自然色になります。OFFではシミュのリング色が記録されます。
開始時に実メッセージとRGBフィールドを検査します。停止は **Ctrl+Cを1回**。
保存先は `~/rosbag2_data/hallway_001/` です。名前を省略すると日時から自動作成します。
同名のbagは上書きしません。校正設定・録画条件も `capture/` に保存します。

エイリアスを使わない場合:

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/record_rosbag_3dmap.sh sim hallway_001
```

必須入力は `/velodyne_points`（RGB付き）、`/imu`、`/odom`、`/clock`。
任意で `/velodyne_points_colored`、`/tf`、`/tf_static`、`/Odometry`、
`/odometry/filtered`、THETA圧縮画像も記録します。シミュでは比較用の `/ground_truth/odom`、ビルド識別用の `/sirius_web/time_status`、`/sirius_web/sensor_config` も記録します。Webの実際の取り付け位置とプロファイルを `capture/web_sensor_profile.json` と校正設定へ保存します。巨大な `/Laser_map` は記録しません。
`/tf_static` はtransient-localで取得します。

- 時間指定: `record_3dmap_sim hallway_001 --duration 60`（実時間の秒数）
- 保存先変更: `--directory /path/to/bags`
- 校正変更: `--config /path/to/fast_lio_config.yaml`
- 実機の記録: `record_3dmap name`。実機ではRGB付き `/velodyne_points_colored` も必須です。
  実機回転LiDARの後処理の制限は下記を参照してください。

## 2. 後処理

Launcherで `offline_3dmap` を押すと、`~/rosbag2_data` 内のBAGが新しい順に表示されます。
ログの入力欄から番号を送信してください（Enter=1、q=中止）。パスの入力は不要です。
生成済みの場合は `offline_3d_2`、`offline_3d_3`…へ自動保存し、元の地図を残します。

ログには全8工程の現在位置を表示します。読み込み・位置合わせ・地図合成は
件数と進捗バー（その工程の割合）を表示し、約3秒ごとに更新します。
最適化は終了割合を計算できないため、経過時間と残差評価回数を表示します。
最終行の **【完了】** が地図保存の終了です。
最適化が収束前に停止した場合は **【保存完了・要確認】** と表示し、地図と `report.json` の確認を促します。

```bash
offline_3dmap
# BAGを直接指定することも可能
offline_3dmap ~/rosbag2_data/hallway_001
```

直接起動する場合:

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/run_offline_3d_mapping.sh \
    ~/rosbag2_data/hallway_001
```

出力は `<bag>/offline_3d/`:

| ファイル | 内容 |
|---|---|
| `map_3d.ply` | XYZのみの3D地図 |
| `map_3d_colored.ply` | **座標・点数・順番が同一**のRGB付き3D地図 |
| `trajectory.csv` | 後処理後の車体姿勢、時刻・XYZ・四元数 |
| `trajectory_wheel.csv` | 比較用の生の車輪オドメトリ基準軌跡（平面） |
| `trajectory_initial.csv` | 最適化前の初期軌跡。平面車輪モデルでは車輪移動＋IMU姿勢で高さを積算 |
| `trajectory_ground_truth.csv` | 真値があるBAGのみ。最初の対応姿勢のYawと位置を揃えた比較用3D真値 |
| `report.json` | ICPの採否、縮退方向の数・固有値、同期欠落数、最適化結果、`loops`（閉じ込み辺数）と`loop_residuals`（最適化後の閉じ残差） |
| `calibration.yaml` | 使用した外部校正 |

既存結果は上書きしません。再計算は `-o /path/to/new_output` を指定します。
後処理はDDSもbag再生も使わないので、録画PCと切り離して実行できます。

比較用に、LiDAR補正を使わず車輪＋IMUの初期軌跡でも生成できます:

```bash
offline_3dmap ~/rosbag2_data/hallway_001 --mode wheel \
    -o ~/rosbag2_data/hallway_001/wheel_baseline
```

古いbagで `capture/calibration.yaml` がない場合は `--config` が必要です。
処理区間は `--start 10 --end 40`（最初の点群からの秒数）、点間隔は `--voxel 0.05`。

### ループ閉じ込み（既定で有効）

隣接キーフレーム間の拘束に加えて、再訪したキーフレーム同士をICPで検証して閉じ込み拘束を追加します。

1. 1回目の最適化（隣接拘束のみ）
2. 現在姿勢で再訪候補を抽出（距離`--loop-radius`以下・時刻差`--loop-min-gap`秒以上・キーフレーム差`--loop-min-keyframes`以上）
3. 候補を近い順にICP検証し、既存と同じ受理条件（fitness・重なり・退化除去）を通ったものを拘束に追加（1ノード最大1本）
4. ループ拘束込みで再最適化し、`report.json`の`loop_residuals`に閉じ残差を記録

オプション: `--no-loop`（無効化）、`--loop-radius 6`（m）、`--loop-min-gap 10`（秒）、`--loop-min-keyframes 8`。
最適化の反復上限は`--max-iterations`（既定300。以前の80は未収束になりやすかった）。

注意: 退化方向（廊下の進行軸などLiDARで拘束できない方向）は従来どおり拘束から除外するため、
ループ閉じ込みでも**その方向の誤差は残ります**（`loop_residuals`で確認できます）。

## 3. Foxgloveで表示

色が物体から床へにじむ場合の共通校正・再着色は [COLOR_CALIBRATION.md](COLOR_CALIBRATION.md) を参照。
`recolor_3dmap` で既存の形状を保って再着色できる。通常生成も、元画像があるBAGは最後に自動で再着色する。
固定カメラ/LiDAR校正と遮蔽判定はライブ・後処理で共通設定を使う。

Launcherの「3次元地図生成・自律移動」→「3D地図の録画・後処理」に `view_3dmap` があります。
引数なしなら生成済み地図の一覧から番号で選択できます（新しい順、Enter=1、q=中止）。
番号を間違えた場合は入力し直せます。bag・出力ディレクトリ・PLYの直接指定も可能です。
表示候補は `~/rosbag2_data` と `maps_waypoints/maps3d` 以下のPLYです。
後処理用BAGも下位フォルダまで検索します（`metadata.yaml` があるもの）。
候補がない場合も終了せず、Enterで再検索できます。候補一覧ではrで再検索します。
検索先を変える場合は、後処理・表示とも `--directory /path/to/bags` を指定します。

```bash
# 別ターミナルでブリッジ起動（すでに起動中なら不要）
foxglove
# 地図配信
view_3dmap ~/rosbag2_data/hallway_001
```

FoxgloveでFoxglove WebSocket接続先を `ws://<ROS PCのIP>:8766` に設定し、3Dパネルを追加します。

移動軌跡は `/offline_map/trajectory`（`nav_msgs/Path`）を有効にすると地図に重ねて表示できる。`view_3dmap` は選択したPLYと同じディレクトリの `trajectory.csv` を自動で読み込む。CSVは地図座標のロボットbase姿勢なので、追加のLiDAR外部変換や高さ補正をしない。保存済みのXYZ・姿勢・撮影時刻を使う推定軌跡であり、計画経路や実測真値ではない。Zカットは点群にだけ適用し、軌跡の高さや連続性は変えない。軌跡CSVがない古い地図では空のPathを配信して以前の表示を消す。

更新前から起動していた `view_3dmap` はCtrl+Cで停止し、再起動すると軌跡も配信される。軌跡の読み込み・表示は地図やCSVを変更しない。
Fixed frame / Display frameを `offline_map` にして、次のトピックを切り替えて表示してください。

- `/offline_map/colored_points`: 残像除去後の色付き地図（Colored_points）。Color modeをRGBに設定。
- `/offline_map/points`: 同じ座標の通常の3D地図。高さによる色分け等も選択可能。
- `/offline_map/removed_points`: 同じ生成結果の `dynamic_candidates.ply`。削除対象だけを赤色で配信。
  Color mode=RGBで色付き地図に重ねると、どこを消したか比較できる。
  削除対象ファイルがない旧地図では空点群を配信する。動的物体の正解ラベルではない。
- `/offline_map/fragment_points`: 仕上げで追加除去した `dynamic_fragments.ply` だけをオレンジで配信。
  全削除対象（赤）の部分集合。小さな浮遊輪郭の除去・誤除去を区別して確認できる。Zカットも共通適用。

後処理の原点は最初の車体位置です。ライブSLAMとは独立した `offline_map` フレームで配信します。
シミュレータやbag再生を起動する必要はありません。3秒ごとに再配信するので、後から接続しても表示できます。
停止はCtrl+C、Launcherでは停止ボタンです。配信とブリッジは同じROS_DOMAIN_IDで起動してください。

色の統合で遠方の画像の影響を下げる設定は
`src/sirius/sirius_navigation/config/theta_colorization.yaml` の
`color_distance_power`（既定2、増やすほど近距離を優先）と
`fusion_weighted_median: true`。
旋回画像の影響は `color_rotation_scale_deg_s`（既定5度/秒、0で無効）で下げられる。
設定後は `recolor_3dmap` で再着色する。
点の位置や数は変えない。詳細は `COLOR_CALIBRATION.md` の距離信頼度の説明を参照。

### 天井を隠してデバッグする（Zカット）

`view_3dmap` は地図を番号選択した後、Zカットを対話で選択できる。
**Enterは「カットなし」**。上側（天井）・下側（床）・上下の範囲を選び、高さを数値入力する。
qで起動を中止できる。`--z-min` / `--z-max` 指定時はZの対話を省略する。
自動起動で対話を不要にする場合は `--no-z-dialog` を指定する。

```bash
view_3dmap --z-max 2.0  # 地図を番号選択し、高さ2m以下だけ配信
view_3dmap --z-min 0.2 --z-max 1.8  # 床・天井を隠す断面表示
# 起動中に別ターミナルから変更可能（同じROS_DOMAIN_ID）
ros2 param set /offline_3d_map_publisher z_max 1.5
ros2 param set /offline_3d_map_publisher z_max 1000000000.0  # 上限カット解除
```

単位はm、`offline_map` のZ座標（最初の車体位置が原点）。センサからの相対高さではない。
坂や階違いでも同じ座標範囲を適用する。境界は含む。通常・色付き・赤い削除対象に同じ範囲を適用し、
変更直後に再配信する。範囲を広げると点が戻り、PLYや地図生成結果は変更しない。既定は全高表示。
更新前から起動中のViewerは一度再起動すると、このパラメータを利用できる。

### 軌跡を変えずに残像除去だけやり直す

壁に接続した残像など、キーフレームの観測が足りない場合は、
`clean_3dmap /path/to/colored_map --all-scans` で全生スキャンを証拠にできる。
元の高頻度軌跡へ最適化のSE(3)補正を補間し、符号付き点時刻でdeskewする。
最適化軌跡の時刻範囲外は使わず、使用・除外数をレポートに保存する。
元地図と一致検証できる `frames`、初期軌跡、元BAGが必要。
既存の削除を維持し、残った点のXYZと選択した地図のRGBを保持する。
色付け済み地図を渡しても古い未除去地図の色へ戻さない。保存先は別ディレクトリ。
通常の地図生成・`--all-scans` なしの再清掃は従来のキーフレーム方式。

共通設定は著者公開の **DUFOMap 1.1.1** を使用する。自由空間の解像度0.1m、
表面余裕0.2m、姿勢余裕2 voxelを全BAGで共通にする。保存地図の解像度0.05mとは別。
初回は `bash ~/sirius_jazzy_ws/bash/startup_bash/setup_3d_map_cleaner.sh` で専用venvに導入する。
これからの `offline_3dmap` はdeskew済み `frames` も保存する（今回の実機例で約50MB）。

```bash
source ~/sirius_jazzy_ws/bash/bash_alias2.sh
clean_3dmap  # 元地図を番号選択。元地図/frames が必要
# 直接指定。別のキャッシュを使う場合は元地図を復元してXYZ/RGB一致を検査する
clean_3dmap /path/to/generated_map --cache /path/to/same_capture/frames -o /path/to/new_map
```

元地図・軌跡は上書きしない。新しい保存先の `map_3d_colored.ply` と `dynamic_candidates.ply`
は元点群の厳密な分割で、保持点を移動・再着色しない。`dynamic_mask.npz` に分割マスクを保存する。
従来のv7方式は `offline_3dmap ... --dynamic-method visibility` で比較できる。
DUFOMap後の孤立輪郭仕上げは共通設定で有効。小さい・動体近傍・再観測が乏しいという
複合条件を使い、単純な小クラスタ一括削除はしない。独自の推定処理なので静的な孤立部品の
誤除去はあり得る。`clean_3dmap ... --no-fragments` / `offline_3dmap ... --no-dynamic-fragments`
で著者DUFOMapのみと比較できる。時間情報がない場合は追加仕上げを行わない。
手法比較・論文・実測の限界は [DYNAMIC_REMOVAL.md](DYNAMIC_REMOVAL.md) を参照。

## 推定の内容と制限

- 車輪の**車体平面内の相対移動・Yaw**を拘束にする。既定は `--wheel-model planar`。ゼロの高さ・Roll・Pitchは未観測として拘束しない。生の `/odom` を使用し、オンラインLIOの出力は使わない。フル3D姿勢センサを入力する場合だけ `--wheel-model se3` を指定する。
- IMUの有効なorientationから相対回転の拘束を追加する。平面車輪モデルでは車輪の距離をIMU姿勢に沿って積算し、坂の高さを含む初期軌跡を作る。加速度を積分して位置を推定する実装ではない。
- 車輪姿勢を初期値に点対平面ICPを実行。平均ヤコビアンの固有値が小さい方向はLiDAR情報から除く。
  車輪・IMU・残ったLiDAR拘束で全キーフレームの姿勢をまとめて最適化する。
- `/ground_truth/odom` は地図生成後の比較にだけ使用する。`report.json` の `ground_truth_evaluation` に位置・高さ・姿勢のRMSEを保存する。真値のない古いBAGも処理可能。
- 外部校正はFAST-LIO設定のLiDAR←→IMUと、wheel設定のIMU←→車体原点から合成する。
- 時刻が合わない点群は外挿せず除外する。シミュ時刻が逆行する録画は別bagに分ける。
- VLP16の `time`（点群headerからの秒）がある場合は、車輪＋IMUの初期軌跡から点ごとの歪みを補正する（`--no-deskew` で比較可能）。時刻のない旧シミュBAGは瞬間点群として扱う。実機の車輪スリップ補償・IMU加速度の事前積分は未実装で、初期軌跡の誤差は歪み補正にも残る。
  ループ閉じ込みはICPベースで実装済み（外観ベースの検出ではない）。幾何が退化した区間では
  閉じ残差が残るため、その場合は外観情報を使う`run_theta_slam_mapping`等と比較してください。
  長距離の絶対精度や、今回の実走行区間での改善を保証するものではない。
- 色のない古いbagは、通常の3D地図と灰色の色付き形式を出力して警告する。

依存: ROS 2 Jazzy (`rosbag2_py`, MCAPプラグイン, `sensor_msgs_py`)、NumPy、SciPy、PyYAML、Open3D。
この環境では導入済みです。

検証:

```bash
source /opt/ros/jazzy/setup.bash
OMP_NUM_THREADS=2 python3 -m pytest -q \
    ~/sirius_jazzy_ws/other_programs/theta_3d_map/tests/test_offline_3d_mapping.py
```

従来の `theta_bag_to_ply.py`（録画TFで蓄積）と `theta_offline_slam.py`（KISS-ICP）は残しています。
新しい `offline_3dmap` は上記の車輪拘束を使う別の処理です。

## 実際に同一平面と分かっている屋内床の補正

新しくBAGから生成する場合は、ターミナルで指定できる。BAGを省略すると従来どおり番号選択になる。`--known-single-floor` を省略した場合は単一床の拘束なし。

```bash
offline_3dmap --known-single-floor
# BAGを直接指定する場合
offline_3dmap ~/rosbag2_data/<bag名> --known-single-floor
```

通常の軌跡推定後、床補正と検証を行ってから点群の融合・動的除去・色付けを行う。床の観測不足、未使用観測の改善不足、再訪する床や壁の検証不足の場合はエラーで停止する。すべての平坦屋内BAGで成功する保証はない。

既存地図を別ディレクトリに再構築する場合:

```bash
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
  ~/.local/share/sirius/dufomap-venv/bin/python \
  other_programs/theta_3d_map/refine_3d_map.py /path/to/source_map \
  --known-single-floor --output /path/to/new_map
```

元地図を復元できるスキャンキャッシュが必要。床観測から姿勢を補正して点群を再融合し、動的除去・色付けも再計算する。床点を直接平面へ押し付けない。坂・段差・複数階のある環境には単一床の仮定を適用しない。新しい地図座標は参照床をZ=0とするので、旧地図用のウェイポイントは再確認する。評価は `report.json` の `floor_refinement` を参照する。

## 浮遊点・残像の追加仕上げ

```bash
# 現在の地図の位置・色・軌跡を保持して、削除だけを追加する
clean_3dmap /path/to/generated_map --polish
# 新しいBAGからの生成時にも同じ仕上げを選べる
offline_3dmap --dynamic-polish
# 実際に同一平面の屋内床である場合は床補正とも併用できる
offline_3dmap --known-single-floor --dynamic-polish
```

小さな孤立群について、直接観測の後、複数時刻のレーザーがその点の奥へ通った証拠を確認する。大きな構造につながった点は保持する。小群は1点でも継続して再観測されていれば群全体を保護し、細長い孤立部材・平面状の小部品・遮蔽や観測不足の点も保護する。BAG名、地図座標、絶対高さによる除去条件は使わない。独自の保守的な後処理で、全ノイズの除去や静的細部の無欠損を保証するものではない。

`--polish` は選択した地図を元に削除を追加するモードで、`--all-scans` / `--no-fragments` とは併用しない。元地図を復元できる `frames` と `dynamic_mask.npz` が必要。新規生成でもマスクを保存する。通常生成の `--dynamic-polish` は既定で無効、DUFOMapとの併用時のみ有効。詳細は [DYNAMIC_REMOVAL.md](DYNAMIC_REMOVAL.md) を参照。
