# bagから3D地図・色付き3D地図を生成する

現在のWebシミュレータ向け手順です。録画・後処理にオンラインのFAST-LIOは必須ではありません。
後処理はbagを直接読み、オンライン推定の `/Odometry` やTFのずれを地図へ引き継ぎません。

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
Fixed frame / Display frameを `offline_map` にして、次のトピックを切り替えて表示してください。

- `/offline_map/colored_points`: 色付き地図。Color modeをRGBに設定。
- `/offline_map/points`: 同じ座標の通常の3D地図。高さによる色分け等も選択可能。

後処理の原点は最初の車体位置です。ライブSLAMとは独立した `offline_map` フレームで配信します。
シミュレータやbag再生を起動する必要はありません。3秒ごとに再配信するので、後から接続しても表示できます。
停止はCtrl+C、Launcherでは停止ボタンです。配信とブリッジは同じROS_DOMAIN_IDで起動してください。

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
