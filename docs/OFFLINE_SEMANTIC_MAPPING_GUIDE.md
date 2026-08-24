# シリウス オフライン SAM3 セマンティックマッピング ガイド

ロボット（NUC）側では軽量なセンサー収録だけを行い、母艦 PC の NVIDIA GPU を使って高精度の **「意味情報付き2次元地図（草地・歩道・車道・点字ブロック等のコストマップ）」** を作成する手順です。

---

## 全体フロー

```mermaid
flowchart LR
    subgraph 1. 録画 (Unity / 実機)
        Unity[Unity シミュレータ / 実機] -->|SBSステレオ映像 + TF + LiDAR| Rec[record_rosbag_offline_sim / real]
        Rec --> Bag[(Rosbag2 MCAP)]
    end

    subgraph 2. オフラインマッピング (母艦GPU)
        Bag --> Play[run_offline_mapping.sh]
        Play --> Docker[SAM3 GPU 推論\n:8080]
        Docker --> RTAB[RTAB-Map + IndexedMapNode]
        RTAB --> Map[2Dカラー地図\n/sam3/colored_map_grid]
    end
```

---

## 1. シミュレーション（Unity）での実行手順

### ステップ 1: シミュレータ環境の起動
端末 1 でシミュレーションと ROS-TCP を起動します：
```bash
cd ~/sirius_jazzy_ws
./bash/startup_bash/launch_simulation.sh
```
（Unity エディタ側で Play ボタンを押してシミュレーションを開始します）

---

### ステップ 2: 走行データの録画
録画前に `rte` → `rviz2sim` → `sf_sim` → `slamtoolbox` の順で起動します。
SLAM Toolboxは録画スクリプから自動起動されません。ここで生成した
補正済みTFが `/tf` としてbagに保存されます。

端末 2 でオフライン録画スクリプトを実行します：
```bash
cd ~/sirius_jazzy_ws
./bash/startup_bash/record_rosbag_offline.sh
```
- SAM3 Dockerが8080番ポートを使用中なら、録画中だけ自動停止し、録画終了時に元の起動状態へ戻します。
- 録画開始前に、ステレオ画像、カメラパラメータ、`/scan3`、`/odom/filtered`、`/clock`、補正済みTF、カメラ取付TFを実メッセージで検査します。不足時はbagを作らず停止します。
- ファイル名を入力（例: `sim_test_01`）
- キーボード操作（`keyboard_teleop_ja` 等）でシリウスロボットを動かし、地図を作成したいエリアを隈なく走行します。
- 録画中は10秒ごとに経過時間とbagサイズを表示します。走行が終わったら **Ctrl + Cを1回だけ** 押して録画を停止・保存します。
- `Ctrl+C`後は、Rosbag終了、MCAP索引、メタデータ、必須トピック検証の現在段階と経過秒数を表示します。
- Launcherの「停止」ボタンでも、rosbagへ先にSIGINTを送り、MCAP索引・検証の完了を待ってから終了します。「Rosbagの保存と検証が完了しました」と表示されるまでPCを終了しないでください。
- 保存直後にもbag内の件数とTF構成を再検査し、欠損があればエラーを表示します。
- データは `~/rosbag2_data/<ファイル名>` に保存されます。

保存済みbagだけを再検査する場合：

```bash
python3 ~/sirius_jazzy_ws/bash/startup_bash/validate_offline_mapping_bag.py \
  ~/rosbag2_data/<ファイル名> --require-clock
```

---

### ステップ 3: オフライン SAM3 セマンティックマッピングの実行
端末 3 でオフラインマッピングスクリプトを実行します：
```bash
cd ~/sirius_jazzy_ws
./bash/startup_bash/run_offline_mapping.sh
```
1. 一覧から先ほど録画した Rosbag の番号を選択します。必須画像または補正TFがないbagは自動的に拒否されます。
2. 再生速度を選択します（推奨: `0.5`）。
3. 認識させたい物体/路面のプロンプトを入力します（デフォルト: `grass, tactile paving, roadway, sidewalk`）。
4. RVizを選んだ場合は一時停止状態から開始できます。端末で`Space`を押すと開始/一時停止、`→`で1メッセージ進み、`↑`/`↓`で速度を変更できます。
5. SAM3 GPU サーバー、RTAB-Map、2D カラーインデックスノードが立ち上がり、Rosbag の再生に合わせてセマンティック地図が構築されます。再生時はSlamToolboxを起動せず、bag内の補正済みTFを使います。
6. 再生完了後、保存確認で **`y`** を入力すると、bag名から`semantic_<bag名>`を自動生成して保存します。同名があれば`_02`、`_03`を付け、既存地図を上書きしません。

---

## 2. RViz2 での可視化確認トピック

| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/sam3/colored_map_grid` | `nav_msgs/OccupancyGrid` | 意味情報（草地・歩道・車道・点字ブロック等）がカラー/コスト化された 2D 地図 |
| `/sam3/colored_map_cloud` | `sensor_msgs/PointCloud2` | 2D 地図を RViz2 用に 3D カラー点群として立体表示したもの |
| `/cloud_map` | `sensor_msgs/PointCloud2` | RTAB-Map が累積したフル 3D カラー点群地図 |
| `/rtabmap/grid_map` | `nav_msgs/OccupancyGrid` | RTAB-Map の幾何構造 2D 占有格子地図 |

---

## 3. 実機ロボット（NVIDIAなしNUC）での録画手順

実機PCでSAM3推論やZED SDKは動かしません。CUDA不要の
[ZED Open Capture](https://github.com/stereolabs/zed-open-capture)でUSBカメラから左右画像を取得し、
工場キャリブレーションで補正したJPEGだけを録画します。

対応カメラはUSB接続のZED、ZED Mini、ZED 2、ZED 2iです。ZED X系は対応しません。

### 初回セットアップ

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/setup_offline_real.sh
```

初回ビルドとカメラの工場キャリブレーション取得時にはインターネット接続が必要です。
取得後は `~/zed/settings/SN<シリアル>.conf` が使われるためオフラインで起動できます。

### 録画手順

Sirius Launcherの「実機オフライン録画準備」は必要ノードだけを起動し、録画は開始しません。

1. 実機センサー、`rviz2real`、`sf_real`、`slamtoolbox_real`、`zed_offline_recorder` を起動します。
2. `check_offline_real` で画像、`/scan3`、`map → base_footprint`、ZED取付TFを確認します。
3. 全項目が成功したら `record_offline_real` を起動します。
4. 走行終了後に `Ctrl+C` で安全にMCAPを保存します。

実機用録画スクリプトは他ノードを自動起動しません。必須トピックや補正TFが不足する場合は、録画を開始せずエラーにします。

保存した `~/rosbag2_data/<実験名>` をGPU搭載PCにコピーし、
`run_offline_mapping` で再生します。再生時のSLAM Toolboxは不要です。

MCAP録画が電源断や強制終了で未完了になりRosbagを開けない場合、
`run_offline_mapping` は自動的に `mcap recover` と再インデックスを実行します。
元データは上書きせず、同じディレクトリ階層に `_recovered` 付きのRosbagを
作成して必須トピックとTFを再検証してから再生します。復旧できても必要な
データが欠けている場合は、安全のため再生を開始しません。

### オンライン実験とのSAM3設定一致

`run_offline_mapping`は開始前に、オンライン実験と同じSAM3設定をサーバーへ
明示的に適用します。

設定を確認・変更する場合はSirius Launcherの`sam3_settings_ui`を起動します。
ブラウザの専用画面（`http://localhost:8080/`）からプロンプト、Confidence、
SAM3解像度、深度方式、深度解像度、最大距離などを変更できます。

| 設定 | オンライン | オフライン |
| :--- | :---: | :---: |
| SAM3モデル／チェックポイント | `sam3.pt` | 同一 |
| SAM3入力解像度 | 512×512 | 512×512 |
| Confidence | 0.5 | 0.5 |
| 色モード | semantic | semantic |
| ROSブリッジ点群間引き | 2 | 2 |
| 背景点群 | 有効 | 有効 |

画像入力元はオンラインのカメラに対してオフラインはrosbag、深度入力は実機PCで
ZED SDKを使えないためオフラインではFastStereoになります。この2点以外のSAM3
認識条件は共通です。
