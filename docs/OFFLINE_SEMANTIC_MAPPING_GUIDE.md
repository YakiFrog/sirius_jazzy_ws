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
- ファイル名を入力（例: `sim_test_01`）
- キーボード操作（`keyboard_teleop_ja` 等）でシリウスロボットを動かし、地図を作成したいエリアを隈なく走行します。
- 走行が終わったら **Ctrl + C** を押して録画を停止・保存します。
- データは `~/rosbag2_data/<ファイル名>` に保存されます。

---

### ステップ 3: オフライン SAM3 セマンティックマッピングの実行
端末 3 でオフラインマッピングスクリプトを実行します：
```bash
cd ~/sirius_jazzy_ws
./bash/startup_bash/run_offline_mapping.sh
```
1. 一覧から先ほど録画した Rosbag の番号を選択します。
2. 再生速度を選択します（推奨: `0.5`）。
3. 認識させたい物体/路面のプロンプトを入力します（デフォルト: `grass, tactile paving, roadway, sidewalk`）。
4. SAM3 GPU サーバー、RTAB-Map、2D カラーインデックスノードが立ち上がり、Rosbag の再生に合わせてセマンティック地図が構築されます。再生時はSlamToolboxを起動せず、bag内の補正済みTFを使います。
5. 再生完了後、保存確認で **`y`** を入力すると、`maps_waypoints/` に 2D カラー地図（`.png`, `.yaml`, `.db`）が保存されます。

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
