# シリウス オフライン SAM3 セマンティックマッピング ガイド

ロボット（NUC）上の計算負荷をゼロにし、母艦 PC の NVIDIA GPU を使って最高精度の **「意味情報付き2次元地図（草地・歩道・車道・点字ブロック等のコストマップ）」** を作成する手順です。

---

## 全体フロー

```mermaid
flowchart LR
    subgraph 1. 録画 (Unity / 実機)
        Unity[Unity シミュレータ / 実機] -->|SBSステレオ映像 + TF + LiDAR| Rec[record_rosbag_offline.sh]
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

## 3. 実機ロボット（NUC）での録画手順

実機でも同様に、NUC 側で以下のトピックを Rosbag に録画し、母艦 PC にコピーして `./run_offline_mapping.sh` を実行するだけで全く同じように高品質なセマンティック地図が作成できます：

```bash
ros2 bag record -s mcap -o ~/rosbag2_data/real_robot_01 \
  /tf \
  /tf_static \
  /odom \
  /odom/filtered \
  /scan3 \
  /camera/stereo_sbs/compressed \
  /clock \
  /imu
```
