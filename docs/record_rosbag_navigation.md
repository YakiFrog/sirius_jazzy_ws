# 自律移動の記録：record_rosbag

`record_mapping` は後処理で地図を作り直すための生LiDAR・画像を含む記録です。
`record_rosbag` は走行中の移動量推定、TF、制御、地図・経路・コストマップの検証用です。

- FAST-LIO：`/Odometry`（大文字O）、`/fastlio/diagnostics`、`/diagnostics`。
- 車輪・IMU：`/odom`、`/odom/filtered`、`/odometry/filtered`、`/roboteq/odom`、`/imu`。
- tf_real：`/tf`、`/tf_static`、`/robot_description`、`/joint_states`。
- SAM3：静的カラー点群に加え、`/sam3/static_colored_map_grid` と `/sam3/static_semantic_cost_grid`。
- Nav2：通常・rawのglobal/localコストマップ、差分更新、footprint、地図・経路。
- 制御：Nav2・Web・テレオペ・twist_mux周辺の速度指令、停止・車輪状態。

記録前に対象トピックと現在検出できるものを表示します。
グラフで検出されることはメッセージの受信確認とは別です。
未検出の任意トピックも、記録中に現れれば購読します。
静的な地図・TF・URDF・コストマップには transient_local の受信設定を指定し、
配信元を起動した後に記録を開始しても保持中のデータを受信します。

BAGと同じ場所に `<bag名>.capture/start` と `finish` ができます。
稼働中のSAM3ローダーの `map_path`、map_serverの `yaml_filename` から
実際に使用しているPGM＋JSON、地図YAMLと参照画像をコピーします。
FAST-LIO、ローダー、map_server、robot_state_publisherの稼働パラメーターと、
元の保存先・コピー先・SHA256も残します。元ファイルは変更しません。
開始時と終了時にローダーが動いていない場合、地図ファイルの控えは取得できません。
配信元が終了済みの静的メッセージは、QoS設定だけで復元されるものではありません。

元から対象だったTF・通常地図・カラー点群・通常コストマップは維持しています。
SAM3を周期配信している場合は繰り返し記録されます。容量を抑える場合はローダーの
「初回のみ配信」を選びます。原点・パレット・意味割り当てはPGM＋JSONの組で保持します。
生3D点群とカメラ画像はこの記録には含みません。FAST-LIO自体の再計算や
映像を使う再解析まで残す場合は `record_mapping` を使います。
