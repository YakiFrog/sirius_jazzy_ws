# 後処理共通録画 record_mapping

実機は `record_mapping`、シミュレーションは `record_mapping_sim`。
任意の名前は `record_mapping survey_20261022`、停止は Ctrl+C を1回。
保存先は `~/rosbag2_data/mapping_all_real_日時/`。開始確認の既定値は y。

## 起動しておくもの（カメラは任意）

- Velodyne（生点群 `/velodyne_points` と `/scan3`）
- IMU、Roboteq等の車輪Odom `/odom`、実機センサーTF（tf_real）
- THETA路面や色付けを使う場合：THETA撮影（theta_capture）
- ZED路面を使う場合：ZEDステレオ撮影（`zed_offline_recorder`。`/camera/stereo_sbs/compressed` と `/camera/stereo_params` の配信元）

全記録対象の受信状況を表示してから「録画を開始しますか？ (Y/n)」と確認します。THETA・ZEDは任意です。カメラ未受信・校正不正・カメラTF未接続は、そのカメラを使う後処理ができない旨を表示しますが録画を妨げません。scan3・TF・LiDAR・IMU・Odomの不足、基本センサーのTF未接続時は開始しません。撮影中にセンサーが停止しないかはTopics等で確認してください。終了時にも必須トピックの記録件数を検査します。

LIO、EKF、RTAB-Map、色付け、SAM3を録画のためだけに起動する必要はありません。動作していればOdomや診断情報を比較用に保存します。MCAP＋zstd_fast圧縮で、LiDAR生点群・カメラ画像は追加で間引きません。容量は両カメラの解像度・配信周期に依存します。

## 保存するもの

- 生LiDAR、IMU、車輪Odom、scan3/scan、TF/TF_static
- THETA圧縮画像、ZED左右結合画像と内部パラメータ
- 配信中のFAST-LIO/EKF Odom、診断、rosout、速度指令、パラメータ更新、実験メタデータ
- `capture/` に録画条件、実行中FAST-LIOの主要設定、THETA校正、設定ファイル群とコードのコミットID

大容量の累積地図・登録済み点群・色付き点群は既定では保存せず、生データから再生成します。必要なら `--include-derived-clouds` で色付き点群も追加できます。BAGとcaptureは一緒に保管してください。設定ファイルは参考スナップショットで、実測の座標変換は記録したTFを使います。

## 後からできる処理

- THETA路面マッピング：THETA画像・校正・TF・Odomを使用。
- ZED路面マッピング：左右画像・内部パラメータ・TF・Odomを使用。
- FAST-LIO再推定・3D地図・色付け・動的候補点除去：生LiDAR・IMU・車輪Odom・画像を使用。`offline_3dmap BAGパス` が入口です。
- slam_toolbox：scan3＋TFと再推定/記録Odomを使用。再生時はuse_sim_timeを有効にし、TFの配信元を1系統にします。

従来の路面マッピング処理/検証ツールが `/odom/filtered` を必須にしている場合、このBAGからEKF/LIOを再実行してその入力を用意するか、適切なOdomへリマップします。録画時にEKFが動いていない場合、同トピックの記録はありません。保存しただけで全方式の地図を自動生成するコマンドではありません。

未撮影範囲、画像のブレ・遮蔽、収録中の欠落は後から復元できません。初回は短いBAGで各後処理の再生確認をしてから本収録してください。

## 10分あたりの容量目安

- THETAあり・ZEDなし：約3.6 GB（既存BAGの30秒をzstd_fastで再記録し、177,585,234 bytesから10分へ換算した実測値）。
- THETA＋ZED：約5〜8 GBという概算。ZEDが8 fps、JPEG 1枚0.3〜0.8 MBと仮定して上記に加算した値で、両カメラ同時の実測値ではありません。

ZED追加容量は「平均画像サイズMB × fps × 600 ÷ 1000」GB。実際は解像度、JPEG品質、景色、配信周期で変動します。カメラなしの場合は減り、`--include-derived-clouds` では増えます。この目安にはBAG外のプロセスログやRTAB-Map DBを含めません。
