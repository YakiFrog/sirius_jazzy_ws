# Velodyne入力の検証（2026-10-21）

実機の `/velodyne_points` は、同じ方位のリング0〜15を順に並べた点群だった。
FAST-LIOの `point_filter_num: 4` が全体の点番号に適用され、リング0・4・8・12だけが残っていた。
Velodyneの特徴抽出OFF経路を、リング内で4点ごとに間引く方式へ修正した。
取得時刻・座標は変えず、同程度の点数で16リングを維持する。
既存のLiDAR–IMU外部回転による補正は継続し、入力は生の `/velodyne_points` を使う。

回帰試験 `velodyne_sampling` は実際の `Preprocess::process()` に
16リング交互配置のPointCloud2を渡し、全リングが残ること、点数、点時刻を検証する。

通常の地図生成用設定でも `/fastlio/diagnostics` を有効にし、地図生成BAGにも
診断・rosout・Roboteq状態を記録する。診断の `estimator_state` には高さ、roll/pitch、
有効対応点数、推定加速度・ジャイロバイアス、重力ベクトルを含める。
診断は推定状態を観測するだけで、推定値を補正しない。

NUCの検証記録は `~/experiment_records/slam3d_real_20261021_205132/`。
生BAGを保存したまま、別ROSドメインで再計算する。

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/recompute_3d_lio.sh \
  ~/rosbag2_data/slam3d_real_20261021_205132 \
  --output /path/to/new-output --extrinsics config --wheel on
```

単体試験通過は走行精度の保証ではない。同じBAGでの修正前後比較に加えて、
別BAGと実機の静止・走行で確認する。固定加速度補正やAHRS傾き補助は単独で改善を
保証できなかったため、今回の恒久修正には含めない。
