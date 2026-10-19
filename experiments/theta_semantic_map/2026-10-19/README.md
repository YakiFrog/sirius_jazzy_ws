# THETA 時系列セマンティック地図・画像 — 2026-10-19

real_both_mapping_20260920_113447_recoveredの検証済み10区間・170フレームを地図化した試作。セマンティックは前後画像512モデル・採用閾値0.5・黄色点字の前後時系列補完版（bidirectional）。通常ROSノードの過去のみ補完とは区別する。全BAG5956画像の意味推論を完了した地図ではない。

成果物：maps_waypoints/maps/theta_semantic_map_20261019/。images.htmlにスマホ向けの固定画像を掲載。seg3300/とoverview/にsemantic.jpg／overlay.jpg／texture.jpg／three_views.jpgがあり、他の検証区間にも同じ3画像を保存した。各3画像の範囲・向き・縮尺は同一。表示色は芝生=緑、点字=黄、車道=青、歩道=灰。表示用RGBを変更してもクラスIDは変更しない。未分類は薄い背景色で、意味を推測して埋めていない。

## 地図の生成

元BAGの撮影時刻、map→sirius3/base_footprintのTF、カメラ取り付けTF、同期scan3を使用。SAM3入力JPEGと元BAG170画像のSHA256一致を確認。視点マスクを地面z=0へ逆投影してBEVラベルを生成し、既存ThetaBevNode／ThetaGroundCloudNodeによる地面点と同一位置へ付与した。画素中心+.5規約に合わせ、BEV→透視画像の既存関数出力から-.5pxを補正。独立したレイと地面の交点で往復誤差1.76µm以下を確認した（これは座標式の整合性であり、実環境での地図精度ではない）。

撮影時刻との差0.15s以内のscan3、記録済みスキャン外部パラメータと時刻間オドメトリ、遮蔽余裕0.10m、障害物余白0.25mを用いる。BEV512²、10m四方、採用半径1.2–4.8m、stride2。観測重みは既存処理の入射角品質重み。地図は既存RGB地図の原点・範囲・解像度0.05mを固定し、同一セル内でクラス別重みを蓄積、重み合計0.3以上で最大クラスを採用。基準版と補完版の全地面点XYZ／RGB／品質重みが一致することを全170フレームで確認。スキップ0。

背景RGBはtheta_road_visibility_20261019_padded025/visible/road_visible.texture.png。元の観測済みRGBA画素は全てビット一致で維持。未観測の1364セルだけ、今回の同じLiDAR通過地面点の重み付きRGB平均で補った。合成後、全セマンティックセルにRGB観測が存在し、意味領域以外の重ね合わせPNGがテクスチャのみPNGと一致することを確認した。既存の観測済み領域を再平滑化していない。

## 結果と制約

点字セルは4075→4128（53セル、0.1325m²増）。既存点字セルの消失0、変更セルの移行先は全て点字。地図融合には既に複数フレームの投票があるため、動画上のちらつき改善に比べて最終地図の差は小さい。意味領域は135847セル（約339.6m²）、10検証区間に限る。元の誤認識、未認識、路面z=0という仮定、記録済み自己位置の誤差は残る。既知の通常舗装誤検出を消したと主張しない。

PGM/JSON/texture PNGは既存と同じindexed semantic形式。Nav2用の完全な占有地図として検証したものではない。アクティブな地図・ナビゲーション設定・SAM3サーバは変更せず、ROS_DOMAIN_ID73でBAGを同期読みして生成した。

## 再実行

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
ROS_DOMAIN_ID=73 python3 bash/startup_bash/map_theta_sam3_tactile.py maps_waypoints/maps/theta_sam3_temporal_20261019 maps_waypoints/maps/theta_road_visibility_20261019_padded025/visible/road_visible.texture.png maps_waypoints/maps/theta_semantic_map_20261019
python3 bash/startup_bash/render_theta_semantic_map.py maps_waypoints/maps/theta_semantic_map_20261019
```

前段の170フレームの入力・クラス／スコアPNG・補完結果が必要。以前のexperiments/theta_sam3_temporalとtheta_sam3_tactileのraw ZIPから同じ配置へ復元できる。元BAG約4.7GBはローカル保存のまま。map_evidence.npzに融合前後のクラス別重みとサンプルカバレッジ、manifest.jsonに全採用時刻・姿勢・対応スキャン時刻・スキップ・入力テクスチャSHA、rendering.jsonに画像範囲、verification.jsonに検証結果を保存した。SHA256SUMSは全地図成果物を検査する。地図PNG/PGM・JPEG画像・実行過程はGitで保管する。
