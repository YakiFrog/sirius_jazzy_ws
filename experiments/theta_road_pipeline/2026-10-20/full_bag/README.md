# 共通パイプラインによる全ROSBAGの地図生成

入力: `real_both_mapping_20260920_113447_recovered`。
**5,956画像を処理し、5,956フレームを地図へ反映。スキップ0。**
出力: `maps_waypoints/maps/theta_road_general_full_20261020/road_temporal`。
スマホ用3画像: `http://100.97.20.32:54369/general_full_map/images.html`。
比較画像: 同じURLの `comparison/images.html`。

## 適用した処理

- `params/theta_road_mapping.yaml` の共通設定。特定の座標・日陰区間を指定しない。
- 通常の前後画像のSAM3結果は、保存済み全区間実験から再利用した。
  全フレームで元JPEGのSHAと時刻、投影後アップロードJPEGのSHA、prompt、
  モデル解像度512、各クラスの閾値0.5を確認した。
  通常推論の再利用フレーム数は5,956。元のキャッシュの11,912ビューの
  メタデータも事前照合し、不一致0。
- 日陰の補助推論は全区間の画像条件に応じて新しく実行。
  推論画像だけを白バランス＋gamma0.45で補正し、元画像の暗さ・黄色成分・
  保護クラスを確認して点字を追加。前向き・逆向きの追跡も実行した。
- RGBは前回選定済みのnative scatter、後方重み0.5、最低相対品質0.5、
  EMA減衰0.7で合成。既存の校正と記録された撮影姿勢を使用した。
- 記録TFとscan3を用いた厳密な遮蔽判定。障害物余白0.25m、時間許容0.15秒。
- 歩道・車道の色境界付き領域補正、連続点字の局所形状と接続に基づく歩道事前項。
  芝生拡張候補はOFF。

領域補正で27,748セル、点字文脈で12,877セルを車道から歩道へ変更。
これは補正前分類に対する変更数であり、正解セル数ではない。

## 確認

- 前回の全区間地図 `theta_texture_quality_map_20261020` と全5,956フレームで
  native地面点数・撮影位置・クォータニオンが完全一致。
- 領域補正前のunknown・壁・床・芝生・点字は領域補正で変更しない。
- 全セマンティックセルが観測RGBとscan可視サンプルの内側。
  RGBの観測支持とnativeサンプル被覆が一致し、未観測を埋めていない。
- 全入力キャッシュのフレーム集合、実行コードSHA、保存ZIPのSHAを照合。
  ZIPは全エントリのCRCを確認。
- 再利用の短区間6フレームでは、通常推論を新しく実行した対照と
  RGB・分類・観測票のファイルがバイト単位で一致。
- 関連40テスト成功。比較画像の世界座標サンプリングも恒等変換で照合。
- 全体の重ね合わせと、日陰2700・横断歩道4500付近の前後比較を目視確認。
  この全区間生成の確認は、独立した正解データによる認識精度や測量誤差の評価ではない。

## 保存物

このフォルダの `observations_part*.zip` に **11,912ファイル**
（5,956フレームのNPZと推論メタデータJSON）を保存した。
元のモデル分類・スコア、補助点字、前向き追跡の中間データを含む。
`observation_sha256.json` は抽出後の各ファイルのSHA。
`archives.json` はZIPごとのSHAとサイズ。

`manifest.json`, `camera_index.json`, `config_used.yaml`, `calibration_used.yaml`,
`run_identity.json`, `input_stream_sha256.json` が入力・設定・コードの記録。
`verification.json` が全区間の整合性確認、`server_restoration.json` がSAM3の
元設定とクラス登録の復元確認。HTTP確認とスマホ幅390pxのブラウザ確認も保存した。
地図・観測票・3種類の全体画像・6区間の比較画像は上記の出力地図フォルダに保存。
元のカメラBAGと通常推論の元キャッシュは、それまでの保存物を参照する。

空き容量のある外付けディスクで計算した生データ一式も保持している。
場所は `artifact_storage.json` に記載。
最初のキャッシュ未利用の実行は、全区間の通常推論を再利用して待ち時間を短くするため
途中で止めた。`initial_attempt.log` を残し、SAM3の復元成功を確認した。

再現コマンド（別の出力先を指定）:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python3 bash/startup_bash/generate_theta_road_map.py \
  /home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered \
  NEW_OUTPUT \
  --calibration src/sirius/sirius_navigation/config/theta_calibration_real.yaml \
  --reuse-study maps_waypoints/maps/theta_sam3_yellow_full_20261019
python3 bash/startup_bash/review_theta_road_map.py NEW_OUTPUT \
  maps_waypoints/maps/theta_texture_quality_map_20261020 \
  --centers 900,1500,2700,3000,4500,5100
```

`--centers` は比較画像だけの切り出し位置であり、地図生成や補正には使わない。
