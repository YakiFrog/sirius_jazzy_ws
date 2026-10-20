# 路面テクスチャの暗部持ち上げ

- 入力: `maps_waypoints/maps/theta_road_general_full_20261020`、5956フレーム。
- 出力: `maps_waypoints/maps/theta_road_bright_map_20261020`。
- 合成済みRGBへの決定的な画素単位トーンカーブ。推論、投影、点群、輪郭、
  格子、分類、投票、観測範囲は再計算していない。
- pivot=180固定、gain=1（元画像）、2、3、4を比較。フレーム2700付近の日陰と
  4500付近の日向を同じ10m四方で確認。地図全体の明るさ統計も保存。
- gain=3を採用。日陰の路面と点字が見やすくなり、gain=4より階調差を残す。
  この選択は目視による可読性判断であり、照度推定精度の評価ではない。
- 元々 `0<V<80` の448177セルの平均V: 42.69→81.90。
  元のV>=180は完全一致、新規V=255の飽和セル0。
- 比率を維持するRGB共通倍率（8bit丸めあり）。影以外の暗い素材も明るくなる。
  影の物理的除去、黒潰れした情報の復元はしていない。
- 出力に未補正 `road_temporal.raw.texture.png` を保存。
  `postprocess_identity.json` に入力・処理コードのSHAと設定を保存。
  分類PGM、格子JSON、投票NPZは入力とバイト一致。6箇所の比較と生成検証は出力内。
- 通常生成もセマンティック領域補正の後で同じトーン処理を使う。
  6フレームの通常生成を再実行し、rawRGB・ラベル・メタデータ・投票が旧生成と
  バイト一致することを確認。SAM3設定・registry復元済み。`smoke_checks.json`参照。
- 48件の対象テスト（トーン8件＋既存40件）を確認。

再現（workspace rootから）:

```bash
python3 experiments/theta_road_pipeline/2026-10-20/shadow_lift/reproduce_pilot.py
python3 bash/startup_bash/brighten_theta_road_map.py \
  maps_waypoints/maps/theta_road_general_full_20261020 NEW_OUTPUT
python3 bash/startup_bash/review_theta_road_map.py NEW_OUTPUT \
  maps_waypoints/maps/theta_road_general_full_20261020 \
  --centers 900,1500,2700,3000,4500,5100
```
