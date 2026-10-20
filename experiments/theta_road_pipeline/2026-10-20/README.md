# 通常生成経路への共通化と短区間の統合確認

実験専用だったTHETA路面マッピングの改善を、
`generate_theta_road_map.py` と `params/theta_road_mapping.yaml` にまとめた。
通常ターミナル `run_offline_theta_mapping` の既定の [1] から呼ぶ。
処理・設定・制約の詳細は `docs/theta_road_mapping_pipeline.md`。

## 確認した範囲

| 入力 | 使用画像 | 推論・追跡・投影・合成・領域補正・保存 |
| --- | ---: | --- |
| real_both_mapping_20260920_113447_recovered | 先頭6フレーム | 成功、スキップ0 |
| real_both_mapping_20260919_110949_recovered | 先頭4フレーム | 成功、スキップ0 |

両方とも同じ既定設定、同じ実機校正を使用。SAM3の元設定とクラス登録が
復元されたことを確認した。BAG1は前回の全区間地図と各フレームのnative点数・
撮影位置・クォータニオンが完全一致した。GUIや稼働中の地図サーバは切り替えていない。

同じ出力に `--resume` を指定した結果、SAM3への新規アップロードは0、
分類PGM、RGB PNG、補正前分類、観測票NPZはバイト単位で一致した。
設定変更時の再開、生成履歴のない別ディレクトリへの再開は拒否される。

38件の関連テストも成功した。共通設定・日陰補完の保護クラスとRGB不変性、
異なる解像度での領域処理、芝生候補の未採用、MCAPの同時刻seek問題、
target-onlyの未登録別名、通常ターミナルからの起動を新規テストに含む。
既存のテクスチャ合成・領域補正・点字文脈・芝生候補テストも実行した。

**これは機能の統合確認であり、別BAGの全区間の認識精度評価ではない。**
日陰や点字が含まれる全区間の改善量は従来の実験を参照する。
今回の先頭6/4フレームだけから、その精度や別環境への性能保証は主張しない。
最新の全区間地図をこの統合パイプラインで再生成したわけでもない。

## 保存内容

`bag1/` と `bag2/` に設定、校正、入力索引、コードSHA、BAG関連ストリームSHA、
推論結果・スコア・画像追跡キャッシュ、アップロードJPEGのSHAとモデル設定、
生成地図・観測票・比較画像・サーバ復元記録を保存した。
元BAGや全画像のコピーは作らず、元の圧縮画像のハッシュで照合している。
`verification.json`、`tests.log`、各実行ログも保存した。
BAG1の最終manifestは再開後の実行記録で、元の推論メタデータは
`observations/*.json` に残っている。

再現コマンド（新しい検証出力先を指定する）:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python3 experiments/theta_road_pipeline/2026-10-20/verify_real_bags.py \
  /home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered \
  /home/kotantu-desktop/rosbag2_data/real_both_mapping_20260919_110949_recovered \
  --calibration src/sirius/sirius_navigation/config/theta_calibration_real.yaml \
  --output /tmp/new_theta_pipeline_check \
  --reference maps_waypoints/maps/theta_texture_quality_map_20261020
```

`verify_real_bags.py` の6/4フレーム制限は、この動作確認用である。
製品の生成経路は制限なしでBAG全区間を処理する。
