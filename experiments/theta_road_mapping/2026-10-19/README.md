# THETA路面マッピング実験記録 — 2026-10-19

論文執筆用の追跡可能な記録。単一BAGで行った探索的な局所RGB比較であり、別BAG・別環境での汎化性能や動的障害物の除去率を示す実験ではない。

## 保存した成果物

- [前後重み・品質保護：4区間10条件の比較](../../../maps_waypoints/maps/theta_direction_trial_20261019/comparison.html)
- [方向重み実験の入力区間・全フレーム・撮影姿勢](../../../maps_waypoints/maps/theta_direction_trial_20261019/manifest.json)
- [区間別評価値](../../../maps_waypoints/maps/theta_direction_trial_20261019/metrics.json)、[集約値](../../../maps_waypoints/maps/theta_direction_trial_20261019/aggregate_metrics.json)、[目視記録](../../../maps_waypoints/maps/theta_direction_trial_20261019/review_notes.json)
- [通常投影とLiDAR遮蔽除外の全区間比較](../../../maps_waypoints/maps/theta_road_visibility_20261019/comparison.html)
- [遮蔽範囲を25cm拡張した全区間比較](../../../maps_waypoints/maps/theta_road_visibility_20261019_padded025/comparison.html)
- 上記3ディレクトリには、40条件別局所地図・4枚の前後カメラ画像・比較画像・PGM/JSON/透過RGB PNG、全区間の地図4組、実行ログがある。
- [入力BAGのファイル名・サイズ・SHA256](inputs/bag_files.json)、[BAGメタデータ](inputs/bag_metadata.yaml)、[環境](environment.json)、[実装リビジョン](source_revisions.json)
- [実験時の実装・設定スナップショット](source_snapshot/source.zip) と [内部ファイルのSHA256](source_snapshot/FILES.json)
- [検証・実行ログ](logs/)、[探索段階の一時スクリプトと人物検出プローブ](exploratory/)

GitHub上ではHTMLのソースが表示されるため、対話比較はチェックアウト後にHTMLをブラウザで開く。

## 入力データの保管

元BAG：`real_both_mapping_20260920_113447_recovered`。ローカル保管先：`/home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered`。

MCAPは約4.7GBで、元データはGitへアップロードしていない。元BAGを別媒体にもバックアップする必要がある場合は、このディレクトリを保管対象とする。Gitには実験の出力、BAGメタデータ、参照先、サイズ、SHA256を保存した。

MCAP SHA256：`ca53cbfec43179ff420dc8ffdf3da91ad6df84b5447625064e4862fe8b733036`。

THETA圧縮画像5956件、記録時間約1581.41秒。最終RGB局所試験ではこのうち260枚を決定論的に選び、208枚を融合、52枚を評価用に保留した。入力画像・TF・スキャンの不一致によるスキップは0枚。

## 実験の過程

1. 画像の人物画素をSAM3で除外する方法をプローブで確認したが、人以外も対象とする要件から採用しなかった。`exploratory/probe_road_person.py` 等はこの探索段階の記録であり、最終比較には人物ラベルを使用していない。
2. 記録済み2D LiDARとTFを用いた遮蔽除外を実装。古いスキャンへのフォールバックや、除外が多いと処理を解除する従来ゲートとは分けた。通常版と同時に全BAGを4倍速で再生して保存した。
3. 遮蔽の周囲に半径25cmの余白を追加して再生成。全区間の観測面積は前回遮蔽版の約87.7%となり、残像低減と引き換えに路面の欠けも増えた。全区間版はROSの処理タイミングによって採用フレームが変わり、完全なビット単位再現は保証しない。
4. 前方優先の重みと品質保護を、3区間8条件で予備比較。その数値は `logs/theta_direction_metrics_first.json` に保存。
5. 保護処理単独の条件2個と区間2200を追加し、最終的に4区間10条件で比較した。最終比較は画像を順番に同期処理し、全条件で同じフレーム・TF・原点・解像度を使う。予備3区間の既存8条件は再実行時に同じ数値を再現した。
6. RGB比較ページをChromeで描画して確認。今回の推奨は候補の保存に留め、既定値と既存の全体地図は変更していない。

## 投影と共通条件

校正済みTHETA前後魚眼画像から、床をロボット座標の `z=0` とした俯瞰画像を生成し、撮影時刻の記録済み `map → base_footprint` TFで地図へ配置する。カメラ取り付け姿勢もTFから解決する。

- BEV：1024×1024、10m四方、カメラ高さ1.135m、入力1920×1080。
- 点の採用半径：1.2〜4.8m。画像サンプリングstride 4。地図解像度0.05m。
- LiDAR：撮影時刻との差0.15秒以下のスキャン。スキャン座標への変換で外部パラメータと時刻間のオドメトリを使用。スキャンまたはTF不明なら観測を取り込まない。
- LiDAR遮蔽の距離方向余裕：0.10m。方向重み比較の共通条件は追加の円形余白0m。`equal_pad025` だけ半径0.25mの追加拡張を使う参考条件。
- 穴埋めなし。RGB指数移動平均の減衰率0.7。最終局所試験はSAM3を実行せず、意味分類の品質は評価対象外。
- 重みの基本項は `q=(h/sqrt(rho^2+h^2))^2`。これは観測品質の経験的重みで、静的物体である確率ではない。
- 前方を1、後方を指定係数とし、ロボット側方を中心とした30度の範囲で滑らかに補間して `w=q×方向係数` とする。地図座標ではなく撮影時のロボット座標で前後を判定する。
- 品質保護ではセルごとの最良重み `b` を記憶し、`w >= r×b` の観測を採用する。`r=0` は保護なし。未観測セルは弱い観測でも埋められる。低品質の観測は何度繰り返しても最良重みの閾値を越えない。

## 局所区間

フレーム番号はTHETAトピック内の0始まりインデックス。中心±64枚を2枚ごとに選択する。時刻・全採用インデックス・位置姿勢は実験manifestに保存してある。

| 中心フレーム | 区間長[s] | 移動距離[m] | ヨー角の変化幅[deg] | 融合 / 保留枚数 |
|---|---:|---:|---:|---:|
| 1500 | 36.07 | 15.15 | 13.08 | 52 / 13 |
| 2200 | 42.97 | 18.20 | 10.46 | 52 / 13 |
| 3000 | 33.28 | 0.50 | 1.41 | 52 / 13 |
| 4500 | 33.88 | 8.18 | 88.40 | 52 / 13 |

区間3000はほぼ停止中、区間4500は旋回を含む。前方にも人物が写る場面があり、「前方は常に人がいない」とは仮定していない。

## 評価方法

各区間で選択順の5枚に1枚を評価用に保留する。参照画像は保留観測だけから作り、方向重みを付けない基本品質0.12以上の観測を使用する。各時刻のセル色を重み付き平均し、3時刻以上の支持があり、時系列中央値に対する絶対偏差中央値のRGB平均が12以下のセルを安定領域とする。

全条件で観測された共通の安定領域57604セル上で、参照色とのRGB各チャンネルの絶対誤差を平均した。各区間の評価セル数で重み付けして全体平均とした。セルは5cm四方である。

参照は実測の正解地図ではない。安定した床の色の整合を評価する補助指標であり、人の正解マスク・物体検出精度・残像除去率・SLAMの幾何精度を測っていない。条件の選択にも使った評価値なので、独立した最終テストセットの成績として扱わない。1つのBAG内の相関した区間であり、統計的独立試行4回とは数えない。

## 最終結果

| 条件 | 後方重み | 最低品質比 | 安定領域のRGB平均絶対誤差 | 観測セル数比 |
|---|---:|---:|---:|---:|
| `equal` | 1 | 0 | 15.867 | 100.00% |
| `rear050` | 0.5 | 0 | 15.352 | 100.00% |
| `rear020` | 0.2 | 0 | 14.317 | 100.00% |
| `rear005` | 0.05 | 0 | 12.073 | 100.00% |
| `equal_keep050` | 1 | 0.5 | 8.976 | 100.00% |
| `equal_keep080` | 1 | 0.8 | 8.096 | 100.00% |
| `rear050_keep050` | 0.5 | 0.5 | 8.197 | 100.00% |
| `rear020_keep050` | 0.2 | 0.5 | 8.774 | 100.00% |
| `rear020_keep080` | 0.2 | 0.8 | 9.336 | 100.00% |
| `equal_pad025` | 1 | 0 | 13.559 | 91.84% |

第一候補：`rear_observation_weight=0.5`, `min_relative_observation_weight=0.5`。基本条件の15.867から8.197へ、参照との色の誤差が約48.34%減った。重み・保護の条件はいずれも観測済みセル数254738を維持した。これは路面の真の面積や動的残像除去率を保証する数値ではない。

前後同じ・品質比0.8の平均誤差は8.096でわずかに小さいが、ほぼ停止中の区間3000では1.27から約3.07へ悪化した。過度な固定を避け、後方0.5・品質比0.5を常用の第一候補とした。別BAGでの再現性は未確認。

## 再実行

親リポジトリとnavigationサブモジュールをこの実験のコミットへ合わせる。navigation実装：`30d2255fc5b93f7db8b64400147e933932455142`。ソースのアーカイブは将来の編集から独立して残してある。

```bash
cd ~/sirius_jazzy_ws
git submodule update --init src/sirius/sirius_navigation
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sirius_navigation --symlink-install
source install/setup.bash

ROS_DOMAIN_ID=154 OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 \
  bash/startup_bash/compare_theta_road_weights.py \
  /home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered \
  --output /tmp/theta_weights_reproduction \
  --calibration src/sirius/sirius_navigation/config/theta_calibration_real.yaml \
  --centers 1500,2200,3000,4500
python3 bash/startup_bash/report_theta_road_weights.py /tmp/theta_weights_reproduction
```

出力先は存在しないディレクトリを指定する。既存の実験ディレクトリへの再実行には明示的な `--resume` が必要。フルBAGの遮蔽比較は `compare_theta_road_visibility.py` に `--obstacle-padding 0.0` または `0.25` を指定する。後者は外部SAM3サーバを路面分類に使用するため、ローカルRGB比較と同等の再現性とは見なさない。`server_before.json` はサーバ変更前の状態であり、実行中の閾値は保存した `theta_sam3.yaml` と共有クラス設定を参照する。

推奨候補を通常のlaunchで使う場合の引数：

```text
rear_observation_weight:=0.5 min_relative_observation_weight:=0.5
```

地図セレクタで局所候補を見る例：`theta_direction_trial_20261019/seg4500_rear050_keep050/road`、表示は路面RGB。

## 検証と完全性

- 実験当時の作業ツリー全体ではTHETA関連51テスト通過。ログを保存した。
- 送信するnavigationコードだけをGit indexから取り出した別ディレクトリで、路面専用15テストが通過。以前からあった別の未コミット作業に依存しないことを確認した。
- 保護が繰り返しの弱い観測で破られないこと、キャンバス拡張で品質記憶が保たれること、前後重みで座標が変化しないことを含む。
- 40個の局所地図、比較ページのリンク、JavaScript構文、Chromeでの描画を確認した。
- ワークスペースのルートから `sha256sum -c experiments/theta_road_mapping/2026-10-19/SHA256SUMS` で成果物を確認できる。元BAGはBAGディレクトリで `inputs/INPUT_SHA256SUMS` の内容を使って検証する。
