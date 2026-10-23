# THETA路面マッピングの共通生成手順

`run_offline_theta_mapping` の **[1] 改善済みの逐次処理** を通常経路にした。
従来のROS再生 + RTAB-Mapは [3]、既存地図の重ね合わせは [2] に残す。
新経路はSAM3処理の完了を待って記録画像を順番に処理する。再生速度に
依存するフレーム落ちを避けるため、ROSのトピック再生ではなくBAGを直接読む。

## 入力と実行

必要なのは、そのBAGの圧縮dual-fisheye画像、記録された撮影時刻、ロボット姿勢TF、
カメラ校正。既定の厳密な遮蔽判定にはscan3とLiDAR外部パラメータTFも必要。
特定の過去地図、経路の座標、日陰のフレーム範囲、エレベータ・白線の位置は不要。
MCAPとsqlite3のストレージ選択はrosbag2に任せる。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 bash/startup_bash/generate_theta_road_map.py BAG_PATH NEW_OUTPUT_DIRECTORY \
  --calibration CAMERA_CALIBRATION.yaml
```

共通設定は `params/theta_road_mapping.yaml`。部分的な上書きYAMLも
`--config CUSTOM.yaml` で渡せる。通常ターミナルでは
`THETA_ROAD_CONFIG=/absolute/custom.yaml run_offline_theta_mapping` に相当する
環境変数を使える（aliasの展開があるため、シェルでは `export THETA_ROAD_CONFIG=...`
を先に実行すると確実）。`--no-semantic` はRGBのみ、`--max-frames N` は
動作確認用で全区間生成ではない。通常は上限を付けない。

## 共通処理

1. BAGのTF・scan・画像メタデータを索引化する。画像は元BAGから読み直し、
   JPEGを複製しない。関連メッセージ列全体のSHA256も保存する。
2. 撮影時刻のTFを使用する。map→robotのTFがないフレームを最新姿勢で代用しない。
   カメラ取り付け姿勢は撮影時刻のTFを優先し、なければ指定した校正YAMLに
   フォールバックする。その由来をフレームごとに記録する。
3. 前後透視画像をSAM3で分類し、黄色の点字の外観を確認する。
   日陰の追加推論は画像の暗さで選び、白バランス＋gamma0.45で推論用画像だけを補正。
   `yellow tactile paving in shadow` の結果のうち、元画像で暗く、黄色成分を
   確認できたID4だけを追加する。芝生・壁・床の分類は追加推論で上書きしない。
   モデルのクラス登録に黄色点字の別名がない場合のID0出力にも、従来の検証と同じ
   target-only/正スコア条件で対応する。サーバのクラス登録を書き換えない。
4. 前向き・逆向きの画像追跡を行う。遮蔽、画角外、時刻の大きな断絶を越えて
   点字を延長しない。通常検出と日陰の検出を別々に追跡し、各クラス自身のスコアを使う。
5. 既存のBEV・地面点群生成とscan3遮蔽判定を再利用する。既定は障害物余白0.25m、
   同期許容0.15秒。TFやscanの不備を理由に遮蔽判定を自動解除しない。
6. RGBは各フレーム・各セルに集約し、距離／入射角の品質を使う。
   最良品質の半分未満の観測による上書きを抑え、EMA減衰0.7で合成。
   後方重みは既定0.5。セマンティックの観測票はRGBの上書き保護とは独立に集約する。
7. 観測された歩道・車道に対して色の境界を考慮した領域補正を行う。
   その後、連続した点字の途中を手掛かりに有限の歩道事前項を加える。
   端点・短い断片・広い警告部分・局所的に線状でない部分は手掛かりから除外する。
   未観測・芝生は接続を遮断し、点字・芝生など他のクラスは変更しない。
   元のクラス画像と観測票を残す。
8. 芝生拡張は既定OFF。`grass.candidates: true` で独立した候補画像だけを保存し、
   最終のクラス地図には採用しない。実験で未確認の候補を自動昇格させない。

地図格子は記録軌跡と設定した投影範囲から求める。地図の形を画像に合わせて
変形する操作はなく、未観測の空間を埋めない。
白線・横断歩道・特定のセマンティック対象だけを直線にする処理もない。

## 環境ごとに見直す設定

- カメラ校正と取り付け高さ、TFのフレーム名、画像・scanトピック。
- 半径、格子解像度、scanの時刻許容と障害物余白。余白を広げるほど本物の床も
  遮蔽物の近くで取り込まれなくなるので、形状を削除する処理とは区別する。
- `texture.rear_weight`: 後方に人が多い今回の収録条件の事前重みであり、
  普遍的な静止確率ではない。前後の品質が同じなら1.0にする。
- `regions.tactile_context`: 黄色い連続誘導ブロックが歩道内にある環境の事前項。
  条件が違う環境ではfalseにする。点字の両側が必ず歩道という断定ではない。
- `regions.policy`: 既定の `road-to-sidewalk` は車道の小さな混入を減らすための
  片方向の採用方針。誤認識を必ず直す保証ではない。`both` は双方向で変更するので
  別の評価が必要。領域補正自体も `regions.enabled: false` で停止できる。

全距離はmで指定し、解像度からセル数に換算する。しきい値は設定として再利用できるが、
別BAGで同じ認識精度を保証するものではない。現状の投影は局所平面の路面仮定を使う。
斜面・凹凸、時刻ずれ、誤校正、SLAMの歪みまで画像合成で修正するものではない。

## 出力、再開、再現

- Map Select: 出力フォルダ名 + `/road_temporal`。
- `road_temporal.colored.pgm/.json`: 分類IDと減色RGBのパレットインデックス、格子・パレット・コスト情報。研究の配信・評価用の正本。
- `road_temporal.texture.png`: 再書き出し・解析のための同じ格子のBGRA生成中間結果。Loaderの入力には使わない。
- `road_temporal.raw.texture.png`: 明るさ補正前の合成RGB（補正有効時）。
- `images.html` と `indexed.png`: PGM＋JSONだけから復元したスマホ向け確認画像。PNG画像をLoaderに与えるものではない。
- `texture.jpg/overlay.jpg/semantic.jpg`: RGB合成や分類を調べる参考画像。Loaderの画質評価には使わない。
- `baseline.class_ids.png`: 領域補正前の分類。
- `map_evidence.npz`: 元の観測票とscan可視サンプル数（配列はworldの下原点）。
- `observations/`: フレーム単位のモデル結果・スコア・前向き追跡、推論設定／アップロードSHA。
- `manifest.json`: 適用設定、処理数、除外理由、撮影姿勢、カメラ姿勢、最終統計。
- `config_used.yaml`, `calibration_used.yaml`, `run_identity.json`, `input_stream_sha256.json`:
  設定、入力とコードのハッシュ、ライブラリ版。
- `server_before.json`, `registry_used.json`, `server_restoration.json`: サーバの状態と復元確認。
- `completed.json`: 生成・保存・画像出力がすべて成功した場合だけ作る。

中断後は同じ引数に `--resume` を追加する。BAG・設定・校正・コードが変わっていたら
既存推論を流用せず拒否する。元画像のハッシュと関連メッセージ列も再確認する。
同条件なら保存済みの推論を再利用し、追跡と合成は最初から再計算する。
新しい通常起動は日時付きの別出力を作るので、過去の地図を上書きしない。

過去の逐次SAM3実験のキャッシュがある場合は `--reuse-study CACHE_DIRECTORY`
も使える。元の画像SHAと時刻、投影後のアップロードJPEGのSHA、prompt・解像度・
クラス閾値が完全一致した通常推論だけを再利用する。不一致は拒否する。
暗所の追加推論・双方向追跡・投影・合成・領域補正は新しく実行し、再利用元と
マスク／スコアのSHAもフレームごとのJSONに残す。過去の地図をコピーする処理ではない。

## PGMのパレット割り当てと表示

認識した芝生・点字・車道・歩道は予約ID 3〜6 の分類色を使う。
それ以外の観測済み路面はRGBを各5段階（0,64,128,192,255）へ量子化し、
125色をID 7〜131へ割り当てる。RGBが分類色と同じでも別インデックスを使い、
色が似ているだけの場所に分類やコストを付けない。未観測のID 0や分類・構造IDは変更しない。

SAM3MapLoaderの既定 `visualization_mode=indexed` はPGMとJSONだけを読み、
全インデックスをJSONの色で復元する。`semantic` は分類だけを抽出する。
PNG直接表示の `texture` / `both` モードは廃止した。

2026-10-23以前の逐次生成版には、PGMが分類IDだけになり、路面のRGBがPNGにだけ
保存されていたものがある。原本を残し、以下で従来形式の別出力を作れる。
生成時にRGBの中間結果から減色するが、出力地図の読み込みにはPNGを必要としない。

```bash
python3 bash/startup_bash/indexed_road_export.py SOURCE_MAP NEW_OUTPUT
```

## 地図テクスチャの暗部補正

`texture.tone` は通常生成にも適用される表示用補正。既定は enabled=true、gain=3、
pivot=180（8bit RGBの最大成分）。v<pivot のとき
`f(v)=gain*v/(1+(gain-1)*v/pivot)`、それ以上は元の値のまま。
RGB全成分を同じ倍率で変え、黒と明るい画素、未観測領域、α、座標を保存する。
局所フィルタを使わず、画素の並びや輪郭を変更しない。8bit丸めによる色差は生じる。
影の推定や除去ではないので、暗い材質も明るくなり、記録されていない細部は復元できない。

セマンティック推論・領域補正がすべて完了してから適用するため、分類ID、投票、
色境界による補正には影響しない。無効化は `texture.tone.enabled: false`。
元のRGBを保存し、manifestに画素統計とパラメータを記録する。

既存の地図にもSAM3や投影の再実行なしで適用できる（別フォルダを必須とする）。
既に補正した地図を入力してもraw画像から作り直し、補正を累積しない。

```bash
python3 bash/startup_bash/brighten_theta_road_map.py SOURCE_MAP NEW_MAP
```

5956フレームの全区間生成と暗部補正を実行済み。別BAGの認識品質評価は別途必要。
