# Lichtblickの低負荷点群表示

`points_preview` を起動し、Lichtblickの3Dパネルで `/velodyne_points_preview` を表示する。
元の `/velodyne_points` と `/velodyne_points_leveled` の表示はオフにする。
入力は既定で `/velodyne_points_leveled`、出力は5Hz・点数約1/2（`point_stride=2`）。
各リング内で2点に1点を残し、全16リング・選択点の全フィールド・座標系・取得時刻を維持する。
入力が20Hzなら、頻度と点数を合わせて送信量はおおむね87.5%減る。
保持するのは最新1メッセージだけで、入力が途絶したときは同じ点群を繰り返さず、リレー内で0.5秒以上待ったデータは捨てる。

生点群を表示したい場合は `points_preview /velodyne_points`、頻度変更は `points_preview --rate 10`。
表示点数も減らす比較用設定は `points_preview --point-stride 4`。
各リング内で4点に1点を残し、全リングと選択点の全フィールド・元時刻を維持する。
`ros2 param set /sirius_pointcloud_preview point_stride 4` で起動中にも変更でき、
`point_stride 2` で既定の半分、`point_stride 1` で全点に戻せる。
全点表示で起動する場合は `points_preview --point-stride 1`。比較時は同じトピック・5Hzを維持する。
1フレームの容量はおおむね1/4になる。これは表示専用の変更。
FAST-LIOの入力、LiDARドライバの頻度、record_3dmap/record_loc3dの記録対象は変更しない。
表示用トピックは既定のBAG記録対象に含めない。

FAST-LIOの累積3D地図自体が不要な場合は `lio3d_lite` を使う。
これは `/Odometry`・TF・診断を維持し、3D地図・軌跡・登録点群の配信を停止する設定。
既存のlio3d系とは排他であり、変更にはFAST-LIOの再起動が必要になる。

NUCでの生成遅延、WebSocketでの受信遅延、Lichtblickの描画・履歴保持は分けて評価する。
表示用リレーは送信負荷を下げる対策であり、Lichtblick自体のメモリリークや時刻追従不良を
解消した証明ではない。ブラウザでの長時間連続表示は別途確認する。

2026-10-22の比較では、全点約321KB、1/4約80KB、1/16約20KB/フレーム（いずれも5Hz、全16リング）。
全点と1/4では送信待ちが約3〜4MB残った。1/16で接続を作り直すと、40秒の確認で送信待ち最大約34KB・最後0となった。
接続し直した影響もあるため、点数削減だけの効果とは断定しない。利用者の表示品質の希望により、最終設定は1/2とした。
1/2は約161KB/フレーム・5Hzで約6.4Mbps。20秒の確認で送信待ちが約3MBまで再発し、遅延解消とは扱わない。
TFエラーによる点群非表示は、点群のバイト欠損と分けて扱う。点群の元時刻とTF経路・時刻範囲を確認する。
Foxgloveブリッジの `max_qos_depth` は標準の10を使用する。点群リレー自身の保持数1と分離し、
TF・静的TFまで一律1に制限しない。この変更だけでクライアントのTFエラーが解消したとは扱わない。
