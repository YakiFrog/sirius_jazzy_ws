# 研究の進展ホーム

Launcherのヘッダーにあるグラフのアイコン、または `/research/` から開きます。
研究テーマの絞り込み、キーワード検索、結果表示、条件・記録の確認ができます。
スマホではカードが1列になります。一覧では軽量な画像だけを読み込みます。

## 記録の追加

`static/research/catalogue.json` に、成果のディレクトリを `bundles`、説明を
`entries` として登録します。ディレクトリには `{home}` と `{workspace}` が使えます。
各カードには日付、テーマ、結果ページ、サムネイル、確認事項、検証の範囲、原記録を指定します。
日付は生成・検証日、使用した収録データは `recording` に別途記録します。
比較した手法同士の一致と、正解データによる精度評価を区別してください。

`maps_waypoints/maps/road_semantic/` 直下の `theta_YYYYMMDD_HHMMSS` / `zed_...`
形式の地図は、生成成功の manifest・completed.json・source_bag.json・images.html
が揃った場合に自動追加します。登録済みの地図は重複しません。
一覧を再読み込みすると反映されます。BAG自体の公開・読み込みは行いません。

## 保管と配信

2026-10-23時点では、NUCの公開済み比較結果をDesktopの
`~/research_archive/nuc/` にコピーしています。このリンクの実体はHDDの
`/media/kotantu-desktop/GS2MESH_HDD/sirius_research_archive/` です。
NUCの電源が切れていてもDesktopから参照できます。DesktopとHDDは必要です。

`research_archive.py` は明示登録された成果ディレクトリだけを配信し、
認証条件をLauncherと共有します。ディレクトリ外への移動・リンク参照は拒否します。
元のHTMLやデータは変更せず、配信時に研究ホームへの戻りリンクと過去の相対リンクを補正します。
原記録のPythonスクリプトはテキストとして表示され、実行しません。
読み取り専用です。新規APIは `/api/research`、結果は `/research/assets/...` です。
