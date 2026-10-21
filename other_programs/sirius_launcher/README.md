# Sirius ROS2 Launch Manager

ROS2ノードやlaunchファイルをGUIボタンから起動できるランチャーアプリケーション

## Sirius Control Console（Web版）

左のアイコンから Home / Terminal / Monitor / 3D / TF Tree / 設定を切り替えます。
左上のSiriusアイコンで画面を再読み込みできます。表示中の画面とターミナルセッション、フルスクリーン状態を引き継ぎます。
Homeではプリセット・個別プログラムを起動し、縦一列のプログラム一覧からログを確認できます。
デスクトップではクイックスタートを右上の縦一列にまとめ、その下に稼働中一覧を表示します。
クイックスタートと稼働中は見出しの矢印で折りたため、状態はブラウザに保存されます。
左右の列は独立してスクロールし、境界をドラッグ（または←→キー）すると幅を変更できます。
小さい画面ではクイックスタートをプログラム一覧の上へ移動します。
停止・再起動は確認メッセージの承認後に実行します。Terminalではプログラムのログと対話入力、
独立したシェルのタブを使えます。ターミナル出力とプログラムのログでは、ERROR/FATALを赤、
WARNを黄、INFOを青、DEBUG/TRACEを淡色で表示します。シェルの「×」はそのセッションを終了します。
入力欄・出力欄でCtrl+Cを押すか、入力欄の横の「Ctrl+C」ボタンを押すと、
現在のタブの実行中コマンドへ割り込みを送ります。テキスト選択中のCtrl+Cはコピーです。
これらの画面変更は、Webサーバーを再起動せずページの再読み込みで反映できます。

Web版の異常表示はログ履歴と現在の状態を区別します。一般プロセスは直近30秒の
ERROR/FATAL/例外を異常とし、時間経過後は「過去エラーあり・現在の状態未確認」にします。
ROS launchの子プロセス異常終了は、その子の再起動を確認するまで異常表示を維持します。
バッテリー専用BLE起動は接続済みSolixStatusの受信も確認し、エラー後の新しいデータで
復帰、5秒を超える受信停止や未接続で異常とします。過去の例外はログと表示のツールチップに残します。

MonitorにはホストのCPU・メモリ・ディスク・CPU温度、コア別負荷、過去2分の推移、
ネットワーク転送速度、プロセス一覧を表示します。Sirius / ROS関連と全プロセスを切り替え、
検索・CPU/メモリ順の並べ替え・詳細確認ができます。プロセス詳細の「停止」から、
子孫プロセスを含めて個別に停止できます。
プリセットは起動済みのプログラムをスキップし、不足しているプログラムだけを起動します。
PIDファイルが失われたWeb起動セッションもコマンドラインから検出し、稼働中表示・
二重起動防止・停止の対象にします。セッションが起動した子プロセスは、外部起動として
重複表示せず、そのセッションにまとめます。
全体CPUは全コアで100%、プロセスCPUは1コアで100%です。
監視値はサーバー側で1秒ごとに共有取得し、Monitorは約2秒ごとに更新します。
取得できないセンサー値は「—」、古い測定値は「更新停止」と表示します。

専用画面を全画面で開く場合:

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/start_sirius_console.sh
```

デスクトップへのログイン後に自動で開く設定（ロボットのプログラムは自動起動しません）:

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/install_sirius_console_autostart.sh
# 解除
bash ~/sirius_jazzy_ws/bash/startup_bash/install_sirius_console_autostart.sh --remove
```

Webサーバーはブラウザ・ターミナルから独立して起動し、既存の8090番サーバーがあれば
それを使います。`SIRIUS_WEB_KIOSK=0` でウィンドウ表示、`SIRIUS_WEB_NO_BROWSER=1` で
サーバーだけ起動できます。ダーク/ライトの切り替えは設定画面にあります。
この自動起動はデスクトップへのログイン後に実行され、自動ログインの設定自体は変更しません。

### 一括再起動と起動入力の復元

設定画面の「稼働中を一括再起動」は、現在稼働中のランチャー管理プログラムを
すべて停止してから起動し直します。停止に失敗したプログラムは再起動しません。
「前回の起動入力を復元」を有効にすると、各プログラムについて記録済みの
ROS Domain IDと起動コマンドを再使用します。記録がないプログラムは、
現在画面に入力されているROS Domain IDと現在のエイリアス定義を使います。
起動入力は `/tmp/sirius_launcher_launch_specs.json` に保存されます。
記録はこの機能を入れた後に起動したプログラムから作成されるため、既存の
稼働中プログラムは初回の一括再起動では現在の入力が使われます。
ロボットが動作中の場合は危険なので、確認メッセージを確認してから実行してください。

### 3D Visualization（Lichtblick）

`3D` ページは Lichtblick Web を iframe で表示し、`foxglove_bridge` 経由の
ROS 2 データを可視化します。SiriusLauncher 側から速度指令や Nav2 ゴールを送る機能は
追加していません。表示のみを目的としたページです。

初回にLichtblickの表示設定を整えたうえで、既定レイアウトを読み込みます。
2回目以降はブラウザに保存されたLichtblick側のレイアウトを使うため、
ユーザーが変更した表示を上書きしません。「レイアウト」の「適用」で
選択したレイアウトを明示的に再読み込みできます。

サーバー側には名前付きレイアウトをJSONで保存できます。レイアウトは
`3D` ページの「JSON取込」から保存し、「適用」で読み込み、「書出」で
JSONをダウンロードできます。削除も可能です。Lichtblickの現在の画面を
アプリが直接読み取ることはできないため、保存するときはLichtblick側で
レイアウトをJSONに書き出してから取り込んでください。保存先は
`sirius_visualization_layouts/` です。

既定レイアウトには次を用意しています。

- 3Dパネル（固定フレーム `map`、斜め俯瞰の初期視点）
- `/velodyne_points`、`/scan`、`/hokuyo_scan`（初期非表示）
- `/map`、`/localization/map`（初期非表示）、`/plan`、`/local_plan`
- `/sirius_markers`
- グリッド、TF

トピックが存在しない場合はエラーにせず、そのレイヤーだけ表示されません。
`map` が無い環境ではLichtblick内の `Display frame` を利用可能なフレームへ変更してください。
Lichtblick標準のPublishツールは上流機能として残っています。誤操作を避けるため、
初期位置（Pose estimate）の送信先は `/initialpose` です。地図上の現在位置から向きへドラッグして設定します。
Pose / Point の送信先は `/sirius_visualization/disabled_*` のままです。
既存ブラウザに古い設定が残る場合は、3Dパネルの設定 → Publish → Pose estimate topic を `/initialpose` に変更するか、更新済みレイアウトを再適用してください。
一括再起動で復元するのはプロセスと起動引数です。FAST-LIOの内部状態や最後の地図内姿勢は復元しません。
Publish全体を無効化する場合は `foxglove_bridge` の起動設定から `clientPublish` を外してください。
既存の `foxglove` エイリアスやbridge設定は自動変更しません。

#### 表示サーバーの起動

公式Dockerイメージをダイジェスト固定で利用します。Composeファイルは
`viewer/docker-compose.yml` です。

```bash
cd ~/sirius_jazzy_ws/other_programs/sirius_launcher/viewer
docker compose up -d
```

- Viewer: `http://<ホスト>:8091/`
- Bridge: `ws://<ホスト>:8766`（NUCの既存bridgeに合わせた既定値）
- `viewer/sirius-start.html` は初回だけLichtblickの設定サイドバーを閉じ、
  3Dビューを広く表示するためにマウントしています。
- イメージは `ghcr.io/lichtblick-suite/lichtblick@sha256:230ed...` に固定しています。
  LichtblickはMPL-2.0で提供されています。
- 8091と8766は、必要な場合だけLAN/Tailscale側へ公開してください。

`foxglove_bridge` は別途起動します。NUCでは既存の
`~/sirius_jazzy_ws/bash/startup_bash/start_foxglove.sh` が8766番を使用します。
標準の8765番はDartsRobotの監視画面が使用しているため、既定では8766番です。

#### 接続設定

`3D` ページ左側の設定で、Viewer URL、bridge URL、自動接続を変更できます。
設定はブラウザのlocalStorageに保存されます。初期値は次の環境変数でも変更できます。

- `SIRIUS_LICHTBLICK_URL`: ViewerのURL
- `SIRIUS_FOXGLOVE_WS`: bridgeのWebSocket URL
- `SIRIUS_LICHTBLICK_VERSION`: 画面に表示するバージョン情報

未指定時は、Launcherへアクセスしたホスト名から `:8091` と `:8766` を組み立てます。
HTTPSでLauncherを公開する場合は、ViewerもHTTPS、bridgeはWSSにしてください。

画面の「Viewer」はiframeの読み込み完了、「Bridge」はブラウザからWebSocketで
到達確認した結果です。Lichtblick内部の接続状態やトピック受信状態は、
クロスオリジンiframeの外から取得できないため区別して表示しています。
表示状態が「表示準備完了」でも、Lichtblick内のData sourceが未接続の場合は、
Lichtblickの接続画面で `foxglove_bridge` を選んでください。

Lichtblickのテーマは上流の設定に従うため、SiriusLauncherのダークテーマとは
一致しない場合があります。iframe内部のCSSや設定はSiriusLauncherから変更しません。

#### 表示トラブルの確認

1. `docker ps` で `sirius-lichtblick` がUpか確認する。
2. ブラウザで `http://<ホスト>:8091/` を直接開く。
3. bridgeが `ss -ltnp | grep 8766` で待受けているか確認する。
4. Viewer設定のbridge URLが、ブラウザから到達できるホスト名か確認する。
5. `ros2 topic list` で `/velodyne_points`、`/map` などが存在するか確認する。
6. 地図や点群が無い場合は、Lichtblickの `Display frame` とトピック表示を確認する。

### 現在のTFツリー

TF Treeでサーバー起動時のROS Domainの `/tf` と `/tf_static` を受信します。
ROS設定を読み込んだ共有の受信プロセスを使い、約1秒ごとに画面を更新します。
フレームを選ぶと親子関係、XYZ・RPY・Quaternion、受信Hz、最終受信、配信元を表示します。
検索、ドラッグ移動、拡大・縮小、全体表示が使えます。TFの配信や設定変更は行いません。
「階層」と「横並び（ROS風）」を切り替えられます。横並びは親から子へ左→右に矢印で接続します。
表示形式はブラウザに保存され、切り替えても選択中のフレームとTF情報は維持します。

静的TFはTransient Localで取得し、再配信がなくても保持します。
動的TFは3秒以上未受信なら更新停止として表示し、60秒で除外します。
複数の親・配信元が同じ子フレームを使う場合や循環も表示します。
他のタブを表示中も監視し、競合・循環があるとTF Treeアイコン右上に対象フレーム数の
赤バッジを表示します。TF Tree内のエラー欄にはフレーム名と配信元を表示し、選択すると
詳細を確認できます。更新停止した動的TFは現在の競合判定から外します。
監視データを取得できない間はバッジを「?」にして、現在の状態が未確認であることを示します。
JazzyのMessageInfoで配信元GIDが得られない環境では、配信元はトピックの候補一覧です。
この場合、同じ親子間にある複数配信元の競合は個別に識別できません。
設定画面のDomain変更は今後起動するコマンド向けで、監視対象Domainは切り替わりません。

## 機能

- **ワンクリック起動**: エイリアスで定義されたコマンドをボタンで簡単に起動
- **自動エイリアス読み込み**: `.bash_alias2`ファイルから自動的にエイリアスを読み込み
- **グループ分け**: `# GROUP: グループ名`コメントでボタンをカテゴリ別に整理
- **グループ説明**: `# GROUP_DESC: 説明`をセクション上部に表示（複数行指定可）
- **プロセス管理**: 個別再起動、停止、全停止、稼働中プログラムの一括再起動
- **起動入力の復元**: 一括再起動時にROS Domain IDと起動コマンドを復元可能
- **稼働時間**: 稼働中プログラムとプロセス一覧に経過時間を表示
- **ウィンドウフォーカス**: 起動中のターミナルウィンドウに切り替え可能
- **プロセス停止**: 各プロセスを個別に停止、または全停止
- **ステータス表示**: 各プロセスの起動状態をリアルタイムで表示
- **Terminator対応**: Terminatorターミナルエミュレータを使用

## Web版のログから対話入力

「3D地図で自己位置推定（実機・録画は別）」プリセットは、車輪・LiDAR・IMU・
ロボットTF・FAST-LIO2・3D地図照合・Foxgloveを起動します。
`loc3d` のログで地図番号を入力すると、そのPLY地図も `/localization/map` に配信します。
Foxgloveの表示フレームは小文字の `map` です。実機では高さ・傾きも補正します。
初期位置・向きが地図の開始位置と異なる場合は `/initialpose` を設定してください。
起動中の自己位置推定は継続します。手押し検証の録画は準備後に `record_loc3d` で開始します。

THETAの取得・校正・プレビュー・点群色付けは、路面マッピングの流れに合わせて「オフライン・マッピング」タブに置いています。3D地図専用の録画・準備確認・生成・表示は「3次元地図生成・自律移動」タブにまとめています。現在のBAG後処理は「3D地図の録画・後処理」の `offline_3dmap` を使います。TF蓄積の `theta_bag_to_ply`、KISS-ICP、RTAB-Mapによる従来の処理と `pub_3dmap` は「従来方式の3D地図生成」にあります。録画の `record_3dmap` / `record_3dmap_sim` は重複表示せず、このタブに1つずつ配置しています。

Webのボタンで起動すると、入力欄付きのログ画面が開きます。番号・パスなどを入力し、
Enterまたは「送信 / Enter」を押すと、起動したコマンドへ入力されます。
空欄のEnterも送信できるため、地図選択の既定値も使えます。
「Ctrl+C」ボタン、または入力欄でCtrl+Cを押すと割り込みを送れます。
ログを閉じてもコマンドは続行し、「起動中」の「ログ」から再び開けます。

例: `view_3dmap` を起動 → ログの候補一覧を確認 → 番号を入力してEnter。
`offline_3dmap` のbagパス入力も同じ操作です。

Web起動はPTYを使い、Terminatorのない環境でも対話できます。ログは自動更新され、
Webサーバー再起動後も実行中のコマンドへ再接続できます。
更新前に起動したコマンドやデスクトップ版の起動はログ閲覧のみです。
それらへのWeb入力が必要な場合、そのコマンドをWebから再起動してください。
画面はテキストログと行入力に対応し、全画面の端末アプリの画面描画には対応していません。

## 必要な依存関係

```bash
# PySide6（既にインストール済み）
python3 -m pip install PySide6 --break-system-packages

# Terminator（ターミナルエミュレータ）
sudo apt install terminator

# wmctrl（ウィンドウフォーカス機能用、オプション）
sudo apt install wmctrl
```

## エイリアスファイルの書き方

`.bash_alias2`ファイルで`# GROUP: グループ名`を使ってグループ分けできます：

```bash
# GROUP: センサー・ハードウェア
# GROUP_DESC: 実機センサーのドライバーを個別に起動します。

# Roboteq起動(udevルール設定済み前提)
alias roboteq='src && ros2 launch roboteq_ros2_driver roboteq_ros2_driver.launch.py'

# IMU起動(udevルール設定済み前提)
alias imu='src && ros2 launch sirius_navigation witmotion_hwt905.launch.py'

# GROUP: シミュレーション

# Simulation起動
alias sim='src && ros2 launch sirius_description sim_with_ui.launch.py'

# GROUP: ナビゲーション

# Nav2起動(既存MAP)
alias nav2='src && ros2 launch nav2_bringup bringup_launch.py params_file:=...'
```

**ルール**:
- `# GROUP: グループ名` でグループを定義
- `# GROUP_DESC: 説明` でグループ上部の説明を定義（繰り返すと改行で連結）
- その後のaliasは同じグループに属する
- alias行の前のコメントが説明として表示される
- `alias install_packages`など、初回のみ実行するものは自動的に除外される

## 使用方法

### 1. 起動

```bash
# ワークスペースのルートディレクトリから
cd ~/sirius_jazzy_ws
python3 other_programs/sirius_launcher/sirius_launcher.py
```

または実行権限を付与して直接実行：

```bash
chmod +x other_programs/sirius_launcher/sirius_launcher.py
./other_programs/sirius_launcher/sirius_launcher.py
```

### 2. エイリアス登録（推奨）

`.bashrc`または`.bash_alias2`に追加：

```bash
alias launcher='python3 ${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher/sirius_launcher.py'
```

その後：

```bash
launcher
```

## 使い方

1. **起動**: 各ボタンをクリックするとTerminatorウィンドウが開き、対応するROSノードが起動
2. **ウィンドウ表示**: 起動中のプロセスのターミナルにフォーカス
3. **停止**: 個別のプロセスを停止、または「全て停止」ボタンで全プロセスを停止
4. **ステータス確認**: 各行の「起動中」「停止中」でステータスを確認

## エイリアスの追加・変更

1. `.bash_alias2`ファイルを編集
2. 必要に応じて`# GROUP:`でグループを定義
3. 新しいaliasを追加
4. ランチャーを再起動すると自動的に反映

例：
```bash
# GROUP: 新しいグループ

# 新しいノード起動
alias newnode='src && ros2 launch my_package my_launch.py'
```

## 注意事項

- 起動中のプロセスボタンは無効化され、再度起動できません
- プロセスが終了すると自動的にボタンが再度有効になります
- ウィンドウフォーカス機能は`wmctrl`がインストールされている場合のみ動作します
- 各プロセスは独立したTerminatorウィンドウで実行されます
- エイリアスファイルは`~/sirius_jazzy_ws/bash/.bash_alias2`を参照します

## トラブルシューティング

### Terminatorが起動しない
Terminatorをインストールしてください：
```bash
sudo apt install terminator
```

### ウィンドウフォーカスが動作しない
`wmctrl`をインストールしてください：
```bash
sudo apt install wmctrl
```

### エイリアスが読み込まれない
- `.bash_alias2`ファイルが`~/sirius_jazzy_ws/bash/`に存在するか確認
- エイリアスの形式が正しいか確認（`alias name='command'`）
- グループ定義が`# GROUP: グループ名`の形式か確認

### ボタンを押してもターミナルが開かない
- Terminatorが正しくインストールされているか確認
- コマンドが正しいか確認（ターミナルから手動で実行してテスト）

## 使用方法

### 1. 起動

```bash
# ワークスペースのルートディレクトリから
cd ~/sirius_jazzy_ws
python3 other_programs/sirius_launcher/sirius_launcher.py
```

または実行権限を付与して直接実行：

```bash
chmod +x other_programs/sirius_launcher/sirius_launcher.py
./other_programs/sirius_launcher/sirius_launcher.py
```

### 2. エイリアス登録（オプション）

`.bashrc`または`.bash_alias2`に追加：

```bash
alias launcher='python3 ${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher/sirius_launcher.py'
```

その後：

```bash
launcher
```

## 使い方

1. **起動**: 各ボタンをクリックするとターミナルウィンドウが開き、対応するROSノードが起動
2. **ウィンドウ表示**: 起動中のプロセスのターミナルにフォーカス
3. **停止**: 個別のプロセスを停止、または「全て停止」ボタンで全プロセスを停止
4. **ステータス確認**: 各行の「起動中」「停止中」でステータスを確認

## ボタン一覧

### センサー・ハードウェア
- **Roboteq**: モータコントローラー
- **Velodyne**: LiDARセンサー
- **Hokuyo**: レーザースキャナー
- **IMU**: IMUセンサー

### シミュレーション
- **Simulation**: Gazeboシミュレーション

### ナビゲーション
- **Sensor Fusion**: センサーフュージョン (EKF)
- **Sensor Fusion + IMU**: センサーフュージョン + IMU自動起動
- **Nav2**: Nav2（既存MAP使用）
- **SLAM Toolbox**: SLAMマッピング

### ユーティリティ
- **Sirius Controller**: キーボード制御

## 注意事項

- 起動中のプロセスボタンは無効化され、再度起動できません
- プロセスが終了すると自動的にボタンが再度有効になります
- ウィンドウフォーカス機能は`wmctrl`がインストールされている場合のみ動作します
- 各プロセスは独立したターミナルウィンドウで実行されます

## トラブルシューティング

### ウィンドウフォーカスが動作しない
`wmctrl`をインストールしてください：
```bash
sudo apt install wmctrl
```

### ボタンを押してもターミナルが開かない
`gnome-terminal`が利用可能か確認してください。別のターミナルエミュレータを使用している場合は、スクリプト内の`gnome-terminal`を変更してください。

### プロセスが停止しない
「全て停止」ボタンを使用するか、Monitorのプロセス詳細から「停止」を実行してください。
PIDファイルがないWeb起動セッションも、起動時の `--name` から検出して停止します。

### ROS_DOMAIN_ID がランチャー経由で反映されない

原因: ランチャーは Terminator を起動するときに `bash --rcfile <一時スクリプト>` を使います。`--rcfile` を使うと通常の `~/.bashrc` は読み込まれず、一時スクリプトだけが初期化ファイルとして読まれるため、そこでエクスポートしている環境変数（例: `ROS_DOMAIN_ID`）が反映されないことがあります。また、ランチャー自身が GUI やデスクトップから起動されている場合は、親プロセスに `ROS_DOMAIN_ID` が設定されていないため継承されないケースがあります。

対処法:

- ランチャーを `ROS_DOMAIN_ID` を設定した端末から起動する（例: `export ROS_DOMAIN_ID=42` の後で `launcher` を起動）。この場合ランチャーが持つ環境が子プロセスに継承されます。
- ランチャー側で一時スクリプトの先頭に `source ~/.bashrc` を試みるように修正しました（もし `~/.bashrc` にエクスポートがあれば読み込まれます）。さらに、ランチャーのプロセス環境に `ROS_DOMAIN_ID` が含まれていれば、その値を一時スクリプトに明示的に `export ROS_DOMAIN_ID=...` するようにしています。
- それでも確実に固定したい場合は `~/.profile` や `~/.bashrc` に `export ROS_DOMAIN_ID=...` を書いて、デスクトップ起動時にも確実に環境が設定されるようにしてください。

テスト方法（手動）:

1. ターミナルで `export ROS_DOMAIN_ID=99` を実行
2. 同じターミナルからランチャーを起動 (`launcher` など)
3. ランチャーのボタンでノードを起動し、起動したターミナル内で `echo $ROS_DOMAIN_ID` を確認 -> `99` が返る

ご不明な点があれば、使っている起動方法（端末から起動／メニューから起動など）を教えてください。

## バッテリー・駆動電源の表示

画面上部のバッテリー表示は、未取得時も残ります。押すと電圧・残量％・電流・温度・
非常停止状態・取得元・最終受信を確認できます。`sensor_msgs/msg/BatteryState` と
`roboteq_ros2_driver/msg/RoboteqStatus` を同じROS Domainで自動検出します。
新鮮な標準BatteryStateを優先し、標準データがなければRoboteqの値を表示します。
受信が5秒以上止まると「更新停止」と表示し、古い値を受信中として扱いません。
残量％は配信値がある場合だけ表示し、電圧から推定しません。

Roboteqの電圧はモータードライバーの駆動側電源です。非常停止で駆動電源が遮断された
場合、バッテリー本体の電圧とは異なることがあります。残量％と充放電状態はこの型には
含まれません。非常停止はドライバーのfault_flagsのbit 4から表示します。
TFツリーと共通の受信プロセスで読み取り、ドライバー起動やロボットへの指令は行いません。

バッテリー本体とRoboteqを同時に表示します。本体の残量・温度と、MD側の駆動電圧・
電源電流・内部電圧・温度・非常停止・通知フラグには、それぞれ取得元と最終受信を表示します。
更新停止は取得元ごとに判定し、駆動電圧0VでもMDの非常停止情報を保持します。

バッテリーの隣には非常停止専用の表示を常設します。物理ボタンの押下推定・電子停止は赤、
電圧による判定中は黄、受信できない間は未確認として表示します。タップすると物理・電子を
分けて確認できます。物理ボタンは電圧からの推定のため、押下直後の検出には遅れがあります。
詳細画面の「電子停止 ON」「電子停止 OFF（解除）」は `/stop` に明示的なON/OFFを送ります。
解除には確認操作と新鮮な電子停止状態が必要です。送信完了と実機の状態確認は分けて表示し、
物理ボタンはこの画面から解除しません。ヘッダーの電源・停止表示は高さ32pxに統一します。

Ankerの入出力・残り時間は `/sirius/battery_status` から独立して受信します。
AC・シガー出力、USB各ポート、AC・ソーラー入力、合計入出力をWで表示します。
USBの入出力方向は未取得のため、ポート電力として表示します。積算Whではありません。
残り時間は本体の推定値のみを使用し、未提供値6553.5時間・負値・非数を除外します。
充電中は残り時間と配信時刻から計算した完了予定時刻（JST）を表示します。


### Chromeでアプリとして利用

`設定 → アプリとして使う` からインストール、または手順の表示ができます。
HTTPでNUCへ接続している場合は、Chromeの `⋮ → キャスト、保存、共有 → ページをアプリとしてインストール` を使用します。
NUC本体の `http://localhost:8090/` またはHTTPS接続では、条件を満たすとChromeのインストール案内も利用できます。
マニフェストは `/static/manifest.webmanifest`、起動先は `/#home`、表示は専用ウィンドウです。
制御APIやセンサー情報をService Workerでキャッシュしません。起動後もNUCへの接続が必要です。


### HTTPSでアプリの接続警告を解消

NUCにTailscale接続中、`python3 other_programs/sirius_launcher/enable_https.py --check` で公開予定を確認できます。
管理者がNUC上で `sudo python3 ~/sirius_jazzy_ws/other_programs/sirius_launcher/enable_https.py` を一度実行すると、Launcherを443、Lichtblickを8443、ROS WebSocketを8767でTailscale内へHTTPS公開します。
HTTP版からインストールしたアプリはHTTPへ接続し続けるため、出力されたHTTPS URLをChromeで開き、アプリを追加し直してください。
既存の他サービスを同じポートで上書きしないよう確認し、適用前のServe設定を `/var/lib/sirius-launcher/https/` に保存します。

### 物理非常停止の表示

電子非常停止はRoboteqの停止フラグ、物理非常停止は駆動電圧からの推定として別表示します。
駆動電圧8V以下で物理停止を推定し、11V以上へ戻るまで推定状態を保持します。初回8〜11Vは「電圧低下・判定中」です。
電圧がゆっくり下がるため押下直後の検出はできません。「未検出」はボタン解除の保証ではありません。受信停止・値欠落時は判定不可に戻します。
