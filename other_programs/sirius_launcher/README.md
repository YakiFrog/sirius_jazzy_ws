# Sirius ROS2 Launch Manager

ROS2ノードやlaunchファイルをGUIボタンから起動できるランチャーアプリケーション

## Sirius Control Console（Web版）

左のアイコンから Home / Terminal / Monitor / TF Tree / 設定を切り替えます。
Homeではプリセット・個別プログラムを起動し、縦一列のプログラム一覧からログを確認できます。
停止・再起動は確認メッセージの承認後に実行します。Terminalではプログラムのログと対話入力、
独立したシェルのタブを使えます。シェルの「×」はそのセッションを終了します。
入力欄・出力欄でCtrl+Cを押すか、入力欄の横の「Ctrl+C」ボタンを押すと、
現在のタブの実行中コマンドへ割り込みを送ります。テキスト選択中のCtrl+Cはコピーです。
これらの画面変更は、Webサーバーを再起動せずページの再読み込みで反映できます。

MonitorにはホストのCPU・メモリ・ディスク・CPU温度、コア別負荷、過去2分の推移、
ネットワーク転送速度、プロセス一覧を表示します。Sirius / ROS関連と全プロセスを切り替え、
検索・CPU/メモリ順の並べ替え・詳細確認ができます。
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
JazzyのMessageInfoで配信元GIDが得られない環境では、配信元はトピックの候補一覧です。
この場合、同じ親子間にある複数配信元の競合は個別に識別できません。
設定画面のDomain変更は今後起動するコマンド向けで、監視対象Domainは切り替わりません。

## 機能

- **ワンクリック起動**: エイリアスで定義されたコマンドをボタンで簡単に起動
- **自動エイリアス読み込み**: `.bash_alias2`ファイルから自動的にエイリアスを読み込み
- **グループ分け**: `# GROUP: グループ名`コメントでボタンをカテゴリ別に整理
- **グループ説明**: `# GROUP_DESC: 説明`をセクション上部に表示（複数行指定可）
- **プロセス管理**: 起動中のプロセスは再起動不可（ボタン無効化）
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
「全て停止」ボタンを使用するか、ターミナルから直接Ctrl+Cで停止してください。

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
