# Sirius ROS2 Launch Manager

ROS2ノードやlaunchファイルをGUIボタンから起動できるランチャーアプリケーション

## 機能

- **ワンクリック起動**: エイリアスで定義されたコマンドをボタンで簡単に起動
- **自動エイリアス読み込み**: `.bash_alias2`ファイルから自動的にエイリアスを読み込み
- **グループ分け**: `# GROUP: グループ名`コメントでボタンをカテゴリ別に整理
- **プロセス管理**: 起動中のプロセスは再起動不可（ボタン無効化）
- **ウィンドウフォーカス**: 起動中のターミナルウィンドウに切り替え可能
- **プロセス停止**: 各プロセスを個別に停止、または全停止
- **ステータス表示**: 各プロセスの起動状態をリアルタイムで表示
- **Terminator対応**: Terminatorターミナルエミュレータを使用

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
