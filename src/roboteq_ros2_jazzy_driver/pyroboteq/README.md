# Pyroboteq

Roboteq モーターコントローラとシリアル通信を行うための Python ライブラリです。

## 概要

このライブラリは、Roboteq SDC21** シリーズのモータードライバとの通信を簡単に行うためのものです。モーター制御、センサー値の読み取り、緊急停止などの機能を提供します。

## ファイル構成

| ファイル | 説明 |
|---------|------|
| `roboteq_handler.py` | Roboteq との通信を処理するメインクラス |
| `roboteq_commands.py` | Roboteq コマンド定数の定義 |
| `sample_program.py` | 使用例を示すサンプルプログラム |

## 依存関係

- Python 3.x
- `pyserial` ライブラリ

```bash
pip install pyserial
```

## クイックスタート

### 基本的な使い方

```python
from roboteq_handler import RoboteqHandler

# ハンドラーを作成
handler = RoboteqHandler(debug_mode=False)

# 接続
if handler.connect("/dev/roboteq", 115200):
    print("接続成功")
    
    # センサー値を読み取り
    voltage = handler.read_value("?V", "2")  # バッテリー電圧
    
    # モーターを動かす
    handler.send_command("!G", "1", "500")  # モーター1にパワー500
    
    # デュアルドライブモード
    handler.dual_motor_control(left_motor=300, right_motor=300)
```

## API リファレンス

### RoboteqHandler クラス

#### コンストラクタ

```python
RoboteqHandler(exit_on_interrupt=False, debug_mode=False)
```

| パラメータ | 型 | デフォルト | 説明 |
|-----------|-----|----------|------|
| `exit_on_interrupt` | bool | False | エラー時にプログラムを終了するか |
| `debug_mode` | bool | False | デバッグ情報を出力するか |

#### メソッド

##### `connect(port, baudrate=115200) -> bool`

コントローラへの接続を確立します。

| パラメータ | 型 | 説明 |
|-----------|-----|------|
| `port` | str | シリアルポート名 (例: `/dev/roboteq`, `/dev/ttyACM0`) |
| `baudrate` | int | ボーレート (デフォルト: 115200) |

**戻り値**: 接続成功時は `True`、失敗時は `False`

##### `send_command(command, first_parameter="", second_parameter="") -> None`

アクションコマンドを送信します。

```python
# モーター1にパワー500を送信
handler.send_command("!G", "1", "500")

# 緊急停止
handler.send_command("!EX")
```

##### `read_value(command, parameter="") -> str`

クエリコマンドを送信し、値を読み取ります。

```python
# バッテリー電圧を読み取り (V * 10 で返る)
voltage = handler.read_value("?V", "2")

# エンコーダカウンタを読み取り
encoder = handler.read_value("?CB", "1")
```

##### `dual_motor_control(left_motor=0, right_motor=0) -> None`

デュアルドライブモードで左右のモーターを制御します。

| パラメータ | 型 | 範囲 | 説明 |
|-----------|-----|------|------|
| `left_motor` | int | -1000 〜 1000 | 左モーターのパワー |
| `right_motor` | int | -1000 〜 1000 | 右モーターのパワー |

##### `request_handler(request) -> str`

生のコマンドを送信し、応答を取得します（汎用メソッド）。

```python
# ファームウェアIDを取得
firmware_id = handler.request_handler("?FID")
```

## 主要コマンド一覧

### アクションコマンド (`!`)

| コマンド | 説明 | 使用例 |
|---------|------|--------|
| `!G` | モーターパワー指令 | `!G 1 500` (チャンネル1にパワー500) |
| `!S` | モーター速度指令 | `!S 1 1000` (チャンネル1に速度1000) |
| `!M` | デュアルドライブ | `!M 500 500` (左500, 右500) |
| `!EX` | 緊急停止 | `!EX` |
| `!MG` | 緊急停止解除 | `!MG` |
| `!MS` | 全モーター停止 | `!MS` |
| `!C` | エンコーダカウンタ設定 | `!C 1 0` (チャンネル1を0にリセット) |
| `!CB` | ブラシレスカウンタ設定 | `!CB 1 0` |

### クエリコマンド (`?`)

| コマンド | 説明 | パラメータ | 戻り値 |
|---------|------|-----------|--------|
| `?V` | 電圧読み取り | 1=内部, 2=バッテリー, 3=5V出力 | V × 10 |
| `?A` | モーター電流 | チャンネル番号 | A × 10 |
| `?T` | 温度 | 1=MCU, 2=CH1ヒートシンク, 3=CH2ヒートシンク | °C |
| `?CB` | ブラシレスエンコーダカウンタ | チャンネル番号 | カウント値 |
| `?BS` | ブラシレスモーターRPM | チャンネル番号 | RPM |
| `?FF` | フォルトフラグ | なし | フラグ値 |
| `?FID` | ファームウェアID | なし | ID文字列 |

## sample_program.py の使い方

### 実行オプション

```bash
python3 sample_program.py [オプション]
```

| オプション | 短縮形 | デフォルト | 説明 |
|-----------|--------|----------|------|
| `--port` | `-p` | `/dev/roboteq` | シリアルポート |
| `--baud` | `-b` | 115200 | ボーレート |
| `--debug` | `-d` | False | デバッグモード |
| `--mode` | `-m` | sensors | 実行モード |
| `--power` | - | 100 | モーターテスト時のパワー値 |

### 実行モード

| モード | 説明 |
|--------|------|
| `sensors` | 基本接続とセンサー値の読み取り |
| `motor` | モーター制御テスト（⚠️ モーターが動きます） |
| `encoder` | エンコーダ値のリアルタイムモニタリング |
| `interactive` | 対話モード（コマンドを直接入力） |
| `all` | すべてのデモを実行 |

### 実行例

```bash
# センサー値を読み取り（デフォルト）
python3 sample_program.py

# 対話モードで起動
python3 sample_program.py -m interactive

# デバッグモードでセンサー読み取り
python3 sample_program.py -m sensors --debug

# エンコーダモニタリング
python3 sample_program.py -m encoder

# 別のポートを使用
python3 sample_program.py -p /dev/ttyUSB0 -m interactive
```

### 対話モード

対話モードでは、Roboteq コマンドを直接入力できます。

```
【対話モード】
コマンドを直接入力できます。
例: !G 1 100  (モーター1にパワー100)
例: ?V 2      (バッテリー電圧読み取り)
例: ?CB 1     (エンコーダ1読み取り)
'quit' または 'exit' で終了

> ?V 2
応答: V=248

> !G 1 100
応答: +

> quit
```

## udev ルールの設定

Roboteq を常に `/dev/roboteq` として認識させるには：

```bash
# /etc/udev/rules.d/99-roboteq.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="20d2", ATTRS{idProduct}=="5740", SYMLINK+="roboteq", MODE="0666"
```

設定後:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## トラブルシューティング

### 接続できない場合

1. デバイスが接続されているか確認
   ```bash
   ls -la /dev/ttyACM* /dev/ttyUSB*
   ```

2. 権限を確認
   ```bash
   sudo chmod 666 /dev/ttyACM0
   # または dialout グループに追加
   sudo usermod -a -G dialout $USER
   ```

3. デバッグモードで実行
   ```bash
   python3 sample_program.py --debug
   ```

### 値が空で返ってくる場合

- ボーレートが正しいか確認（デフォルト: 115200）
- デバッグモードで通信内容を確認
- Roboteq のシリアル設定が正しいか確認

## 参考リンク

- [Roboteq Controllers User Manual](https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file)
