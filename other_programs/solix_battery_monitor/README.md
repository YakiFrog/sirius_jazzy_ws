# Anker SOLIX C300 BLE Battery Monitor

Anker SOLIX C300 ポータブル電源と Bluetooth Low Energy (BLE) で直接ローカル通信し、バッテリー残量や入出力電力を取得・監視するプログラムです。
非公式の [flip-dots/SolixBLE](https://github.com/flip-dots/SolixBLE) ライブラリを使用しています。

## 構成
- `solix_monitor.py`: 本体スクリプト (Python 3)
- `run.sh`: 仮想環境の Python を用いて実行するためのランナー
- `venv/`: 依存ライブラリ (`bleak`, `SolixBLE`) がインストールされた仮想環境

## 準備 (実機の操作)
1. **IoT (Bluetooth) を有効化**:
   Anker C300 本体の IoTボタン（またはBluetoothボタン）を押し、ディスプレイ上の Bluetooth アイコンが点滅または点灯していることを確認してください。
2. **他の接続を遮断**:
   スマートフォンアプリ等が接続していると、BLE接続が競合してこのプログラムから繋がらなくなります。アプリは一旦終了させてください。

## 使い方

このディレクトリ (`/home/kotantu-desktop/sirius_jazzy_ws/other_programs/solix_battery_monitor`) に移動して、以下のコマンドを実行します。

### 1. 周辺の Solix デバイスをスキャンする
```bash
./run.sh --scan
```
スキャンに成功すると、デバイス名と MAC アドレスが表示されます。
```
Found 1 Solix device(s):
  [0] Name: Solix C300  |  MAC Address: XX:XX:XX:XX:XX:XX
```

### 2. 特定のデバイスに接続して監視する
表示された MAC アドレスを指定して実行します。
```bash
./run.sh --mac XX:XX:XX:XX:XX:XX
```
5秒おきにステータス（バッテリー残量 %、AC/DC 出力、入力電力、温度など）を自動更新して表示します。

```
2台目のバッテリーの MAC Address: F4:9D:8A:57:90:E8
1台目のバッテリーの MAC Address: 
```

### 3. 一度だけステータスを表示して終了する (Single-Shot)
```bash
./run.sh --mac XX:XX:XX:XX:XX:XX --once
```

### 4. 更新インターバルを変更する (例: 10秒おき)
```bash
./run.sh --mac XX:XX:XX:XX:XX:XX --interval 10
```

### 5. デバッグログを出力して接続フローを確認する
接続の交渉フェーズや受信パケットの詳細をダンプする場合は、`--debug` を付与します。
```bash
./run.sh --mac XX:XX:XX:XX:XX:XX --debug
```

## 注意事項
- Bluetooth 通信は物理的な距離や障害物に影響されます。なるべく PC の近くに C300 を置いて実行してください。
- 接続中にエラーが出る場合は、一度 C300 本体の Bluetooth を OFF にしてから再度 ON にして試してみてください。
