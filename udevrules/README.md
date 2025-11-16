# Udevルール設定

このディレクトリには以下のデバイス用のudevルールファイルが含まれています：
- `99-hokuyo.rules` - Hokuyoレーザーセンサー用
- `99-ichimill.rules` - Ichimillデバイス用
- `99-roboteq-serial.rules` - Roboteqモータコントローラー用
- `99-wt905.rules` - WT905 IMUセンサー用

## udevルールの適用手順

### 1. ルールファイルをシステムディレクトリにコピー
```bash
sudo cp ${HOME}/sirius_jazzy_ws/udevrules/*.rules /etc/udev/rules.d/
```

### 2. udevルールを再読み込み
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 確認
```bash
# コピーしたファイルが存在するか確認
ls -la /etc/udev/rules.d/ | grep -E "(hokuyo|ichimill|roboteq|wt905)"

# デバイスの認識確認（デバイス接続後）
ls -la /dev/tty* | grep -E "(roboteq|hokuyo|wt905)"
```

### 4. デバイスの再接続
udevルールを適用するため、対象デバイスを一度取り外してから再接続するか、システムを再起動してください。

## 注意事項
- udevルールの変更後は必ずリロードが必要です
- デバイスのベンダーIDやプロダクトIDが変更された場合は、ルールファイルの更新が必要です
- 権限の問題でデバイスにアクセスできない場合は、ユーザーを適切なグループに追加してください