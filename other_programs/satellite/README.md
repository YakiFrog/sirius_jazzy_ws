# RViz Satellite Demo

ROS2でrviz_satelliteを使用して地図を表示するデモです。

## 📋 目次

1. [オンライン地図の使用](#オンライン地図の使用)
2. [オフライン地図の使用](#オフライン地図の使用)
3. [地図タイプの変更](#地図タイプの変更)

---

## オンライン地図の使用

### 起動方法

```bash
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo.launch.xml
```

### 設定

`demo.rviz`の`Object URI`で地図ソースを設定:

- **OpenStreetMap** (デフォルト):
  ```
  https://tile.openstreetmap.org/{z}/{x}/{y}.png
  ```

- **Google Maps 衛星画像**:
  ```
  https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}
  ```

- **Google Maps 道路地図**:
  ```
  https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}
  ```

- **Google Maps 衛星+道路**:
  ```
  https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}
  ```

---

## オフライン地図の使用

インターネット接続なしで地図を使用する方法です。

### クイックスタート（推奨）

タイルをダウンロード済みの場合、以下のコマンド1つで起動できます:

```bash
cd ~/sirius_jazzy_ws/satellite
./launch_offline.sh
```

このスクリプトは自動的に:
1. ローカルタイルサーバーを起動
2. RVizをオフライン設定で起動
3. 終了時にサーバーを自動停止

### 手動セットアップ

#### 方法1: オフライン専用Launchファイルを使用（簡単）

```bash
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo_offline.launch.xml
```

このlaunchファイルは自動的にタイルサーバーも起動します。

#### 方法2: 個別に起動

タイルサーバーとRVizを別々に起動したい場合:

```bash
# ターミナル1: タイルサーバー起動
cd ~/sirius_jazzy_ws/satellite
python3 tile_server.py 8000 ./tiles

# ターミナル2: RViz起動
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo_offline.launch.xml
```

### 初回セットアップ手順

### 初回セットアップ手順

タイルをまだダウンロードしていない場合:

### 1. 依存パッケージのインストール

```bash
# Ubuntuのパッケージマネージャーを使用
sudo apt install python3-requests python3-tqdm

# または仮想環境を使用
python3 -m venv venv
source venv/bin/activate
pip install requests tqdm
```

### 2. 地図タイルのダウンロード

`download_tiles.py`を編集して、対象エリアを設定:

```python
# あなたのエリアの中心座標
LAT_CENTER = 35.6812  # 緯度
LON_CENTER = 139.7671  # 経度

# ダウンロード範囲（度単位）
LAT_DELTA = 0.005  # 約500m
LON_DELTA = 0.005  # 約500m

# ズームレベル（大きいほど詳細）
zoom_levels = [16, 17, 18]
```

タイルをダウンロード:

```bash
cd ~/sirius_jazzy_ws/satellite
python3 download_tiles.py
```

---

## 起動方法まとめ

### オンライン地図
```bash
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo.launch.xml
```

### オフライン地図（タイルダウンロード済み）

**方法1: 自動起動スクリプト（推奨）**
```bash
cd ~/sirius_jazzy_ws/satellite
./launch_offline.sh
```

**方法2: Launchファイル**
```bash
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo_offline.launch.xml
```

**方法3: 手動起動**
```bash
# ターミナル1
cd ~/sirius_jazzy_ws/satellite
python3 tile_server.py 8000 ./tiles

# ターミナル2
cd ~/sirius_jazzy_ws/satellite
ros2 launch demo_offline.launch.xml
```

---

## 地図タイプの変更

### Google Maps の各種地図タイプ

`demo.rviz`の`Object URI`の`lyrs`パラメータを変更:

| タイプ | パラメータ | URL |
|--------|-----------|-----|
| 道路地図 | `lyrs=m` | `https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}` |
| 地形図 | `lyrs=p` | `https://mt1.google.com/vt/lyrs=p&x={x}&y={y}&z={z}` |
| 衛星画像 | `lyrs=s` | `https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}` |
| 衛星+道路 | `lyrs=y` | `https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}` |
| ハイブリッド | `lyrs=h` | `https://mt1.google.com/vt/lyrs=h&x={x}&y={y}&z={z}` |

---

## ファイル構成

```
satellite/
├── README.md                 # このファイル
├── OFFLINE_MAP_README.md     # オフライン地図の詳細ガイド
├── demo.launch.xml           # オンライン地図用起動ファイル
├── demo_offline.launch.xml   # オフライン地図用起動ファイル（NEW!）
├── demo.rviz                 # オンライン地図用RViz設定
├── demo_offline.rviz         # オフライン地図用RViz設定（NEW!）
├── publish_demo_data.py      # デモデータパブリッシャー
├── download_tiles.py         # 地図タイルダウンローダー
├── tile_server.py            # ローカルタイルサーバー
├── launch_offline.sh         # オフライン地図自動起動スクリプト（NEW!）
└── tiles/                    # ダウンロードした地図タイル（作成後）
```

---

## トラブルシューティング

### 地図が表示されない

1. **インターネット接続を確認**（オンライン使用時）
2. **タイルサーバーが起動しているか確認**（オフライン使用時）
   ```bash
   # ブラウザでアクセスして確認
   http://localhost:8000/16/58210/25806.png
   ```
3. **RVizのトピックを確認**
   ```bash
   ros2 topic echo /fix
   ```

### 404エラー「Tile not found」が表示される

オフライン使用時に一部のタイルが見つからない場合:

**原因**: ダウンロード範囲が狭く、表示領域がカバーされていない

**解決策**: より広い範囲をダウンロード

```python
# download_tiles.pyで範囲を広げる
LAT_DELTA = 0.01  # 0.005 → 0.01 に変更（約1km）
LON_DELTA = 0.01  # 0.005 → 0.01 に変更（約1km）
```

境界部分のタイルだけ追加ダウンロードする場合:
```bash
# 再度download_tiles.pyを実行（既存タイルはスキップされます）
python3 download_tiles.py
```

### pipインストールエラー

Ubuntu 24.04以降では、システムPythonへの直接インストールが制限されています:

```bash
# 方法1: システムパッケージを使用（推奨）
sudo apt install python3-requests python3-tqdm

# 方法2: 仮想環境を使用
python3 -m venv venv
source venv/bin/activate
pip install requests tqdm

# 方法3: pipxを使用
sudo apt install pipx
pipx install requests tqdm
```

---

## 注意事項

### 地図タイル使用に関する注意

- **OpenStreetMap**: [Tile Usage Policy](https://operations.osmfoundation.org/policies/tiles/)に従ってください
- **Google Maps**: 利用規約に注意。大量ダウンロードは避けてください

### タイルサイズの目安

| ズームレベル | 1km²あたりのタイル数 |
|-------------|---------------------|
| Zoom 16 | 約16タイル |
| Zoom 17 | 約64タイル |
| Zoom 18 | 約256タイル |

1km²エリアでzoom 16-18をダウンロードすると、約336タイル、数MBになります。

---

## 詳細情報

オフライン地図の詳細な使用方法については、`OFFLINE_MAP_README.md`を参照してください。