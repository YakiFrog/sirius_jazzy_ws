# オフライン地図の使用方法

このディレクトリには、rviz_satelliteでオフライン地図を使用するためのツールが含まれています。

## ファイル構成

- `download_tiles.py` - 地図タイルをダウンロードするスクリプト
- `tile_server.py` - ローカルタイルサーバー
- `demo.rviz` - RViz設定ファイル
- `demo.launch.xml` - 起動ファイル

## セットアップ手順

### 1. 依存関係のインストール

```bash
pip3 install requests tqdm
```

### 2. 地図タイルのダウンロード

`download_tiles.py`を編集して、対象エリアの座標を設定します:

```python
# 中心座標を設定
LAT_CENTER = 35.6812  # あなたのエリアの緯度
LON_CENTER = 139.7671  # あなたのエリアの経度

# エリアの範囲を設定（度単位）
LAT_DELTA = 0.005  # ~500m
LON_DELTA = 0.005  # ~500m

# ズームレベルを設定（大きいほど詳細、タイル数も増加）
zoom_levels = [16, 17, 18]
```

スクリプトを実行:

```bash
python3 download_tiles.py
```

タイルは`./tiles/`ディレクトリにダウンロードされます。

### 3. ローカルタイルサーバーの起動

```bash
python3 tile_server.py 8000 ./tiles
```

サーバーは `http://localhost:8000` で起動します。

### 4. RViz設定の変更

`demo.rviz`ファイルの`Object URI`を以下のように変更:

```yaml
Object URI: http://localhost:8000/{z}/{x}/{y}.png
```

### 5. RVizの起動

```bash
ros2 launch demo.launch.xml
```

## 注意事項

### タイル使用に関する注意

1. **OpenStreetMap**: 
   - 非商用利用推奨
   - Tile Usage Policy に従ってください
   - https://operations.osmfoundation.org/policies/tiles/

2. **Google Maps**:
   - 利用規約に注意（商用利用には制限があります）
   - 大量ダウンロードは避けてください

### タイルサイズの見積もり

- Zoom 16: ~1km² あたり ~16 タイル
- Zoom 17: ~1km² あたり ~64 タイル
- Zoom 18: ~1km² あたり ~256 タイル

1km²エリアでzoom 16-18をダウンロードすると、約336タイル、数MBになります。

## トラブルシューティング

### タイルが表示されない

1. タイルサーバーが起動しているか確認
2. RVizの設定でURLが正しいか確認
3. ブラウザで `http://localhost:8000/{z}/{x}/{y}.png` にアクセスして確認

### ダウンロードが遅い

- `download_tiles.py`の`time.sleep(0.1)`の値を増やす（サーバーへの負荷軽減）
- ズームレベルを減らす
- エリアを小さくする

## 別の方法: MapProxyを使用

より高度な機能が必要な場合は、MapProxyの使用を検討してください:

```bash
pip3 install MapProxy
```

MapProxyは、タイルのキャッシング、座標系の変換、複数のソースの統合などができます。
