# Rosbag修復ツール

録画中の電源断や強制終了で正常に閉じられなかったROS 2 MCAP bagを、元データを変更せずに復旧するGUIです。

## 復旧内容

1. 選択したbagと空き容量を読み取り専用で診断
2. MCAPの完全に読み取れるチャンクを兄弟ディレクトリへコピー
3. MCAPのサマリーとインデックスを再生成
4. `ros2 bag reindex` で `metadata.yaml` を再生成
5. `ros2 bag info` で出力を検証

復旧先は `<元のbag名>_recovered` です。既に存在する場合は `_recovered_2` のように連番になります。

Foxgloveでは、復旧先フォルダ内の `.mcap` ファイルを開いてください。

## 起動

```bash
./run_rosbag_repair.sh
```

現在はMCAP形式に対応しています。SQLite3 (`.db3`) は対象外です。
