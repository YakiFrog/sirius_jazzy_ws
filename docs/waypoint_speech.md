# ウェイポイント到達時の音声

WaypointEditorのAdd Actionsにある「到達時の発話」へ文章を入力します。
空欄なら発話しません。保存形式は次のとおりです。

```yaml
waypoints:
  - number: 1
    x: 0.0
    y: 0.0
    angle_radians: 0.0
    speech: こんにちは
```

通常の距離による到達判定で、`/sirius/speech_text`（`std_msgs/msg/String`）へ1回送ります。
待機・人物探索中の繰り返し判定では再送せず、周回して再到達したときは再び発話します。
InitialPose指定がある場合は、到達時に送信し、その後にInitialPoseの処理を行います。
発話終了は待たず、従来の走行・待機処理を続けます。

更新版`mv_goal`は音声トピックから既存SiriusFace発話APIへの橋渡しも起動します。
同一ユーザー・ホスト・ROS Domain内で橋渡しが重複しないようロックしています。
SiriusFaceの発話サーバー（50052番）が必要です。未起動やエラーの場合はログへ警告し、
走行は継続します。失敗した文章は自動再送しません。

200文字以内の文章に対応し、制御命令との混同を避けるため角括弧タグは使えません。
`/sirius/speech_status`（String内のJSON）に`accepted`または`error`を通知します。
`accepted`は発話APIが受け付けた意味で、音声再生終了の通知ではありません。
トピックはvolatileで、橋渡しが後から起動しても過去の発話を再生しません。

既に起動している`mv_goal`にはコード更新は反映されません。次回起動時に読み込みます。
既存のルートへ発話設定を自動追加することはありません。

単独で音声の橋渡しを使う場合:

```bash
python3 -m sirius_navigation.waypoint_speech
# 別のターミナルから、同一ROS Domainで:
ros2 topic pub --once /sirius/speech_text std_msgs/msg/String "{data: 'こんにちは'}"
```

2026-10-25の実機試験では、走行用ActionClientを持たないテスト用オブジェクトで
実際の`get_position`到達処理を実行し、「こんにちは」のトピック受信・API受付・
SiriusFaceバックエンドのWAV再生開始を確認しました。実際の走行到達による確認は未実施です。
