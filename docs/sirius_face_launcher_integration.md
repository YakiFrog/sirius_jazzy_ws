# SiriusFaceとSiriusLauncherの連携案

2026-10-23にNUCの実装・稼働状態を確認した設計案。手動操作UIを実装し、NUCの実行環境に反映。実験状態との自動連動は未実装。

## 実装済みの操作

HomeにFaceカードを置き、API接続状態と現在の表情を表示する。
カードから操作パネルを開き、「通常」「笑顔」「視線」「短い字幕」「読み上げ」を
試せるようにする。日常の起動・実験中の動作確認に使う。
プロセスが存在すること、APIが応答すること、描画画面があることは分けて確認する。

## 次に加えるもの

ROSBAG記録や実験の状態変化を、顔・短い字幕で知らせる。
バッテリー低下やセンサー異常も通知対象にできる。自動連動は初期状態でOFFとし、
対象イベントを選べるようにする。同じ異常を繰り返し発話しない。
会話中の表情や手動操作を上書きし続けないよう、通知後の復帰と優先度を設計する。

現在NUCには1台のモニターがあり、LauncherとFaceは別ワークスペースにある。
画面表示の切り替えを加える場合、FaceアプリとLauncherの既存ウィンドウを再利用する。

## 接続方法と確認済みの機能

スマホ・PCのブラウザ → HTTPSのLauncher → NUC内のFace API の構成を使う。
ブラウザからFaceの制御ポートを直接公開する必要はない。

- `127.0.0.1:50051` の `face_control.FaceService`
- `GetStatus`: 実機から normal / standard / standard の応答を確認済み。
- `SetExpression`, `SetStyles`, `LookAt`, `ToggleAutoMotion`: 表情・視線・自動動作。
- `ShowText`, `SetSystemError`: 字幕・通知表示。
- `PythonControlService.Speak`: 発話APIの定義あり。今回、発話テストは実行していない。

API定義: `~/sirius_face_anim2/main/proto/face_control.proto`。
Launcher側は既存認証を共有するサーバーAPIを設け、短いタイムアウトと
許可した操作・値の範囲だけを扱う。Face停止時にもLauncherの他機能を待たせない。

## NUCの全画面自動起動

自動ログインは既に有効。`~/.config/autostart/sirius-console.desktop` から
`bash/startup_bash/start_sirius_console.sh` を起動する。
画面名の変更に依存せず、専用ブラウザのプロファイル・URL・ウィンドウIDで既存画面を
特定して全画面化する。再実行時も同じウィンドウを使う。
Web画面の自動起動設定であり、実験プログラムを追加起動する設定ではない。
実機再起動は行わず、現行セッションで起動経路を2回実行して全画面状態・重複なしを確認した。

## 起動元と実装

NUCの `~/Desktop/sirius_face.desktop` を `gio launch` で起動する。
Execはビルド済みAppImageを指し、開発用コマンドに置き換えない。起動済みや起動待機中なら重複起動しない。
`face_bridge.py` が同じLauncher認証で状態取得と許可した手動操作だけを公開する。
`face_worker.py` はFaceのvenvで実行し、ROSやLauncherのPython依存へ影響を与えない。
Homeまたは操作パネルが表示されている間だけ約3秒ごとに確認し、サーバーも状態を2秒間共有する。
音声APIの接続確認は既存の発音しない `[ping]` を使う。手動の読み上げ欄では角括弧タグを拒否し、
移動指示・LLM制御の経路へ送らない。Faceの停止時には操作を無効にし、エラー表示から再確認できる。

## 顔画面とLauncherの切り替え

- Launcherのヘッダーの顔アイコンで、NUCの既存Faceウィンドウを前面にする。
- Face表示中、右上にマウスを寄せたときだけ「Launcherへ戻る」を表示する。マウスを離すと約1.5秒で隠れる。
- スマホからはHome → SiriusFace → 操作で「顔画面を表示」「Launcher画面を表示」を選べる。
- NUC本体で `Ctrl+Alt+F` は両画面を切り替え、`Ctrl+Alt+W` はLauncherを表示する。

`screen_switch.py install` でGNOMEのユーザーショートカットと
`~/.config/autostart/sirius-screen-switch.desktop` を登録する。
既存の他のショートカットを保存し、再実行でも重複させない。
`screen_switch.py overlay` はGTK3/X11の小さな常駐表示で、ロックにより1個だけ起動する。
NUCの既存X11環境を利用する。FaceのAppImageには変更を加えない。
画面切り替えはFace APIが応答しなくても実行できる。
対象ウィンドウが閉じている場合はエラーを表示するため、Homeから顔を起動する。
次回ログインから戻るボタンも自動起動する。今回の確認にNUCの再起動は含めない。
