# Siriusの速度指令経路

Nav2の制御・リカバリー・ドッキング指令は、次の経路で出力します。

`cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel_nav_safe → twist_mux → cmd_vel → Roboteq`

Webテレオペは `cmd_vel_web → twist_mux`（優先度105）、通常の手動操作は
`cmd_vel_teleop → twist_mux`（100）、Nav2は10です。`/stop`のロックは255です。

衝突監視はNav2の経路を監視します。この配置ではWebの手動指令は衝突監視を通りません。
物理・電子緊急停止の扱いは変えません。

`nav2_mux.launch.py`はNav2の起動範囲内だけで`cmd_vel`を`cmd_vel_nav`へ
リマップします。ドッキングサーバーも最終出力を直接配信しません。
実機・シミュレーションの通常起動、SLAM起動、3D Nav2、Unity、Assisted Teleopの
Nav2起動はこの共通launchを利用します。上流Nav2のコードは変更していません。

Collision Monitorの`enable: false`は無効化の指定として機能しないため削除しました。
起動・停止はlifecycleで管理します。ローカル・グローバルのinflation_radiusは0.775です。

Launcherは使う速度トピックだけのPublisherを作ります。twist_mux経由を選択しているときに
`cmd_vel`へ別ノードが直接配信している場合は、操作の有効化を拒否します。
従来の直接出力は「自動検出・直接出力あり」で明示的に選ぶ場合のみ使います。

## 反映と確認

設定変更後はNav2・twist_muxを停止状態で再起動します。新しいlaunchを追加した環境では
`sirius_navigation`の再ビルド（symlink-install）も必要です。
今回のDesktop・NUCでは既存のsymlink-installに新launchのリンクを追加して反映しました。
AMCLを再起動した場合は現在位置・向きを再設定してください。

`cmd_vel`のPublisherはtwist_muxだけ、`cmd_vel_nav_safe`のPublisherはcollision_monitor、
そのSubscriberはtwist_mux、ドッキングサーバーの出力は`cmd_vel_nav`になっていることを確認します。
単なるPublisher数に加え、ノード名・配信先・lifecycleの状態も確認します。
