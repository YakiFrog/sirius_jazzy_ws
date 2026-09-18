# move_goal ハイブリッド方式メモ（NavigateThroughPoses + NavigateToPose）

状態: **未実装（設計メモのみ）**。今は適用しない。
対象: `src/sirius/sirius_navigation/sirius_navigation/move_goal.py`

## 背景・課題

- 現状は各ウェイポイント(WP)を **`NavigateToPose` の個別ゴール**として送信し、到達判定
  (`get_arrival_threshold`, 既定 2.0m)で次WPへ切り替える。
- MPPI の参照パスは**そのWPで終わる**ため、ゴールが予測ホライズン
  `= vx_max × time_steps × model_dt`（normal: 0.9×60×0.1 = **5.4m**）内に入ると、
  `PathFollowCritic` / `PathAlignCritic` により**自然に減速**する（オーバーシュートがペナルティ）。
- 結果、WP手前 3〜4m から減速（例: 0.50 → 0.38 m/s）し、2mで次WPへ切替→再加速、を繰り返す。
  これは「ゴール＝停止点」という NavigateToPose の性質上、不可避。

### 検討したが採用しない案

- **到達 threshold を spacing 近くまで上げる**: 非停止化できるが、`threshold ≧ spacing` で
  細WPをスキップする（ユーザ指摘）。
- **全WPを NavigateThroughPoses で一括**: 中間WPを通過するため、WPに付与したタスク
  (`stop` / `wait_time` / `person_area` / `change_map` / `rotate` / `threshold`) が実行できない。

### 成立条件の整理

非停止（減速窓ゼロ）には「次のゴールが常にホライズン外」が必要:
`spacing ≧ threshold ≧ horizon`。
4m間隔・horizon 5.4m では成立しないため、**タスク無し区間だけ through-poses 化**する
ハイブリッドが唯一の両立案。

## ハイブリッド方式（採用候補）

### 考え方

- **タスク無しWPの連続区間**をひとつの `NavigateThroughPoses` ゴールとして送信 → 非停止で通過。
- **タスク付きWP**（`stop` / `wait_time>0` / `person_area` / `change_map`）でバッチを区切り、
  バッチ終端＝そのタスクWP。バッチ完了時に**従来ロジックでタスクを実行**。
- タスク実行後、次のバッチを送信。→ タスクは維持しつつ、間は流れる。

### バッチ規則

- 開始: `self.count`。
- 進める: 連続するタスク無しWPを追加。
- 終端: 最初に現れた「タスク付きWP」を**含めて**終端（＝そこで停止してタスク実行）。
  タスク付きWPが無ければ最終WPを終端。
- 上限: 安全のため最大件数（例 50）で打ち切り。
- `threshold`（個別到達距離）は through-poses では使えないため、バッチ終端WPのみ
  `NavigateToPose` で精密到達させる案も可（下記「代替案」）。

### 状態・コールバック

- `NavigateThroughPoses` 用の `ActionClient` を追加（`navigate_through_poses`）。
- `send_goal()` を分岐: `self.through_poses` が有効なら `_send_through_batch()`、それ以外は従来。
- バッチ送信時に `self._batch_start` / `self._batch_end` を保持。
- `get_result_async()` のコールバック `_through_result_callback` を追加:
  - 成功: `last_wp = waypoints[batch_end]` を見てタスク処理。
    - `change_map`: `change_map_command()`（内部で waypoints 再読込・count リセット）→ 次バッチ。
    - `person_area`: `hold_at_person_area(batch_end)`。
    - `wait_time>0`: `publish_stop_command(True)` → `_waiting_until` 設定 → `get_position` で再開。
    - `stop`: `publish_stop_command(True/False)`。
    - それ以外（最終WP）: `count = batch_end + 1` → 次バッチ or 終了/loop。
  - 失敗/キャンセル: ログ。必要なら従来の per-WP `NavigateToPose` にフォールバック。

### `get_position()` の扱い

- through-poses 中は**WPごとの到達判定・10秒ごとの再送をスキップ**（アクション結果で進む）。
  - 到達判定ブロック（`if self.distance < threshold_distance:`）と
    `elif self.loop_count % 100 == 0:`（再送）を `if not self.through_poses:` でガード。
  - 待機(`_waiting_until`)・pause/cancel・person_area の状態処理は従来通り残す。
- ログの `[WP:n]` 表示は「バッチ先頭/終端」を出すように調整。

### pause / resume / cancel 連携

- `cancel_current_goal` / `pause_current_goal` / `resume_current_goal` は
  `self.current_goal_handle`（NavigateToPose/ThroughPoses どちらのハンドルでも可）を対象に動く。
- `resume_current_goal` → `send_goal()` が through-poses バッチを再送するようにする。

### CLI / 起動

- `--through-poses`（または `--flow`）フラグを追加（既定 False で後方互換）。
- 環境変数 `SIRIUS_WAYPOINTS` は従来通り。
- 起動例:
  ```
  ros2 run sirius_navigation move_goal --through-poses --waypoints atc_1F
  ```

## 代替案（バッチ終端だけ NavigateToPose で精密停止）

- タスク無し区間を through-poses で通過し、**タスクWPの手前でバッチを終端**。
- その後、タスクWPを `NavigateToPose`（既存の `threshold` 尊重）で精密に到達 → タスク実行。
- `person_area`（2m手前で停止）等の閾値挙動を厳密に保てる。

## エッジケース

- タスク付きWPが連続する場合: 各バッチが1WPになり、実質 per-WP 動作（従来同等）。
- 全WPがタスク無し: 1バッチで全通過（完全非停止、最終WPのみ停止）。
- ループ(`--loop`): バッチ完了→最終WPで `count=0` に戻して次バッチ。
- `change_map`: バッチ終端で地図切替→WP再読込→新WPでバッチ再開。
- 経路が再計画される場合（1Hz）: through-poses の BT でも `ComputePathThroughPoses` が再計画。
- コントローラ選択: through-poses 用BT `navigate_through_poses_w_replanning_and_recovery.xml`
  には現状 `Switch2`(WaitPath/FollowPath) が無い。**wait系モードで使うなら要改修**。

## 関連ファイル

- `src/sirius/sirius_navigation/sirius_navigation/move_goal.py`
  - `send_goal()` / `get_position()` / `goal_response_callback()` /
    `change_map_command()` / `hold_at_person_area()` / `get_arrival_threshold()`
- `src/navigation2/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`
- `src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
  （待機/Clear の独自改修は to_pose 側のみ）
- MPPI: `nav2_mppi_controller/src/critics/path_follow_critic.cpp`, `path_align_critic.cpp`,
  `tools/utils.hpp` の `findPathFurthestReachedPoint`
- 到達/タスク属性: `WAYPOINT_ATTRIBUTE_DEFAULTS`（`rotate`/`stop`/`wait_time`/`change_map`/`threshold`/`person_area`）

## 補足（非停止化の代替レバー）

ハイブリッドを使わない場合の簡易策（参考）:

- NORMAL の `time_steps` を下げる（60→40 で horizon 3.6m、減速窓 ≒1.6m）。
  回避の先読みは減る。
- 到達 `threshold` を spacing 直前まで上げる（4m間隔で 3.0m 程度）。曲がり角でスキップ注意。
- いずれも「真の非停止」にはならない（WPごとに減速は残る）。
