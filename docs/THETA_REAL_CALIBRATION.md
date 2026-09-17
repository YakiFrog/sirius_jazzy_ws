# THETA 実機マウント校正メモ

RICOH THETA S を HDMI→USB キャプチャで取り込み、オフライン路面/セマンティックマッピングに
使うための**実機マウント外部パラメータ**の記録。レンズ内部パラメータ（center/focal/歪み）は
工場校正取得済みで、ここでは扱わない。

## 実測値（2026-09-17）

| 項目 | 値 | 状態 |
| :--- | :--- | :--- |
| カメラ光軸の地上高 (z) | **1.135 m** | 実測（暫定1.1から更新） |
| 前後/左右の傾き (pitch / roll) | 未計測（仮値 0°） | 要実測 |
| yaw | 0°（光軸=ロボット前方向を仮定） | 要確認 |

> 高さの測り方: ロボット接地面（base_footprint 相当）から THETA のレンズ光軸中心までを
> メジャー等で実測。個体・取付金具で変わるため、再取付時は必ず測り直す。

## パラメータの反映先

カメラ姿勢の解決優先順位（`theta_bev_node.resolve_pose`）:

1. **TF** `sirius3/base_footprint <- sirius3/theta_link`
2. 姿勢トピック `/theta/mount_pose`（アプリ同期・未使用）
3. **校正YAML** `config/theta_calibration_real.yaml`（最終フォールバック）

実機は現在TFを配信していないため、プレビューでは **(3) YAML** が使われる。録画時の
検証スクリプトは TF チェーンを必須とするため、録画・オフライン生成では **(1) TF** が必要。

### 更新箇所

- `src/sirius/sirius_navigation/config/theta_calibration_real.yaml`
  - `camera_position: [0.0, 0.0, 1.135]`（更新済み）
  - `rpy_degrees: [roll, pitch, 0]`（未計測のため `[0,0,0]` のまま）
- `src/sirius/sirius_description/sdf/sirius3.sdf`
  - `theta_link` の `<pose relative_to="base_link">0 0 1.135 0 0 0</pose>`（更新済み）
  - このSDFは `rviz2real`（`display.launch.py`）の `robot_state_publisher` が
    `sirius3/theta_link` としてTF配信する実機・sim共通の定義。シミュレーション側の
    THETA姿勢も1.135になるため、simで路面マッピングする場合は整合を確認すること。

TFを一時的に手で出す場合:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 1.135 0 0 0 sirius3/base_footprint sirius3/theta_link
```

## 解像度と自動スケール

内部パラメータYAMLの `image_size` は 1920x1080 基準だが、投影側
（`theta_bev_projection.py` / `theta_perspective.py`）が
`scale = 実フレーム寸法 / image_size` を center・focal に掛けるため、
**1280x720 でもそのまま使える**（既定は720p）。YAMLの書き換えは不要。

## 高さ誤差の影響

BEV逆投影は `camera_position.z` を高さ `h` として地面レイを計算する。高さを `h'` と誤ると
地面点の動径方向位置が `h'/h` 倍にスケールする。

- 例: 真値 1.135 m を 1.1 m で計算 → 比 0.969（**約3.1%の動径収縮**）
- BEV端 r=5 m で約 **15 cm**、r=1.2 m で約 3.7 cm の位置ずれ
- 索引地図グリッドは 5 cm（`theta_indexed_map_node`）なので、地図外周では
  テクスチャ/意味ラベルとSLAM幾何の不一致が無視できない

pitch/roll も同様に投影を歪めるため、取付後に実測して `rpy_degrees` を更新するのが望ましい
（UI規約: pitch 正=前レンズが上向き）。
