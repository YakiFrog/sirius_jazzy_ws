# AMCL自己位置推定精度モニター（GUI版）

AMCLの共分散をリアルタイムで可視化するPySide6ベースのGUIアプリケーション。

## 機能

- **リアルタイム表示**: AMCLの位置推定精度をリアルタイムで監視
- **視覚的評価**: 
  - ✓ 優秀（緑）: 高精度
  - ⚠ 許容範囲（黄）: 許容できる精度
  - ✗ 要改善（赤）: 精度が低い
- **共分散表示**: X, Y, Yawの共分散を数値で表示

## インストール

```bash
cd /home/kotantu-nuc/sirius_jazzy_ws/amcl_monitor_gui

# PySide6をインストール
pip install PySide6

# または、requirements.txtから一括インストール
pip install -r requirements.txt
```

## 使い方

### 1. シミュレーションとNav2を起動

```bash
# ターミナル1: シミュレーション
sim

# ターミナル2: Nav2
nav2
```

### 2. モニターGUIを起動

```bash
# ターミナル3
cd /home/kotantu-nuc/sirius_jazzy_ws/amcl_monitor_gui
source ../install/setup.bash
python3 amcl_accuracy_monitor.py
```

## 評価基準

### X/Y方向の位置精度（標準偏差）
- ✓ 優秀: < 0.1m
- ⚠ 許容: 0.1 ~ 0.3m
- ✗ 要改善: > 0.3m

### Yaw角の精度（標準偏差）
- ✓ 優秀: < 5.7° (0.1 rad)
- ⚠ 許容: 5.7° ~ 17° (0.1 ~ 0.3 rad)
- ✗ 要改善: > 17° (0.3 rad)

## トラブルシューティング

### PySide6が見つからない
```bash
pip install --upgrade PySide6
```

### ROSトピックが見つからない
- Nav2とAMCLが正しく起動しているか確認
```bash
ros2 topic list | grep amcl_pose
```

### GUIが表示されない
- X11転送が有効か確認
- または、ローカルマシンで実行

## .bashrcへのエイリアス追加

```bash
# AMCL精度モニターGUI
alias amcl_gui='cd /home/kotantu-nuc/sirius_jazzy_ws/amcl_monitor_gui && source ../install/setup.bash && python3 amcl_accuracy_monitor.py'
```

使い方:
```bash
amcl_gui
```

## 位置推定精度の計算方法

このGUIアプリケーションでは、**AMCLの共分散行列**から位置推定精度を計算しています。

### 計算式

```python
# 共分散行列から標準偏差を計算
std_x = √(cov[0])    # X方向の標準偏差 [m]
std_y = √(cov[7])    # Y方向の標準偏差 [m]
std_yaw = √(cov[35]) # Yaw角の標準偏差 [rad]
```

### 共分散行列の構造

AMCLの`/amcl_pose`トピック（`PoseWithCovarianceStamped`）には、6x6の共分散行列（36要素）が含まれています：

```
[ cov[0]  cov[1]  cov[2]  cov[3]  cov[4]  cov[5]  ]   <- X
[ cov[6]  cov[7]  cov[8]  cov[9]  cov[10] cov[11] ]   <- Y
[ cov[12] cov[13] cov[14] cov[15] cov[16] cov[17] ]   <- Z
[ cov[18] cov[19] cov[20] cov[21] cov[22] cov[23] ]   <- Roll
[ cov[24] cov[25] cov[26] cov[27] cov[28] cov[29] ]   <- Pitch
[ cov[30] cov[31] cov[32] cov[33] cov[34] cov[35] ]   <- Yaw
```

- **cov[0]**: X方向の分散（σ²ₓ）
- **cov[7]**: Y方向の分散（σ²ᵧ）
- **cov[35]**: Yaw角の分散（σ²ᵧₐw）

### 精度の意味

**標準偏差（Standard Deviation）**は、位置推定の**不確実性**を表します：

- **std_x = 0.05m** → 真の位置がX方向に±0.05m以内にある確率が約68%
- **std_x = 0.10m** → 真の位置がX方向に±0.10m以内にある確率が約68%
- **std_yaw = 5°** → 真の向きが±5°以内にある確率が約68%

### 評価の根拠

標準偏差は正規分布の性質に基づいています：

- **±1σ (標準偏差)**: 約68.3%の確率で真の値がこの範囲内
- **±2σ**: 約95.4%の確率で真の値がこの範囲内
- **±3σ**: 約99.7%の確率で真の値がこの範囲内

このGUIでは、**1σ（標準偏差）の値**を精度指標として使用しています。

### 評価閾値の設定根拠

| 項目 | 🟢 優秀 | 🟡 許容範囲 | 🔴 要改善 | 理由 |
|------|---------|------------|----------|------|
| X/Y方向 | < 0.1m | 0.1～0.3m | > 0.3m | 室内ナビゲーションでは10cm以下が理想的。30cm超は障害物回避に支障 |
| Yaw角 | < 0.1rad (5.7°) | 0.1～0.2rad (5.7～11.4°) | > 0.2rad (11.4°) | 5度以下なら進行方向が正確。11度超は経路追従が困難 |

### AMCLパラメータとの関係

共分散（精度）は、以下のAMCLパラメータに影響されます：

- **max_particles**: パーティクル数が多いと精度向上（計算負荷増）
- **z_hit / z_rand**: センサーモデルの重み（z_hit高いほど精度向上）
- **laser_likelihood_max_dist**: レーザーの最大距離（環境に応じて調整）
- **update_min_d / update_min_a**: 更新頻度（頻繁な更新で精度向上）

この計算方法は、**統計的に位置推定の信頼性**を定量化しており、AMCLのパーティクルフィルタがどれだけ収束しているかを示します。値が小さいほど、位置推定が正確で信頼性が高いことを意味します。
