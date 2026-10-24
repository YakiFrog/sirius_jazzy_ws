"""Shared Nav2 navigation mode definitions and remote-command helpers."""

import json
import os


# wait系は「停止・待機」専用BT(Spin/BackUp/コストマップクリア無し)、
# それ以外はnav2標準BT(リカバリ込み)を使う。'@pkg/relative/path' 形式は
# resolve_navigation_mode_config() でインストール先share配下へ展開される。
_WAIT_MODES = ("wait_normal", "wait_active")
_WAIT_BT = "@sirius_navigation/behavior_trees/navigate_to_pose_wait_only.xml"
_DEFAULT_BT = (
    "@nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
)


NAVIGATION_MODE_CONFIGS = {
    "normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.90, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "normal_active": {
        "/controller_server": {
            "FollowPath.vx_max": 1.00,
            "FollowPath.time_steps": 54,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 1.00,
            "FollowPath.vx_std": 0.40,
            "FollowPath.wz_std": 0.48,
            "FollowPath.ax_max": 1.50,
            "FollowPath.ax_min": -1.50,
            "FollowPath.az_max": 2.20,
            "FollowPath.CostCritic.cost_weight": 20.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [1.00, 0.0, 1.00],
            "min_velocity": [-0.60, 0.0, -1.00],
            "max_accel": [1.50, 0.0, 2.20],
            "max_decel": [-1.50, 0.0, -2.20],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    # fast: 通常(0.90)より高速な1.20m/s。今回の同定・接地実験の知見を反映。
    #   - ドライバのソフト閉ループ(既定ON)が左右輪差・蛇行を抑制する前提。
    #   - 回生過電圧(OVL15V/FF=2)を抑えるため、減速は緩め(ax_min/max_decel=-0.55)。
    #     停止距離 d=v^2/2a = 1.2^2/(2*0.55) = 1.31m。狭所では normal を使うこと。
    #   - 旋回半径 R=v/w=1.0 を維持 (vx_max=wz_max=1.20)。
    #   - 供給(C300 10A/120W)が上限のため、これ以上はサグ/失速の恐れ。
    "fast": {
        "/controller_server": {
            "FollowPath.vx_max": 1.20,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 1.20,
            "FollowPath.vx_std": 0.30,
            "FollowPath.wz_std": 0.35,
            "FollowPath.ax_max": 1.00,
            "FollowPath.ax_min": -0.55,
            "FollowPath.az_max": 1.80,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [1.20, 0.0, 1.20],
            "min_velocity": [-0.60, 0.0, -1.20],
            "max_accel": [1.00, 0.0, 1.80],
            "max_decel": [-0.55, 0.0, -1.80],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "safe": {
        "/controller_server": {
            "FollowPath.vx_max": 0.40,
            "FollowPath.time_steps": 135,
            "FollowPath.vx_min": -0.20,
            "FollowPath.wz_max": 0.40,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.40,
            "FollowPath.ax_min": -0.40,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 15.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.40, 0.0, 0.40],
            "min_velocity": [-0.20, 0.0, -0.40],
            "max_accel": [0.40, 0.0, 1.00],
            "max_decel": [-0.40, 0.0, -1.00],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    # slow(4): 速度が低く障害物前で膠着しやすい。
    #   ・CostCritic 20->10: 障害物コストを下げ、過剰回避で固まるのを防ぐ。
    #   ・vx_std/wz_std 拡大: 探索ノイズを増やし、逃げ道となる軌道を見つけやすくする。
    "slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.time_steps": 200,  # 低速でも先読み距離(約4m)を確保するため維持
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    # wait_normal: 待機優先。巡航速度は0.60m/s(実測normal巡航相当)。
    #   RPPはパス追従中の角速度上限を持たない(omega = v * 曲率)ため、低速でも
    #   先読み距離Lが小さいと曲率 k=2y/L^2 が増幅し角速度が激しくなる。よって
    #   L(min_lookahead含む)を大きめに固定する。
    #   曲率減速の式 v = raw * r / R より、カーブ中は omega = raw / R 一定になる。
    #   raw=0.60, R=1.50 で omega=0.40 となり velocity_smoother の角速度上限
    #   0.60rad/sを常に下回る。
    #   障害物にはもっと手前で減速・停止させるため cost_scaling_dist と
    #   max_allowed_time_to_collision_up_to_carrot を base より強める。
    "wait_normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.60,
            "FollowPath.time_steps": 82,  # 0.60*82*0.1=4.92m 先読み維持(RPP未使用)
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.60,
            "FollowPath.vx_std": 0.22,
            "FollowPath.wz_std": 0.27,
            "FollowPath.ax_max": 0.60,
            "FollowPath.ax_min": -0.60,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
            "WaitPath.desired_linear_vel": 0.60,
            # 角速度の激しさ対策: 先読みを短くしすぎない
            "WaitPath.lookahead_dist": 1.20,
            "WaitPath.min_lookahead_dist": 1.00,
            "WaitPath.max_lookahead_dist": 2.00,
            "WaitPath.lookahead_time": 1.20,
            # カーブでは v=raw*r/R で減速し omega=raw/R=0.40 に抑える
            "WaitPath.regulated_linear_scaling_min_radius": 1.50,
            "WaitPath.regulated_linear_scaling_min_speed": 0.10,
            # 障害物手前で早めに減速・停止 (base: 0.70 / 2.0)
            "WaitPath.cost_scaling_dist": 1.00,
            "WaitPath.max_allowed_time_to_collision_up_to_carrot": 3.0,
            # パスが背後にあるときもその場旋回して向き直れるようにする。
            # use_rotate_to_heading=falseだと shouldRotateToPath が常にfalseとなり、
            # 背後パスでは曲率≈0で旋回せず前進してしまう(旋回しない問題の原因)。
            # 障害物が正面にある場合は angle_to_path が小さく旋回条件を満たさず、
            # 従来どおり停止・待機する。旋回時に壁が干渉すれば isCollisionImminent が
            # NoValidControl を返し、BTが待機・再試行する(安全側)。
            "WaitPath.use_rotate_to_heading": True,
            "WaitPath.rotate_to_heading_angular_vel": 0.60,
            "WaitPath.rotate_to_heading_min_angle": 0.785,
        },
        "/velocity_smoother": {
            "max_velocity": [0.60, 0.0, 0.60],  # 角速度上限0.60rad/sで激しい旋回を抑制
            "min_velocity": [-0.60, 0.0, -0.60],
            "max_accel": [0.60, 0.0, 1.00],
            "max_decel": [-0.60, 0.0, -1.00],
        },
        # The local costmap remains enabled for collision detection. Only the
        # global dynamic-obstacle layer is disabled so replanning keeps the
        # original static-map route while the wait controller is selected.
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
    # wait_active: 待機優先の速度違い。巡航速度はwait_normal(0.60)の1.2倍=0.72m/s。
    #   角速度の激しさ対策(先読み固定・ωクランプ)はwait_normalと同じ方針。
    #   raw=0.72, R=1.50 で omega=raw/R=0.48 となり角速度上限0.60rad/sに余裕を持つ。
    #   障害物手前の減速・停止もwait_normalと同じく強める。
    "wait_active": {
        "/controller_server": {
            "FollowPath.vx_max": 0.72,
            "FollowPath.time_steps": 68,  # 0.72*68*0.1=4.90m 先読み維持(RPP未使用)
            "FollowPath.vx_min": -0.66,
            "FollowPath.wz_max": 0.60,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.72,
            "FollowPath.ax_min": -0.72,
            "FollowPath.az_max": 1.20,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 8.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
            "WaitPath.desired_linear_vel": 0.72,
            # 角速度の激しさ対策: 先読みを短くしすぎない
            "WaitPath.lookahead_dist": 1.20,
            "WaitPath.min_lookahead_dist": 1.00,
            "WaitPath.max_lookahead_dist": 2.00,
            "WaitPath.lookahead_time": 1.20,
            # カーブでは v=raw*r/R で減速し omega=raw/R=0.48 に抑える
            "WaitPath.regulated_linear_scaling_min_radius": 1.50,
            "WaitPath.regulated_linear_scaling_min_speed": 0.10,
            # 障害物手前で早めに減速・停止 (base: 0.70 / 2.0)
            "WaitPath.cost_scaling_dist": 1.00,
            "WaitPath.max_allowed_time_to_collision_up_to_carrot": 3.0,
            # 背後パスでもその場旋回できるようwait_normalと同じく有効化する。
            "WaitPath.use_rotate_to_heading": True,
            "WaitPath.rotate_to_heading_angular_vel": 0.60,
            "WaitPath.rotate_to_heading_min_angle": 0.785,
        },
        "/velocity_smoother": {
            "max_velocity": [0.72, 0.0, 0.60],  # 角速度上限0.60rad/sで激しい旋回を抑制
            "min_velocity": [-0.66, 0.0, -0.60],
            "max_accel": [0.72, 0.0, 1.00],
            "max_decel": [-0.72, 0.0, -1.00],
        },
        # 待機モード同様、再計画では迂回せず元のパスを維持して停止待機する。
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": False},
    },
    # strict系(6/7/8): 「回避しつつパス追従を重視」する設計。
    #   ・global obstacle_layer を False->True: 再計画で障害物を迂回させる。
    #     （旧設定はパスが障害物を貫通し、全軌道衝突→NoValidControl→
    #       BTリカバリのSpinで回り続ける原因になっていた）
    #   ・PathAlignCritic 60->12: normal(8)より高くして追従を強めるが、
    #     過剰な横偏差ペナルティで回避できなくなるのを防ぐ。
    #   ・CostCritic/PathFollow/PathAngle/Twirling/PreferForward/Goal は
    #     normal相当に揃え、strict同士で挙動が発散しないようにする。
    "strict_normal": {
        "/controller_server": {
            "FollowPath.vx_max": 0.90,
            "FollowPath.time_steps": 60,
            "FollowPath.vx_min": -0.60,
            "FollowPath.wz_max": 0.90,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.90,
            "FollowPath.ax_min": -0.90,
            "FollowPath.az_max": 1.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.90, 0.0, 0.90],
            "min_velocity": [-0.60, 0.0, -0.90],
            "max_accel": [0.90, 0.0, 1.50],
            "max_decel": [-0.90, 0.0, -1.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "strict_safe": {
        "/controller_server": {
            "FollowPath.vx_max": 0.40,
            "FollowPath.time_steps": 135,
            "FollowPath.vx_min": -0.20,
            "FollowPath.wz_max": 0.40,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.20,
            "FollowPath.ax_max": 0.40,
            "FollowPath.ax_min": -0.40,
            "FollowPath.az_max": 1.00,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.40, 0.0, 0.40],
            "min_velocity": [-0.20, 0.0, -0.40],
            "max_accel": [0.40, 0.0, 1.00],
            "max_decel": [-0.40, 0.0, -1.00],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
    "strict_slow": {
        "/controller_server": {
            "FollowPath.vx_max": 0.20,
            "FollowPath.time_steps": 200,
            "FollowPath.vx_min": -0.10,
            "FollowPath.wz_max": 0.20,
            "FollowPath.vx_std": 0.25,
            "FollowPath.wz_std": 0.30,
            "FollowPath.ax_max": 0.20,
            "FollowPath.ax_min": -0.20,
            "FollowPath.az_max": 0.50,
            "FollowPath.CostCritic.cost_weight": 10.0,
            "FollowPath.PathAlignCritic.cost_weight": 12.0,
            "FollowPath.PathFollowCritic.cost_weight": 8.0,
            "FollowPath.PathAngleCritic.cost_weight": 2.0,
            "FollowPath.TwirlingCritic.cost_weight": 3.0,
            "FollowPath.PreferForwardCritic.cost_weight": 15.0,
            "FollowPath.GoalCritic.cost_weight": 3.0,
            "FollowPath.GoalAngleCritic.cost_weight": 1.0,
        },
        "/velocity_smoother": {
            "max_velocity": [0.20, 0.0, 0.20],
            "min_velocity": [-0.10, 0.0, -0.20],
            "max_accel": [0.20, 0.0, 0.50],
            "max_decel": [-0.20, 0.0, -0.50],
        },
        "/global_costmap/global_costmap": {"obstacle_layer.enabled": True},
    },
}


NAVIGATION_MODE_INFO = {
    "normal": {"label": "通常", "speed": 0.90, "strict": False},
    "normal_active": {"label": "通常・探索強化", "speed": 1.00, "strict": False},
    "fast": {"label": "高速(減速緩和・OVL抑制)", "speed": 1.20, "strict": False},
    "safe": {"label": "安全", "speed": 0.40, "strict": False},
    "slow": {"label": "超低速", "speed": 0.20, "strict": False},
    "wait_normal": {"label": "待機優先", "speed": 0.60, "strict": False},
    "wait_active": {"label": "待機優先・高速", "speed": 0.72, "strict": False},
    "strict_normal": {"label": "パス厳守・通常", "speed": 0.90, "strict": True},
    "strict_safe": {"label": "パス厳守・安全", "speed": 0.40, "strict": True},
    "strict_slow": {"label": "パス厳守・超低速", "speed": 0.20, "strict": True},
}


# Speed-only parameter subset shared by the legacy numeric speed command.
# NAVIGATION_MODE_CONFIGS stays the single source of truth for these values so
# the named modes and the numeric speed command cannot drift apart.
SPEED_CONTROLLER_PARAM_KEYS = (
    "FollowPath.vx_max",
    "FollowPath.vx_min",
    "FollowPath.wz_max",
    "FollowPath.vx_std",
    "FollowPath.wz_std",
    "FollowPath.ax_max",
    "FollowPath.ax_min",
    "FollowPath.az_max",
    "FollowPath.time_steps",
)

# String buckets accepted by the legacy speed command, mapped to named modes.
_SPEED_STRING_MODES = {
    "slow": "slow",
    "safe": "safe",
    "normal": "normal",
    "fast": "normal_active",
}

# Numeric speed upper bounds -> named mode (checked in ascending order).
_SPEED_NUMERIC_BUCKETS = (
    (0.25, "slow"),
    (0.55, "safe"),
    (0.95, "normal"),
)
_SPEED_NUMERIC_DEFAULT_MODE = "normal_active"


def speed_setting_to_mode(speed_setting):
    """Map a legacy numeric/string speed setting to a named navigation mode."""
    if isinstance(speed_setting, str):
        return _SPEED_STRING_MODES.get(speed_setting.strip().lower(), "normal")
    try:
        value = float(speed_setting)
    except (TypeError, ValueError):
        return "normal"
    for threshold, mode in _SPEED_NUMERIC_BUCKETS:
        if value <= threshold:
            return mode
    return _SPEED_NUMERIC_DEFAULT_MODE


def build_speed_parameters(mode):
    """Return only the speed-related params of a named mode (no critics/costmap)."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    config = NAVIGATION_MODE_CONFIGS[canonical]
    controller = {
        key: value
        for key, value in config.get("/controller_server", {}).items()
        if key in SPEED_CONTROLLER_PARAM_KEYS
    }
    params = {"/controller_server": controller}
    if "/velocity_smoother" in config:
        params["/velocity_smoother"] = dict(config["/velocity_smoother"])
    return params


def navigation_mode_controller(mode):
    """Return the controller plugin selected by a navigation mode."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    return "WaitPath" if canonical in ("wait_normal", "wait_active") else "FollowPath"


def normalize_navigation_mode(mode):
    """Return a canonical navigation mode, including the legacy strict alias."""
    normalized = str(mode or "").strip().lower()
    if normalized == "strict":
        normalized = "strict_safe"
    return normalized if normalized in NAVIGATION_MODE_CONFIGS else None


def resolve_navigation_mode_config(mode):
    """Return a mode config with the BT selector applied and '@pkg/path' expanded.

    wait系には停止・待機専用BT、それ以外はnav2標準BTを bt_navigator に設定する。
    '@package/relative/path' 形式の値はインストール済み share 配下の絶対パスへ
    展開する（bt_navigator.default_nav_to_pose_bt_xml は絶対パスを要求するため）。
    """
    from ament_index_python.packages import get_package_share_directory

    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")

    default_bt = _WAIT_BT if canonical in _WAIT_MODES else _DEFAULT_BT
    config = {"/bt_navigator": {"default_nav_to_pose_bt_xml": default_bt}}
    config.update(NAVIGATION_MODE_CONFIGS[canonical])

    resolved = {}
    for node_name, params in config.items():
        resolved_params = {}
        for key, value in params.items():
            if isinstance(value, str) and value.startswith("@"):
                package, _, relative = value[1:].partition("/")
                value = os.path.join(get_package_share_directory(package), relative)
            resolved_params[key] = value
        resolved[node_name] = resolved_params

    return resolved


def parse_navigation_mode_command(instruction):
    """Parse a remote nav_mode JSON command; return None for other commands."""
    stripped = str(instruction or "").strip()
    if not (stripped.startswith("{") and stripped.endswith("}")):
        return None
    try:
        payload = json.loads(stripped)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or payload.get("type") != "nav_mode":
        return None
    mode = normalize_navigation_mode(payload.get("mode"))
    if mode is None:
        raise ValueError(f"unknown navigation mode: {payload.get('mode')!r}")
    return mode


def navigation_mode_confirmation(mode):
    """Build an explicit Japanese confirmation for an applied mode."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    info = NAVIGATION_MODE_INFO[canonical]
    return (
        f"[happy]Nav2走行モードを{info['label']}、"
        f"最高速度{info['speed']:.1f}メートル毎秒に変更したのだ！"
    )


# UI and spoken presentation for the browser/CLI mode switch. Speeds stay in
# NAVIGATION_MODE_INFO; this table only adds human-facing wording and keeps the
# mode definitions in one place.
NAVIGATION_MODE_PRESENTATION = {
    "normal": {
        "label": "通常",
        "description": "標準の走行モード。通常の屋内移動向け。",
    },
    "normal_active": {
        "label": "通常・探索強化",
        "description": "通常より少し速く、探索と加速を強めたモード。",
    },
    "safe": {
        "label": "ゆっくり安全",
        "description": "速度と加減速を抑えた安全寄りのモード。",
    },
    "slow": {
        "label": "超低速安全",
        "description": "周囲確認を優先する超低速モード。",
    },
    "wait_normal": {
        "label": "待機優先",
        "description": "障害物の前で停止・待機し、再試行するモード。",
    },
    "strict_normal": {
        "label": "パス追従優先・通常",
        "description": "パスへの追従を強めつつ、通常速度で走行するモード。",
    },
    "strict_safe": {
        "label": "パス追従優先・ゆっくり",
        "description": "パス追従を優先し、ゆっくり安全に走行するモード。",
    },
    "strict_slow": {
        "label": "パス追従優先・超低速",
        "description": "パス追従を優先し、超低速で走行するモード。",
    },
    "wait_active": {
        "label": "待機優先・高速",
        "description": "待機優先の挙動を保ちながら、待機優先より速く走るモード。",
    },
    "fast": {
        "label": "高速",
        "description": "高速走行向け。減速を緩和して回生過電圧を抑える設定。",
    },
}


def navigation_mode_speech(mode):
    """Plain speech text for a confirmed mode (no Face control tags)."""
    canonical = normalize_navigation_mode(mode)
    if canonical is None:
        raise ValueError(f"unknown navigation mode: {mode!r}")
    info = NAVIGATION_MODE_INFO[canonical]
    label = NAVIGATION_MODE_PRESENTATION[canonical]["label"]
    return f"{label}モード、秒速{info['speed']:.1f}メートルに変更しました"
