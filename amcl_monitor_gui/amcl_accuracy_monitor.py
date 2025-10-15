#!/usr/bin/env python3
"""
AMCL自己位置推定精度モニター（PySide6 GUI版）
AMCLの共分散をリアルタイムで可視化
"""

import sys
import math
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QLabel, QProgressBar, QGroupBox)
from PySide6.QtCore import QTimer, Qt, Signal, QObject
from PySide6.QtGui import QFont, QColor, QPalette

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading


# ========================================
# 精度評価パラメータ設定
# ========================================
class AccuracyThresholds:
    """
    AMCL精度評価の閾値設定
    各値を調整することで評価基準を変更できます
    """
    
    # X方向の精度評価閾値 [m]
    X_EXCELLENT = 0.1    # これ以下なら優秀（緑）
    X_ACCEPTABLE = 0.3   # これ以下なら許容範囲（黄色）、これ以上なら失敗（赤）
    X_MAX_DISPLAY = 1.5  # プログレスバーの最大表示値
    
    # Y方向の精度評価閾値 [m]
    Y_EXCELLENT = 0.1    # これ以下なら優秀（緑）
    Y_ACCEPTABLE = 0.3   # これ以下なら許容範囲（黄色）、これ以上なら失敗（赤）
    Y_MAX_DISPLAY = 1.5  # プログレスバーの最大表示値
    
    # Yaw角の精度評価閾値 [度]
    YAW_EXCELLENT = 5.7   # これ以下なら優秀（緑）: 0.1 rad ≈ 5.7°
    YAW_ACCEPTABLE = 17.0 # これ以下なら許容範囲（黄色）、これ以上なら失敗（赤）: 0.3 rad ≈ 17°
    YAW_MAX_DISPLAY = 45.0 # プログレスバーの最大表示値
    
    # GUI更新設定
    CONNECTION_CHECK_INTERVAL = 1000  # 接続チェック間隔 [ms]
    DATA_TIMEOUT = 3.0                # データ受信タイムアウト [秒]


# ========================================
# ROSノード＆ワーカー
# ========================================
class ROSWorker(Node, QObject):
    """ROSノードを別スレッドで実行"""
    amcl_updated = Signal(dict)
    
    def __init__(self):
        Node.__init__(self, 'amcl_monitor_gui')
        QObject.__init__(self)
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10)
        
        self.get_logger().info('AMCL Monitor GUI Node started')
    
    def amcl_callback(self, msg):
        """AMCLデータを受信してシグナルを発行"""
        # 共分散から標準偏差を計算
        cov = msg.pose.covariance
        std_x = math.sqrt(cov[0]) if cov[0] > 0 else 0.0
        std_y = math.sqrt(cov[7]) if cov[7] > 0 else 0.0
        std_yaw = math.sqrt(cov[35]) if cov[35] > 0 else 0.0
        std_yaw_deg = math.degrees(std_yaw)
        
        # クォータニオンからYaw角を計算
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        yaw_deg = math.degrees(yaw)
        
        data = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw_deg,
            'std_x': std_x,
            'std_y': std_y,
            'std_yaw': std_yaw_deg,
            'cov_x': cov[0],
            'cov_y': cov[7],
            'cov_yaw': cov[35]
        }
        
        self.amcl_updated.emit(data)


# ========================================
# 精度表示ウィジェット
# ========================================
class AccuracyBar(QWidget):
    """精度を表示するカスタムウィジェット"""
    def __init__(self, label_text, max_value, threshold_good, threshold_ok, unit="m"):
        super().__init__()
        self.max_value = max_value
        self.threshold_good = threshold_good
        self.threshold_ok = threshold_ok
        self.unit = unit
        
        layout = QVBoxLayout()
        
        # ラベル
        self.label = QLabel(label_text)
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.label.setFont(font)
        layout.addWidget(self.label)
        
        # 値表示
        self.value_label = QLabel("-- " + unit)
        value_font = QFont()
        value_font.setPointSize(14)
        self.value_label.setFont(value_font)
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)
        
        # プログレスバー
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(int(max_value * 1000))  # ミリ単位で精度向上
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setMinimumHeight(30)
        layout.addWidget(self.progress_bar)
        
        # 評価ラベル
        self.status_label = QLabel("待機中")
        status_font = QFont()
        status_font.setPointSize(11)
        self.status_label.setFont(status_font)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        self.setLayout(layout)
    
    def update_value(self, value):
        """値を更新"""
        self.value_label.setText(f"{value:.4f} {self.unit}")
        
        # プログレスバーを更新
        progress_value = min(int(value * 1000), self.max_value * 1000)
        self.progress_bar.setValue(progress_value)
        
        # 色と評価を更新
        if value < self.threshold_good:
            # 優秀（緑）
            self.progress_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid grey;
                    border-radius: 5px;
                    background-color: #2b2b2b;
                }
                QProgressBar::chunk {
                    background-color: #4CAF50;
                }
            """)
            self.status_label.setText("✓ 優秀")
            self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold; background-color: transparent;")
            self.value_label.setStyleSheet("color: #4CAF50; font-weight: bold; background-color: transparent;")
        elif value < self.threshold_ok:
            # 許容（黄色）
            self.progress_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid grey;
                    border-radius: 5px;
                    background-color: #2b2b2b;
                }
                QProgressBar::chunk {
                    background-color: #FFC107;
                }
            """)
            self.status_label.setText("⚠ 許容範囲")
            self.status_label.setStyleSheet("color: #FFC107; font-weight: bold; background-color: transparent;")
            self.value_label.setStyleSheet("color: #FFC107; font-weight: bold; background-color: transparent;")
        else:
            # 推定失敗（赤）
            self.progress_bar.setStyleSheet("""
                QProgressBar {
                    border: 2px solid grey;
                    border-radius: 5px;
                    background-color: #2b2b2b;
                }
                QProgressBar::chunk {
                    background-color: #F44336;
                }
            """)
            self.status_label.setText("✗ 推定失敗")
            self.status_label.setStyleSheet("color: #F44336; font-weight: bold; background-color: transparent;")
            self.value_label.setStyleSheet("color: #F44336; font-weight: bold; background-color: transparent;")
        
        # 失敗判定を返す
        return value >= self.threshold_ok


# ========================================
# メインウィンドウ
# ========================================
class AMCLMonitorWindow(QMainWindow):
    """メインウィンドウ"""
    def __init__(self, ros_worker):
        super().__init__()
        self.ros_worker = ros_worker
        
        self.setWindowTitle("AMCL 自己位置推定精度モニター")
        self.setMinimumSize(800, 600)
        
        # メインウィジェット
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # メインレイアウト
        main_layout = QVBoxLayout()
        
        # タイトル
        title_label = QLabel("🎯 AMCL 位置推定精度モニター")
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color: #2196F3; padding: 10px;")
        main_layout.addWidget(title_label)
        
        # 位置情報グループ
        position_group = QGroupBox("現在位置")
        position_layout = QHBoxLayout()
        
        self.pos_x_label = QLabel("X: -- m")
        self.pos_y_label = QLabel("Y: -- m")
        self.pos_yaw_label = QLabel("Yaw: -- °")
        
        for label in [self.pos_x_label, self.pos_y_label, self.pos_yaw_label]:
            font = QFont()
            font.setPointSize(13)
            label.setFont(font)
            label.setStyleSheet("padding: 10px; background-color: #1a1a1a; color: white; border-radius: 5px; border: 1px solid #2196F3;")
            position_layout.addWidget(label)
        
        position_group.setLayout(position_layout)
        main_layout.addWidget(position_group)
        
        # 精度情報グループ
        accuracy_group = QGroupBox("位置推定精度（標準偏差）")
        accuracy_layout = QHBoxLayout()
        
        # X方向の精度（パラメータクラスから閾値を取得）
        self.x_bar = AccuracyBar(
            "X方向", 
            AccuracyThresholds.X_MAX_DISPLAY,
            AccuracyThresholds.X_EXCELLENT,
            AccuracyThresholds.X_ACCEPTABLE,
            "m"
        )
        accuracy_layout.addWidget(self.x_bar)
        
        # Y方向の精度（パラメータクラスから閾値を取得）
        self.y_bar = AccuracyBar(
            "Y方向",
            AccuracyThresholds.Y_MAX_DISPLAY,
            AccuracyThresholds.Y_EXCELLENT,
            AccuracyThresholds.Y_ACCEPTABLE,
            "m"
        )
        accuracy_layout.addWidget(self.y_bar)
        
        # Yaw方向の精度（パラメータクラスから閾値を取得）
        self.yaw_bar = AccuracyBar(
            "Yaw角",
            AccuracyThresholds.YAW_MAX_DISPLAY,
            AccuracyThresholds.YAW_EXCELLENT,
            AccuracyThresholds.YAW_ACCEPTABLE,
            "°"
        )
        accuracy_layout.addWidget(self.yaw_bar)
        
        accuracy_group.setLayout(accuracy_layout)
        main_layout.addWidget(accuracy_group)
        
        # 共分散情報グループ
        covariance_group = QGroupBox("共分散行列")
        covariance_layout = QVBoxLayout()
        
        self.cov_x_label = QLabel("Variance X: --")
        self.cov_y_label = QLabel("Variance Y: --")
        self.cov_yaw_label = QLabel("Variance Yaw: --")
        
        for label in [self.cov_x_label, self.cov_y_label, self.cov_yaw_label]:
            font = QFont()
            font.setPointSize(11)
            label.setFont(font)
            label.setStyleSheet("padding: 5px;")
            covariance_layout.addWidget(label)
        
        covariance_group.setLayout(covariance_layout)
        main_layout.addWidget(covariance_group)
        
        # 自己位置推定状態ボックス（常に表示）
        localization_status_group = QGroupBox("自己位置推定状態")
        localization_status_layout = QVBoxLayout()
        
        self.localization_status_label = QLabel("待機中...")
        self.localization_status_label.setAlignment(Qt.AlignCenter)
        loc_font = QFont()
        loc_font.setPointSize(16)
        loc_font.setBold(True)
        self.localization_status_label.setFont(loc_font)
        self.localization_status_label.setStyleSheet(
            "padding: 20px; background-color: #1a1a1a; color: white; "
            "border-radius: 5px; border: 2px solid grey;"
        )
        localization_status_layout.addWidget(self.localization_status_label)
        
        localization_status_group.setLayout(localization_status_layout)
        main_layout.addWidget(localization_status_group)
        
        # ステータスバー
        self.status_label = QLabel("ROSノードに接続中...")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("padding: 10px; background-color: #1a1a1a; color: white; border-radius: 5px; border: 1px solid #4CAF50;")
        main_layout.addWidget(self.status_label)
        
        main_widget.setLayout(main_layout)
        
        # ROSワーカーからのシグナルを接続
        self.ros_worker.amcl_updated.connect(self.update_display)
        
        # タイマーで接続状態を確認（パラメータクラスから間隔を取得）
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self.check_connection)
        self.connection_timer.start(AccuracyThresholds.CONNECTION_CHECK_INTERVAL)
        
        self.last_update_time = None
    
    def update_display(self, data):
        """表示を更新"""
        import time
        self.last_update_time = time.time()
        
        # 位置情報を更新
        self.pos_x_label.setText(f"X: {data['x']:.3f} m")
        self.pos_y_label.setText(f"Y: {data['y']:.3f} m")
        self.pos_yaw_label.setText(f"Yaw: {data['yaw']:.1f} °")
        
        # 精度バーを更新（失敗判定を取得）
        x_failed = self.x_bar.update_value(data['std_x'])
        y_failed = self.y_bar.update_value(data['std_y'])
        yaw_failed = self.yaw_bar.update_value(data['std_yaw'])
        
        # 共分散を更新
        self.cov_x_label.setText(f"Variance X: {data['cov_x']:.6f}")
        self.cov_y_label.setText(f"Variance Y: {data['cov_y']:.6f}")
        self.cov_yaw_label.setText(f"Variance Yaw: {data['cov_yaw']:.6f}")
        
        # 自己位置推定の状態を判定して表示
        if x_failed or y_failed or yaw_failed:
            # 失敗している
            failed_axes = []
            if x_failed:
                failed_axes.append("X")
            if y_failed:
                failed_axes.append("Y")
            if yaw_failed:
                failed_axes.append("Yaw")
            
            self.localization_status_label.setText(
                f"✗ 自己位置推定失敗\n[{', '.join(failed_axes)}方向で精度不足]"
            )
            self.localization_status_label.setStyleSheet(
                "padding: 20px; background-color: #4A148C; color: white; "
                "border-radius: 5px; border: 3px solid #F44336;"
            )
        else:
            # 成功している
            self.localization_status_label.setText("✓ 自己位置推定成功\n[全方向で良好な精度]")
            self.localization_status_label.setStyleSheet(
                "padding: 20px; background-color: #1B5E20; color: white; "
                "border-radius: 5px; border: 3px solid #4CAF50;"
            )
        
        # ステータスを更新
        self.status_label.setText("✓ AMCLデータ受信中")
        self.status_label.setStyleSheet("padding: 10px; background-color: #C8E6C9; border-radius: 5px; color: #2E7D32; font-weight: bold;")
    
    def check_connection(self):
        """接続状態を確認"""
        import time
        if self.last_update_time is None:
            return
        
        # パラメータで設定したタイムアウト時間以上更新がない場合
        if time.time() - self.last_update_time > AccuracyThresholds.DATA_TIMEOUT:
            self.status_label.setText("⚠ AMCLデータが受信されていません")
            self.status_label.setStyleSheet("padding: 10px; background-color: #FFCDD2; border-radius: 5px; color: #C62828; font-weight: bold;")


def ros_spin(node):
    """ROSノードをスピン"""
    rclpy.spin(node)


def main():
    # ROS 2の初期化
    rclpy.init()
    
    # ROSワーカーノードを作成
    ros_worker = ROSWorker()
    
    # ROSを別スレッドで実行
    ros_thread = threading.Thread(target=ros_spin, args=(ros_worker,), daemon=True)
    ros_thread.start()
    
    # Qt アプリケーションを作成
    app = QApplication(sys.argv)
    
    # ウィンドウを作成して表示
    window = AMCLMonitorWindow(ros_worker)
    window.show()
    
    # アプリケーションを実行
    exit_code = app.exec()
    
    # クリーンアップ
    ros_worker.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
