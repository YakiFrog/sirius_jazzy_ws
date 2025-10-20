#!/usr/bin/env python3
"""
Nav2 Behavior Tree Log Viewer - GUI版
PySide6を使用したビジュアルモニター
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_msgs.msg import BehaviorTreeLog
from datetime import datetime

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QFont


class ROSSignals(QObject):
    """ROS callbackからQtへのシグナル橋渡し"""
    update_signal = Signal(object)


class BTLogViewerGUI(Node):
    def __init__(self, signals):
        super().__init__('bt_log_viewer_gui')
        
        self.signals = signals
        self.node_states = {}
        
        # ステータスの日本語訳
        self.status_jp = {
            'IDLE': '待機中',
            'RUNNING': '実行中',
            'SUCCESS': '成功',
            'FAILURE': '失敗'
        }
        
        # ノード名の日本語訳
        self.node_name_jp = {
            'ComputePathToPose': '経路計算',
            'FollowPath': '経路追従',
            'NavigateWithReplanning': '再計画ナビゲーション',
            'NavigateRecovery': 'リカバリーナビゲーション',
            'NavigateToPose': 'ゴールへ移動',
            'RateController': 'レート制御',
            'PlannerSelector': 'プランナー選択',
            'ControllerSelector': 'コントローラー選択',
            'GoalChecker': 'ゴール判定',
            'Sequence': 'シーケンス',
            'RecoveryNode': 'リカバリーノード',
            'PipelineSequence': 'パイプライン処理',
            'ClearCostmapService': 'コストマップクリア',
            'Spin': '回転動作',
            'Wait': '待機動作',
            'BackUp': '後退動作'
        }
        
        # QoS設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.log_callback,
            qos_profile
        )
        
        self.get_logger().info('シンプルGUI版 起動')
    
    def log_callback(self, msg):
        """ログを受信してGUIへ送信"""
        if not msg.event_log:
            return
        
        # データを処理してGUIに送信
        data = {
            'events': msg.event_log,
            'timestamp': datetime.now(),
            'node_states': self.node_states.copy()
        }
        
        # 状態を更新
        for event in msg.event_log:
            if event.current_status != 'IDLE':
                self.node_states[event.node_name] = event.current_status
            elif event.node_name in self.node_states:
                del self.node_states[event.node_name]
        
        self.signals.update_signal.emit(data)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ステータスの日本語訳
        self.status_jp = {
            'IDLE': '待機中',
            'RUNNING': '実行中',
            'SUCCESS': '成功',
            'FAILURE': '失敗'
        }
        
        # ノード名の日本語訳
        self.node_name_jp = {
            'ComputePathToPose': '経路計算',
            'FollowPath': '経路追従',
            'NavigateWithReplanning': '再計画ナビゲーション',
            'NavigateRecovery': 'リカバリーナビゲーション',
            'NavigateToPose': 'ゴールへ移動',
            'RateController': 'レート制御',
            'PlannerSelector': 'プランナー選択',
            'ControllerSelector': 'コントローラー選択',
            'GoalChecker': 'ゴール判定',
            'Sequence': 'シーケンス',
            'RecoveryNode': 'リカバリーノード',
            'PipelineSequence': 'パイプライン処理',
            'ClearCostmapService': 'コストマップクリア',
            'Spin': '回転動作',
            'Wait': '待機動作',
            'BackUp': '後退動作'
        }
        
        self.setup_ui()
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.setWindowTitle('Nav2 アクティブノード')
        self.setGeometry(100, 100, 400, 350)
        
        # メインウィジェット
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # メインレイアウト
        layout = QVBoxLayout(main_widget)
        layout.setSpacing(3)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # ヘッダー
        header_layout = QHBoxLayout()
        
        title_label = QLabel('アクティブノード')
        title_label.setFont(QFont('Arial', 10, QFont.Bold))
        header_layout.addWidget(title_label)
        
        self.status_label = QLabel('')
        self.status_label.setFont(QFont('Arial', 8))
        self.status_label.setAlignment(Qt.AlignRight)
        header_layout.addWidget(self.status_label)
        
        layout.addLayout(header_layout)
        
        # ノードリスト表示エリア（固定高さ、スクロールなし）
        self.nodes_widget = QWidget()
        self.nodes_layout = QVBoxLayout(self.nodes_widget)
        self.nodes_layout.setAlignment(Qt.AlignTop)
        self.nodes_layout.setSpacing(2)
        self.nodes_layout.setContentsMargins(0, 0, 0, 0)
        
        # 最大10個のラベルを事前に作成
        self.node_labels = []
        for i in range(10):
            label = QLabel('')
            label.setFont(QFont('Arial', 9))
            label.setStyleSheet('padding: 3px; border: 1px solid #ddd;')
            label.setVisible(False)
            self.nodes_layout.addWidget(label)
            self.node_labels.append(label)
        
        layout.addWidget(self.nodes_widget)
    
    def update_display(self, data):
        """表示を更新"""
        timestamp = data['timestamp']
        node_states = data['node_states']
        
        # タイムスタンプ更新
        self.status_label.setText(timestamp.strftime("%H:%M:%S"))
        
        # すべてのラベルを非表示に
        for label in self.node_labels:
            label.setVisible(False)
        
        # アクティブノードを表示（最大10個）
        if node_states:
            items = list(node_states.items())[:10]  # 最大10個
            for i, (node, status) in enumerate(items):
                node_jp = self.node_name_jp.get(node, node)
                status_jp = self.status_jp.get(status, status)
                
                self.node_labels[i].setText(f'{node_jp}: {status_jp}')
                self.node_labels[i].setVisible(True)
        else:
            # ノードがない場合
            self.node_labels[0].setText('アクティブなノードなし')
            self.node_labels[0].setVisible(True)


def main(args=None):
    # Qt Application
    app = QApplication(sys.argv)
    
    # ROS2初期化
    rclpy.init(args=args)
    
    # シグナル
    signals = ROSSignals()
    
    # メインウィンドウ
    window = MainWindow()
    window.show()
    
    # ROSノード
    ros_node = BTLogViewerGUI(signals)
    
    # シグナル接続
    signals.update_signal.connect(window.update_display)
    
    # ROSスピンタイマー
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    ros_timer.start(10)  # 10ms間隔
    
    # 終了処理
    def cleanup():
        ros_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()
    
    app.aboutToQuit.connect(cleanup)
    
    # 実行
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
