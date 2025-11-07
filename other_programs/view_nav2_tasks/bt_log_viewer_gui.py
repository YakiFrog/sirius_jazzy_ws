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
        
        # 表示しないノードのリスト
        self.hidden_nodes = [
            'RateController',
            'NavigateRecovery',
            'RecoveryNode',
            'PipelineSequence',
            'Sequence'
        ]
        
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
        
        # 表示しないノードのリスト
        self.hidden_nodes = [
            'RateController',
            'NavigateRecovery',
            'RecoveryNode',
            'PipelineSequence',
            'Sequence'
        ]
        
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
        
        # ウィンドウサイズを固定（4セクション対応で高さ拡大）
        self.setFixedSize(450, 550)
        
        # メインウィジェット
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # メインレイアウト
        layout = QVBoxLayout(main_widget)
        layout.setSpacing(2)
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
        
        # 実行中セクション
        running_group = QGroupBox('🔵 実行中')
        running_group.setFont(QFont('Arial', 9, QFont.Bold))
        running_layout = QVBoxLayout()
        running_layout.setSpacing(2)
        running_layout.setContentsMargins(3, 3, 3, 3)
        
        self.running_labels = []
        for i in range(3):
            label = QLabel('-')
            label.setFont(QFont('Arial', 9))
            label.setStyleSheet('padding: 3px; border: 1px solid #ddd;')
            label.setFixedHeight(22)
            running_layout.addWidget(label)
            self.running_labels.append(label)
        
        running_group.setLayout(running_layout)
        layout.addWidget(running_group)
        
        # 成功セクション
        success_group = QGroupBox('✅ 成功')
        success_group.setFont(QFont('Arial', 9, QFont.Bold))
        success_layout = QVBoxLayout()
        success_layout.setSpacing(2)
        success_layout.setContentsMargins(3, 3, 3, 3)
        
        self.success_labels = []
        for i in range(3):
            label = QLabel('-')
            label.setFont(QFont('Arial', 9))
            label.setStyleSheet('padding: 3px; border: 1px solid #ddd;')
            label.setFixedHeight(22)
            success_layout.addWidget(label)
            self.success_labels.append(label)
        
        success_group.setLayout(success_layout)
        layout.addWidget(success_group)
        
        # 失敗セクション
        failure_group = QGroupBox('❌ 失敗')
        failure_group.setFont(QFont('Arial', 9, QFont.Bold))
        failure_layout = QVBoxLayout()
        failure_layout.setSpacing(2)
        failure_layout.setContentsMargins(3, 3, 3, 3)
        
        self.failure_labels = []
        for i in range(2):
            label = QLabel('-')
            label.setFont(QFont('Arial', 9))
            label.setStyleSheet('padding: 3px; border: 1px solid #ddd;')
            label.setFixedHeight(22)
            failure_layout.addWidget(label)
            self.failure_labels.append(label)
        
        failure_group.setLayout(failure_layout)
        layout.addWidget(failure_group)
        
        # 待機中セクション
        idle_group = QGroupBox('⚪ 待機中')
        idle_group.setFont(QFont('Arial', 9, QFont.Bold))
        idle_layout = QVBoxLayout()
        idle_layout.setSpacing(2)
        idle_layout.setContentsMargins(3, 3, 3, 3)
        
        self.idle_labels = []
        for i in range(2):
            label = QLabel('-')
            label.setFont(QFont('Arial', 9))
            label.setStyleSheet('padding: 3px; border: 1px solid #ddd;')
            label.setFixedHeight(22)
            idle_layout.addWidget(label)
            self.idle_labels.append(label)
        
        idle_group.setLayout(idle_layout)
        layout.addWidget(idle_group)
    
    def update_display(self, data):
        """表示を更新"""
        timestamp = data['timestamp']
        node_states = data['node_states']
        
        # タイムスタンプ更新
        self.status_label.setText(timestamp.strftime("%H:%M:%S"))
        
        # ステータスごとにノードを分類（非表示リストを除外）
        running_nodes = []
        success_nodes = []
        failure_nodes = []
        idle_nodes = []
        
        for node, status in node_states.items():
            # 非表示リストに含まれるノードはスキップ
            if node in self.hidden_nodes:
                continue
            
            node_jp = self.node_name_jp.get(node, node)
            
            if status == 'RUNNING':
                running_nodes.append(node_jp)
            elif status == 'SUCCESS':
                success_nodes.append(node_jp)
            elif status == 'FAILURE':
                failure_nodes.append(node_jp)
            elif status == 'IDLE':
                idle_nodes.append(node_jp)
        
        # 実行中ラベルを更新
        for i, label in enumerate(self.running_labels):
            if i < len(running_nodes):
                label.setText(running_nodes[i])
            else:
                label.setText('-')
        
        # 成功ラベルを更新
        for i, label in enumerate(self.success_labels):
            if i < len(success_nodes):
                label.setText(success_nodes[i])
            else:
                label.setText('-')
        
        # 失敗ラベルを更新
        for i, label in enumerate(self.failure_labels):
            if i < len(failure_nodes):
                label.setText(failure_nodes[i])
            else:
                label.setText('-')
        
        # 待機中ラベルを更新
        for i, label in enumerate(self.idle_labels):
            if i < len(idle_nodes):
                label.setText(idle_nodes[i])
            else:
                label.setText('-')
        
        # すべて空の場合
        if not running_nodes and not success_nodes and not failure_nodes and not idle_nodes:
            self.running_labels[0].setText('アクティブなノードなし')
            self.success_labels[0].setText('-')
            self.failure_labels[0].setText('-')
            self.idle_labels[0].setText('-')


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
