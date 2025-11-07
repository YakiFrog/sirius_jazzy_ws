#!/usr/bin/env python3
"""
Behavior Tree Log Viewer
Nav2の動作ツリーログを見やすく表示するツール
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_msgs.msg import BehaviorTreeLog, BehaviorTreeStatusChange
from collections import defaultdict
from datetime import datetime
import sys


class BTLogViewer(Node):
    def __init__(self):
        super().__init__('bt_log_viewer')
        
        # ノードの状態を追跡
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
            'RateController': 'レート制御',
            'PlannerSelector': 'プランナー選択',
            'ControllerSelector': 'コントローラー選択',
            'GoalChecker': 'ゴール判定',
            'Sequence': 'シーケンス',
            'RecoveryNode': 'リカバリーノード',
            'PipelineSequence': 'パイプライン処理',
            'ClearCostmapService': 'コストマップクリア',
            'Spin': '回転',
            'Wait': '待機',
            'BackUp': '後退'
        }
        
        # QoSプロファイルの設定（Nav2のデフォルトに合わせる）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # サブスクライバーの作成
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.log_callback,
            qos_profile
        )
        
        self.get_logger().info('🌲 動作ツリー ログビューアー 起動')
        self.get_logger().info('━' * 60)
    
    def get_status_emoji(self, status):
        """状態に応じた絵文字を返す"""
        emoji_map = {
            'IDLE': '⚪',
            'RUNNING': '🔵',
            'SUCCESS': '✅',
            'FAILURE': '❌'
        }
        return emoji_map.get(status, '❓')
    
    def log_callback(self, msg):
        """ログメッセージを受信したときの処理"""
        if not msg.event_log:
            return
        
        # 最新のイベントのみを処理(表示をシンプルに)
        recent_events = msg.event_log[-5:]  # 最新5件
        
        # 変化があったノードのみ表示
        changes = []
        for event in recent_events:
            node_name = event.node_name
            prev_status = event.previous_status  # 文字列で取得
            curr_status = event.current_status   # 文字列で取得
            
            # 状態変化があった場合のみ記録
            if prev_status != curr_status and curr_status != 'IDLE':
                changes.append({
                    'name': node_name,
                    'prev': prev_status,
                    'curr': curr_status,
                    'timestamp': event.timestamp
                })
                
                # 現在の状態を更新
                self.node_states[node_name] = curr_status
        
        # 変化があった場合のみ表示
        if changes:
            print()
            print(f"{'=' * 60}")
            timestamp = datetime.now().strftime('%H:%M:%S')
            print(f"🕒 {timestamp}")
            print(f"{'-' * 60}")
            
            for change in changes:
                emoji = self.get_status_emoji(change['curr'])
                node_name_jp = self.node_name_jp.get(change['name'], change['name'])
                prev_jp = self.status_jp.get(change['prev'], change['prev'])
                curr_jp = self.status_jp.get(change['curr'], change['curr'])
                print(f"{emoji} {node_name_jp:20} | {prev_jp:8} → {curr_jp}")
            
            # 現在アクティブなノードの概要を表示
            active_nodes = {k: v for k, v in self.node_states.items() 
                          if v in ['RUNNING', 'SUCCESS']}
            
            if active_nodes:
                print(f"{'-' * 60}")
                print("📊 現在の状態:")
                for node, status in active_nodes.items():
                    emoji = self.get_status_emoji(status)
                    node_name_jp = self.node_name_jp.get(node, node)
                    status_jp = self.status_jp.get(status, status)
                    print(f"   {emoji} {node_name_jp}: {status_jp}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = BTLogViewer()
        print("\n" + "=" * 60)
        print("🌲 Nav2 動作ツリー ログビューアー")
        print("=" * 60)
        print("Ctrl+C で終了")
        print("=" * 60 + "\n")
        
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\n\n終了します...")
    finally:
        if rclpy.ok():
            viewer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
