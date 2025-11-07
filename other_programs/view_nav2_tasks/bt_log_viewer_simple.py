#!/usr/bin/env python3
"""
Behavior Tree Log Viewer (Simple Version)
シンプルに状態変化のみを表示
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_msgs.msg import BehaviorTreeLog


class SimpleBTViewer(Node):
    def __init__(self):
        super().__init__('simple_bt_viewer')
        
        # ステータスは文字列で送られてくる
        self.emoji_map = {
            'IDLE': '⚪', 
            'RUNNING': '🔵', 
            'SUCCESS': '✅', 
            'FAILURE': '❌'
        }
        
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
        
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            '/behavior_tree_log',
            self.callback,
            qos_profile
        )
        
        print("\n" + "=" * 70)
        print("🌲 Nav2 動作ツリー モニター")
        print("=" * 70 + "\n")
    
    def callback(self, msg):
        """ログを受信して表示"""
        if not msg.event_log:
            return
        
        # 最新イベントをチェック
        for event in msg.event_log[-3:]:  # 最新3件
            prev = event.previous_status
            curr = event.current_status
            
            # 意味のある変化のみ表示 (IDLE以外への変化)
            if prev != curr and curr in ['RUNNING', 'SUCCESS', 'FAILURE']:
                emoji = self.emoji_map.get(curr, '❓')
                
                # 日本語に翻訳
                node_name_jp = self.node_name_jp.get(event.node_name, event.node_name)
                prev_jp = self.status_jp.get(prev, prev)
                curr_jp = self.status_jp.get(curr, curr)
                
                print(f"{emoji} {node_name_jp:20} {prev_jp:8} → {curr_jp}")


def main():
    rclpy.init()
    try:
        node = SimpleBTViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n終了")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
