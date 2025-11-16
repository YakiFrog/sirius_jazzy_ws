#!/usr/bin/env python3
"""
Sirius Assisted Teleop サンプルプログラム

このプログラムはNav2のAssisted Teleopアクションを使用して、
障害物を避けながら手動操縦を支援します。

使用方法:
1. Nav2が起動していることを確認
2. このノードを実行: ros2 run sirius_navigation assisted_teleop
3. 別ウィンドウでteleop_twist_keyboardを実行してロボットを操縦

Assisted Teleopは以下を行います:
- 操縦コマンド(cmd_vel)を受信
- コストマップをチェックして障害物を検出
- 障害物に衝突しないようにコマンドを修正/フィルタリング
- 安全な速度コマンドをロボットに送信
"""

import sys
from time import sleep

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from nav2_msgs.action import AssistedTeleop
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node


class SiriusAssistedTeleop(Node):
    """Sirius用のAssisted Teleopノード"""

    def __init__(self):
        super().__init__('sirius_assisted_teleop')
        
        # パラメータの宣言
        self.declare_parameter('time_allowance', 300)  # デフォルト5分(300秒)
        self.declare_parameter('set_initial_pose', True)  # 初期ポーズを設定するか
        
        # 初期ポーズのパブリッシャー（AMCLに初期位置を通知）
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.initial_pose_pub = self.create_publisher(
            PoseStamped,
            'initialpose',
            qos_profile
        )
        
        # Assisted Teleopアクションクライアント
        self.assisted_teleop_client = ActionClient(
            self, AssistedTeleop, 'assisted_teleop'
        )
        
        self.goal_handle = None
        self.result_future = None
        
        self.get_logger().info('Sirius Assisted Teleop ノードを起動しました')

    def set_initial_pose(self):
        """
        初期ポーズを設定（AMCLのローカライゼーションを開始）
        """
        if not self.get_parameter('set_initial_pose').value:
            return
        
        self.get_logger().info('初期ポーズを設定中...')
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # ロボットの初期位置（原点）
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        
        # 初期姿勢（正面向き）
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        
        # 初期ポーズをパブリッシュ
        self.initial_pose_pub.publish(initial_pose)
        
        self.get_logger().info('初期ポーズを設定しました (x=0.0, y=0.0, yaw=0.0)')
        
        # AMCLがローカライズするまで少し待機
        self.get_logger().info('AMCLのローカライゼーションを待機中...')
        sleep(2.0)

    def start_assisted_teleop(self, time_allowance=None):
        """
        Assisted Teleopアクションを開始
        
        Args:
            time_allowance: 操縦を許可する時間(秒)。Noneの場合はパラメータから取得
        """
        if time_allowance is None:
            time_allowance = self.get_parameter('time_allowance').value
        
        # 初期ポーズを設定
        # self.set_initial_pose()
        
        self.get_logger().info("'assisted_teleop' アクションサーバーを待機中...")
        
        # アクションサーバーが利用可能になるまで待機
        timeout_counter = 0
        while not self.assisted_teleop_client.wait_for_server(timeout_sec=1.0):
            timeout_counter += 1
            if timeout_counter % 5 == 0:  # 5秒ごとにメッセージ
                self.get_logger().info(
                    "'assisted_teleop' アクションサーバーが利用できません。待機中..."
                )
                self.get_logger().info(
                    "Nav2が起動していることを確認してください！"
                )
            if timeout_counter > 30:  # 30秒でタイムアウト
                self.get_logger().error(
                    "タイムアウト: 'assisted_teleop' アクションサーバーが見つかりません"
                )
                return False
        
        # ゴールメッセージの作成
        goal_msg = AssistedTeleop.Goal()
        goal_msg.time_allowance = Duration(sec=time_allowance)
        
        self.get_logger().info(
            f"Assisted Teleop を開始します (制限時間: {time_allowance}秒)..."
        )
        self.get_logger().info(
            "別のターミナルで 'teleop_twist_keyboard' を起動して操縦してください！"
        )
        
        # ゴールを送信
        send_goal_future = self.assisted_teleop_client.send_goal_async(
            goal_msg, self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Assisted Teleop リクエストが拒否されました！')
            return False
        
        self.get_logger().info('Assisted Teleop が開始されました！')
        self.result_future = self.goal_handle.get_result_async()
        return True

    def _feedback_callback(self, feedback_msg):
        """
        アクションのフィードバックを受信
        """
        feedback = feedback_msg.feedback
        # AssistedTeleopアクションにはフィードバックがないため、
        # このコールバックは主にデバッグ用
        pass

    def wait_for_completion(self):
        """
        Assisted Teleopアクションの完了を待機
        """
        if self.result_future is None:
            self.get_logger().error('Assisted Teleop がまだ開始されていません')
            return None
        
        self.get_logger().info('Assisted Teleop 実行中...')
        self.get_logger().info('キャンセルするには Ctrl+C を押してください')
        
        try:
            # 結果を待機（定期的にスピンして他のコールバックも処理）
            while not self.result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                sleep(0.1)
            
            result = self.result_future.result()
            
            if result.status == 4:  # SUCCEEDED
                self.get_logger().info('Assisted Teleop が正常に完了しました')
                return True
            elif result.status == 5:  # CANCELED
                self.get_logger().warn('Assisted Teleop がキャンセルされました')
                return False
            else:
                self.get_logger().error(
                    f'Assisted Teleop が失敗しました (status: {result.status})'
                )
                return False
                
        except KeyboardInterrupt:
            self.get_logger().info('ユーザーによって中断されました')
            self.cancel_assisted_teleop()
            return False

    def cancel_assisted_teleop(self):
        """Assisted Teleopアクションをキャンセル"""
        if self.goal_handle is not None:
            self.get_logger().info('Assisted Teleop をキャンセル中...')
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info('Assisted Teleop がキャンセルされました')


def main(args=None):
    """メイン関数"""
    rclpy.init(args=args)
    
    try:
        # ノードの作成
        assisted_teleop_node = SiriusAssistedTeleop()
        
        # 使用方法の表示
        print("\n" + "="*60)
        print("Sirius Assisted Teleop")
        print("="*60)
        print("\n使用方法:")
        print("1. このプログラムを実行 (実行中)")
        print("2. 別のターミナルで以下を実行:")
        print("   ros2 run teleop_twist_keyboard teleop_twist_keyboard")
        print("3. キーボードでロボットを操縦")
        print("\nAssisted Teleopの機能:")
        print("- 障害物を検出して自動的に速度を調整")
        print("- 衝突しないように操縦コマンドをフィルタリング")
        print("- 安全な操縦をサポート")
        print("\n終了するには Ctrl+C を押してください")
        print("="*60 + "\n")
        
        # Assisted Teleopの開始
        if assisted_teleop_node.start_assisted_teleop():
            # 完了を待機
            assisted_teleop_node.wait_for_completion()
        
        # クリーンアップ
        assisted_teleop_node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        print("\n\nプログラムを終了します...")
        if 'assisted_teleop_node' in locals():
            assisted_teleop_node.cancel_assisted_teleop()
            assisted_teleop_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
