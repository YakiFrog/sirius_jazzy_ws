#!/usr/bin/env python3
# Copyright 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
自律移動をキャンセルするプログラム
このプログラムは実行中のナビゲーションタスクをキャンセルします。
RViz2から送信されたNav2Goalも含めて、すべてのナビゲーションタスクをキャンセルできます。
"""

import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo


class NavigationCanceler(Node):
    """ナビゲーションタスクをキャンセルするノード"""

    def __init__(self):
        super().__init__('navigation_canceler')
        
        # キャンセルサービスのリスト
        self.cancel_services = [
            ('NavigateToPose', '/navigate_to_pose/_action/cancel_goal'),
            ('NavigateThroughPoses', '/navigate_through_poses/_action/cancel_goal'),
            ('FollowWaypoints', '/follow_waypoints/_action/cancel_goal'),
        ]

    def cancel_all_navigation_tasks(self):
        """すべてのナビゲーションタスクをキャンセル"""
        print('ナビゲーションシステムに接続中...')
        print()
        
        canceled_any = False
        
        # すべてのキャンセルクライアントを事前に作成して準備
        cancel_clients = []
        for action_name, service_name in self.cancel_services:
            client = self.create_client(CancelGoal, service_name)
            cancel_clients.append((action_name, service_name, client))
        
        # 各サービスが利用可能になるまで待つ（最大5秒）
        print('アクションサーバーの準備を待っています...')
        import time
        start_time = time.time()
        ready_clients = []
        
        while time.time() - start_time < 5.0:
            for action_name, service_name, client in cancel_clients:
                if (action_name, service_name, client) not in ready_clients:
                    if client.wait_for_service(timeout_sec=0.1):
                        ready_clients.append((action_name, service_name, client))
                        print(f'  ✓ {action_name} の準備完了')
            
            if len(ready_clients) == len(cancel_clients):
                break
        
        print()
        
        # 各アクションのゴールをキャンセル
        for action_name, service_name, client in ready_clients:
            try:
                # 空のリクエストを送信（すべてのゴールをキャンセル）
                request = CancelGoal.Request()
                request.goal_info = GoalInfo()
                
                print(f'{action_name} のゴールをキャンセル中...')
                future = client.call_async(request)
                
                # 結果を待つ
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                
                if future.done():
                    result = future.result()
                    if result and len(result.goals_canceling) > 0:
                        print(f'  ✓ {len(result.goals_canceling)}個のゴールをキャンセルしました。')
                        canceled_any = True
                    else:
                        print(f'  → キャンセルするゴールがありません。')
                else:
                    print(f'  ✗ タイムアウトしました。')
                    
            except Exception as e:
                print(f'  ✗ エラー: {e}')
        
        # クライアントをクリーンアップ
        for _, _, client in cancel_clients:
            client.destroy()
        
        print()
        if not canceled_any:
            print('実行中のナビゲーションタスクはありませんでした。')
        else:
            print('✓ ナビゲーションタスクのキャンセルが完了しました。')


def main():
    """メイン関数：ナビゲーションタスクをキャンセル"""
    rclpy.init()

    canceler = NavigationCanceler()
    
    try:
        canceler.cancel_all_navigation_tasks()
    except Exception as e:
        print(f'エラーが発生しました: {e}')
    finally:
        # クリーンアップ
        canceler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
