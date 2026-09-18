#!/bin/bash
# Nav2 のビヘイビアツリー(XML)をスタック再起動なしで再読込する。
#
# 方式: lifecycle_manager の manage_nodes を使って「RESET -> STARTUP」する。
#   - RESET    : 全 managed node を unconfigured へ（active なら deactivate+cleanup）
#   - STARTUP  : 全 managed node を configure + activate
#   bt_action_server は on_cleanup() で BT のファイル名キャッシュをクリアし、
#   on_activate() で XML をディスクから読み直すため、STARTUP 完了時に
#   編集済み BT が反映される。
#
# 以前の「bt_navigator だけを lifecycle set で cycle する」方式は、bond が
# 途切れて lifecycle_manager が全ノードを落とすことがあったため廃止。
# この方式は manager 自身が遷移するので bond は壊れない。
#
# 注意: ナビゲーションは数十秒停止する（goal はキャンセルされる）。

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "${HOME}/sirius_jazzy_ws/install/setup.bash" ]; then
    source "${HOME}/sirius_jazzy_ws/install/setup.bash"
fi

exec python3 - <<'PY'
import sys
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger

MANAGER = '/lifecycle_manager_navigation'
rclpy.init()
node = rclpy.create_node('bt_reload_client')

manage = node.create_client(ManageLifecycleNodes, f'{MANAGER}/manage_nodes')
is_active_cli = node.create_client(Trigger, f'{MANAGER}/is_active')
if not manage.wait_for_service(timeout_sec=10.0):
    print(f'ERROR: service {MANAGER}/manage_nodes not available', file=sys.stderr)
    node.destroy_node(); rclpy.shutdown(); sys.exit(1)
is_active_cli.wait_for_service(timeout_sec=5.0)


def call_manage(command, timeout):
    req = ManageLifecycleNodes.Request()
    req.command = command
    future = manage.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        return False
    return bool(future.result().success)


def is_active():
    future = is_active_cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done() or future.result() is None:
        return None
    return bool(future.result().success)


start = time.time()
print('== RESET (unconfigure all managed nodes) ==')
# ManageLifecycleNodes: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4
reset_ok = call_manage(3, 90.0)
print(f'RESET success={reset_ok}')

print('== STARTUP (configure + activate, reload BT from disk) ==')
startup_ok = call_manage(0, 120.0)
active = is_active()
print(f'STARTUP success={startup_ok} manager_is_active={active} elapsed={time.time()-start:.1f}s')

node.destroy_node()
rclpy.shutdown()
sys.exit(0 if (startup_ok and active) else 1)
PY
