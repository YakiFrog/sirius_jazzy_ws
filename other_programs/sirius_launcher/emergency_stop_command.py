"""One-shot /stop command. Importing this module never connects to ROS."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


def send_emergency_stop(active, domain):
    script = Path(__file__).resolve()
    workspace = script.parents[2]
    command = ('source /opt/ros/jazzy/setup.bash && '
               f'source {shlex.quote(str(workspace/"install/setup.bash"))} && '
               f'exec python3 {shlex.quote(str(script))} --publish {"on" if active else "off"}')
    try:
        result = subprocess.run(['bash', '-c', command], stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=8,
                                env={**os.environ, 'ROS_DOMAIN_ID': str(domain or '0')})
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'sent': False, 'error': '停止指令の送信を確認できません: '+str(error)[:200]}
    if result.returncode:
        return {'sent': False, 'error': result.stderr.strip()[-400:] or '停止指令の送信に失敗しました'}
    return {'sent': True, 'active': active}


def wait_for_driver(node, publisher, spin, timeout=3):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        subscribers = node.get_subscriptions_info_by_topic('/stop')
        if (any(s.node_name == 'roboteq_ros2_driver' for s in subscribers)
                and publisher.get_subscription_count() >= len(subscribers)):
            break
        spin(node, timeout_sec=.1)
    else:
        raise RuntimeError('Roboteqの /stop 接続を確認できないため送信しませんでした')


def publish_stop(node, publisher, active, spin, duration, timeout=3, feedback=None):
    wait_for_driver(node, publisher, spin, timeout)
    from std_msgs.msg import Bool
    message = Bool()
    message.data = active
    if feedback is not None:
        feedback.clear()
    publisher.publish(message)
    if feedback is not None:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            spin(node, timeout_sec=.1)
            if feedback.get('active') is active:
                return
        raise RuntimeError('実機の停止状態の反映が未確認です。停止表示を確認してください')
    if not publisher.wait_for_all_acked(duration(seconds=1)):
        raise RuntimeError('送信後の応答が未確認です。実機の停止表示を確認してください')


def main():
    import argparse
    import rclpy
    from rclpy.duration import Duration
    from std_msgs.msg import Bool
    from roboteq_ros2_driver.msg import RoboteqStatus
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--publish', choices=('on', 'off'))
    mode.add_argument('--check', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('sirius_launcher_emergency_stop')
    try:
        publisher = node.create_publisher(Bool, '/stop', 10)
        if args.check:
            wait_for_driver(node, publisher, rclpy.spin_once)
            print(json.dumps({'ready': True, 'published': False}))
            return
        feedback = {}
        subscription = node.create_subscription(
            RoboteqStatus, '/roboteq/status',
            lambda msg: feedback.update(active=bool(msg.fault_flags & 16)), 10)
        publish_stop(node, publisher, args.publish == 'on', rclpy.spin_once, Duration,
                     feedback=feedback)
        print(json.dumps({'sent': True}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
