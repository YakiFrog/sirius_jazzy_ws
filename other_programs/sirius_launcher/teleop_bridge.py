#!/usr/bin/env python3
"""Browser teleop -> Twist. JSON lines on stdin/stdout; EOF stops output."""
import fcntl
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

from teleop_control import VelocityControl
from battery_monitor import PHYSICAL_STOP_LOW_VOLTAGE, PHYSICAL_STOP_RECOVERED_VOLTAGE


def velocity_route(mode, outputs, inputs):
    """Mux-only mode never falls back to the motor command topic."""
    mux_output = any('twist_mux' in name for name in outputs)
    topic = '/cmd_vel_web' if mode == 'mux' else ('/cmd_vel_teleop' if mux_output else '/cmd_vel')
    if mode == 'mux' and not (mux_output and any('twist_mux' in name for name in inputs)):
        return topic, 'twist_muxの入力・出力接続を確認できません（直接出力しません）'
    return topic, ''


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from roboteq_ros2_driver.msg import RoboteqStatus

    lock = open(Path('/tmp') / f'sirius_launcher_velocity_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    rclpy.init()
    node = rclpy.create_node('sirius_launcher_velocity')
    model = VelocityControl()
    commands = queue.Queue(maxsize=64)
    eof = threading.Event()
    driver = {'at': 0., 'physical': None, 'electronic': None}
    route = {'topic': '/cmd_vel', 'blocked': '接続確認中', 'at': 0., 'mode': 'auto'}
    publishers = {t: node.create_publisher(Twist, t, 1)
                  for t in ('/cmd_vel', '/cmd_vel_teleop', '/cmd_vel_web')}

    def telemetry(msg):
        driver['at'] = time.monotonic()
        driver['electronic'] = bool(msg.fault_flags & 16)
        voltage = float(msg.voltage)
        if voltage <= PHYSICAL_STOP_LOW_VOLTAGE:
            driver['physical'] = True
        elif voltage >= PHYSICAL_STOP_RECOVERED_VOLTAGE:
            driver['physical'] = False

    subscription = node.create_subscription(RoboteqStatus, '/roboteq/status', telemetry,
                                             qos_profile_sensor_data)

    def input_lines():
        try:
            for line in sys.stdin:
                try:
                    data = json.loads(line)
                    if not isinstance(data, dict):
                        continue
                    commands.put_nowait((time.monotonic(), data))
                except (ValueError, queue.Full):
                    eof.set()
                    return
        finally:
            eof.set()
    threading.Thread(target=input_lines, daemon=True).start()

    def refresh_route(now):
        if now-route['at'] < .5:
            return
        route['at'] = now
        names = {p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')
                 if p.node_name != node.get_name()}
        inputs = {p.node_name for p in node.get_subscriptions_info_by_topic('/cmd_vel_web')}
        topic, blocked = velocity_route(route['mode'], names, inputs)
        other = {p.node_name for p in node.get_publishers_info_by_topic(topic)
                 if p.node_name != node.get_name()}
        route['blocked'] = blocked or (('他の速度配信元が起動中: '+', '.join(sorted(other))) if other else '')
        if topic != route['topic'] and model.owner:
            publishers[route['topic']].publish(Twist())
            model.stop('速度指令の経路が変わったため停止しました')
        route['topic'] = topic

    def interlock(now):
        if now-driver['at'] > 2.:
            return 'Roboteqの状態を受信できません'
        if driver['electronic']:
            return '電子緊急停止がONです'
        if driver['physical'] is not False:
            return '物理緊急停止中、または駆動電圧の復帰待ちです'
        return route['blocked']

    def report(request_id=None, error=''):
        data = model.snapshot()
        data.update(owner=model.owner, topic=route['topic'], route_mode=route['mode'], route_blocked=route['blocked'], blocked=interlock(time.monotonic()))
        if request_id is not None:
            data.update(request_id=request_id, ok=not error, error=error)
        print(json.dumps(data, ensure_ascii=False), flush=True)

    previous = time.monotonic()
    zero_ticks = 0
    def tick():
        nonlocal previous, zero_ticks
        now = time.monotonic()
        refresh_route(now)
        was_active = model.owner is not None
        model.tick(now, 0., interlock(now))
        while not commands.empty():
            received_at, data = commands.get_nowait()
            if now-received_at > .5 and data.get('action') not in ('stop', 'disable'):
                error = '遅れて届いた操作を破棄しました'
            else:
                if (data.get('action') == 'arm' and not model.owner
                        and data.get('route_mode', 'auto') in ('auto', 'mux')
                        and type(data.get('seq')) is int
                        and data['seq'] > model.sequences.get(data.get('session'), 0)):
                    route['mode'] = data.get('route_mode', 'auto')
                    route['at'] = 0.
                    refresh_route(now)
                error = model.command(data.get('action'), data.get('session'), data.get('seq'),
                                      now, data.get('key'), interlock(now))
            if data.get('action') in ('stop', 'disable') or (was_active and not model.owner):
                zero_ticks = 3
            report(data.get('request_id'), error)
        if eof.is_set():
            model.stop('操作画面との接続が終了しました')
            publishers[route['topic']].publish(Twist())
            return
        model.tick(now, now-previous, interlock(now))
        previous = now
        if was_active and not model.owner:
            zero_ticks = 3
        if model.owner or zero_ticks:
            msg = Twist()
            msg.linear.x = model.linear
            msg.angular.z = model.angular
            publishers[route['topic']].publish(msg)
            zero_ticks = max(0, zero_ticks-1)
        report()

    timer = node.create_timer(.1, tick)
    try:
        while rclpy.ok() and not eof.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            publishers[route['topic']].publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
