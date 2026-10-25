"""Read-only navigation telemetry. No publishers, services or action clients."""
import math
import time

PREFLIGHT_NODES = ('bt_navigator', 'planner_server', 'controller_server',
                   'amcl', 'map_server', 'velocity_smoother', 'collision_monitor')

class NavigationState:
    def __init__(self):
        self.values = {}

    def ingest(self, key, value, now):
        self.values[key] = (now, value)

    def snapshot(self, now):
        def entry(key, stale_after=3):
            item = self.values.get(key)
            age = None if item is None else max(0, now-item[0])
            return {'age': age, 'fresh': age is not None and age < stale_after,
                    'value': item[1] if item else None}
        data = {key: entry(key) for key in (
            'goal', 'path', 'nav', 'safe', 'output', 'lock', 'driver', 'motion', 'event')}
        def fresh_value(key):
            return data[key]['value'] if data[key]['fresh'] else None
        nav, safe = fresh_value('nav'), fresh_value('safe')
        driver, lock = fresh_value('driver'), fresh_value('lock')
        mismatch = (bool(safe and safe['moving']) and lock is True
                    and driver is False and not data['output']['fresh'])
        if mismatch:
            summary = '停止状態の不一致：Roboteq解除済み・muxロック中・最終速度出力なし'
        elif driver is True:
            summary = 'Roboteqの緊急停止が有効です'
        elif lock is True:
            summary = 'muxの停止ロックが有効です'
        elif safe and safe['moving'] and not data['output']['fresh']:
            summary = 'mux入力あり・最終速度出力なし。経路や接続状態の確認が必要です'
        elif nav and nav['moving'] and not (safe and safe['moving']):
            summary = 'Nav2指令あり・安全監視通過後の非ゼロ指令を確認できません'
        elif fresh_value('output') and fresh_value('output')['moving']:
            summary = ('速度指令と車輪Odomの動きを確認' if fresh_value('motion')
                       else '最終速度指令あり。車輪Odomの動きは未確認です')
        else:
            summary = '受信データを下の各段階で確認してください'
        configured = entry('scan_topic', 30)
        topic = configured['value'] if configured['fresh'] else None
        scan = entry('laser/'+topic) if topic else entry('missing_scan')
        return {'summary': summary, 'mismatch': mismatch, 'stages': data,
                'preflight': {'nodes': {name: entry('lifecycle/'+name, 6) for name in PREFLIGHT_NODES},
                              'scan': {**scan, 'topic': topic},
                              'communication': entry('driver_communication'),
                              'routing': entry('routing', 6)}}


class NavigationReceiver:
    def __init__(self, node):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Path, Odometry
        from action_msgs.msg import GoalStatusArray
        from diagnostic_msgs.msg import DiagnosticArray
        from roboteq_ros2_driver.msg import RoboteqDiagnostics
        from rcl_interfaces.msg import Log
        from rcl_interfaces.srv import GetParameters
        from lifecycle_msgs.srv import GetState
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        self.state = NavigationState()
        self.subscriptions = []
        def receive(kind, convert):
            return lambda msg: self.state.ingest(kind, convert(msg), time.monotonic())
        def velocity(msg):
            x, yaw = msg.linear.x, msg.angular.z
            return {'x': x if math.isfinite(x) else None,
                    'yaw': yaw if math.isfinite(yaw) else None,
                    'moving': abs(x) > .001 or abs(yaw) > .001}
        def subscribe(typ, topic, cb):
            self.subscriptions.append(node.create_subscription(typ, topic, cb, qos_profile_sensor_data))
        for key, topic in [('nav', '/cmd_vel_nav'), ('safe', '/cmd_vel_nav_safe'), ('output', '/cmd_vel')]:
            subscribe(Twist, topic, receive(key, velocity))
        subscribe(Path, '/plan', receive('path', lambda m: len(m.poses)))
        subscribe(Odometry, '/odom', receive('motion', lambda m: velocity(m.twist.twist)['moving']))
        def driver(msg):
            now = time.monotonic()
            self.state.ingest('driver', bool(msg.estop_active), now)
            self.state.ingest('driver_communication', bool(msg.status_flags_valid
                and msg.status_flags_age < 3 and msg.command_write_ok_ch1
                and msg.command_write_ok_ch2), now)
        subscribe(RoboteqDiagnostics, '/roboteq/diagnostics', driver)
        def goals(msg):
            if msg.status_list:
                latest = max(msg.status_list, key=lambda s: (
                    s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
                self.state.ingest('goal', latest.status, time.monotonic())
        subscribe(GoalStatusArray, '/navigate_to_pose/_action/status', goals)
        def diagnostics(msg):
            for status in msg.status:
                if status.name != 'twist_mux: Twist mux status':
                    continue
                for value in status.values:
                    text = value.value.strip()
                    if value.key == 'lock locks.stop' and text.startswith(('locked ', 'free ')):
                        self.state.ingest('lock', text.startswith('locked '), time.monotonic())
        subscribe(DiagnosticArray, '/diagnostics', diagnostics)
        def logs(msg):
            if msg.name in ('bt_navigator', 'controller_server', 'planner_server', 'nav2_goal_client'):
                if msg.level >= 30 or any(s in msg.msg for s in (
                        'Goal accepted', 'Goal REJECTED', 'InitialPose', 'Goal succeeded', 'Goal failed')):
                    self.state.ingest('event', {'node': msg.name, 'text': msg.msg[:500]}, time.monotonic())
        subscribe(Log, '/rosout', logs)
        # Only lifecycle state/configuration queries: never change state, goals,
        # parameters, power outputs or emergency stops.
        clients = {name: node.create_client(GetState, '/'+name+'/get_state')
                   for name in PREFLIGHT_NODES}
        scan_client = node.create_client(GetParameters, '/amcl/get_parameters')
        pending = {}
        self.scan_subscription = None
        self.scan_topic = None
        self.last_scan_query = -float('inf')

        def read_scan(msg, topic):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            source_age = time.time()-stamp
            self.state.ingest('laser/'+topic,
                {'samples': len(msg.ranges), 'valid': any(not math.isnan(v) for v in msg.ranges),
                 'stamp_current': stamp > 0 and -.5 < source_age < 3}, time.monotonic())

        def poll():
            now = time.monotonic()
            routes = [('/cmd_vel_nav', 'controller_server', 'velocity_smoother'),
                      ('/cmd_vel_smoothed', 'velocity_smoother', 'collision_monitor'),
                      ('/cmd_vel_nav_safe', 'collision_monitor', 'twist_mux'),
                      ('/cmd_vel', 'twist_mux', 'roboteq_ros2_driver')]
            missing = []
            for topic, publisher, subscriber in routes:
                pubs = node.get_publishers_info_by_topic(topic)
                subs = node.get_subscriptions_info_by_topic(topic)
                if not any(e.node_name == publisher and e.topic_type == 'geometry_msgs/msg/Twist' for e in pubs):
                    missing.append(topic+' 配信元 '+publisher)
                if not any(e.node_name == subscriber and e.topic_type == 'geometry_msgs/msg/Twist' for e in subs):
                    missing.append(topic+' 購読先 '+subscriber)
            self.state.ingest('routing', {'connected': not missing, 'missing': missing}, now)
            for key, client in [*clients.items(), ('scan_topic', scan_client)]:
                previous = pending.get(key)
                if previous and not previous[0].done():
                    if now-previous[1] < 4:
                        continue
                    previous[0].cancel()
                if key == 'scan_topic' and now-self.last_scan_query < 10:
                    continue
                if not client.service_is_ready():
                    continue
                if key == 'scan_topic':
                    request = GetParameters.Request(); request.names = ['scan_topic']
                    self.last_scan_query = now
                else:
                    request = GetState.Request()
                future = client.call_async(request)
                pending[key] = (future, now)
                def done(f, key=key, sent=now):
                    if f.cancelled() or pending.get(key, (None,))[0] is not f or time.monotonic()-sent > 4:
                        return
                    try:
                        response = f.result()
                        if key != 'scan_topic':
                            state = response.current_state
                            self.state.ingest('lifecycle/'+key,
                                {'active': state.id == 3, 'label': state.label}, time.monotonic())
                        else:
                            topic = response.values[0].string_value
                            if not topic:
                                return
                            topic = '/amcl/'+topic[2:] if topic.startswith('~/') else '/'+topic.lstrip('/')
                            if topic != self.scan_topic:
                                if self.scan_subscription:
                                    node.destroy_subscription(self.scan_subscription)
                                self.scan_topic = topic
                                self.scan_subscription = node.create_subscription(
                                    LaserScan, topic, lambda msg, topic=topic: read_scan(msg, topic), qos_profile_sensor_data)
                            self.state.ingest('scan_topic', topic, time.monotonic())
                    except Exception:
                        # Existing state expires to unknown; a failed read is not
                        # evidence that the robot can run.
                        pass
                future.add_done_callback(done)
        self.preflight_timer = node.create_timer(2.0, poll)
