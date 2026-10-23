#!/usr/bin/env python3
"""Record the raw inputs needed for reproducible offline 3D mapping.

録画前に必須トピック・TF（シミュではRGBフィールド）の受信を検査して表示し、
録画可否を判定する（NGがあれば開始しない）。対話端末では bag名・メモ・
開始可否を確認する。`--yes` を付けると確認をスキップ（自動化向け）。
"""
import argparse
import hashlib
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import numpy as np
import yaml


def _line(mark, message):
    print(f'  {mark} {message}', flush=True)


def required_topics(mode, profile='mapping'):
    topics = ['/velodyne_points', '/imu', '/odom']
    if mode == 'sim':
        topics.append('/clock')
    if profile == 'all_mapping':
        topics += ['/tf', '/tf_static', '/scan3']
    if profile == 'localization':
        topics += ['/Odometry', '/tf', '/tf_static', '/map_localizer/pose',
                   '/map_localizer/status', '/localization/map', '/localization_ready',
                   '/fastlio/diagnostics', '/map_localizer/tf_status', '/map_localizer/correction']
    return topics


def recording_topics(mode, profile='mapping', include_derived_clouds=False):
    topics = ['/velodyne_points', '/imu', '/odom', '/scan3', '/scan',
              '/odom/filtered', '/odometry/filtered', '/magnetometer', '/Odometry',
              '/tf', '/tf_static', '/theta/dual_fisheye/image_raw/compressed',
              '/fastlio/diagnostics', '/diagnostics', '/rosout', '/roboteq/status',
              '/camera/stereo_sbs/compressed', '/camera/stereo_params',
              '/robot_description', '/joint_states', '/cmd_vel', '/cmd_vel_nav',
              '/cmd_vel_teleop', '/parameter_events', '/roboteq/odom', '/experiment_metadata']
    if mode == 'sim':
        topics += ['/clock', '/ground_truth/odom', '/sirius_web/time_status', '/sirius_web/sensor_config']
    if profile == 'localization':
        topics += ['/map_localizer/pose', '/map_localizer/status', '/initialpose',
                   '/localization/map', '/cloud_registered_body',
                   '/map_localizer/correction', '/map_localizer/tf_status', '/localization_ready']
    if include_derived_clouds:
        topics.append('/velodyne_points_colored')
        if profile == 'localization':
            topics.append('/velodyne_points_leveled')
    return topics


def validate_all_mapping(received, transforms):
    """Require core sensor TF; report optional camera limitations without blocking."""
    import math
    if not received['/scan3'].ranges:
        raise ValueError('/scan3: スキャンが空です')
    graph = {}
    for transform in transforms:
        a, b = transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/')
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)
    base = received['/odom'].child_frame_id.lstrip('/')
    if not base:
        raise ValueError('/odom: child_frame_id が空です')
    reachable, pending = set(), [base]
    while pending:
        frame = pending.pop()
        if frame not in reachable:
            reachable.add(frame)
            pending.extend(graph.get(frame, ()))
    for topic in ('/velodyne_points', '/imu', '/scan3'):
        frame = received[topic].header.frame_id.lstrip('/')
        if not frame or frame not in reachable:
            raise ValueError(f'{topic}: {base} から {frame!r} へのTFがありません')

    warnings = []
    for label, topic in [('THETA', '/theta/dual_fisheye/image_raw/compressed'),
                         ('ZED', '/camera/stereo_sbs/compressed')]:
        image = received.get(topic)
        reason = None
        if image is None or not image.data:
            reason = '画像未受信'
        elif image.header.frame_id.lstrip('/') not in reachable:
            reason = 'カメラTF未接続'
        if label == 'ZED' and reason is None:
            try:
                params = json.loads(received['/camera/stereo_params'].data)
                for key in ('fx', 'fy', 'baseline', 'width', 'height', 'cx', 'cy'):
                    value = float(params[key])
                    if not math.isfinite(value) or (key not in ('cx', 'cy') and value <= 0):
                        raise ValueError(key)
            except (ValueError, TypeError, KeyError):
                reason = '内部パラメータ未受信/不正'
        if reason:
            warnings.append(f'{label}: {reason}。{label}路面マッピング' +
                            ('・THETA色付け' if label == 'THETA' else '') + 'はこの状態ではできません')
    return warnings


def snapshot_mapping(capture, ws):
    """Keep configuration sources beside raw inputs; TF messages remain authoritative."""
    paths = ['src/sirius/sirius_navigation/config', 'params',
             'bash/startup_bash/start_3d_lio.sh', 'bash/startup_bash/start_theta_capture.sh']
    for relative in paths:
        source = ws/relative
        if source.is_file():
            shutil.copyfile(source, capture/source.name)
        elif source.is_dir():
            target = capture/('navigation_config' if relative.endswith('/config') else 'params')
            shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__'))
    revision = subprocess.run(['git', '-C', str(ws), 'rev-parse', 'HEAD'],
                              capture_output=True, text=True, timeout=5)
    (capture/'workspace_revision.txt').write_text(revision.stdout)


def dump_runtime_parameters(node):
    result = subprocess.run(['ros2', 'param', 'dump', node, '--no-daemon',
                             '--spin-time', '2', '--timeout', '8'],
                            capture_output=True, text=True, timeout=20)
    if result.returncode:
        raise RuntimeError(f'実行時設定を取得できません ({node}): {result.stderr.strip()}')
    return result.stdout


def snapshot_localization(capture, ws):
    dumped = dump_runtime_parameters('/map_localizer')
    params = yaml.safe_load(dumped)['/map_localizer']['ros__parameters']
    (capture/'map_localizer.yaml').write_text(dumped)
    (capture/'localization_tf.yaml').write_text(dump_runtime_parameters('/localization_tf'))
    try:
        (capture/'witmotion.yaml').write_text(dump_runtime_parameters('/witmotion'))
    except RuntimeError as error:
        # Simulated IMU publishers do not run the hardware driver.
        (capture/'witmotion_unavailable.txt').write_text(str(error)+'\n')
    source = Path(params['map_path']).expanduser().resolve(strict=True)
    reference = capture/('reference_map'+source.suffix)
    shutil.copyfile(source, reference)
    versions = {}
    for name, root in [('workspace', ws), ('navigation', ws/'src/sirius/sirius_navigation'),
                       ('fast_lio', ws/'src/FAST_LIO_ROS2')]:
        revision = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                                  capture_output=True, text=True, timeout=5)
        versions[name] = revision.stdout.strip() if revision.returncode == 0 else None
    (capture/'localization.json').write_text(json.dumps({
        'source_map': str(source), 'reference_map': reference.name,
        'map_sha256': hashlib.sha256(reference.read_bytes()).hexdigest(),
        'versions': versions, 'absolute_ground_truth': 'not recorded on a real robot'}, indent=2)+'\n')


def preflight(mode, timeout, domain, profile='mapping'):
    """必須トピックの受信・RGBフィールド・TFを検査し、結果を表示する。"""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, Imu, CompressedImage, LaserScan
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_msgs.msg import TFMessage

    print('=================================================', flush=True)
    print(f' 3D録画 事前チェック  (profile={profile}, mode={mode}, ROS_DOMAIN_ID={domain})', flush=True)
    print('=================================================', flush=True)
    print('[必須トピック]', flush=True)

    types = {'/velodyne_points': PointCloud2, '/imu': Imu, '/odom': Odometry,
             '/clock': Clock, '/velodyne_points_colored': PointCloud2,
             '/Odometry': Odometry, '/tf': TFMessage, '/tf_static': TFMessage,
             '/map_localizer/pose': PoseStamped, '/map_localizer/status': String,
             '/localization/map': PointCloud2, '/localization_ready': Bool,
             '/fastlio/diagnostics': String, '/map_localizer/tf_status': String,
             '/map_localizer/correction': TransformStamped, '/scan3': LaserScan,
             '/theta/dual_fisheye/image_raw/compressed': CompressedImage,
             '/camera/stereo_sbs/compressed': CompressedImage, '/camera/stereo_params': String}
    required = {topic: types[topic] for topic in required_topics(mode, profile)}
    rgb_topic = '/velodyne_points' if mode == 'sim' else None
    # 任意トピックはメッセージ受信ではなくPublisher有無で判定する（/tf_staticはlatchedのため）
    optional = {
        '/tf': '任意。RTAB-Map再生や他ツール向け（offline_3dmapでは未使用）',
        '/tf_static': '任意。同上（latched配信）',
        '/Odometry': '任意。FAST-LIOのLiDARオドメトリ（大文字O。/odomとは別物。offline_3dmapでは未使用・比較用）',
        '/theta/dual_fisheye/image_raw/compressed': '任意。THETA色付け/BEV系ツール用',
        '/scan3': '任意。slam_toolbox用の2Dスキャン（配信があれば記録）',
        '/scan': '任意。別系統の2Dスキャン（配信があれば記録）',
        '/camera/stereo_sbs/compressed': '任意。ZED画像を後処理する場合に必要',
        '/camera/stereo_params': '任意。ZED画像の内部パラメータ',
    }

    rclpy.init(args=[])
    node = rclpy.create_node('record_3d_preflight')
    received = {}
    transforms = []

    def receive(topic, message):
        received[topic] = message
        if topic in ('/tf', '/tf_static'):
            transforms.extend(message.transforms)

    sensor_profiles = []
    profile_sub = node.create_subscription(String, '/sirius_web/sensor_config', lambda msg: sensor_profiles.append(msg.data), 10) if mode == 'sim' else None
    subscription_types = dict(required)
    if profile == 'all_mapping':
        from rosidl_runtime_py.utilities import get_message
        discovery_end = time.monotonic() + 1.0
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=.1)
        wanted = set(recording_topics(mode, profile))
        subscription_types.update({topic: typ for topic, typ in types.items() if topic in wanted})
        for topic, names in node.get_topic_names_and_types():
            if topic in wanted and topic not in subscription_types and len(names) == 1:
                subscription_types[topic] = get_message(names[0])
    subs = [node.create_subscription(typ, topic,
                                     lambda msg, t=topic: receive(t, msg),
                                     QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                                     if topic in ('/tf_static', '/camera/stereo_params', '/robot_description', '/localization/map', '/map_localizer/correction', '/localization_ready') else qos_profile_sensor_data)
            for topic, typ in subscription_types.items()]
    end = time.monotonic() + timeout
    while (profile == 'all_mapping' or not set(required) <= set(received)) and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)

    # 任意トピックのPublisher検出はディスカバリ完了を少し待つ
    settle_end = time.monotonic() + 1.0
    while time.monotonic() < settle_end:
        rclpy.spin_once(node, timeout_sec=.1)

    ng = 0
    for topic in required:
        if topic in received:
            _line('✓', f'{topic} を受信')
        else:
            _line('✗', f'{topic} を受信できません（未起動/型違い/時間切れ）')
            ng += 1
    if profile == 'localization' and '/localization_ready' in received:
        if not received['/localization_ready'].data:
            _line('✗', '自己位置推定が準備未完了です。/map_localizer/tf_status を確認してください')
            ng += 1
    if profile == 'all_mapping':
        print('[記録対象トピックの受信状況：未配信も録画対象に残し、途中起動時に収録]', flush=True)
        for topic in recording_topics(mode, profile):
            status = '受信済み' if topic in received else '配信元あり・未受信' if node.count_publishers(topic) else '未配信'
            _line('✓' if topic in received else '▲', f'{topic}: {status}')
    for topic, note in ({} if profile == 'all_mapping' else optional).items():
        if topic in required:
            continue
        try:
            pubs = node.get_publishers_info_by_topic(topic)
        except Exception:  # noqa: BLE001
            pubs = []
        if not pubs:
            try:
                pubs = [1] if node.count_publishers(topic) else []
            except Exception:  # noqa: BLE001
                pubs = []
        if pubs:
            _line('✓', f'{topic} Publisherあり（{note}）')
        else:
            _line('▲', f'{topic} がありません（{note}）')

    cloud = received.get(rgb_topic) if profile == 'mapping' else None
    if cloud is not None:
        fields = {f.name for f in cloud.fields}
        if {'rgb', 'rgba'} & fields:
            _line('✓', f'{rgb_topic} にRGBフィールドあり')
            if mode == 'sim':
                # リング色(16色)かTHETA色かを色数で見分ける。色付けOFFでもRGBは付くため。
                try:
                    from sensor_msgs_py.point_cloud2 import read_points
                    p = read_points(cloud, skip_nans=True)
                    f = 'rgb' if 'rgb' in p.dtype.names else 'rgba'
                    packed = np.ascontiguousarray(p[f]).reshape(-1)
                    packed = packed.view(np.uint32) if packed.dtype.kind == 'f' else packed.astype(np.uint32)
                    colors = np.unique(np.stack([(packed >> 16) & 255, (packed >> 8) & 255,
                                                 packed & 255], 1), axis=0)
                    unique = len(colors)
                    if unique == 1 and colors[0, 0] == colors[0, 1] == colors[0, 2]:
                        _line('▲', '点群は灰色です。THETA映像の取得・更新待ちの可能性があります。'
                                   'Webシミュを表示して「THETA映像で色付け中」を確認してください')
                    elif unique <= 32:
                        _line('▲', f'点群の色が{unique}色のみです。リング色か自然色かは色数だけでは断定できません。'
                                   'WebシミュのTHETA色付け状態を確認してください')
                    else:
                        _line('✓', f'点群の色数={unique}（THETA色付けONの可能性）')
                except Exception as error:  # noqa: BLE001
                    _line('▲', f'色数の判定に失敗: {error}')
        else:
            _line('✗', f'{rgb_topic} にRGBがありません'
                       + ('（Webシミュの「THETA 360°映像で色付け」をON）' if mode == 'sim'
                          else '（theta_colorize を起動）'))
            ng += 1
    # A source YAML may differ from the installed/running node. Record the actual
    # wheel-fusion switch so the offline backend does not count encoders twice.
    fastlio_runtime = None
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import parameter_value_to_python
    for name, namespace in node.get_node_names_and_namespaces():
        if name not in ('laserMapping', 'laser_mapping', 'fast_lio', 'lio_mapping'):
            continue
        remote = namespace.rstrip('/')+'/'+name
        client = AsyncParameterClient(node, remote)
        if not client.wait_for_services(timeout_sec=.3):
            continue
        names = ['wheel.enabled', 'wheel.topic', 'wheel.child_frame', 'wheel.imu_to_base_translation',
                 'wheel.imu_from_base_rotation', 'wheel.velocity_std', 'wheel.yaw_rate_std',
                 'common.lid_topic', 'common.imu_topic', 'common.time_offset_lidar_to_imu',
                 'preprocess.timestamp_unit', 'preprocess.scan_rate', 'preprocess.normalize_point_time',
                 'mapping.extrinsic_R', 'mapping.extrinsic_T', 'mapping.extrinsic_est_en',
                 'mapping.acc_cov', 'mapping.gyr_cov',
                 'lidar_quality.enabled', 'lidar_quality.min_points', 'lidar_quality.min_ratio',
                 'lidar_quality.max_residual', 'lidar_quality.weak_geometry_ratio',
                 'lidar_quality.degraded_weight', 'lidar_quality.recovery_scans',
                 'lidar_quality.max_position_correction', 'lidar_quality.max_rotation_correction']
        future = client.get_parameters(names)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        if future.done() and future.exception() is None and future.result() is not None:
            values = future.result().values
            fastlio_runtime = {'node': remote, 'wheel_enabled': parameter_value_to_python(values[0]),
                               'wheel_topic': parameter_value_to_python(values[1]), 'source': 'running node parameters',
                               'parameters': {key: parameter_value_to_python(value) for key, value in zip(names, values)
                                              if value.type != 0}}
            break
    if profile == 'all_mapping' and set(required) <= set(received):
        try:
            warnings = validate_all_mapping(received, transforms)
            for warning in warnings:
                _line('▲', warning)
            _line('✓', '3D・2D地図用の基本入力とセンサーTFを確認（カメラは任意）')
        except ValueError as error:
            _line('✗', str(error))
            ng += 1
    node.destroy_node()
    rclpy.shutdown()

    if ng:
        raise ValueError(f'必須項目が{ng}件不足しています（上記✗を解消してください）')
    print('✓ 必須センサーを受信しました', flush=True)
    cloud = received['/velodyne_points']
    wheel = received['/odom']
    if mode == 'sim' and 'time' in {f.name for f in cloud.fields} and not sensor_profiles:
        raise ValueError('回転点群を受信しましたが /sirius_web/sensor_config が未受信です。WebのROS接続を確認して再実行してください')
    return {'lidar_frame': cloud.header.frame_id, 'wheel_frame': wheel.header.frame_id,
            'wheel_child_frame': wheel.child_frame_id,
            '_sensor_profile': json.loads(sensor_profiles[-1]) if sensor_profiles else None,
            '_fastlio_runtime': fastlio_runtime}


def ask(prompt, default=''):
    try:
        value = input(prompt).strip()
    except EOFError:
        return None
    return value if value else default


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['sim', 'real'], nargs='?', default='sim')
    parser.add_argument('name', nargs='?')
    parser.add_argument('--directory', default=str(Path.home()/'rosbag2_data'))
    parser.add_argument('--config')
    parser.add_argument('--profile', choices=['mapping', 'localization', 'all_mapping'], default='mapping')
    parser.add_argument('--include-derived-clouds', action='store_true',
                        help='色付き・傾き補正済み点群も記録（通常は生点群と画像から再生成）')
    parser.add_argument('--duration', type=float, help='Stop after this many wall-clock seconds')
    parser.add_argument('--preflight-timeout', type=float, default=10.)
    parser.add_argument('--yes', action='store_true', help='確認プロンプトをスキップして録画開始')
    args = parser.parse_intermixed_args(argv)
    if args.duration is not None and args.duration <= 0:
        parser.error('--duration must be positive')

    def validate_name(value):
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', value) or value in ('.', '..'):
            parser.error('Bag name must use letters, numbers, periods, hyphens or underscores')
        return value

    name = validate_name(args.name) if args.name else datetime.now(
        ZoneInfo('Asia/Tokyo')).strftime(f'{"localization3d" if args.profile == "localization" else "mapping_all" if args.profile == "all_mapping" else "slam3d"}_{args.mode}_%Y%m%d_%H%M%S')
    ws = Path(__file__).resolve().parents[2]
    config = Path(args.config).expanduser() if args.config else (
        ws/'params/fast_lio_sirius_sim.yaml' if args.mode == 'sim' else
        ws/'src/sirius/sirius_navigation/config/fast_lio_sirius.yaml')
    if not config.is_file():
        parser.error(f'Calibration config not found: {config}')

    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    frames = preflight(args.mode, args.preflight_timeout, domain, args.profile)
    topics = recording_topics(args.mode, args.profile, args.include_derived_clouds)
    omitted = sorted(set(recording_topics(args.mode, args.profile, True)) - set(topics))

    interactive = (sys.stdin.isatty() or args.profile == 'all_mapping') and not args.yes
    memo = ''
    print('', flush=True)
    print('-------------------------------------------------', flush=True)
    print(f' 録画モード : {args.mode}', flush=True)
    print(f' 設定ファイル: {config}', flush=True)
    print(f' トピック   : {", ".join(topics)}', flush=True)
    if omitted:
        print(f' 容量削減   : {", ".join(omitted)} は記録しません（再生成可能）', flush=True)
    if interactive:
        entered_name = ask(f' bag名 [{name}]: ', name)
        if entered_name is None:
            print('入力が閉じているため中止しました。自動実行は --yes を指定してください。', flush=True)
            return 0
        name = validate_name(entered_name)
        bag = Path(args.directory).expanduser().resolve()/name
        if bag.exists():
            parser.error(f'Bag already exists: {bag}')
        memo = ask(' メモ（任意、Enterでスキップ）: ', '')
        if memo is None:
            memo = ''
        print(f' 保存先     : {bag}', flush=True)
        ans = ask(' 録画を開始しますか？ (Y/n): ', 'y')
        if ans is None or ans.lower() not in ('y', 'yes'):
            print('中止しました（録画していません）。', flush=True)
            return 0
    else:
        bag = Path(args.directory).expanduser().resolve()/name
        if bag.exists():
            parser.error(f'Bag already exists: {bag}')
        print(f' 保存先     : {bag}', flush=True)
        print(' 非対話環境のため確認なしで開始します（--yes相当）。', flush=True)

    # Keep sidecars outside the output URI until rosbag creates the directory.
    capture = bag.with_name(bag.name+'.capture')
    capture.mkdir(parents=True, exist_ok=False)
    if args.profile == 'localization':
        snapshot_localization(capture, ws)
    if args.profile == 'all_mapping':
        snapshot_mapping(capture, ws)
    profile = frames.pop('_sensor_profile', None)
    fastlio_runtime = frames.pop('_fastlio_runtime', None)
    if fastlio_runtime is not None:
        (capture/'fastlio_runtime.json').write_text(json.dumps(fastlio_runtime, indent=2)+'\n')
        if args.profile == 'localization':
            (capture/'fastlio.yaml').write_text(dump_runtime_parameters(fastlio_runtime['node']))
    theta_config = ws/'src/sirius/sirius_navigation/config'/('theta_calibration_real.yaml' if args.mode == 'real' else 'theta_calibration.yaml')
    if theta_config.exists():
        shutil.copyfile(theta_config, capture/'theta_calibration.yaml')
    color_policy = ws/'src/sirius/sirius_navigation/config/theta_colorization.yaml'
    if color_policy.exists():
        shutil.copyfile(color_policy, capture/'theta_colorization.yaml')
    if profile is not None and not args.config:
        from web_sensor_profile import apply_profile
        (capture/'calibration.yaml').write_text(yaml.safe_dump(apply_profile(yaml.safe_load(config.read_text()), profile), sort_keys=False))
    else:
        snapshot = config
        if args.mode == 'sim' and profile is None and not args.config:
            snapshot = ws/'params/fast_lio_sirius_sim_ideal.yaml'
        shutil.copyfile(snapshot, capture/'calibration.yaml')
    if profile is not None:
        (capture/'web_sensor_profile.json').write_text(json.dumps(profile, indent=2)+'\n')
    qos = {'/tf_static': {'reliability': 'reliable', 'durability': 'transient_local', 'history': 'keep_last', 'depth': 100},
           '/velodyne_points': {'reliability': 'best_effort', 'durability': 'volatile', 'history': 'keep_last', 'depth': 20},
           '/velodyne_points_colored': {'reliability': 'best_effort', 'durability': 'volatile', 'history': 'keep_last', 'depth': 20},
           '/imu': {'reliability': 'best_effort', 'durability': 'volatile', 'history': 'keep_last', 'depth': 200}}
    if args.profile == 'localization':
        for topic in ('/localization/map', '/robot_description', '/map_localizer/correction', '/localization_ready'):
            qos[topic] = {'reliability': 'reliable', 'durability': 'transient_local', 'history': 'keep_last', 'depth': 1}
    qos_path = capture/'qos.yaml'
    for topic in ('/robot_description', '/camera/stereo_params'):
        qos[topic] = {'reliability': 'reliable', 'durability': 'transient_local', 'history': 'keep_last', 'depth': 1}
    qos_path.write_text(yaml.safe_dump(qos))
    (capture/'recording.json').write_text(json.dumps({'mode': args.mode, 'profile': args.profile, 'topics': topics,
        'include_derived_clouds': args.include_derived_clouds, 'omitted_derived_clouds': omitted,
        'ros_domain_id': domain, 'frames': frames, 'memo': memo,
        'created': datetime.now(ZoneInfo('Asia/Tokyo')).isoformat()}, indent=2)+'\n')
    command = ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
               '--qos-profile-overrides-path', str(qos_path), '-o', str(bag), '--topics', *topics]
    print(f'Recording: {bag}\nStop with Ctrl+C once. Wheel/IMU/LiDAR are required; live SLAM is optional.', flush=True)
    if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    child = subprocess.Popen(command, start_new_session=True)

    def stop(_signum=None, _frame=None):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    try:
        if args.duration:
            try:
                code = child.wait(timeout=args.duration)
            except subprocess.TimeoutExpired:
                stop()
                code = child.wait()
        else:
            code = child.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if bag.is_dir():
        shutil.move(str(capture), str(bag/'capture'))
    metadata = bag/'metadata.yaml'
    if not metadata.is_file():
        raise RuntimeError(f'No finalized bag metadata: {bag}; recorder exit={code}')
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
    required = required_topics(args.mode, args.profile)
    missing = [t for t in required if not counts.get(t, 0)]
    if missing:
        raise RuntimeError('Bag missing required recorded messages: '+', '.join(missing))
    print('Saved counts:', {t: counts[t] for t in required}, flush=True)
    if args.profile == 'localization':
        print('自己位置推定検証用BAGを保存しました。地図・設定は capture/、表示はFoxgloveでMCAPを開いてください。', flush=True)
    elif args.profile == 'all_mapping':
        print('THETA/ZED路面・3D・2D地図用BAGを保存しました。手順: docs/record_mapping.md', flush=True)
    else:
        print(f'Generate maps:\n  offline_3dmap "{bag}"', flush=True)
    return 0 if code in (0, 130, -signal.SIGINT) else code


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, RuntimeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
