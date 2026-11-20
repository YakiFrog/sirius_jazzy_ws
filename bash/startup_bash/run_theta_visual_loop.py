#!/usr/bin/env python3
"""Generate matched open/closed THETA maps from a saved SLAM Toolbox graph."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
from scipy.spatial import cKDTree
import yaml

from export_theta_costmaps import export_costmaps
from generate_theta_road_map import write_json
from theta_road_pipeline import load_config
from theta_visual_loop.measure_loop import Bag, Odometry, LaserScan, deserialize_message, stamp

HERE = Path(__file__).resolve().parent
NATIVE = HERE/'theta_visual_loop'
GRAPH_OUTPUTS = ('map.pgm', 'map.yaml')


def csv(path):
    return np.genfromtxt(path, delimiter=',', names=True)


def read_dense(bag, topic, pose_npz):
    if pose_npz is not None:
        with np.load(pose_npz, allow_pickle=False) as data:
            return np.column_stack((data['stamp_ns']/1e9, data['poses']))
    rows = []
    for _, _, record in Bag(bag).iter_messages([topic]):
        od = deserialize_message(record.data, Odometry)
        p, q = od.pose.pose.position, od.pose.pose.orientation
        yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        rows.append((stamp(od.header), p.x, p.y, yaw))
    return np.asarray(rows)


def with_graph(dense, graph, output):
    if dense.ndim != 2 or dense.shape[1] != 4 or len(dense) < 2:
        raise ValueError('No continuous planar odometry; supply an EKF trajectory or another topic')
    dense = dense[np.argsort(dense[:, 0], kind='stable')]
    dense = dense[np.r_[True, np.diff(dense[:, 0]) > 0]]
    t = dense[:, 0]
    knots = graph['stamp']
    if (not np.isfinite(dense).all() or len(t) < 2 or np.any(np.diff(knots) <= 0)
            or knots[0] < t[0] or knots[-1] > t[-1]):
        raise ValueError('Continuous odometry must cover both graph endpoints')
    nearest = np.clip(np.searchsorted(t, knots), 1, len(t)-1)
    if np.max(np.minimum(t[nearest]-knots, knots-t[nearest-1])) > .20:
        raise ValueError('Dense odometry does not support a graph timestamp within 0.20 s')
    yaw = np.unwrap(dense[:, 3])
    xx, yy = (np.interp(knots, t, dense[:, i]) for i in (1, 2))
    delta = np.unwrap(graph['yaw']-np.interp(knots, t, yaw))
    c, s = np.cos(delta), np.sin(delta)
    dx, dy = graph['x']-c*xx+s*yy, graph['y']-s*xx-c*yy
    a = np.interp(t, knots, delta)
    c, s = np.cos(a), np.sin(a)
    poses = np.column_stack((np.interp(t, knots, dx)+c*dense[:, 1]-s*dense[:, 2],
                             np.interp(t, knots, dy)+s*dense[:, 1]+c*dense[:, 2],
                             a+yaw))
    np.savez(output, stamp_ns=np.round(t*1e9).astype(np.int64), poses=poses)


def verify_graph_bag(bag, graph_dir, scan_topic):
    poses, ranges = (csv(graph_dir/f) for f in ('poses.csv','endpoint_ranges.csv'))
    for pose in (poses[0], poses[-1]):
        ts = pose['stamp']
        scan = None
        for _, _, row in Bag(bag).iter_messages([scan_topic], int((ts-.3)*1e9), int((ts+.6)*1e9)):
            msg = deserialize_message(row.data, LaserScan)
            if abs(stamp(msg.header)-ts) < 1e-5:
                scan = np.asarray(msg.ranges)
                break
        graph_ranges = ranges[ranges['id']==pose['id']]['range']
        if scan is None or scan.shape != graph_ranges.shape:
            raise ValueError('Saved graph endpoint scan is absent from the selected BAG')
        valid = np.isfinite(graph_ranges) & (graph_ranges > 0) & (graph_ranges < 30)
        if valid.sum() < 30 or np.mean(np.isclose(scan[valid], graph_ranges[valid], atol=1e-4)) < .98:
            raise ValueError('Saved graph scans do not match the selected BAG')


def graph_metrics(folder):
    p = csv(folder/'poses.csv')
    end = csv(folder/'endpoint_points.csv')
    keep = (end['range'] >= 4.) & (end['range'] <= 20.)
    cloud = []
    for mask in (end['id'] <= p['id'][0]+4, end['id'] >= p['id'][-1]-4):
        pts = np.column_stack((end['x'][keep & mask], end['y'][keep & mask]))
        if len(pts) < 30:
            raise ValueError('Too few independent endpoint scan points')
        _, indexes = np.unique(np.floor(pts/.15), axis=0, return_index=True)
        cloud.append(pts[indexes])
    d = np.r_[cKDTree(cloud[0]).query(cloud[1])[0], cKDTree(cloud[1]).query(cloud[0])[0]]
    return {'gap_m': float(np.hypot(p['x'][-1]-p['x'][0], p['y'][-1]-p['y'][0])),
            'lidar_median_m': float(np.median(d)), 'lidar_within_0_5': float(np.mean(d < .5)),
            'path_m': float(np.hypot(np.diff(p['x']), np.diff(p['y'])).sum())}


def passes_open_closed(before, after):
    return (after['gap_m'] <= min(1., before['gap_m']*.6)
            and after['lidar_median_m'] <= min(1., before['lidar_median_m']*.8)
            and after['lidar_within_0_5'] > before['lidar_within_0_5']
            and abs(after['path_m']/before['path_m']-1.) <= .05)


def generate(args, output, trajectory, graph, cache=None):
    command = [sys.executable, HERE/'generate_theta_road_map.py', args.bag, output,
               '--calibration', args.calibration, '--config', args.config, '--trajectory',
               trajectory, '--map-prefix', 'map', '--domain', str(args.domain)]
    if args.no_semantic:
        command.append('--no-semantic')
    if args.max_frames:
        command.extend(['--max-frames', str(args.max_frames)])
    if cache is not None:
        command.extend(['--observations-from', cache])
    if (output/'run_identity.json').is_file():
        command.append('--resume')
    subprocess.run(command, check=True)
    if json.loads((output/'completed.json').read_text()).get('status') != 'complete':
        raise ValueError('Road map did not finish')
    for name in GRAPH_OUTPUTS:
        shutil.copy2(graph/name, output/name)
    export_costmaps(output)


def run(args):
    args.bag = args.bag.resolve()
    root = args.output.resolve()
    if root == args.bag or args.bag in root.parents:
        raise ValueError('Output must be outside the source BAG')
    root.mkdir(parents=True, exist_ok=True)
    with (root/'.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return run_locked(args, root)


def run_locked(args, root):
    state = {'bag': str(args.bag), 'graph': str(args.graph.resolve()), 'open': 'pending',
             'closed': 'pending', 'full_bag': not bool(args.max_frames)}
    status = root/'loop-status.json'

    def save():
        write_json(status, state)

    if (root/'open').exists() or (root/'closed').exists():
        raise ValueError('Select a new paired-output directory; existing results are preserved')
    save()
    work = root/'.theta-loop-work'
    work.mkdir(exist_ok=True)
    build = work/'build'
    subprocess.run(['cmake', '-S', NATIVE, '-B', build, '-DCMAKE_BUILD_TYPE=Release'], check=True)
    subprocess.run(['cmake', '--build', build, '-j2'], check=True)
    exe = build/'apply_loop'
    before, after = (work/k for k in ('before', 'after'))
    subprocess.run([exe, args.graph, before], check=True)
    config = load_config(args.config)
    verify_graph_bag(args.bag, before, config['scan_topic'])
    dense = read_dense(args.bag, args.odom_topic, args.pose_npz)
    pose_before = csv(before/'poses.csv')
    open_track = work/'open-trajectory.npz'
    with_graph(dense, pose_before, open_track)
    try:
        generate(args, work/'open-working', open_track, before)
        (work/'open-working').rename(root/'open')
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        state.update(open='failed', closed='skipped', reason=str(error))
        save()
        raise
    state['open'] = 'complete'
    save()
    print(f'Open map ready: {root/"open"}', flush=True)

    try:
        cmd = [sys.executable, NATIVE/'measure_loop.py', args.bag, before/'poses.csv',
               args.calibration, work, '--camera-topic', config['camera_topic'],
               '--scan-topic', config['scan_topic'], '--odom-topic', args.odom_topic,
               '--camera-frame', config['camera_frame'], '--robot-frame', config['robot_frame']]
        if args.mask_bottom_right:
            cmd.append('--mask-bottom-right')
        measurement_log = work/'measurement.log'
        with measurement_log.open('w') as log:
            attempt = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
        if attempt.returncode:
            tail = measurement_log.read_text().splitlines()
            reason = tail[-1] if tail else f'Image matching exited with status {attempt.returncode}'
            raise ValueError(f'{reason} (see {measurement_log})')
        edge = json.loads((work/'visual_measurement.json').read_text())
        subprocess.run([exe, args.graph, after, *map(str, [edge['start_node'], edge['end_node'],
                                                         *edge['edge'], .30, math.radians(5)])], check=True)
        state['metrics'] = dict(open=graph_metrics(before), closed=graph_metrics(after))
        if not passes_open_closed(state['metrics']['open'], state['metrics']['closed']):
            raise ValueError('Closed graph failed endpoint and LiDAR consistency checks')
        state['closed'] = 'projecting'
        save()
        pose_after = csv(after/'poses.csv')
        if not np.array_equal(pose_before['id'], pose_after['id']) or not np.array_equal(
                pose_before['stamp'], pose_after['stamp']):
            raise ValueError('Optimization changed graph node identities')
        closed_track = work/'closed-trajectory.npz'
        with_graph(dense, pose_after, closed_track)
        generate(args, work/'closed-working', closed_track, after,
                 cache=None if args.no_semantic else root/'open')
        (work/'closed-working').rename(root/'closed')
        state['closed'] = 'complete'
        save()
        print(f'Closed map ready: {root/"closed"}', flush=True)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        state.update(closed='failed', reason=str(error))
        save()
        print(f'Closed map unavailable: {error}\nOpen map remains ready: {root/"open"}', file=sys.stderr)
        return 2
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--graph', required=True, type=Path, help='Serialized SLAM Toolbox graph prefix')
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--odom-topic', default='/odom', help='Continuous odometry for local timing and pose interpolation')
    parser.add_argument('--pose-npz', type=Path, help='Optional original dense EKF poses (stamp_ns, poses)')
    parser.add_argument('--mask-bottom-right', action='store_true')
    parser.add_argument('--no-semantic', action='store_true')
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Smoke test only (requires --no-semantic)')
    parser.add_argument('--domain', type=int, default=226)
    args = parser.parse_args()
    if args.max_frames < 0 or (args.max_frames and not args.no_semantic):
        parser.error('--max-frames requires --no-semantic and a nonnegative limit')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    sys.exit(run(args))


if __name__ == '__main__':
    main()
