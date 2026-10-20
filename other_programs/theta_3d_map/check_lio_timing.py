#!/usr/bin/env python3
"""Audit recorded sensor stamps without assuming recorder time is acquisition time."""
import argparse
import json
from pathlib import Path

import numpy as np


def distribution(values):
    a = np.asarray(values, dtype=float)
    return None if not len(a) else dict(zip(
        ('min', 'p50', 'p95', 'p99', 'max'), map(float, np.percentile(a, [0, 50, 95, 99, 100]))))


def summarize(stamps, receipts):
    stamps, receipts = np.asarray(stamps, dtype=np.int64), np.asarray(receipts, dtype=np.int64)
    delta = np.diff(stamps)
    positive = delta[delta > 0]
    median = float(np.median(positive)) if len(positive) else 0
    stable = positive[positive <= 2.5*median] if median else positive
    nominal = float(np.mean(stable)) if len(stable) else 0
    average = ((len(stamps)-1)*1e9/int(stamps[-1]-stamps[0])
               if len(stamps)>1 and stamps[-1]>stamps[0] else None)
    long_gaps = positive[positive > 2.5*nominal] if nominal else np.array([])
    return {
        'samples': len(stamps), 'header_interval_ms': distribution(delta / 1e6),
        'rate_hz': average, 'nominal_interval_ms': nominal/1e6 if nominal else None,
        'average_rate_hz': (len(stamps)-1)*1e9/int(stamps[-1]-stamps[0])
        if len(stamps)>1 and stamps[-1]>stamps[0] else None,
        'duplicate_stamps': int(np.sum(delta == 0)), 'backward_stamps': int(np.sum(delta < 0)),
        'gaps_over_2_5_nominal': len(long_gaps),
        'long_gap_missing_candidates': int(np.maximum(np.rint(long_gaps/nominal)-1, 0).sum()) if nominal else 0,
        'recorder_minus_header_ms': distribution((receipts-stamps)/1e6),
        'recorder_interval_ms': distribution(np.diff(receipts)/1e6),
    }


def analyze(bag):
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    selected = {'/imu', '/velodyne_points', '/Odometry', '/odom'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in selected}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    stamps, receipts = {t: [] for t in types}, {t: [] for t in types}
    scan_start, scan_end, scan_receipt, point_offsets = [], [], [], []
    while reader.has_next():
        topic, data, ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        stamps[topic].append(stamp); receipts[topic].append(ns)
        if topic == '/velodyne_points':
            field = next((f for f in msg.fields if f.name == 'time'), None)
            if field is not None and field.datatype in (7, 8) and msg.width*msg.height:
                dtype = ('>' if msg.is_bigendian else '<')+('f4' if field.datatype == 7 else 'f8')
                values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                    offset=field.offset, strides=(msg.row_step, msg.point_step))
                valid = values[np.isfinite(values)]
                if valid.size:
                    lo, hi = float(valid.min()), float(valid.max())
                    point_offsets.append([lo, hi]); scan_start.append(stamp+round(lo*1e9))
                    scan_end.append(stamp+round(hi*1e9)); scan_receipt.append(ns)
    result = {'bag': str(bag), 'topics': {t: summarize(stamps[t], receipts[t]) for t in types},
              'caveat': 'Recorder latency is not hardware clock calibration. Burst timing can mimic loss; long-gap candidates do not prove missing sensor samples.'}
    if point_offsets:
        result['point_time_min_s'] = distribution(np.array(point_offsets)[:, 0])
        result['point_time_max_s'] = distribution(np.array(point_offsets)[:, 1])
    if scan_end and stamps.get('/imu'):
        order = np.argsort(stamps['/imu']); imu = np.asarray(stamps['/imu'], dtype=np.int64)[order]
        recv = np.asarray(receipts['/imu'], dtype=np.int64)[order]
        starts, ends = np.array(scan_start), np.array(scan_end)
        after = np.searchsorted(imu, ends, side='left')
        before = np.searchsorted(imu, starts, side='right')-1
        covered = (after < len(imu)) & (before >= 0)
        idx = np.flatnonzero(covered)
        result['scan_imu_coverage'] = {
            'scans': len(ends), 'fully_bracketed': int(covered.sum()),
            'imu_samples_in_scan': distribution(np.searchsorted(imu, ends, side='right')-
                                                np.searchsorted(imu, starts, side='left')),
            'imu_after_scan_end_ms': distribution((imu[after[idx]]-ends[idx])/1e6),
            'observed_receipt_wait_ms': distribution(np.maximum(
                recv[after[idx]]-np.asarray(scan_receipt)[idx], 0)/1e6),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = [analyze(bag.expanduser().resolve()) for bag in args.bags]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
