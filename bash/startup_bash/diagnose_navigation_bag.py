#!/usr/bin/env python3
"""Read recorded messages only; never create a ROS node or send commands."""
import argparse
import collections
import datetime
import html
import json
from pathlib import Path


def classify(nav, output, locked, driver_stopped):
    if not nav:
        return 'no_motion_request'
    if driver_stopped is True:
        return 'driver_stopped'
    if locked is True and not output:
        return 'mux_lock_mismatch' if driver_stopped is False else 'mux_locked'
    if output:
        return 'command_output_present'
    return 'output_missing_unknown'


def analyze(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    wanted = {'/cmd_vel_nav_safe', '/cmd_vel', '/diagnostics', '/roboteq/diagnostics', '/stop'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()
             if t.name in wanted}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    rows = collections.defaultdict(lambda: {'nav': 0, 'output': 0})
    counts = collections.Counter()
    while reader.has_next():
        topic, raw, ns = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        sec = ns // 1_000_000_000
        row = rows[sec]
        counts[topic] += 1
        if topic == '/cmd_vel_nav_safe':
            row['nav'] += int(abs(msg.linear.x) > .001 or abs(msg.angular.z) > .001)
        elif topic == '/cmd_vel':
            row['output'] += 1
        elif topic == '/roboteq/diagnostics':
            row['driver_stopped'] = bool(msg.estop_active)
        elif topic == '/diagnostics':
            for status in msg.status:
                if status.name != 'twist_mux: Twist mux status':
                    continue
                for value in status.values:
                    text = value.value.strip()
                    if value.key == 'lock locks.stop' and text.startswith(('locked ', 'free ')):
                        row.setdefault('locks', set()).add(text.startswith('locked '))
    if not rows:
        raise ValueError('対象の診断・速度トピックがありません')
    state = {}
    segments = []
    for sec in range(min(rows), max(rows) + 1):
        row = rows[sec]
        if 'driver_stopped' in row:
            state['driver'] = (sec, row['driver_stopped'])
        if 'locks' in row:
            locks = row['locks']
            state['lock'] = (sec, next(iter(locks)) if len(locks) == 1 else None)
        def fresh(key):
            entry = state.get(key)
            return entry[1] if entry and sec - entry[0] <= 3 else None
        kind = classify(row['nav'] > 0, row['output'] > 0, fresh('lock'), fresh('driver'))
        if segments and segments[-1]['kind'] == kind:
            segments[-1]['end'] = sec + 1
        else:
            segments.append({'start': sec, 'end': sec + 1, 'kind': kind})
    # One-second boundary effects cannot establish a sustained output blockage.
    for segment in segments:
        if segment['end'] - segment['start'] < 2 and segment['kind'] in (
                'mux_lock_mismatch', 'mux_locked', 'output_missing_unknown'):
            segment['kind'] = 'short_transition'
    return {'bag': str(path), 'message_counts': dict(counts), 'segments': segments}


LABELS = {
    'no_motion_request': '非ゼロのNav2入力なし（目標終了・経路待ち等は別途確認）',
    'driver_stopped': 'Nav2入力あり／ドライバの緊急停止が有効',
    'mux_lock_mismatch': '停止状態の不一致：ドライバ解除済み／muxロック中／出力なし',
    'mux_locked': 'Nav2入力あり／muxロック中／出力なし（ドライバ状態は未確認）',
    'command_output_present': 'Nav2入力・最終速度出力あり（実機の移動は保証しません）',
    'output_missing_unknown': 'Nav2入力あり／出力なし（遮断理由は未確定）',
    'short_transition': '短時間の遷移（原因判定の対象外）',
}


def render(report):
    def clock(sec):
        return datetime.datetime.fromtimestamp(sec, datetime.timezone(
            datetime.timedelta(hours=9))).strftime('%H:%M:%S')
    rows = ''.join('<tr><td>'+clock(s['start'])+'〜'+clock(s['end'])+'</td><td>'
                   +str(s['end']-s['start'])+'秒</td><td>'+LABELS[s['kind']]+'</td></tr>'
                   for s in report['segments'])
    return ('<!doctype html><html lang="ja"><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>自律移動の停止診断</title><style>'
            'body{font:16px/1.7 system-ui;margin:24px auto;padding:0 16px;max-width:1000px;'
            'color:#172338;background:#f6f8fc}h1{font-size:24px}table{border-collapse:collapse;'
            'width:100%;background:white}td,th{text-align:left;padding:12px;border-bottom:1px solid #dce2eb}'
            'td:first-child{white-space:nowrap}.table{overflow:auto}code{overflow-wrap:anywhere}</style>'
            '<h1>自律移動の停止診断</h1><p><code>'+html.escape(report['bag'])+'</code></p>'
            '<p>BAGを読むだけの診断です。走行・停止・解除・プロセス再起動は行いません。'
            '時刻はJST、1秒単位です。診断値が3秒を超えて古い場合は未確認として扱います。</p>'
            '<div class="table"><table><tr><th>時刻</th><th>継続</th><th>記録から確認できる状態</th></tr>'
            +rows+'</table></div><p>/stop記録件数：'+str(report['message_counts'].get('/stop', 0))+
            '。0件の場合、解除操作の送信元・未達理由はこの記録だけでは特定できません。</p></html>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = analyze(args.bag)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'diagnosis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (args.output/'index.html').write_text(render(report))
    print(json.dumps({'segments': len(report['segments']), 'output': str(args.output)}))


if __name__ == '__main__':
    main()
