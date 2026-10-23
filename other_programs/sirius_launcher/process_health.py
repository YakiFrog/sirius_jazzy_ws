"""Current observations and historical log errors are separate signals."""
import os
import re
import threading
import time

ANSI = re.compile(r"\x1b\[[0-9;]*m")
STAMP = re.compile(r"\[(\d{9,}(?:\.\d+)?)\]")
ERROR = re.compile(r"\[(?:ERROR|FATAL|CRITICAL)\]|Traceback \(most recent call last\):|"
                   r"Task exception was never retrieved|(?:^|\s)[\w.]+(?:Error|Exception):")
CHILD = re.compile(r"\[([^\[\]]+)\]: process (has died|started with pid)")
RECENT_SECONDS = 30
READINESS = {
    'hokuyo': {
        'pattern': re.compile(r"Hardware ID:\s*\S+"),
        'timeout': 10,
        'waiting': 'Hokuyo接続待ち（Hardware ID未検出）',
        'missing': 'Hokuyo未接続（Hardware ID未検出）',
    },
}


def readiness_pattern(name):
    spec = READINESS.get(name)
    return spec['pattern'] if spec else None


def readiness_health(health, name, uptime=None, now=None):
    spec = READINESS.get(name)
    if not spec or health['state'] == 'error' or health.get('failed_children'):
        return health
    if health.get('ready'):
        return health
    elapsed = uptime if uptime is not None else health.get('log_age_seconds')
    detail = {'marker': spec['pattern'].pattern, 'timeout_seconds': spec['timeout'],
              'elapsed_seconds': elapsed}
    if elapsed is not None and elapsed >= spec['timeout']:
        return {**health, 'state': 'error', 'source': 'readiness',
                'summary': spec['missing'], 'readiness': detail}
    return {**health, 'state': 'unknown', 'source': 'readiness',
            'summary': spec['waiting'], 'readiness': detail}


class LogHealth:
    def __init__(self):
        self._cache = {}
        self._lock = threading.Lock()

    def snapshot(self, path, now=None, required=None):
        now = time.time() if now is None else now
        marker = required.pattern if required else None
        try:
            with self._lock:
                stat = os.stat(path)
                key = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
                previous = self._cache.get(path)
                if (previous and previous['key'] == key
                        and previous.get('marker') == marker):
                    event = previous['event']
                    failures = previous['failures']
                    ready = previous.get('ready')
                else:
                    with open(path, 'rb') as stream:
                        stream.seek(max(0, stat.st_size-65536))
                        offset = stream.tell()
                        raw_lines = stream.read().splitlines(keepends=True)
                    lines, offsets = [], []
                    for raw in raw_lines:
                        offsets.append(offset)
                        lines.append(ANSI.sub('', raw.decode(errors='replace')).rstrip())
                        offset += len(raw)
                    stamps = [STAMP.search(line) for line in lines]
                    same_log = (previous and previous['key'][0] == key[0]
                                and key[1] >= previous['key'][1])
                    failures = dict(previous['failures']) if same_log else {}
                    for line in lines:
                        child = CHILD.search(line)
                        if child:
                            if child[2] == 'has died':
                                failures[child[1]] = line.strip()[:500]
                            else:
                                failures.pop(child[1], None)
                    next_stamp = None
                    event = None
                    event_offset = None
                    # An unstamped Python traceback is bounded by the next ROS
                    # log timestamp, or file modification time at the tail.
                    for line, stamp, offset in reversed(list(zip(lines, stamps, offsets))):
                        if stamp:
                            next_stamp = float(stamp[1])
                        if ERROR.search(line):
                            event = {'message': line.strip()[:500],
                                     'timestamp': next_stamp if next_stamp is not None else stat.st_mtime}
                            event_offset = offset
                            if (previous and previous['key'][0] == key[0]
                                    and key[1] >= previous['key'][1]
                                    and previous['event_offset'] == offset
                                    and previous['event']['message'] == event['message']):
                                event = previous['event']
                            break
                    if (event is None and previous and
                            previous['key'][0] == key[0] and key[1] >= previous['key'][1]):
                        event = previous['event']
                        event_offset = previous['event_offset']
                    ready = (any(required.search(line) for line in lines)
                             if required else None)
                    self._cache[path] = {'key': key, 'event': event, 'event_offset': event_offset,
                                         'failures': failures, 'marker': marker, 'ready': ready}
        except OSError:
            return {'state': 'unknown', 'summary': 'ログ未取得', 'source': 'log',
                    'last_error': None}
        age = max(0, now-event['timestamp']) if event else None
        state = 'error' if failures or event and age <= RECENT_SECONDS else 'unknown' if event else 'ok'
        return {'state': state, 'source': 'log', 'last_error': event,
                'failed_children': failures, 'ready': ready,
                'log_age_seconds': max(0, now-stat.st_mtime),
                'error_age_seconds': age, 'recent_error_seconds': RECENT_SECONDS,
                'summary': ('子プロセス異常終了: '+', '.join(failures) if failures else
                            '直近30秒にエラー' if state == 'error' else
                            '過去エラーあり・現在の状態未確認' if event else 'ログ異常なし')}


def battery_health(log, telemetry, now=None):
    """Only the battery-only alias may recover from successful Solix polls."""
    now = time.time() if now is None else now
    if log.get('failed_children'):
        return log
    details = (telemetry.get('battery') or {}).get('details') or {}
    if not telemetry.get('available'):
        return log if log['state'] == 'error' else {
            **log, 'state': 'unknown', 'summary': 'バッテリー監視データ未取得'}
    age = details.get('age_seconds')
    if age is not None:
        age += max(0, now-telemetry.get('timestamp', now))
    if (details.get('state') == 'live' and details.get('present') is True
            and age is not None and age <= 5):
        error = log.get('last_error')
        if error and now-age <= error['timestamp']:
            return log
        return {**log, 'state': 'ok', 'source': 'solix',
                'summary': 'バッテリー受信正常' + ('（過去エラーから復帰）' if error else ''),
                'data_age_seconds': age}
    if details.get('state') in ('stale', 'absent') or age is not None and age > 5:
        return {**log, 'state': 'error', 'source': 'solix', 'data_age_seconds': age,
                'summary': 'バッテリー未接続・再接続中' if details.get('state') == 'absent' else 'バッテリー受信停止'}
    return log if log['state'] == 'error' else {
        **log, 'state': 'unknown', 'summary': 'バッテリー受信待ち'}
