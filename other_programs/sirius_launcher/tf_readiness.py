"""Sirius TF readiness derived from a TF monitor snapshot."""
from collections import deque

REQUIRED_FRAMES = ('sirius3/odom', 'sirius3/base_footprint', 'sirius3/base_link')
REQUIRED_PATHS = (
    ('sirius3/odom', 'sirius3/base_footprint'),
    ('sirius3/base_footprint', 'sirius3/base_link'),
)
ODOM_PATH = REQUIRED_PATHS[0]


def _path(edges, start, end):
    """Return edge indexes on an active (non-stale) undirected path."""
    graph = {}
    for index, edge in enumerate(edges):
        if edge.get('stale'):
            continue
        parent, child = edge.get('parent'), edge.get('child')
        if not parent or not child:
            continue
        graph.setdefault(parent, []).append((child, index))
        graph.setdefault(child, []).append((parent, index))
    queue = deque([start])
    previous = {start: None}
    while queue:
        frame = queue.popleft()
        if frame == end:
            indexes = []
            while previous[frame] is not None:
                frame, index = previous[frame]
                indexes.append(index)
            return indexes
        for neighbour, index in graph.get(frame, ()):
            if neighbour not in previous:
                previous[neighbour] = (frame, index)
                queue.append(neighbour)
    return None


def _result(state, summary, **extra):
    return {'state': state, 'ready': state == 'ok', 'summary': summary,
            'required_frames': list(REQUIRED_FRAMES),
            'required_paths': [list(path) for path in REQUIRED_PATHS], **extra}


def tf_readiness(snapshot):
    """Classify Sirius frame readiness without requiring a map frame."""
    if not snapshot or not snapshot.get('available'):
        return _result('unknown', 'TF受信を待っています', missing=[], disconnected=[],
                       dynamic_path=False, conflicts=[], cycles=[])

    edges = snapshot.get('edges') or []
    conflicts = list(snapshot.get('conflicts') or [])
    cycles = list(snapshot.get('cycles') or [])
    if conflicts or cycles:
        return _result('error', 'TF構成エラー', missing=[], disconnected=[],
                       dynamic_path=False, conflicts=conflicts, cycles=cycles)

    frames = set(snapshot.get('frames') or [])
    missing = [frame for frame in REQUIRED_FRAMES if frame not in frames]
    if missing:
        return _result('waiting', 'TF準備待ち: ' + ', '.join(missing), missing=missing,
                       disconnected=[], dynamic_path=False, conflicts=[], cycles=[])

    disconnected = [list(path) for path in REQUIRED_PATHS if _path(edges, *path) is None]
    if disconnected:
        labels = [' → '.join(path) for path in disconnected]
        return _result('waiting', 'TF経路未接続: ' + ', '.join(labels), missing=[],
                       disconnected=disconnected, dynamic_path=False,
                       conflicts=[], cycles=[])

    odom_path = _path(edges, *ODOM_PATH)
    dynamic_path = any(not edges[index].get('static') for index in odom_path)
    if not dynamic_path:
        return _result('waiting', 'odom → base_footprint の動的TFが未更新', missing=[],
                       disconnected=[], dynamic_path=False, conflicts=[], cycles=[])

    return _result('ok', 'シリウス準備OK', missing=[], disconnected=[], dynamic_path=True,
                   conflicts=[], cycles=[])
