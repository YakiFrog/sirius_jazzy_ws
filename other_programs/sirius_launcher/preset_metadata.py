"""Presentation settings stored as comments alongside existing launch presets."""
import json


def default_category(name, items):
    if 'シミュ' in name or any(item.endswith('_sim') for item in items):
        return 'シミュレーション'
    if '自律' in name or any(item.startswith('nav2') for item in items):
        return '自律移動'
    if '自己位置' in name or 'loc3d' in items or 'lio3d_loc' in items:
        return '自己位置推定'
    if '地図' in name or '録画' in name or any('slamtoolbox' in item for item in items):
        return '地図生成'
    if 'センサ' in name or any(item in items for item in ('velodyne', 'imu', 'theta_capture', 'zed_offline_recorder')):
        return 'センサー'
    return 'その他'


def read_preset_metadata(path):
    result, name = {}, None
    with open(path, encoding='utf-8') as source:
        for line in source:
            line = line.strip()
            if line.startswith(('# TAB:', '# GROUP:')):
                break
            if line.startswith('# PRESET:'):
                name = line.split(':', 1)[1].strip()
            elif name and line.startswith('# PRESET_UI:'):
                try:
                    value = json.loads(line.split(':', 1)[1])
                    if isinstance(value, dict):
                        result[name] = value
                except (ValueError, TypeError):
                    pass
    return result


def preset_presentation(name, items, metadata):
    settings = metadata.get(name, {})
    category = settings.get('category')
    if not isinstance(category, str) or not category.strip() or len(category) > 60:
        category = default_category(name, items)
    note = settings.get('note', '')
    return {'category': category.strip(), 'visible': settings.get('visible') is not False,
            'note': note[:2000] if isinstance(note, str) else ''}
