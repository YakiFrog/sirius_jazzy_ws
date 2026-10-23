"""Shared preset presentation survives reloads and legacy API clients."""
import importlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from preset_metadata import default_category, read_preset_metadata


@pytest.fixture
def launcher(tmp_path, monkeypatch):
    web = importlib.import_module('web_server')
    alias = tmp_path / 'aliases.sh'
    alias.write_text('# PRESET: 地図生成セット\n# PRESET_ITEMS: lidar,imu\n\n'
                     '# PRESET: 自律移動セット\n# PRESET_ITEMS: nav2_real\n\n'
                     '# GROUP: センサー\nalias lidar=\'echo lidar\'\nalias imu=\'echo imu\'\n'
                     '# GROUP: ナビ\nalias nav2_real=\'echo nav\'\n', encoding='utf-8')
    for name, value in [('ALIAS_FILE', str(alias)), ('PASSWORD', ''), ('BUTTONS', {}),
                        ('GROUPS', []), ('PRESETS', [])]:
        monkeypatch.setattr(web, name, value)
    web.load_aliases(str(alias))
    return web, web.app.test_client(), alias


def test_categories_and_visibility_survive_save_reload_without_changing_commands(launcher):
    web, client, path = launcher
    original_tail = path.read_text().split('# GROUP:', 1)[1]
    data = client.get('/api/state').get_json()['presets']
    assert data[0]['category'] == '地図生成' and data[1]['category'] == '自律移動'
    data[0].update(category='屋外実験', visible=False, note='カメラを確認\n録画してから移動 <script>literal</script>')
    result = client.post('/api/presets/save', json={'presets': data})
    assert result.status_code == 200
    web.load_aliases(str(path))
    saved = client.get('/api/state').get_json()['presets']
    assert saved == result.get_json()['presets']
    assert saved[0]['category'] == '屋外実験' and saved[0]['visible'] is False
    assert saved[0]['note'] == data[0]['note']
    assert saved[0]['items'] == ['lidar', 'imu']
    assert path.read_text().split('# GROUP:', 1)[1] == original_tail
    # Old API clients do not erase the new metadata.
    legacy = [{key: item[key] for key in ('name', 'items')} for item in saved]
    assert client.post('/api/presets/save', json={'presets': legacy}).status_code == 200
    assert read_preset_metadata(path)['地図生成セット']['visible'] is False
    assert read_preset_metadata(path)['地図生成セット']['note'] == data[0]['note']


@pytest.mark.parametrize('override', [{'category': 'bad\n# GROUP: malicious'},
                                     {'visible': 'false'}, {'category': 'x' * 61},
                                     {'name': 'bad\nalias unsafe=oops'}])
def test_invalid_metadata_does_not_modify_alias_file(launcher, override):
    web, client, path = launcher
    before = path.read_bytes()
    preset = {**web.PRESETS[0], **override}
    assert client.post('/api/presets/save', json={'presets': [preset]}).status_code == 400
    assert path.read_bytes() == before


def test_hidden_preset_remains_available_and_can_be_restored(launcher):
    web, client, _ = launcher
    presets = [dict(p) for p in web.PRESETS]
    presets[0]['visible'] = False
    hidden = client.post('/api/presets/save', json={'presets': presets}).get_json()['presets']
    assert len(hidden) == 2 and hidden[0]['items'] == presets[0]['items']
    hidden[0]['visible'] = True
    restored = client.post('/api/presets/save', json={'presets': hidden}).get_json()['presets']
    assert restored[0]['visible'] is True


def test_default_classification_prioritizes_simulation_and_navigation():
    assert default_category('3D地図生成(Webシミュ)', ['lio3d_sim']) == 'シミュレーション'
    assert default_category('セット', ['velodyne', 'nav2_3d']) == '自律移動'
    assert default_category('セット', ['velodyne', 'loc3d']) == '自己位置推定'


@pytest.mark.parametrize('override', [{'note': 'x'*2001}, {'note': 123},
                                     {'items': ['typo_program']}, {'items': []}, {'name': ''}])
def test_mistyped_or_empty_presets_are_rejected_without_losing_existing_data(launcher, override):
    web, client, path = launcher
    before = path.read_bytes()
    preset = {**web.PRESETS[0], **override}
    assert client.post('/api/presets/save', json={'presets': [preset]}).status_code == 400
    assert path.read_bytes() == before
