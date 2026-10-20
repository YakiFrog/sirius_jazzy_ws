"""Recording must retain both localization inputs and rejected-match diagnostics."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'record_3d_bag.py'
spec = importlib.util.spec_from_file_location('record_3d_bag', SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


@pytest.mark.parametrize('mode', ['real', 'sim'])
def test_localization_records_inputs_results_and_latched_map(mode):
    required = set(recorder.required_topics(mode, 'localization'))
    topics = recorder.recording_topics(mode, 'localization')
    assert required <= set(topics)
    assert len(topics) == len(set(topics))
    assert {'/velodyne_points', '/Odometry', '/map_localizer/pose',
            '/map_localizer/status', '/tf', '/tf_static', '/localization/map'} <= required
    assert '/velodyne_points_colored' not in required
    assert {'/initialpose', '/cloud_registered_body', '/roboteq/status',
            '/parameter_events'} <= set(topics)
    assert ('/clock' in required) == (mode == 'sim')


def test_existing_real_mapping_still_requires_rgb():
    assert '/velodyne_points_colored' in recorder.required_topics('real')
    assert '/map_localizer/pose' not in recorder.required_topics('real')


def test_launcher_alias_accepts_a_bag_name(monkeypatch):
    import shlex
    aliases = SCRIPT.parents[2] / 'bash/bash_alias2.sh'
    line = next(line for line in aliases.read_text().splitlines()
                if line.startswith('alias record_loc3d='))
    command = shlex.split(line.split('=', 1)[1])[0]
    arguments = shlex.split(command)[2:] + ['push_test_001']

    class Checked(Exception):
        pass

    def preflight(mode, timeout, domain, profile):
        assert mode == 'real' and profile == 'localization'
        raise Checked

    monkeypatch.setattr(recorder, 'preflight', preflight)
    with pytest.raises(Checked):
        recorder.main(arguments)


def test_snapshot_copies_the_running_map_and_keeps_parameters(tmp_path, monkeypatch):
    source = tmp_path / 'actual_map.ply'
    source.write_bytes(b'original reference map bytes')
    capture = tmp_path / 'capture'
    capture.mkdir()
    params = f'/map_localizer:\n  ros__parameters:\n    map_path: {source}\n    use_3dof: false\n'

    def run(command, **kwargs):
        return SimpleNamespace(returncode=0, stdout=params if command[0] == 'ros2' else 'abc123\n')

    monkeypatch.setattr(recorder.subprocess, 'run', run)
    recorder.snapshot_localization(capture, tmp_path)
    metadata = json.loads((capture / 'localization.json').read_text())
    assert (capture / metadata['reference_map']).read_bytes() == source.read_bytes()
    assert metadata['map_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert (capture / 'map_localizer.yaml').read_text() == params
    assert metadata['versions'] == dict(workspace='abc123', navigation='abc123', fast_lio='abc123')
