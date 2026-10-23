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


def test_real_mapping_keeps_raw_inputs_without_requiring_live_colorization():
    assert {'/velodyne_points', '/imu', '/odom'} <= set(recorder.required_topics('real'))
    assert '/velodyne_points_colored' not in recorder.required_topics('real')
    assert '/map_localizer/pose' not in recorder.required_topics('real')


@pytest.mark.parametrize('mode', ['real', 'sim'])
@pytest.mark.parametrize('profile', ['mapping', 'localization'])
def test_compact_recording_preserves_sources_and_localization_evidence(mode, profile):
    topics = set(recorder.recording_topics(mode, profile))
    assert set(recorder.required_topics(mode, profile)) <= topics
    assert {'/velodyne_points', '/imu', '/odom', '/tf', '/tf_static',
            '/theta/dual_fisheye/image_raw/compressed'} <= topics
    assert not {'/velodyne_points_colored', '/velodyne_points_leveled'} & topics
    assert ('/cloud_registered_body' in topics) == (profile == 'localization')
    full = recorder.recording_topics(mode, profile, include_derived_clouds=True)
    assert topics <= set(full)
    assert '/velodyne_points_colored' in full
    assert ('/velodyne_points_leveled' in full) == (profile == 'localization')
    assert len(full) == len(set(full))


@pytest.mark.parametrize('mode', ['real', 'sim'])
def test_mapping_retains_estimator_and_sensor_failure_evidence(mode):
    topics = recorder.recording_topics(mode)
    assert {'/fastlio/diagnostics', '/diagnostics', '/rosout', '/roboteq/status'} <= set(topics)
    assert len(topics) == len(set(topics))


@pytest.mark.parametrize('mode', ['real', 'sim'])
def test_mapping_can_also_replay_2d_slam_and_optional_stereo_camera(mode):
    topics = set(recorder.recording_topics(mode))
    assert {'/scan3', '/scan', '/velodyne_points', '/imu', '/odom', '/Odometry',
            '/tf', '/tf_static', '/camera/stereo_sbs/compressed', '/camera/stereo_params',
            '/robot_description', '/joint_states', '/cmd_vel', '/parameter_events'} <= topics


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


@pytest.mark.parametrize('mode', ['real', 'sim'])
def test_all_mapping_cameras_are_recorded_but_not_required(mode):
    required = set(recorder.required_topics(mode, 'all_mapping'))
    topics = recorder.recording_topics(mode, 'all_mapping')
    assert {'/theta/dual_fisheye/image_raw/compressed', '/camera/stereo_sbs/compressed',
            '/camera/stereo_params'} <= set(topics) - required
    assert {'/scan3', '/tf', '/tf_static', '/velodyne_points', '/imu', '/odom'} <= required <= set(topics)
    assert '/Odometry' not in required
    assert '/odom/filtered' not in required
    assert '/velodyne_points_colored' not in topics
    assert len(topics) == len(set(topics))


def test_all_mapping_rejects_invalid_calibration_and_disconnected_camera():
    def msg(frame):
        return SimpleNamespace(header=SimpleNamespace(frame_id=frame), data=b'jpeg', ranges=[1.])
    received = {topic: msg(frame) for topic, frame in [
        ('/velodyne_points', 'lidar'), ('/imu', 'imu'), ('/scan3', 'lidar'),
        ('/theta/dual_fisheye/image_raw/compressed', 'theta'),
        ('/camera/stereo_sbs/compressed', 'zed')]}
    received['/odom'] = SimpleNamespace(child_frame_id='base')
    received['/camera/stereo_params'] = SimpleNamespace(data=json.dumps(
        dict(fx=100, fy=100, cx=50, cy=50, baseline=.12, width=100, height=100)))
    transforms = [SimpleNamespace(header=SimpleNamespace(frame_id='base'), child_frame_id=f)
                  for f in ['lidar', 'imu', 'theta', 'zed']]
    recorder.validate_all_mapping(received, transforms)
    assert any('ZED' in w and 'TF' in w for w in recorder.validate_all_mapping(received, transforms[:-1]))
    received['/camera/stereo_params'].data = '{}'
    assert any('内部パラメータ' in w for w in recorder.validate_all_mapping(received, transforms))
    del received['/camera/stereo_sbs/compressed']
    del received['/theta/dual_fisheye/image_raw/compressed']
    assert len(recorder.validate_all_mapping(received, transforms)) == 2
    with pytest.raises(ValueError, match='TF'):
        recorder.validate_all_mapping(received, [])


def test_all_mapping_alias_accepts_optional_bag_name_after_profile(monkeypatch):
    def checked(*args):
        assert args[0] == 'real'
        assert args[3] == 'all_mapping'
        raise RuntimeError('preflight reached')
    monkeypatch.setattr(recorder, 'preflight', checked)
    with pytest.raises(RuntimeError, match='preflight reached'):
        recorder.main(['real', '--profile', 'all_mapping', 'survey_test'])


def test_all_mapping_confirms_after_topic_log_even_with_launcher_pipe(tmp_path, monkeypatch, capsys):
    def preflight(*args):
        print('/theta/dual_fisheye/image_raw/compressed: 未配信')
        return {}
    prompts = []
    answers = iter(['', '', 'n'])
    def input_line(prompt):
        assert '未配信' in capsys.readouterr().out if not prompts else True
        prompts.append(prompt)
        return next(answers)
    monkeypatch.setattr(recorder, 'preflight', preflight)
    monkeypatch.setattr(recorder.sys, 'stdin', SimpleNamespace(isatty=lambda: False))
    monkeypatch.setattr('builtins.input', input_line)
    assert recorder.main(['real', '--profile', 'all_mapping', '--directory', str(tmp_path)]) == 0
    assert 'Y/n' in prompts[-1]
    assert not list(tmp_path.iterdir())
