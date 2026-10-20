"""Reusable pipeline guards and detector/geometry boundaries."""
import copy
import sys
import os
import subprocess
import json
import hashlib

import cv2
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation'))
from theta_road_pipeline import load_config, enhanced_shadow, add_shadow, refine_semantics
from generate_theta_road_map import read_camera, SemanticProcessor
from theta_road_cache import SemanticBaselineCache


def test_partial_config_overrides_and_unknown_keys_fail(tmp_path):
    p = tmp_path/'custom.yaml'
    p.write_text('texture:\n  rear_weight: 1.0\n')
    config = load_config(p)
    assert config['texture']['rear_weight'] == 1
    assert config['semantic']['bidirectional']
    p.write_text('texture:\n  raer_weight: 1.0\n')
    with pytest.raises(ValueError, match='Unknown'):
        load_config(p)
    p.write_text('resolution: 0\n')
    with pytest.raises(ValueError):
        load_config(p)


def test_shadow_preserves_original_rgb_and_protected_labels():
    config = load_config()['semantic']['shadow']
    original = np.full((12, 20, 3), [0, 40, 65], np.uint8)
    before = original.copy()
    enhanced = enhanced_shadow(original, config)
    assert np.array_equal(original, before)
    assert enhanced.max() > original.max()
    classes = np.tile(np.arange(7, dtype=np.uint8), (12, 3))[:, :20]
    scores = np.full(classes.shape, 230, np.uint8)
    extra = np.full(classes.shape, 4, np.uint8)
    result, confidence = add_shadow(classes, scores, extra, np.full(classes.shape, 150, np.uint8), original, config)
    assert np.array_equal(result[np.isin(classes, (1, 2, 3))], classes[np.isin(classes, (1, 2, 3))])
    assert np.all(result[np.isin(classes, (0, 4, 5, 6))] == 4)
    assert np.all(confidence[classes == 4] == 230)
    bright = np.full_like(original, 240)
    assert np.array_equal(add_shadow(classes, scores, extra, scores, bright, config)[0], classes)


@pytest.mark.parametrize('resolution', [.05, .1])
def test_region_refinement_keeps_protected_and_unobserved_cells(resolution):
    c = load_config(); c['regions']['tactile_context'] = False
    labels = np.full((24, 32), 6, np.uint8)
    labels[8:10, 12:14] = 5
    labels[:4] = np.arange(32, dtype=np.uint8) % 5
    texture = np.full((24, 32, 4), 180, np.uint8); texture[..., 3] = 255
    texture[:, :2, 3] = 0
    labels[:, :2] = 0
    votes = np.zeros((24, 32, 4), np.float32)
    votes[..., 3] = .9; votes[..., 2] = .1
    original = labels.copy(); rgb = texture.copy()
    refined, report, _ = refine_semantics(labels, texture, votes, np.ones(labels.shape), resolution, c)
    assert np.array_equal(texture, rgb)
    assert np.array_equal(refined[original < 5], original[original < 5])
    assert not ((refined > 0) & (texture[..., 3] == 0)).any()
    assert report['regions']['road_to_sidewalk'] > 0


def test_grass_is_candidate_only_even_when_enabled():
    c = load_config(); c['regions']['enabled'] = False; c['grass']['candidates'] = True
    labels = np.full((8, 10), 6, np.uint8); labels[:, 0] = 3
    texture = np.full((8, 10, 4), [30, 130, 40, 255], np.uint8)
    votes = np.zeros((8, 10, 4), np.float32); votes[..., 0] = .4; votes[..., 3] = .6
    refined, report, layers = refine_semantics(labels, texture, votes, np.ones(labels.shape), .1, c)
    assert np.array_equal(refined, labels)
    assert report['grass_candidates']['added_cells'] > 0
    assert not report['grass_candidates']['adopted']
    assert (layers['grass_candidates'] > 0).any()


def test_indexed_reread_handles_same_timestamp_seek_noop(monkeypatch):
    # Exercise the MCAP issue without requiring ROS in this unit test.
    import types, hashlib
    fake = types.ModuleType('rclpy.serialization')
    msg = types.SimpleNamespace(data=b'image', header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=4)))
    fake.deserialize_message = lambda data, typ: msg
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', fake)
    sensor = types.ModuleType('sensor_msgs.msg'); sensor.CompressedImage = object
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', sensor)
    class Reader:
        target = None
        cursor = 0
        def seek(self, timestamp):
            if timestamp != self.target:
                self.target = timestamp; self.cursor = 0
        def has_next(self):
            return True
        def read_next(self):
            self.cursor += 1
            return 'camera', b'data', self.target + (self.cursor-1)*100
    reader = Reader()
    record = dict(bag_ns=20, occurrence=0, stamp_ns=1000000004, sha256=hashlib.sha256(b'image').hexdigest())
    assert read_camera(reader, record) is msg
    assert read_camera(reader, record) is msg


def test_target_only_shadow_prompt_can_recover_dark_yellow_without_registry_mutation():
    c = load_config(); c['semantic']['view_yaws'] = [0]
    class Session:
        registry_before = {'ids': {'grass': 3, 'roadway': 5, 'sidewalk': 6}}
        prompt = ''
        def configure(self, prompt, thresholds, **kwargs):
            self.prompt = prompt
        def infer(self, image):
            if self.prompt == c['semantic']['prompt']:
                return np.full(image.shape[:2], 5, np.uint8), np.full(image.shape[:2], 180, np.uint8), {}
            ids = np.zeros(image.shape[:2], np.uint8); score = ids.copy()
            score[32:56, 24:56] = 220
            return ids, score, {}
    session = Session(); before = copy.deepcopy(session.registry_before)
    processor = SemanticProcessor(c, {}, session)
    image = np.full((64, 80, 3), 80, np.uint8)
    image[32:56, 24:56] = [0, 50, 70]
    packet = processor.infer([image])
    forward = processor.track([image], packet, 1.)
    merged = processor.merge([image], packet, forward, forward)
    assert (merged['classes'][0, 32:56, 24:56] == 4).all()
    assert (merged['classes'][0, :20] == 5).all()
    assert session.registry_before == before


def test_normal_terminal_default_reaches_sequential_pipeline(tmp_path):
    home = tmp_path/'home'
    workspace = home/'sirius_jazzy_ws'
    scripts = workspace/'bash/startup_bash'; scripts.mkdir(parents=True)
    install = workspace/'install'; install.mkdir(); (install/'setup.bash').write_text('')
    bag = home/'rosbag2_data/real_smoke'; bag.mkdir(parents=True)
    (bag/'recording.mcap').touch()
    (scripts/'validate_theta_offline_mapping_bag.py').touch()
    tools = tmp_path/'bin'; tools.mkdir()
    log = tmp_path/'called.txt'
    stub = tools/'python3'
    stub.write_text('#!/bin/bash\nprintf "%s\\n" "$@" >> "$CALL_LOG"\n')
    stub.chmod(0o755)
    launcher = Path(__file__).with_name('run_offline_theta_mapping.sh')
    env = dict(os.environ, HOME=str(home), PATH=str(tools)+':'+os.environ['PATH'], CALL_LOG=str(log))
    result = subprocess.run(['bash', str(launcher)], input='\n\n\nn\n', text=True,
        capture_output=True, env=env, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = log.read_text()
    assert str(scripts/'generate_theta_road_map.py') in calls
    assert str(bag) in calls
    assert '--no-semantic' in calls
    assert str(workspace/'params/theta_road_mapping.yaml') in calls
    assert 'Map Select:' in result.stdout


def test_baseline_reuse_requires_exact_camera_jpeg_and_settings(tmp_path):
    cfg = load_config()['semantic']; cfg['view_yaws'] = [0]
    record = dict(frame=17, sha256='original-camera-hash', stamp_ns=100)
    (tmp_path/'manifest.json').write_text(json.dumps(dict(centers=[20], frames={'17': record})))
    folder = tmp_path/'yellow/seg20/frame17'; folder.mkdir(parents=True)
    image = np.full((32, 48, 3), 60, np.uint8)
    _, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
    meta = dict(settings=dict(prompt=cfg['prompt'], resolution=cfg['resolution'], confidence_threshold=1.,
        class_thresholds={p.strip(): cfg['score_min'] for p in cfg['prompt'].split(',')}),
        uploaded_jpeg_sha256=hashlib.sha256(jpeg.tobytes()).hexdigest())
    (folder/'yaw0.json').write_text(json.dumps(meta))
    cv2.imwrite(str(folder/'yaw0.classes.png'), np.full((32, 48), 5, np.uint8))
    cv2.imwrite(str(folder/'yaw0.scores.png'), np.full((32, 48), 200, np.uint8))
    cache = SemanticBaselineCache(tmp_path)
    result = cache.load(record, [image], cfg)
    assert (result['classes'][0] == 5).all()
    with pytest.raises(ValueError, match='camera mismatch'):
        cache.load(dict(record, sha256='different-camera'), [image], cfg)
    with pytest.raises(ValueError, match='projection/JPEG mismatch'):
        cache.load(record, [np.full_like(image, 250)], cfg)
    with pytest.raises(ValueError, match='settings mismatch'):
        cache.load(record, [image], dict(cfg, score_min=.6))


def test_baseline_reuse_never_uploads_normal_view_again():
    c = load_config(); c['semantic']['view_yaws'] = [0]
    class Session:
        registry_before = {'ids': {}}
        count = 0
        def configure(self, prompt, *args, **kwargs):
            assert prompt == c['semantic']['shadow']['prompt']
        def infer(self, image):
            self.count += 1
            return np.zeros(image.shape[:2], np.uint8), np.zeros(image.shape[:2], np.uint8), {}
    session = Session(); processor = SemanticProcessor(c, {}, session)
    view = np.full((32, 48, 3), 50, np.uint8)
    base = dict(classes=[np.full((32, 48), 5, np.uint8)], scores=[np.full((32, 48), 200, np.uint8)], metadata=[{'kind':'base_cached'}])
    packet = processor.infer([view], baseline=base)
    assert session.count == 1 and (packet['raw'] == 5).all()
