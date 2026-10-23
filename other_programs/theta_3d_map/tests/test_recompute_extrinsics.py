"""Calibration replay must really use the candidate rotation being compared."""
import copy
import json
import importlib.util
import sys
from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('recompute_fastlio', ROOT/'recompute_fastlio.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_config_calibration_survives_recorded_identity_tf():
    rotation = Rotation.from_euler('y', 1.3, degrees=True).as_matrix()
    params = {'mapping': {'extrinsic_R': rotation.ravel().tolist(), 'extrinsic_T': [0, 0, .2]}}
    original = copy.deepcopy(params)
    module.select_extrinsics(params, np.eye(4), 'config')
    assert params['mapping']['extrinsic_R'] == original['mapping']['extrinsic_R']
    assert params['mapping']['extrinsic_T'] == original['mapping']['extrinsic_T']


def test_recorded_tf_remains_the_default_calibration_source():
    transform = np.eye(4)
    transform[2, 3] = .2
    params = {'mapping': {}}
    module.select_extrinsics(params, transform, 'recorded-tf')
    assert params['mapping']['extrinsic_T'] == [0, 0, .2]
    np.testing.assert_array_equal(params['mapping']['extrinsic_R'], np.eye(3).ravel())


def test_invalid_candidate_rotation_is_rejected():
    params = {'mapping': {'extrinsic_R': [0.]*9, 'extrinsic_T': [0, 0, .2]}}
    with pytest.raises(ValueError):
        module.select_extrinsics(params, np.eye(4), 'config')


def test_default_replay_restores_running_mount_instead_of_stale_base_yaml(tmp_path):
    capture = tmp_path/'capture'
    capture.mkdir()
    base = {'/**': {'ros__parameters': {'mapping': {'extrinsic_R': np.eye(3).ravel().tolist()}}}}
    (capture/'calibration.yaml').write_text(yaml.safe_dump(base))
    actual = Rotation.from_euler('y', 1.3, degrees=True).as_matrix().ravel().tolist()
    (capture/'fastlio_runtime.json').write_text(json.dumps({'parameters': {
        'mapping.extrinsic_R': actual, 'preprocess.normalize_point_time': True}}))
    params = module.load_replay_config(tmp_path)['/**']['ros__parameters']
    assert params['mapping']['extrinsic_R'] == actual
    assert params['preprocess']['normalize_point_time'] is True
    # A deliberate calibration/noise experiment must not be overwritten.
    explicit = module.load_replay_config(tmp_path, capture/'calibration.yaml')
    assert explicit == base
