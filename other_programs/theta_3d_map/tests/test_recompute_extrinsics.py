"""Calibration replay must really use the candidate rotation being compared."""
import copy
import importlib.util
import sys
from pathlib import Path
import numpy as np
import pytest
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
