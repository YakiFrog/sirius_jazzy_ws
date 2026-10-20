"""Recoloring must preserve optimized poses and fuse only valid RGB evidence."""
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mapping_color as color
from offline_3d_mapping import transform, save_trajectory, interpolate_pose
from types import SimpleNamespace


def test_rotation_confidence_preserves_pose_bounds_and_reduces_fast_turns():
    def at(t):
        if t < 0 or t > 1:
            raise AssertionError('Motion weights must not extrapolate')
        return transform(Rotation.from_euler('z',18*t,degrees=True).as_matrix(),[t,0,0])
    trajectory = SimpleNamespace(times=np.array([0.,1.]),at=at)
    for t in (0.,.5,1.):
        weight, speed = color.rotation_observation_weight(trajectory,t,5.)
        assert speed == pytest.approx(18.)
        assert weight == pytest.approx(1./(1+(18/5)**2))
    assert color.rotation_observation_weight(trajectory,.5,0.) == (1.,None)
    missing = SimpleNamespace(times=np.array([0.,1.]),at=lambda _: None)
    assert color.rotation_observation_weight(missing,.5,5.) == (0.,None)


def test_rotation_weight_allows_stable_black_view_to_replace_near_white_turn_views():
    fusion = color.ColorFusion(1,5,weighted_median=True)
    near_turn = .117/(1+(18/5)**2)
    for _ in range(5):
        fusion.add(np.array([0]),np.array([0xbcbcbc]),np.array([near_turn]))
    for _ in range(3):
        fusion.add(np.array([0]),np.array([0x080808]),np.array([.04]))
    np.testing.assert_array_equal(fusion.finish()[0],[[8,8,8]])


def test_high_rate_motion_preserves_optimized_keyframes_and_respects_pose_bounds(tmp_path, monkeypatch):
    times = np.linspace(0,1,5)
    seed = [transform(Rotation.from_euler('z',.2*t).as_matrix(),[t,0,.1*t]) for t in times]
    alignment = transform(Rotation.from_euler('z',.4).as_matrix(),[3,-2,.4])
    initial = [alignment @ seed[i] for i in (0,4)]
    corrections = [np.eye(4), transform(Rotation.from_euler('z',.1).as_matrix(),[.2,.1,.05])]
    optimized = [c @ p for c,p in zip(corrections,initial)]
    save_trajectory(tmp_path/'trajectory_initial.csv',[0,1],initial,np.eye(4))
    save_trajectory(tmp_path/'trajectory.csv',[0,1],optimized,np.eye(4))
    report = dict(resolved_base_from_lidar=np.eye(4).tolist(),options=dict(max_sync_gap=.3,wheel_model='se3'))
    monkeypatch.setattr(color,'load_motion',lambda _: (times,seed,np.array([]),[],('odom','base')))
    monkeypatch.setattr(color,'calibration',lambda _: (np.eye(4),np.eye(3),None))
    trajectory = color.ColorTrajectory(tmp_path,report,tmp_path,[],None)
    for t,pose in zip([0,1],optimized):
        np.testing.assert_allclose(trajectory.at(t),pose,atol=1.e-12)
    expected = interpolate_pose(.5,[0,1],corrections,1.) @ alignment @ seed[2]
    np.testing.assert_allclose(trajectory.at(.5),expected,atol=1.e-12)
    assert trajectory.at(-.1) is None and trajectory.at(1.1) is None
    assert trajectory.report['ground_truth_used'] is False
    bad = initial[1].copy();bad[0,3] += 1
    save_trajectory(tmp_path/'trajectory_initial.csv',[0,1],[initial[0],bad],np.eye(4))
    with pytest.raises(ValueError,match='一致しません'):
        color.ColorTrajectory(tmp_path,report,tmp_path,[],None)


def test_multiview_median_rejects_invalid_samples_and_single_color_outlier():
    fusion = color.ColorFusion(2,3)
    for rgb in (0xff0000,0x00ff00,0x00ff00):
        fusion.add(np.array([0,1]),np.array([rgb,0xff0000],np.uint32),np.array([1.,0.]))
    fusion.add(np.array([0]),np.array([0x0000ff],np.uint32),np.array([.1]))
    rgb, counts = fusion.finish()
    np.testing.assert_array_equal(rgb,[[0,255,0],[128,128,128]])
    np.testing.assert_array_equal(counts,[3,0])


def test_weighted_median_near_view_outweighs_four_distant_views():
    fusion = color.ColorFusion(2,5,weighted_median=True)
    fusion.add(np.array([0]),np.array([0x101010],np.uint32),np.array([1./4]))
    for _ in range(4):
        fusion.add(np.array([0]),np.array([0xf0f0f0],np.uint32),np.array([1./100]))
    rgb, counts = fusion.finish()
    np.testing.assert_array_equal(rgb,[[16,16,16],[128,128,128]])
    np.testing.assert_array_equal(counts,[5,0])


@pytest.mark.parametrize('views',[1,2,3,4,5])
def test_equal_confidence_weighted_median_matches_ordinary_median(views):
    ordinary = color.ColorFusion(3,5)
    weighted = color.ColorFusion(3,5,weighted_median=True)
    for i in range(views):
        for fusion in (ordinary,weighted):
            fusion.add(np.array([0,1]),np.array([0x102030+i*0x101010,0xe0c0a0-i*0x101010]),np.array([1.,1.]))
    np.testing.assert_array_equal(ordinary.finish()[0],weighted.finish()[0])


@pytest.mark.parametrize('move_geometry', [False, True])
def test_automatic_recolor_preserves_geometry_and_keeps_recorded_rgb(tmp_path,monkeypatch,move_geometry):
    import recolor_3d_map as recolor
    from offline_3d_mapping import write_map
    xyz=np.array([[1.,2.,3.],[4.,5.,6.]])
    write_map(tmp_path/'map_3d.ply',xyz)
    write_map(tmp_path/'map_3d_colored.ply',xyz,np.array([[10,20,30],[40,50,60]]))
    before=(tmp_path/'map_3d_colored.ply').read_bytes()
    def fake(source,output,*args):
        output.mkdir()
        changed=xyz.copy()
        if move_geometry:changed[0,0]+=.01
        write_map(output/'map_3d_colored.ply',changed,np.full((2,3),200))
        for name in ('report.json','color_observations.npz','color_image_motion.npz','colorization_policy.yaml','theta_calibration_used.yaml'):
            (output/name).write_bytes(b'checked')
        return {'geometry_modified':False}
    monkeypatch.setattr(recolor,'recolor',fake)
    if move_geometry:
        with pytest.raises(ValueError,match='XYZ'):
            recolor.recolor_new_output(tmp_path,tmp_path)
        assert (tmp_path/'map_3d_colored.ply').read_bytes()==before
    else:
        recolor.recolor_new_output(tmp_path,tmp_path)
        assert (tmp_path/'map_3d_colored_recorded.ply').read_bytes()==before
        assert not (tmp_path/'.recolor_work').exists()
        new_xyz,_=recolor.read_ply(tmp_path/'map_3d_colored.ply')
        np.testing.assert_array_equal(new_xyz,xyz)
