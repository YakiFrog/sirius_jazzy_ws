"""Exercise the official native backend on occlusion, free space and frame transforms."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mapping_dynamic import dufomap_filter, refine_fragments


@pytest.mark.parametrize('streaming', [False, True])
def test_official_dufomap_removes_departed_surface_preserves_wall_and_unknown(streaming):
    pytest.importorskip('dufomap')
    yy, zz = np.meshgrid(np.linspace(-3, 3, 121), np.linspace(-3, 3, 121))
    wall = np.column_stack([np.full(yy.size, 8.), yy.ravel(), zz.ravel()])
    # The middle scan has a foreground object. Hide the wall behind it.
    foreground = wall[(abs(wall[:, 1]) < 1.5) & (abs(wall[:, 2]) < 1.5)]/2
    occluded = (abs(wall[:, 1]) < 1.5) & (abs(wall[:, 2]) < 1.5)
    scans = [wall, np.concatenate([wall[~occluded], foreground]), wall]
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('zy', [40, 12], degrees=True).as_matrix()
    pose[:3, 3] = [2, -3, .85]
    # A point outside all scans remains unknown. A stationary surface stays.
    local = np.array([[4, 0, 0], [8, 0, 0], [-5, 0, 0]], dtype=float)
    xyz = local @ pose[:3, :3].T + pose[:3, 3]
    original = xyz.copy()
    def unused_keyframe(i):
        raise AssertionError('Raw evidence must replace keyframe evidence')
    keep, report = dufomap_filter(xyz, unused_keyframe if streaming else scans.__getitem__,
                                   [pose] if streaming else [pose]*3,
                                   SimpleNamespace(dynamic_range=30., no_dynamic_fragments=True),
                                   observations=((pose, scan) for scan in scans) if streaming else None)
    np.testing.assert_array_equal(keep, [False, True, True])
    np.testing.assert_array_equal(xyz, original)
    assert report['removed_points'] == 1
    assert report['trajectory_modified'] is False
    assert report['tested_scans'] == 3
    assert report['tested_keyframes'] == (1 if streaming else 3)


def test_fragment_refinement_protects_revisits_large_surfaces_and_unrelated_islands():
    # Equal-size floating groups differ only by temporal support, not by XYZ size.
    ghost = np.array([[i*.04, 0, 1] for i in range(3)])
    stationary = ghost+[2, 0, 0]
    unrelated = ghost+[4, 0, 0]
    unseen = ghost+[5, 0, 0]
    surface = np.array([[6+i*.04, j*.04, 1] for i in range(16) for j in range(10)])
    seeds = np.array([[0, .18, 1], [2, .18, 1], [5, .18, 1], [6, -.18, 1]])
    local = np.concatenate([ghost, stationary, unrelated, unseen, surface, seeds])
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('zy', [40, 12], degrees=True).as_matrix()
    pose[:3, 3] = [2, -3, .85]
    xyz = local @ pose[:3, :3].T + pose[:3, 3]
    keep = np.r_[np.ones(len(local)-len(seeds), bool), np.zeros(len(seeds), bool)]
    scans = [np.concatenate([ghost, stationary, surface]),
             np.concatenate([ghost, stationary, surface]), np.concatenate([stationary, surface])]
    output, report = refine_fragments(xyz, keep, scans.__getitem__, [pose]*3, [0, 1, 12], SimpleNamespace())
    expected = keep.copy()
    expected[:3] = False
    np.testing.assert_array_equal(output, expected)
    np.testing.assert_array_equal(keep[:3], [True]*3)  # Caller mask is not mutated.
    assert report['removed_points'] == 3
    assert report['protected_components'] == 2  # Persistent surface and unobserved island.
    disabled, _ = refine_fragments(xyz, keep, scans.__getitem__, [pose]*3, [0, 1, 12],
                                    SimpleNamespace(no_dynamic_fragments=True))
    np.testing.assert_array_equal(disabled, keep)
    no_stamps, _ = refine_fragments(xyz, keep, scans.__getitem__, [pose]*3, None, SimpleNamespace())
    np.testing.assert_array_equal(no_stamps, keep)
    with pytest.raises(ValueError, match='timestamps'):
        refine_fragments(xyz, keep, scans.__getitem__, [pose]*3, [0, 2, 1], SimpleNamespace())


def test_raw_evidence_deskews_signed_time_and_uses_corrected_header_pose(tmp_path, monkeypatch):
    import clean_3d_map as cleaner
    import mapping_color
    import mapping_fusion
    import offline_3d_mapping as mapping
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import PointField
    from sensor_msgs_py.point_cloud2 import create_cloud
    from std_msgs.msg import Header
    header = Header(frame_id='lidar')
    header.stamp.sec = 10
    fields = [PointField(name=k, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i,k in enumerate(('x','y','z','time'))]
    scan = create_cloud(header, fields, [(1.,0.,0.,-.05),(1.,0.,0.,.05)])
    missing = create_cloud(Header(frame_id='lidar'), fields, [(1.,0.,0.,0.)])
    corrected = np.eye(4); corrected[0,3] = 3.
    seeds = [np.eye(4),np.eye(4),np.eye(4)]
    for pose,x in zip(seeds,[-.2,0.,.2]): pose[0,3] = x
    trajectory = SimpleNamespace(seed_times=np.array([9.8,10.,10.2]),seed=seeds,
        mount=np.eye(4),gap=.3,report={'verified':True},
        at=lambda t: corrected.copy() if t == 10. else None)
    monkeypatch.setattr(mapping_color,'ColorTrajectory',lambda *args: trajectory)
    monkeypatch.setattr(mapping_fusion,'read_static_calibration',
                        lambda *args: (None,None,[],{'/raw':'lidar'}))
    entries = iter([('/raw',serialize_message(missing),0),('/raw',serialize_message(scan),0)])
    class Reader:
        remaining = 2
        def has_next(self): return self.remaining > 0
        def read_next(self):
            self.remaining -= 1
            return next(entries)
    monkeypatch.setattr(mapping,'reader',lambda *args: Reader())
    report = dict(bag=str(tmp_path),options={'raw_cloud_topic':'/raw','min_range':.1,'max_range':30.},
                  wheel_frames=['odom','base'],imu_topic='/imu')
    diagnostic = {}
    evidence = list(cleaner.raw_scan_observations(tmp_path,report,diagnostic))
    assert len(evidence) == 1
    np.testing.assert_array_equal(evidence[0][0],corrected)
    np.testing.assert_allclose(evidence[0][1],[[.95,0.,0.],[1.05,0.,0.]],atol=1.e-6)
    assert diagnostic['scans_without_pose'] == 1
    assert diagnostic['scans_used'] == 1
