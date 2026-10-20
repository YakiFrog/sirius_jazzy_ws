"""Finishing must require later free rays and protect static/occluded details."""
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mapping_cleanup import polish_map
from mapping_dynamic import finishing_indices


def beams(point, factor):
    return np.asarray(point)*factor+np.array([[0,-.004,0],[0,0,0],[0,.004,0]])*factor


@pytest.mark.parametrize('tilted',[False,True])
def test_finishing_requires_free_consensus_and_preserves_revisits_occlusion_unknown_and_wires(tilted):
    ghost=np.array([4.,0,0]);static=np.array([4.,1,0]);occluded=np.array([4.,-1,0])
    unseen=np.array([4.,-2,0]);late=np.array([4.,2,0])
    wire=np.array([[4.,3,z] for z in np.arange(0,.51,.05)])
    local=np.vstack([ghost,static,occluded,unseen,late,wire])
    scans=[]
    for i in range(6):
        parts=[beams(static,1),beams(unseen,2),beams(ghost,1 if i==0 else 2),
               beams(occluded,1 if i==0 else .5),beams(late,1 if i in (0,5) else 2)]
        parts.extend(beams(p,1 if i==0 else 2) for p in wire)
        scans.append(np.vstack(parts))
    pose=np.eye(4)
    if tilted:
        pose[:3,:3]=Rotation.from_euler('zyx',[37,18,12],degrees=True).as_matrix()
        pose[:3,3]=[2.,-3.,.7]
    xyz=local@pose[:3,:3].T+pose[:3,3];keep=np.ones(len(xyz),bool)
    result,report=polish_map(xyz,keep,scans.__getitem__,[pose]*6,[0,1,2,10,11,12],SimpleNamespace())
    expected=keep.copy();expected[0]=False
    np.testing.assert_array_equal(result,expected)
    assert keep.all()
    assert report['protected_slender_points']==len(wire)
    assert report['removed_points']==1
    assert report['floor_or_roi_mask'] is False


def test_finishing_preserves_large_wall_and_attached_tip_despite_ray_evidence():
    y,z=np.meshgrid(np.arange(-1,1.01,.1),np.arange(-1,1.01,.1))
    wall=np.column_stack([np.full(y.size,4.),y.ravel(),z.ravel()])
    arm=np.array([[x,0.,0.] for x in [3.7,3.8,3.9]])
    seed=np.array([[3.65,0.,.05]])
    xyz=np.vstack([wall,arm,seed]);keep=np.r_[np.ones(len(wall)+3,bool),False]
    scans=[np.vstack([wall,arm])]+[np.vstack([wall,beams(arm[0],4/3.7)])]*3
    result,report=polish_map(xyz,keep,scans.__getitem__,[np.eye(4)]*4,[0,2,3,5],SimpleNamespace())
    assert result[:len(wall)].all()
    assert result[len(wall):-1].all()
    assert not result[-1]
    assert report['removed_points']==0
    assert report['protected_connected_points']>=1


def test_single_free_frame_or_late_missing_rays_are_not_enough():
    p=np.array([[4.,0,0.]])
    scans=[beams(p[0],1),beams(p[0],2),np.empty((0,3)),np.empty((0,3))]
    result,_=polish_map(p,np.ones(1,bool),scans.__getitem__,[np.eye(4)]*4,[0,2,3,5],SimpleNamespace())
    assert result.all()
    with pytest.raises(ValueError,match='timestamps'):
        polish_map(p,np.ones(1,bool),scans.__getitem__,[np.eye(4)]*4,[0,2,1,5],SimpleNamespace())


def test_overlay_contains_both_kinds_of_additional_removals():
    np.testing.assert_array_equal(finishing_indices(dict(fragment_filter={'removed_indices':[2,4]},
        polish_filter={'removed_indices':[4,9]})),[2,4,9])


def test_foreground_return_anywhere_in_beam_footprint_vetoes_free_space():
    p=np.array([[4.,0,0.]])
    # Three rays point almost straight ahead and return behind the point.
    # A fourth ray lies farther to the side, still within the footprint, and
    # returns in front. Treat the footprint as occluded/uncertain.
    later=np.vstack([beams(p[0],2),[2.,.012,0.]])
    scans=[beams(p[0],1)]+[later]*3
    keep,report=polish_map(p,np.ones(1,bool),scans.__getitem__,[np.eye(4)]*4,[0,2,3,5],SimpleNamespace())
    assert keep.all() and report['removed_points']==0


def test_one_persistent_point_protects_entire_small_feature():
    center=np.array([4.,0.,0.])
    detail=center+np.vstack([np.eye(3),-np.eye(3)])*.1
    xyz=np.vstack([center,detail]);scans=[]
    for i in range(6):
        scans.append(np.vstack([beams(center,1)]+[beams(p,1 if i==0 else 2) for p in detail]))
    keep,report=polish_map(xyz,np.ones(len(xyz),bool),scans.__getitem__,[np.eye(4)]*6,
                           [0,2,3,10,11,12],SimpleNamespace())
    assert keep.all()
    assert report['persistent_points']==1
    assert report['protected_component_points']==len(xyz)


def test_coherent_small_planar_part_is_preserved_even_with_single_view():
    y,z=np.meshgrid([-.1,0,.1],[-.05,.05])
    part=np.column_stack([np.full(y.size,4.),y.ravel(),z.ravel()])
    scans=[np.vstack([beams(p,1) for p in part])]+[np.vstack([beams(p,2) for p in part])]*3
    keep,report=polish_map(part,np.ones(len(part),bool),scans.__getitem__,[np.eye(4)]*4,
                          [0,2,3,5],SimpleNamespace())
    assert keep.all()
    assert report['protected_planar_points']==len(part)


def test_seed_proximity_cannot_override_large_extent_feature_protection():
    line=np.column_stack([np.arange(4.,5.41,.1),np.zeros(15),np.zeros(15)])
    xyz=np.vstack([line,[4.,.15,0.]]);keep=np.r_[np.ones(len(line),bool),False]
    scans=[line]+[beams(line[0],2)]*3
    result,_=polish_map(xyz,keep,scans.__getitem__,[np.eye(4)]*4,[0,2,3,5],SimpleNamespace())
    np.testing.assert_array_equal(result,keep)
