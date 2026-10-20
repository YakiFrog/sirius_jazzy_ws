"""Explicit planar-floor refinement must correct poses, preserving scan geometry."""
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mapping_floor import refine_single_floor,floor_frame
from offline_3d_mapping import transform


@pytest.mark.parametrize('slope',[0.,.15])
def test_pose_refinement_preserves_measured_plane_slope_and_unflattened_returns(slope):
    xx,yy=np.meshgrid(np.linspace(-2,6,20),np.linspace(-2,2,12))
    world=np.column_stack([xx.ravel(),yy.ravel(),slope*xx.ravel()])
    normal=np.array([-slope,0.,1.]);normal/=np.linalg.norm(normal)
    initial=[];truth=[];observations=[]
    for i,distance in enumerate([0.,1.,2.,3.,4.,3.,2.,1.,0.,1.]):
        pose=transform(translation=[distance,0.,.85+slope*distance]);truth.append(pose)
        local=world-pose[:3,3]
        observations.append(dict(view=i,normal=normal.copy(),center=local.mean(axis=0),
            points=local,heldout=local,local_heldout_p95_m=0.))
        drift=pose.copy()
        if i:
            drift[2,3]+=.1*np.sin(i/9*np.pi)
            drift[:3,:3]=Rotation.from_euler('y',.02*np.sin(i/9*np.pi)).as_matrix()
        initial.append(drift)
    new,report=refine_single_floor(initial,observations)
    assert report['success']
    assert report['heldout_after']['point_p95_distance_m']<.03
    assert report['heldout_after']['point_p95_distance_m']<.2*report['heldout_before']['point_p95_distance_m']
    assert max(abs(p[2,3]-q[2,3]) for p,q in zip(new,truth))<.025
    np.testing.assert_array_equal(np.array(new)[:,:2,3],np.array(initial)[:,:2,3])
    if slope:assert new[4][2,3]-new[0][2,3]>.55
    # A real object 10cm above the plane is retained by rigid scan transformation.
    local_floor=(world[100]-truth[3][:3,3])
    local_object=local_floor+normal*.1
    difference=(new[3][:3,:3]@local_object+new[3][:3,3])-(new[3][:3,:3]@local_floor+new[3][:3,3])
    assert np.linalg.norm(difference)==pytest.approx(.1)
    assert difference@normal>.099


def test_floor_frame_is_rigid_gauge_and_does_not_change_surface_distances():
    normal=np.array([-.03,.02,1.]);normal/=np.linalg.norm(normal)
    frame=floor_frame([*normal,.07])
    np.testing.assert_allclose(frame[:3,:3]@normal,[0,0,1],atol=1.e-12)
    points=np.array([[1.,2.,.03-0.04-.07],[3.,1.,.09-.02-.07]])
    # General rigidity, independent of particular plane point coordinates.
    out=points@frame[:3,:3].T+frame[:3,3]
    assert np.linalg.norm(out[0]-out[1])==pytest.approx(np.linalg.norm(points[0]-points[1]))


def test_rebuild_requires_explicit_known_floor_declaration(tmp_path):
    from refine_3d_map import rebuild
    with pytest.raises(ValueError,match='explicit --known-single-floor'):
        rebuild(tmp_path,tmp_path/'output')
    assert not (tmp_path/'output').exists()
