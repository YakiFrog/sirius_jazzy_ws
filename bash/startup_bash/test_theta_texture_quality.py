"""Geometry parity and general quality/photometric fusion behavior."""
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/sirius/sirius_navigation'))
from theta_texture_quality import sampling_maps,TextureFusion,aggregate_frame
from sirius_navigation.theta_bev_projection import build_blend_maps


def test_direct_sampler_matches_calibrated_bev_at_exact_same_ground_points():
    c=yaml.safe_load((ROOT/'maps_waypoints/maps/theta_semantic_tactile_context_map_20261020/calibration_used.yaml').read_text())
    c['bev_size']=64;c['rpy_degrees']=[1.2,-.7,4.0]
    u,v=np.meshgrid((np.arange(64)+.5)/64,(np.arange(64)+.5)/64)
    actual=sampling_maps(c,(.5-v)*10,(.5-u)*10,(1080,1920))
    expected=build_blend_maps(c,1920,1080,6)
    for a,b in zip(actual[:5],expected):np.testing.assert_allclose(a,b,atol=1e-4,rtol=0)


def test_same_color_blends_but_inconsistent_lower_quality_cannot_ghost_it():
    f=TextureFusion(2,'robust',.65,25)
    f.update([0],[[220,220,220]],[.8])
    for _ in range(30):f.update([0],[[30,30,30]],[.7])
    color,seen=f.result();assert seen[0] and np.array_equal(color[0],[220]*3)


def test_better_view_resets_stale_incompatible_color():
    f=TextureFusion(2,'robust',.65,25)
    f.update([0],[[30,30,30]],[.4]);f.update([0],[[220,220,220]],[.8])
    color,_=f.result();assert np.array_equal(color[0],[220]*3)


def test_first_weak_observation_fills_without_losing_support():
    f=TextureFusion(3,'robust',.8,25)
    f.update([0,2],[[10,20,30],[120,130,140]],[.01,.9]);_,seen=f.result()
    assert np.array_equal(seen,[True,False,True])


def test_repeated_low_quality_does_not_replace_a_good_view():
    f=TextureFusion(1,'ema',.5)
    f.update([0],[[100,110,120]],[1])
    for _ in range(50):f.update([0],[[20,30,40]],[.2])
    color,_=f.result();assert np.array_equal(color[0],[100,110,120])


def test_native_frame_aggregation_preserves_weighted_color_without_density_quality_boost():
    a=aggregate_frame(np.array([5,5]),np.array([[100,0,0],[0,100,0]]),np.array([.2,.6]),np.ones(2),np.ones(2))
    np.testing.assert_allclose(a['bgr'],[[25,75,0]],atol=1e-5)
    np.testing.assert_allclose(a['q'],[.6]);np.testing.assert_allclose(a['weight'],[.8])


def test_best_view_selection_does_not_move_or_fill_an_unobserved_cell():
    f=TextureFusion(10,'best');f.update([4],[[100,120,140]],[.9]);f.update([4],[[0,0,0]],[.2])
    color,seen=f.result();assert np.flatnonzero(seen).tolist()==[4]
    assert np.array_equal(color[4],[100,120,140])
