"""Context seeds exclude unsafe geometry and respect observed floor barriers."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]))
from experiment_theta_tactile_context import tactile_anchors, local_shape_anchors, tactile_distances, add_prior
from refine_theta_road_regions import prepare_graph, solve


def scene():
    labels=np.full((200,160),5,np.uint8)
    labels[10:190,77:83]=4
    texture=np.full((*labels.shape,4),130,np.uint8);texture[...,3]=255
    votes=np.zeros((*labels.shape,4),np.float32);votes[...,2][labels==5]=1
    return labels,texture,votes


def test_long_strip_interior_seeds_but_endpoints_do_not():
    labels,_,_=scene()
    safe,_,_=tactile_anchors(labels,.05)
    assert safe[70:130].any()
    assert not safe[:30].any() and not safe[170:].any()


def test_short_tactile_piece_is_not_a_seed():
    labels=np.full((100,100),5,np.uint8);labels[30:60,47:53]=4
    safe,_,_=tactile_anchors(labels,.05)
    assert not safe.any()


def test_branch_and_warning_pad_neighborhoods_are_excluded():
    labels,_,_=scene();labels[80:86,80:145]=4;labels[130:150,70:90]=4
    safe,_,_=tactile_anchors(labels,.05)
    assert not safe[70:95,70:95].any()
    assert not safe[120:160,70:90].any()
    assert safe[40:60].any()


def test_geodesic_prior_stops_at_unknown_gap():
    labels,texture,votes=scene();labels[:,90]=0
    graph=prepare_graph(labels,texture,votes,.05,color_sigma=3)
    anchors,_,_=tactile_anchors(labels,.05)
    distance=tactile_distances(labels,graph,.05,anchors,2)
    full=np.full(labels.shape,np.inf);full[graph['active']]=distance
    assert np.isfinite(full[100,85])
    assert not np.isfinite(full[:,91:]).any()


def test_strong_image_edge_reduces_prior_reach():
    labels,texture,votes=scene();texture[:,90:,:3]=230
    graph=prepare_graph(labels,texture,votes,.05,color_sigma=3,blur_m=0)
    anchors,_,_=tactile_anchors(labels,.05)
    distance=tactile_distances(labels,graph,.05,anchors,2)
    full=np.full(labels.shape,np.inf);full[graph['active']]=distance
    assert np.isfinite(full[100,85]) and not np.isfinite(full[100,95])


def test_soft_prior_can_recover_floor_beside_strip_without_relabeling_tactile():
    labels,texture,votes=scene()
    graph=prepare_graph(labels,texture,votes,.05,color_sigma=3)
    anchors,_,_=tactile_anchors(labels,.05)
    distance=tactile_distances(labels,graph,.05,anchors,.9)
    updated,_=add_prior(graph,distance,.05,.9,5)
    refined,_=solve(labels,updated,.8,policy='road-to-sidewalk')
    assert refined[100,74]==6 and refined[100,86]==6
    assert np.all(refined[labels==4]==4)
    assert refined[100,20]==5


def test_without_anchors_the_added_prior_is_zero():
    labels,texture,votes=scene();labels[labels==4]=5;votes[...,2]=1
    graph=prepare_graph(labels,texture,votes,.05,color_sigma=3)
    anchors,_,_=tactile_anchors(labels,.05)
    distance=tactile_distances(labels,graph,.05,anchors,.9)
    updated,prior=add_prior(graph,distance,.05,.9,5)
    assert not prior.any()
    assert np.array_equal(updated['road_cost'],graph['road_cost'])


def test_local_shape_anchors_ignore_small_skeleton_spurs_but_exclude_endpoints():
    labels,_,_=scene();labels[70:73,83:86]=4
    safe,_,_=local_shape_anchors(labels,.05)
    assert safe[70:130].any()
    assert not safe[:30].any() and not safe[170:].any()


def test_connectivity_distance_can_cross_shadow_color_without_crossing_unknown():
    labels,texture,votes=scene();texture[:,87:,:3]=230;labels[:,95]=0
    graph=prepare_graph(labels,texture,votes,.05,color_sigma=3,blur_m=0)
    anchors,_,_=local_shape_anchors(labels,.05)
    distance=tactile_distances(labels,graph,.05,anchors,2,use_appearance=False)
    full=np.full(labels.shape,np.inf);full[graph['active']]=distance
    assert np.isfinite(full[100,90]) and not np.isfinite(full[100,96])


def test_local_shape_rejects_a_real_branch_and_broad_warning_pad():
    labels,_,_=scene();labels[80:86,80:145]=4;labels[130:150,70:90]=4
    safe,_,_=local_shape_anchors(labels,.05)
    assert not safe[75:95,70:95].any()
    assert not safe[125:155,70:90].any()
    assert safe[40:60].any()
