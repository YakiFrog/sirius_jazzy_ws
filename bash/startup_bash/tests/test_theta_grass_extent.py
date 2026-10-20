"""Grass proposals require evidence, connectivity, and observed support."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]))
from experiment_theta_grass_extent import expand_grass


def scene():
    labels=np.full((20,20),6,np.uint8);labels[:,2:5]=3
    texture=np.empty((20,20,4),np.uint8);texture[...,:3]=[30,110,50];texture[...,3]=255
    votes=np.zeros((20,20,4),np.float32);votes[...,0]=.4;votes[...,3]=.6
    coverage=np.ones((20,20),np.uint32)
    return labels,texture,votes,coverage


def test_supported_grass_can_extend_only_by_the_configured_distance():
    labels,t,v,c=scene();new,added,_=expand_grass(labels,t,v,c,.05,.1)
    assert added[10,5] and added[10,6] and not added[10,7]
    assert np.all(new[labels==3]==3)


def test_green_appearance_without_grass_observations_does_not_extend():
    labels,t,v,c=scene();v[...,0]=0
    _,added,_=expand_grass(labels,t,v,c,.05,.5)
    assert not added.any()


def test_tactile_and_unobserved_space_block_growth():
    labels,t,v,c=scene();labels[:,6]=4;t[:,8,3]=0;c[:,10]=0
    new,added,_=expand_grass(labels,t,v,c,.05,1)
    assert np.all(new[:,6]==4) and not added[:,6:].any()
    labels[:,6]=6
    _,added,_=expand_grass(labels,t,v,c,.05,1)
    assert added[:,7].any() and not added[:,8:].any()


def test_native_visibility_support_is_required_even_if_rgb_exists():
    labels,t,v,c=scene();c[:,6]=0
    _,added,_=expand_grass(labels,t,v,c,.05,1)
    assert added[:,5].any() and not added[:,6:].any()


def test_incompatible_pavement_color_is_not_crossed():
    labels,t,v,c=scene();t[:,6,:3]=[130,130,130]
    _,added,_=expand_grass(labels,t,v,c,.05,1)
    assert added[:,5].any() and not added[:,6:].any()


def test_low_grass_fraction_blocks_a_high_absolute_vote():
    labels,t,v,c=scene();v[:,6,0]=1;v[:,6,3]=9
    _,added,_=expand_grass(labels,t,v,c,.05,1)
    assert added[:,5].any() and not added[:,6:].any()
