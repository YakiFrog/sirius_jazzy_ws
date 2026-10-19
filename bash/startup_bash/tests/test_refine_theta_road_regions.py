"""Checks for over-smoothing real boundaries and changing protected classes."""
import importlib.util
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location('regions', Path(__file__).parents[1] / 'refine_theta_road_regions.py')
regions = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(regions)


def inputs(labels):
    texture = np.full((*labels.shape, 4), 150, np.uint8)
    texture[..., 3] = 255
    votes = np.zeros((*labels.shape, 4), np.float32)
    votes[..., 2][labels == 5] = 1
    votes[..., 3][labels == 6] = 1
    return texture, votes


def test_spurious_island_removed_on_continuous_surface():
    labels = np.full((60, 80), 6, np.uint8)
    labels[25:29, 35:39] = 5
    texture, votes = inputs(labels)
    after, _ = regions.solve(labels, regions.prepare_graph(labels, texture, votes, .05), .2)
    assert np.all(after == 6)


def test_extended_real_road_boundary_is_retained():
    labels = np.full((60, 80), 6, np.uint8)
    labels[:, :35] = 5
    texture, votes = inputs(labels)
    after, _ = regions.solve(labels, regions.prepare_graph(labels, texture, votes, .05), .2)
    assert np.array_equal(after, labels)


def test_small_region_behind_strong_visual_boundary_is_retained():
    labels = np.full((60, 80), 6, np.uint8)
    labels[25:33, 35:43] = 5
    texture, votes = inputs(labels)
    texture[labels == 5, :3] = 20
    graph = regions.prepare_graph(labels, texture, votes, .05, color_sigma=3, blur_m=0)
    after, _ = regions.solve(labels, graph, .2)
    assert np.array_equal(after, labels)


def test_unknown_grass_and_tactile_are_preserved_and_unknown_blocks_propagation():
    labels = np.full((60, 80), 6, np.uint8)
    labels[25:30] = 0
    labels[30:] = 5
    labels[4:8, 4:8] = 3
    labels[10:14, 35:39] = 4
    labels[17:21, 35:39] = 5
    texture, votes = inputs(labels)
    texture[labels == 0, 3] = 0
    before = labels.copy()
    after, _ = regions.solve(labels, regions.prepare_graph(labels, texture, votes, .05), .2)
    assert np.all(after[30:] == 5)
    assert np.all(after[17:21, 35:39] == 6)
    assert np.array_equal(after[np.isin(labels, (0, 3, 4))], labels[np.isin(labels, (0, 3, 4))])
    assert np.array_equal(labels, before)


def test_zero_strength_reproduces_original_binary_evidence():
    rng = np.random.default_rng(41)
    labels = rng.choice(np.array([5, 6], np.uint8), (40, 50))
    texture, votes = inputs(labels)
    after, _ = regions.solve(labels, regions.prepare_graph(labels, texture, votes, .05), 0)
    assert np.array_equal(after, labels)


def test_diagonal_neighbors_cannot_bridge_unknown_corners():
    labels = np.zeros((4, 4), np.uint8)
    labels[1, 1] = 5
    labels[2, 2] = 6
    texture, votes = inputs(labels)
    graph = regions.prepare_graph(labels, texture, votes, .05)
    assert graph['pairs'].shape == (2, 0)
    after, _ = regions.solve(labels, graph, .8)
    assert np.array_equal(after, labels)


def test_conservative_policy_cannot_expand_a_road_into_sidewalk():
    labels = np.full((60, 80), 5, np.uint8)
    labels[25:29, 35:39] = 6
    texture, votes = inputs(labels)
    graph = regions.prepare_graph(labels, texture, votes, .05)
    symmetric, _ = regions.solve(labels, graph, .8)
    conservative, _ = regions.solve(labels, graph, .8, policy='road-to-sidewalk')
    assert np.all(symmetric == 5)
    assert np.array_equal(conservative, labels)
