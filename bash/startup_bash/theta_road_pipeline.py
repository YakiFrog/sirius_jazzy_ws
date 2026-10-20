"""Recording-independent configuration, shadow gates and semantic refinement.

This module contains no ROS or SAM3 network calls. Existing evaluated algorithms
are reused; parameters are collection priors, not cross-BAG accuracy guarantees.
"""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / 'params/theta_road_mapping.yaml'


def load_config(path=DEFAULT_CONFIG):
    base = yaml.safe_load(DEFAULT_CONFIG.read_text())
    def merge(target, update):
        for key, value in update.items():
            if key not in target:
                raise ValueError(f'Unknown mapping setting: {key}')
            if isinstance(target[key], dict):
                if not isinstance(value, dict):
                    raise ValueError(f'{key} must be a mapping')
                merge(target[key], value)
            else:
                target[key] = value
    merge(base, yaml.safe_load(Path(path).read_text()) or {})
    validate_config(base)
    return base


def validate_config(c):
    positive = [c['resolution'], c['max_grid_cells'], c['projection']['bev_size'],
                c['projection']['stride'], c['projection']['max_radius'],
                c['semantic']['min_weight'], c['semantic']['max_gap_sec'],
                c['regions']['color_sigma'], c['regions']['radius_m'],
                c['semantic']['shadow']['gamma'], c['semantic']['shadow']['max_gain']]
    if not all(np.isfinite(x) and x > 0 for x in positive):
        raise ValueError('Resolution, sizes, radii and positive thresholds must be finite and > 0')
    for value in (c['texture']['rear_weight'], c['texture']['relative_quality'],
                  c['texture']['decay'], c['semantic']['score_min'],
                  c['semantic']['yellow_fraction'], c['semantic']['shadow']['yellow_fraction']):
        if not np.isfinite(value) or not 0 <= value <= 1:
            raise ValueError('Weights, score thresholds and fractions must lie in [0, 1]')
    if not 0 <= c['projection']['min_radius'] < c['projection']['max_radius']:
        raise ValueError('Require 0 <= min_radius < max_radius')
    for value in (c['visibility']['max_age_sec'], c['visibility']['obstacle_padding_m'],
                  c['visibility']['margin_m'], c['texture']['quality_power'],
                  c['regions']['strength_m'], c['regions']['prior_strength'],
                  c['grass']['radius_m'], c['grass']['weight_min']):
        if not np.isfinite(value) or value < 0:
            raise ValueError('Visibility, quality and regularization parameters must be finite and nonnegative')
    if c['regions']['policy'] not in ('road-to-sidewalk', 'both'):
        raise ValueError('Unknown region adoption policy')
    if not c['semantic']['view_yaws'] or len(c['semantic']['view_size']) != 2:
        raise ValueError('Semantic views must be nonempty')
    if not all(isinstance(v, int) and v >= 16 for v in c['semantic']['view_size']):
        raise ValueError('Semantic view dimensions must be integers >= 16')
    if not 0 < c['semantic']['hfov_deg'] < 180:
        raise ValueError('Perspective field of view must lie in (0, 180)')


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def enhanced_shadow(image, cfg):
    """The evaluated shadow white balance + bounded gamma, for inference only."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value = hsv[..., 2].astype(np.float32)
    region = np.zeros(value.shape, bool)
    region[int(value.shape[0]*.6):int(value.shape[0]*.95)] = True
    neutral = region & (value >= 8) & (value <= 100) & (hsv[..., 1] < 120)
    if neutral.sum() < 128:
        neutral = region & (value >= 8) & (value <= 120)
    balanced = image.copy()
    if neutral.sum() >= 128:
        median = np.maximum(np.median(image[neutral], axis=0), 1.)
        gains = np.clip(np.mean(median)/median, .65, 1.8)
        shade = np.clip((110.-value)/70., 0, 1)
        balanced = np.clip(np.rint(image*(1+shade[..., None]*(gains-1))), 0, 255).astype(np.uint8)
    value = balanced.max(axis=2).astype(np.float32)
    target = 255.*np.power(value/255., cfg['gamma'])
    gain = np.minimum(target/np.maximum(value, 1), cfg['max_gain'])
    return np.clip(np.rint(balanced.astype(np.float32)*gain[..., None]), 0, 255).astype(np.uint8)


def dark_mask(image, mask, cfg):
    n, cc, sizes, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    keep = np.zeros(n, bool)
    value = image.max(axis=2)
    for k in range(1, n):
        if sizes[k, cv2.CC_STAT_AREA] >= cfg['min_component_area']:
            keep[k] = np.median(value[cc == k]) <= cfg['max_original_value']
    return keep[cc]


def add_shadow(base, scores, extra, extra_scores, original, cfg):
    """Only add ID4; keep existing tactile scores and protected classes intact."""
    take = dark_mask(original, extra == 4, cfg) & ~np.isin(base, (1, 2, 3))
    result, confidence = base.copy(), scores.copy()
    confidence[take] = np.maximum(np.where(base[take] == 4, scores[take], 0), extra_scores[take])
    result[take] = 4
    return result, confidence


def refine_semantics(labels, texture, votes, coverage, resolution, cfg):
    """Images use top-left origin; observation votes passed in the same order."""
    from refine_theta_road_regions import prepare_graph, solve, statistics
    before = labels.copy()
    report, layers = {}, {}
    r = cfg['regions']
    if r['enabled'] and np.isin(labels, (5, 6)).any():
        graph = prepare_graph(labels, texture, votes, resolution, color_sigma=r['color_sigma'])
        labels, timing = solve(labels, graph, r['strength_m'], policy=r['policy'])
        report['regions'] = {**statistics(before, labels, resolution), **timing}
        if r['tactile_context'] and (labels == 4).any():
            from experiment_theta_tactile_context import local_shape_anchors, tactile_distances, add_prior
            anchors, unsafe, counts = local_shape_anchors(labels, resolution,
                guard_m=r['endpoint_guard_m'], min_length_m=r['min_length_m'], max_width_m=r['max_width_m'])
            graph = prepare_graph(labels, texture, votes, resolution, color_sigma=r['color_sigma'])
            distance = tactile_distances(labels, graph, resolution, anchors, r['radius_m'], use_appearance=False)
            graph, _ = add_prior(graph, distance, resolution, r['radius_m'], r['prior_strength'])
            prior_labels = labels.copy()
            labels, timing = solve(labels, graph, r['strength_m'], policy=r['policy'])
            report['tactile_context'] = {**counts, **statistics(prior_labels, labels, resolution), **timing}
            layers['tactile_anchors'] = anchors.astype(np.uint8)*255
    if cfg['grass']['candidates']:
        from experiment_theta_grass_extent import expand_grass
        g = cfg['grass']
        proposed, added, stats = expand_grass(labels, texture, votes, coverage, resolution,
            g['radius_m'], fraction_min=g['fraction_min'], grass_weight_min=g['weight_min'])
        layers.update(grass_candidate_classes=proposed, grass_candidates=added.astype(np.uint8)*255)
        report['grass_candidates'] = {**stats, 'adopted': False}
    if not np.array_equal(labels[~np.isin(before, (5, 6))], before[~np.isin(before, (5, 6))]):
        raise AssertionError('Refinement modified protected classes')
    return labels, report, layers
