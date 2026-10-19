"""Test a soft sidewalk prior near the interior of continuous tactile strips.

Tactile paving is contextual evidence, not a universal prohibition of roadway.
Short pieces, branch/end neighborhoods and broad warning pads do not seed it.
Distances follow observed floor connectivity and image similarity, not circles.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from refine_theta_road_regions import prepare_graph, solve, statistics


def tactile_anchors(labels, resolution, *, guard_m=1.0, min_length_m=3.0,
                    max_width_m=0.45):
    tactile = (labels == 4).astype(np.uint8)
    # Close single-pixel registration holes, but never fill a wider gap.
    cleaned = cv2.morphologyEx(tactile, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    skeleton = cv2.ximgproc.thinning(cleaned * 255) > 0
    n, component, sizes, _ = cv2.connectedComponentsWithStats(skeleton.astype(np.uint8), 8)
    long_enough = np.zeros(n, bool)
    long_enough[1:] = sizes[1:, cv2.CC_STAT_AREA] * resolution >= min_length_m
    skeleton &= long_enough[component]
    padded = np.pad(skeleton, 1)
    degree = np.zeros(labels.shape, np.uint8)
    h, w = labels.shape
    for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
        neighbor = padded[1+dy:1+dy+h, 1+dx:1+dx+w].copy()
        if dx and dy:
            # Remove redundant triangle edges at digital corners.
            neighbor &= ~(padded[1:1+h, 1+dx:1+dx+w] | padded[1+dy:1+dy+h, 1:1+w])
        degree += neighbor
    half_width = cv2.distanceTransform(cleaned, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * resolution
    unsafe = skeleton & ((degree != 2) | (half_width > max_width_m / 2))
    safe = skeleton.copy()
    if unsafe.any():
        distance_from_unsafe = ndimage.distance_transform_edt(~unsafe) * resolution
        safe &= distance_from_unsafe >= guard_m
    safe &= tactile > 0
    return safe, unsafe, dict(raw_tactile_cells=int(tactile.sum()), skeleton_cells=int(skeleton.sum()),
                             safe_anchor_cells=int(safe.sum()), unsafe_skeleton_cells=int(unsafe.sum()))


def local_shape_anchors(labels, resolution, *, guard_m=1.0, min_length_m=3.0,
                        max_width_m=0.45):
    """Use meter-scale strip shape instead of degree at noisy skeleton pixels."""
    tactile=(labels==4).astype(np.uint8)
    cleaned=cv2.morphologyEx(tactile,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    skeleton=cv2.ximgproc.thinning(cleaned*255)>0
    n,component,sizes,_=cv2.connectedComponentsWithStats(skeleton.astype(np.uint8),8)
    keep=np.zeros(n,bool);keep[1:]=sizes[1:,cv2.CC_STAT_AREA]*resolution>=min_length_m
    skeleton&=keep[component]
    safe=np.zeros(labels.shape,bool);radius=int(np.ceil((guard_m+.3)/resolution));h,w=labels.shape
    for y,x in zip(*np.nonzero(skeleton & (tactile>0))):
        y0=max(0,y-radius);y1=min(h,y+radius+1);x0=max(0,x-radius);x1=min(w,x+radius+1)
        coords=np.argwhere(cleaned[y0:y1,x0:x1]>0).astype(np.float64)
        coords=(coords+np.array([y0-y,x0-x]))*resolution
        if len(coords)<8:continue
        covariance=np.cov(coords.T);values,vectors=np.linalg.eigh(covariance)
        if values[0]/max(values[1],1e-9)>.10 or np.sqrt(12*max(values[0],0))>max_width_m:continue
        along=coords@vectors[:,1];lo,hi=np.quantile(along,[.03,.97])
        if lo<=-guard_m and hi>=guard_m:safe[y,x]=True
    unsafe=skeleton&~safe
    return safe,unsafe,dict(raw_tactile_cells=int(tactile.sum()),skeleton_cells=int(skeleton.sum()),
                            safe_anchor_cells=int(safe.sum()),unsafe_skeleton_cells=int(unsafe.sum()))


def tactile_distances(labels, graph, resolution, anchors, limit_m, *, use_appearance=True):
    """Geodesic distance penalizes a visual boundary; unobserved cells block it."""
    n = graph['count']
    seeds = np.flatnonzero(anchors[graph['active']])
    if seeds.size == 0:
        return np.full(n, np.inf)
    first, last = graph['pairs']
    flat = np.flatnonzero(graph['active'])
    gap = np.abs(flat[first] - flat[last])
    diagonal = gap != 1
    diagonal &= gap != labels.shape[1]
    geometric_step = resolution * np.where(diagonal, np.sqrt(2), 1.0)
    direction_weight = np.where(diagonal, 1-1/np.sqrt(2), np.sqrt(2)-1)
    similarity = graph['lengths'] / (resolution * direction_weight)
    costs = geometric_step / np.maximum(similarity, 1e-6) if use_appearance else geometric_step
    adjacency = csr_matrix((np.concatenate((costs, costs)),
                            (np.concatenate((first, last)), np.concatenate((last, first)))), shape=(n, n))
    return dijkstra(adjacency, directed=True, indices=seeds, min_only=True, limit=limit_m)


def add_prior(graph, distances, resolution, radius_m, strength):
    # A strong but finite preference fades to zero at the geodesic radius.
    prior = np.clip(1 - distances / radius_m, 0, 1).astype(np.float32)
    updated = dict(graph)
    updated['road_cost'] = graph['road_cost'] + strength * resolution**2 * prior
    return updated, prior


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--radii', type=float, nargs='+', default=[0.35, 0.6, 0.9])
    parser.add_argument('--prior-strengths', type=float, nargs='+', default=[2.5, 5.0])
    parser.add_argument('--endpoint-guard', type=float, default=1.0)
    parser.add_argument('--anchor-mode',choices=('junction-guard','local-shape'),default='junction-guard')
    parser.add_argument('--distance-mode',choices=('appearance','connectivity'),default='appearance')
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    labels = cv2.imread(str(args.reference/'road_temporal.colored.pgm'), 0)
    texture = cv2.imread(str(args.reference/'road_temporal.texture.png'), -1)
    meta = json.loads((args.reference/'manifest.json').read_text())
    res = meta['grid']['resolution']
    extractor=tactile_anchors if args.anchor_mode=='junction-guard' else local_shape_anchors
    anchors, unsafe, counts = extractor(labels, res, guard_m=args.endpoint_guard)
    cv2.imwrite(str(out/'safe_anchors.png'), anchors.astype(np.uint8)*255)
    cv2.imwrite(str(out/'excluded_skeleton.png'), unsafe.astype(np.uint8)*255)
    with np.load(args.reference/'map_evidence.npz') as evidence:
        graph = prepare_graph(labels, texture, evidence['temporal_votes'][::-1], res, color_sigma=3, connectivity=8)
    distances = tactile_distances(labels, graph, res, anchors, max(args.radii),use_appearance=args.distance_mode=='appearance')
    full_distance = np.full(labels.shape, np.inf, np.float32)
    full_distance[graph['active']] = distances
    np.savez_compressed(out/'anchor_evidence.npz', geodesic_distance_m=full_distance,
                        anchors=anchors, excluded_skeleton=unsafe)
    cfg = dict(radii_m=args.radii, prior_strengths=args.prior_strengths, endpoint_guard_m=args.endpoint_guard,
               anchor_mode=args.anchor_mode,distance_mode=args.distance_mode,
               minimum_strip_length_m=3.0, maximum_strip_width_m=0.45, color_sigma_lab=3,
               guide_blur_m=0.08, connectivity=8, smoothing_strength_m=0.8,
               adoption_policy='road-to-sidewalk', **counts)
    report = dict(reference=str(args.reference.resolve()), config=cfg,
                  input_sha256={name:hashlib.sha256((args.reference/name).read_bytes()).hexdigest()
                                for name in ('road_temporal.colored.pgm','road_temporal.texture.png','map_evidence.npz')}, trials={})
    for radius in args.radii:
        for strength in args.prior_strengths:
            name=f'radius{radius}_strength{strength}'.replace('.', '_')
            trial=out/name
            trial.mkdir(exist_ok=True)
            updated, prior=add_prior(graph, distances, res, radius, strength)
            refined, timing=solve(labels, updated, .8, policy='road-to-sidewalk')
            cv2.imwrite(str(trial/'road_temporal.colored.pgm'), refined)
            stats={**statistics(labels, refined, res),**timing,
                   'prior_floor_cells':int(((prior>0)&graph['eligible'][graph['active']]).sum())}
            settings={**cfg,'radius_m':radius,'prior_strength':strength,**stats}
            trialmeta={**meta, 'tactile_context':settings,
                       'road_region_consistency':{**meta['road_region_consistency'],'tactile_context':settings},
                       'baseline_definition':'input road-region map before tactile-context prior',
                       'baseline_cells':{str(k):int((labels==k).sum()) for k in range(3,7)},
                       'temporal_cells':{str(k):int((refined==k).sum()) for k in range(3,7)},
                       'changed_semantic_cells':stats['changed_cells'],
                       'label_policy':'Soft tactile-context prior; only road-to-sidewalk changes accepted; all RGB/other labels immutable.'}
            (trial/'manifest.json').write_text(json.dumps(trialmeta,indent=2))
            palette=json.loads((args.reference/'road_temporal.colored.json').read_text())
            palette.update(semantic_source='road-region consistency with tactile-context prior',tactile_context=settings)
            (trial/'road_temporal.colored.json').write_text(json.dumps(palette,indent=2))
            for filename in ('road_temporal.texture.png','calibration_used.yaml','semantic_coverage.png'):
                if (args.reference/filename).exists():shutil.copy2(args.reference/filename,trial/filename)
            report['trials'][name]=settings
            (out/'evaluation.json').write_text(json.dumps(report,indent=2))
            print(name,stats,flush=True)


if __name__=='__main__':main()
