"""Boundary-weighted spatial consistency of accumulated roadway/sidewalk votes.

Binary Potts energy: sum(-log(p) * cell_area) plus lambda[m] * boundary_length.
Color boundaries reduce the latter cost. Only observed class 5/6 cells may change;
class 4 carries connectivity without being relabeled. Unknown/grass are barriers.
This is a semantic prior, not a measurement of curb heights or ground truth.
"""
import argparse
import hashlib
import json
import shutil
import time
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import breadth_first_order, maximum_flow


def prepare_graph(labels, texture, votes, resolution, *, color_sigma=12.0,
                  blur_m=0.08, probability_clip=0.1, connectivity=8):
    """Build graph in image row order; eight directions reduce grid bias."""
    eligible = np.isin(labels, (5, 6)) & (texture[..., 3] > 0)
    active = (eligible | (labels == 4)) & (texture[..., 3] > 0)
    ids = np.full(labels.shape, -1, np.int32)
    count = int(active.sum())
    ids[active] = np.arange(count, dtype=np.int32)
    # Yellow paving is a semantic overlay on a floor: do not mistake its color
    # boundary for a curb. All other physical image edges remain in the guide.
    guide = texture[..., :3].copy()
    tactile = (labels == 4) & active
    if tactile.any() and eligible.any():
        nearest = ndimage.distance_transform_edt(~eligible, return_distances=False,
                                               return_indices=True)
        guide[tactile] = guide[tuple(nearest[:, tactile])]
    lab = cv2.cvtColor(guide.astype(np.float32) / 255, cv2.COLOR_BGR2LAB)
    if blur_m:
        lab = cv2.GaussianBlur(lab, (0, 0), blur_m / resolution)
    pairs, lengths = [], []
    if connectivity not in (4, 8):
        raise ValueError('connectivity must be 4 or 8')
    axial_weight = np.sqrt(2) - 1 if connectivity == 8 else 1.0
    directions = [((slice(None, -1), slice(None)), (slice(1, None), slice(None)), axial_weight),
                  ((slice(None), slice(None, -1)), (slice(None), slice(1, None)), axial_weight)]
    if connectivity == 8:
        diagonal_weight = 1 - 1 / np.sqrt(2)
        directions += [((slice(None, -1), slice(None, -1)), (slice(1, None), slice(1, None)), diagonal_weight),
                       ((slice(None, -1), slice(1, None)), (slice(1, None), slice(None, -1)), diagonal_weight)]
    for first, last, direction_weight in directions:
        connected = active[first] & active[last]
        if connectivity == 8 and direction_weight != axial_weight:
            # Diagonal connections must not jump across a protected/unknown
            # corner even when the diagonally opposite cells are both known.
            connected &= active[first[0], last[1]] & active[last[0], first[1]]
        difference = lab[first] - lab[last]
        contrast = np.sum(difference * difference, axis=-1)[connected]
        weight = np.exp(-contrast / (2 * color_sigma**2))
        pairs.append(np.stack((ids[first][connected], ids[last][connected])))
        lengths.append(resolution * direction_weight * weight)
    pairs = np.concatenate(pairs, axis=1)
    lengths = np.concatenate(lengths)
    total = votes[..., 2] + votes[..., 3]
    p = np.divide(votes[..., 2], total, out=np.full(labels.shape, 0.5, np.float32), where=total > 0)
    p = np.clip(p, probability_clip, 1 - probability_clip)
    # Repeated observations are correlated. Normalize evidence rather than
    # allowing frame count alone to overwhelm the spatial prior.
    road_cost = -np.log(p) * resolution**2
    sidewalk_cost = -np.log1p(-p) * resolution**2
    road_cost[~eligible] = 0
    sidewalk_cost[~eligible] = 0
    return dict(active=active, eligible=eligible, count=count, pairs=pairs,
                lengths=lengths, road_cost=road_cost[active],
                sidewalk_cost=sidewalk_cost[active])


def solve(labels, graph, strength_m, *, scale=10000, policy='both'):
    """Exact minimum s-t cut; source side denotes roadway (ID 5)."""
    n = graph['count']
    if n == 0:
        return labels.copy(), {'flow': 0, 'seconds': 0.0}
    started = time.monotonic()
    ids = np.arange(n, dtype=np.int32)
    first, last = graph['pairs']
    pair_cost = np.rint(graph['lengths'] * strength_m * scale).astype(np.int32)
    rows = np.concatenate((np.full(n, n, np.int32), ids, first, last))
    cols = np.concatenate((ids, np.full(n, n + 1, np.int32), last, first))
    values = np.concatenate((np.rint(graph['sidewalk_cost'] * scale).astype(np.int32),
                             np.rint(graph['road_cost'] * scale).astype(np.int32),
                             pair_cost, pair_cost))
    capacity = csr_matrix((values, (rows, cols)), shape=(n + 2, n + 2))
    capacity.eliminate_zeros()
    result = maximum_flow(capacity, n, n + 1)
    residual = capacity - result.flow
    residual.eliminate_zeros()
    reachable = breadth_first_order(residual, n, directed=True, return_predecessors=False)
    road = np.zeros(n + 2, bool)
    road[reachable] = True
    proposed = labels.copy()
    proposed[graph['active']] = np.where(road[:n], 5, 6)
    proposed[~graph['eligible']] = labels[~graph['eligible']]
    if policy == 'road-to-sidewalk':
        # Conservative adoption of the symmetric solution: do not grow an
        # already mistaken road patch into previously recognized sidewalk.
        proposed[labels == 6] = 6
    elif policy != 'both':
        raise ValueError('unknown adoption policy')
    assert np.all(np.isin(proposed[proposed != labels], (5, 6)))
    return proposed, {'flow': int(result.flow_value), 'seconds': round(time.monotonic() - started, 3)}


def statistics(before, after, resolution):
    report = {}
    for cid in (5, 6):
        n, _, components, _ = cv2.connectedComponentsWithStats((after == cid).astype(np.uint8), 8)
        areas = components[1:, cv2.CC_STAT_AREA] * resolution**2
        report[str(cid)] = {'cells': int((after == cid).sum()), 'components': n - 1,
                            'components_below_025_m2': int((areas < 0.25).sum())}
    report.update(changed_cells=int((before != after).sum()),
                  road_to_sidewalk=int(((before == 5) & (after == 6)).sum()),
                  sidewalk_to_road=int(((before == 6) & (after == 5)).sum()),
                  protected_cells_unchanged=bool(np.array_equal(after[~np.isin(before, (5, 6))], before[~np.isin(before, (5, 6))])))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--strengths', type=float, nargs='+', default=[0.8])
    parser.add_argument('--color-sigma', type=float, default=3.0)
    parser.add_argument('--connectivity', type=int, choices=(4, 8), default=8)
    parser.add_argument('--policy', choices=('both', 'road-to-sidewalk'), default='road-to-sidewalk')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    labels = cv2.imread(str(args.reference / 'road_temporal.colored.pgm'), 0)
    texture = cv2.imread(str(args.reference / 'road_temporal.texture.png'), -1)
    meta = json.loads((args.reference / 'manifest.json').read_text())
    resolution = meta['grid']['resolution']
    with np.load(args.reference / 'map_evidence.npz') as evidence:
        votes = evidence['temporal_votes'][::-1].copy()
    graph = prepare_graph(labels, texture, votes, resolution, color_sigma=args.color_sigma,
                          connectivity=args.connectivity)
    del votes
    report = {'reference': str(args.reference.resolve()), 'parameters': {
        'strengths_m': args.strengths, 'color_sigma_lab': args.color_sigma,
        'guide_blur_m': 0.08, 'probability_clip': 0.1, 'integer_scale': 10000,
        'connectivity': args.connectivity, 'adoption_policy': args.policy, 'tactile_connectivity_only': True},
        'reference_sha256': {name: hashlib.sha256((args.reference / name).read_bytes()).hexdigest()
                             for name in ('road_temporal.colored.pgm', 'road_temporal.texture.png', 'map_evidence.npz')},
        'baseline': statistics(labels, labels, resolution), 'trials': {}}
    for strength in args.strengths:
        trial = args.output / ('lambda' + str(strength).replace('.', '_'))
        trial.mkdir(exist_ok=True)
        refined, timing = solve(labels, graph, strength, policy=args.policy)
        cv2.imwrite(str(trial / 'road_temporal.colored.pgm'), refined)
        for name in ('road_temporal.texture.png', 'calibration_used.yaml', 'semantic_coverage.png'):
            if (args.reference / name).exists():
                shutil.copy2(args.reference / name, trial / name)
        stats = {**statistics(labels, refined, resolution), **timing}
        trialmeta = {**meta, 'road_region_consistency': {**report['parameters'], 'strength_m': strength,
                     'reference_map': str(args.reference.resolve()), **stats},
                     'label_policy': f'Spatial binary Potts proposal; adoption policy {args.policy}; all other labels and RGB preserved.',
                     'baseline_definition': 'input reference map before spatial road-region refinement',
                     'baseline_cells': {str(k): int((labels == k).sum()) for k in range(3, 7)},
                     'temporal_cells': {str(k): int((refined == k).sum()) for k in range(3, 7)},
                     'changed_semantic_cells': stats['changed_cells']}
        (trial / 'manifest.json').write_text(json.dumps(trialmeta, indent=2))
        palette = json.loads((args.reference / 'road_temporal.colored.json').read_text())
        palette.update(semantic_source='boundary-weighted region consistency after shadow rescue',
                       road_region_consistency=trialmeta['road_region_consistency'])
        (trial / 'road_temporal.colored.json').write_text(json.dumps(palette, indent=2))
        report['trials'][str(strength)] = stats
        (args.output / 'evaluation.json').write_text(json.dumps(report, indent=2))
        print(strength, stats, flush=True)


if __name__ == '__main__':
    main()
