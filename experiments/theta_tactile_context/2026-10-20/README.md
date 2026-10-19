# Tactile paving as sidewalk context: 16 exploratory trials

The user identified a useful contextual cue: continuous tactile guidance paving
surrounded by roadway labels is implausible in this environment, while roadway
and sidewalk colors are similar. We tested a finite sidewalk preference near
continuous tactile strips. This is a local layout prior, not a universal rule
that both sides of every tactile marking must be sidewalk. Crossings, endpoints,
warning pads and false tactile detections require care.

Reference: `maps_waypoints/maps/theta_semantic_regions_map_20261020`.
Comparison map: `maps_waypoints/maps/theta_semantic_tactile_context_map_20261020`.
Map Select prefix: `theta_semantic_tactile_context_map_20261020/road_temporal`.
The previous map and all inputs remain available. SAM3 is not rerun or modified.
This is an experimental postprocessing variant, not a newly trained model.

## Method and sequence

1. Thin the existing class-4 map; close one-pixel holes for anchor extraction
   only. Ignore skeleton components shorter than 3 m. Initial anchors reject
   branch/endpoint neighborhoods within 1 m and half-width above 0.225 m.
   Redundant diagonal skeleton edges are suppressed at digital corners.
2. Initial geodesic distance divides grid step length by image similarity. The
   radius/strength grid is `[0.35, 0.6, 0.9] m x [2.5, 5]` (6 trials).
   Strong photometric boundaries, including shadow, limited useful propagation.
3. A second anchor extractor uses meter-scale local strip shape, avoiding the
   sensitivity of skeleton degree to tiny spurs. For each long-component
   skeleton pixel, collect tactile cells in a square of radius `guard+0.3 m`,
   fit PCA, require minor/major variance <=0.10 and `sqrt(12*minor_variance)`
   <=0.45 m, and require longitudinal 3rd/97th percentiles to extend at least
   1 m on both sides. This rejects ends, broad pads and substantive branches.
4. Repeat the same 6 radius/strength trials using these anchors and connectivity
   distance, which does not treat a shadow's color edge as a physical barrier.
   Unknown/grass still block propagation, including diagonal corner shortcuts.
5. Narrow priors correct strips but leave broad wrong regions. Try radii
   `[1.2, 1.6] m x strengths [5, 8]` (4 additional trials).

The original eight-neighbor, image-weighted Potts graph remains in use. Add
`strength * cell_area * max(1 - geodesic_distance/radius, 0)` to the cost of
choosing road. This finite term prefers sidewalk near reliable tactile anchors.
The distance is capped; it does not propagate across the entire connected map.
Only original road-to-sidewalk proposals are accepted. Existing sidewalk,
tactile, grass, other classes, RGB, coordinates and scale remain immutable.
No curb height is measured. Connectivity distance is a prior on observed floor
support, not evidence that two connected surfaces share the same physical use.

Selected comparison: local-shape anchors, connectivity distance, radius 1.6 m,
strength 8, original spatial smoothing lambda 0.8 m and Lab sigma 3. This condition
gave the strongest correction in the inspected point-adjacent floor regions
without changes in the selected roadway controls. Selection and ROIs were
exploratory on the same recording; there is no independent test-BAG result.
The defaults retain the first exploratory configuration; reproduce the selected
configuration explicitly using the command below.

## Results and limits

- 3,402 safe anchor pixels from 6,140 long-component skeleton pixels.
- 13,967 cells change road to sidewalk (34.9175 square meters). No other class
  transitions; zero sidewalk-to-road changes. All 5,956 original camera-frame
  observations remain the input basis. Observation votes are copied unchanged;
  refined labels additionally include a spatial prior.
- Four provisional point-adjacent floor interior ROIs: 24/2,114 cells initially
  labeled sidewalk, 1,975/2,114 afterward. These are interior-label agreement
  counts under exploratory source-texture annotations, not general accuracy.
- Three added roadway controls at a curb/crossing: all 1,047 cell labels are
  unchanged (923 were already roadway; existing mistakes are not repaired).
  The previous four roadway interiors also retain all 6,870 roadway cells.
  Their combined unchanged sample does not prove every real roadway is intact.
- Original 14 interior ROIs from the region-consistency study do not change.
  They alone would not measure the additional point-adjacent improvement.
- Broad wrong regions and false road bands outside the prior's reach remain.
  Even the strongest tested prior is incomplete; expanding it without geometric
  or contextual validation could consume real roadway. No perfect recognition
  or cross-recording robustness is claimed.
- Entire texture file is byte-identical. Rendered semantic/overlay images are
  checked against the output labels. Protected classes and observation support
  are unchanged. Seventeen unit tests cover both the original spatial method
  and anchor geometry, shadow edges, unknown barriers and finite prior behavior.
- Both galleries load at a 390-px phone viewport with all images decoded and
  no horizontal overflow. The full comparison map is generated for review;
  no live Nav2 map or SAM3 backend settings are switched.

## Reproduction and preservation

From the workspace root, with OpenCV-contrib, NumPy and SciPy:

```bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 bash/startup_bash/experiment_theta_tactile_context.py \
  maps_waypoints/maps/theta_semantic_regions_map_20261020 /tmp/tactile_context_reproduction \
  --anchor-mode local-shape --distance-mode connectivity --radii 1.6 --prior-strengths 8.0
cp experiments/theta_tactile_context/2026-10-20/context_rois.json /tmp/tactile_context_reproduction/
python3 bash/startup_bash/review_theta_tactile_context.py \
  maps_waypoints/maps/theta_semantic_regions_map_20261020 /tmp/tactile_context_reproduction \
  /tmp/tactile_context_map --select radius1_6_strength8_0
python3 -m pytest -q bash/startup_bash/tests
```

The fixed source-texture ROI coordinates and display crops are review-only and
do not enter the algorithm. All 16 trial class-ID rasters (lossless PNG), full
manifests, palette metadata, three sets of anchor/exclusion masks, float geodesic
distance evidence and evaluations are in `trial_artifacts.zip`. PNG pixel hashes
are checked against the original PGM labels; original PGM hashes are retained.
The first trial settings predate explicit mode fields; their modes are the
default `junction-guard` / `appearance`, recorded here and in the final code.
CRC checks, source/library hashes, input-map hashes, all evaluation counts,
logs and review images are preserved. Original RGB and observational evidence
are already versioned in the reference map and are not duplicated in the ZIP.
