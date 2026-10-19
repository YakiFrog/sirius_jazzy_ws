# Boundary-weighted road / sidewalk region consistency

Input: `maps_waypoints/maps/theta_semantic_shadow_map_20261020` (the full
5,956-camera-frame map, including the 701-frame shadow tactile update).
Output: `maps_waypoints/maps/theta_semantic_regions_map_20261020`.
Map Select prefix: `theta_semantic_regions_map_20261020/road_temporal`.

The user observed that small roadway patches scattered inside a continuous
sidewalk are usually incorrect. This experiment adds an optional spatial prior
after observation fusion. It does not rerun SAM3 or change its live settings.

## Logic

For observed class-5/6 cells, normalize the accumulated roadway/sidewalk votes.
Use their clipped fractions (0.1–0.9) as unary evidence, rather than multiplying
confidence indefinitely by the number of correlated observations. These fractions
are not calibrated probabilities. Class 4 is allowed to transmit connectivity;
unknown and grass are barriers. Output class 4, grass and all other classes are
immutable.

Find a minimum s-t cut of a binary Potts energy: unary `-log(fraction)` times cell
area plus `lambda * boundary length * image similarity`. Similarity is
`exp(-squared_Lab_difference / (2*sigma^2))`, using the original map texture
blurred by 0.08 m. For this guide only, replace tactile color by the nearest
observed class-5/6 floor color, avoiding a spurious boundary along the tactile
strip. No replacement is made in the output texture. Strong image differences
reduce the cost of retaining a class boundary. This is a color boundary proxy,
not a measured curb height, instance boundary or proof of physical continuity.

Four-neighbor trials showed axis-aligned artifacts. Eight-neighbor trials use
cardinal weight `sqrt(2)-1` and diagonal weight `1-1/sqrt(2)` to reduce grid bias.
Diagonal edges cannot cross an unknown or protected corner. Costs are rounded
to integers at scale 10,000 for SciPy maximum_flow. Source-side cells mean road.

The symmetric cut can expand an erroneous roadway region. The final adoption
policy therefore accepts only road-to-sidewalk proposals; original sidewalk
cells cannot become roadway. This conservative result is a filtered proposal,
not itself the unrestricted minimum-energy solution. It intentionally does not
correct sidewalk islands inside real road. Original observation votes are copied
unchanged into the final `map_evidence.npz`, separately from the refined labels.

No location, frame index, ROI or hand-selected class seed enters this algorithm.
Resolution and grid origin come from the input metadata; spatial parameters are
in meters. It can be applied to another map with the same class/vote encoding,
but performance on an independent BAG has not been established.

## Sequential trials and choice

All 9 outputs and settings are retained in `trial_artifacts.zip`; class-ID PNGs
are lossless representations of the original PGM pixels. Their original PGM and
decoded-pixel hashes are recorded in `trial_label_hashes.json`.

| Stage | Connectivity | Lab sigma | Lambda (m) | Adoption |
| --- | --- | --- | --- | --- |
| Initial | 4 | 12 | 0.08, 0.2, 0.4 | symmetric |
| Stronger image boundaries | 4 | 3 | 0.2, 0.4, 0.8 | symmetric |
| Reduced grid bias | 8 | 3 | 0.4, 0.8 | symmetric |
| Selected conservative output | 8 | 3 | 0.8 | road to sidewalk only |

Choice was exploratory on this recording: reduce speckled errors, retain visible
crosswalk/curb boundaries, and reject the observed expansion of road into
sidewalk. The 14 source-texture interior ROIs cover 10 existing review intervals;
their coordinates, source-image views and every trial's counts are retained.
They are provisional annotations made during exploration, not an independent
test set or expert-confirmed ground truth. They never drive output labels.

Selected output:

- 27,687 cells change from road to sidewalk (69.2175 square meters at 0.05 m).
  Zero sidewalk-to-road changes.
- Road components below 0.25 square meters: 2,448 to 850 (8-connected component
  statistics). Counts are a fragmentation measure, not an accuracy score. Some
  residual components are isolated by unobserved map holes.
- In provisional sidewalk interiors: 25,413 / 30,713 matching cells (82.74%) to
  27,193 / 30,713 (88.54%). Provisional road interiors: 6,870 / 6,870 matching
  cells before and after. These aggregate figures do not demonstrate universal
  correctness or boundary accuracy.
- Large wrong regions remain: sidewalk ROIs in intervals 300 and 5650 remain
  poorly classified. The symmetric cut worsened the latter; the conservative
  adoption avoids that additional regression. Spatial continuity cannot recover
  a correctly named class when a broad region's evidence is consistently wrong.
- Entire texture file is byte-identical to the reference. All protected class
  labels, observed support, dimensions, origin and resolution are preserved.
  No point geometry, robot pose or camera projection is modified.

## Reproduction

From the workspace root, using Python with OpenCV-contrib, NumPy and SciPy:

```bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 bash/startup_bash/refine_theta_road_regions.py \
  maps_waypoints/maps/theta_semantic_shadow_map_20261020 /tmp/road_region_reproduction \
  --strengths 0.8 --color-sigma 3 --connectivity 8 --policy road-to-sidewalk
python3 bash/startup_bash/evaluate_theta_road_regions.py \
  maps_waypoints/maps/theta_semantic_shadow_map_20261020 /tmp/road_region_reproduction \
  /tmp/road_region_map --select lambda0_8
python3 bash/startup_bash/render_theta_semantic_map.py /tmp/road_region_map
python3 -m pytest -q bash/startup_bash/tests/test_refine_theta_road_regions.py
```

Reproduce the earlier trials by explicitly selecting their connectivity, sigma,
strengths and `--policy both`. The initial trials predate the explicit adoption
option; their effective symmetric mode is recorded in the archived settings.

Seven meaningful tests cover speckle removal, retaining an extended real class
boundary, retaining a small region behind a strong image boundary, protected
classes and unknown barriers, zero-strength parity, diagonal-corner barriers and
the conservative no-road-expansion rule. Final rendered semantic/overlay rasters
are checked against the actual output labels. Both phone galleries were checked
at 390 px: HTTP 200, all images decoded, no horizontal overflow.

`source_versions.json` records working-source hashes and library versions.
`verification.json`, `browser_checks.json`, logs, evaluation JSONs, all trial
labels/metadata and phone-sized comparison images are preserved here. The source
map, texture and observational votes are already versioned in the repository;
their SHA256 values appear in the evaluation records. Full map images and loadable
map files are in the output folder above.
