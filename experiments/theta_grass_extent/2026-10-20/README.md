# Connected grass extent candidates

The user reports that recognized grass is mostly correct, but nearby true grass
is missing. The exact missing location was requested asynchronously and was not
specified when this comparison was generated. These are candidate-only results,
not a confirmed correction or replacement of the current map.

Input: `maps_waypoints/maps/theta_semantic_tactile_context_map_20261020`.
Outputs and phone gallery: `maps_waypoints/maps/theta_grass_extent_20261020`.
The existing map, texture, geometry and runtime configuration are unchanged.

## Existing grass classification

The input map uses the SAM3 `grass` prompt, class ID 3, with mask confidence
threshold 0.5. Inferred classes are projected onto the ground under the recorded
TF and `/scan3` visibility gates. Each observed semantic sample contributes its
geometric/directional quality weight to a class vote grid. A cell receives the
largest-vote class if total semantic weight is at least 0.3; otherwise it is
unknown. The recent sidewalk/tactile postprocessing preserved class 3.

Thus some adjacent cells have nonzero grass evidence but lose to sidewalk/road,
or remain unknown because their total evidence is too small. The observational
vote fraction is not calibrated probability. Scores/masks discarded below the
original SAM3 cutoff cannot be reconstructed from the map votes; recovering
completely missed regions would need source-image review or new inference.

## Candidate method

Start from existing grass cells with RGB and native sample support. Grow through
four-connected neighboring cells only if each traversed cell:

- Has original observed RGB alpha and nonzero native `sample_coverage`.
- Is originally unknown, roadway or sidewalk; tactile and other classes block it.
- Has absolute grass vote weight >=0.2 and a minimum grass/total vote fraction.
- Has compatible green appearance: OpenCV HSV H 25–90, S >=40, V >=12.

Each iteration moves at most one map cell; the number of iterations is
`floor(radius/resolution)`. This bounds four-neighbor path length, including
around barriers. It is not an unconstrained mask dilation. There is no expansion
of RGB, laser visibility, point geometry or unobserved ground. Native coverage
counts samples, not independent camera frames; no per-cell multi-frame quorum is
claimed. Low-total-weight unknown cells are retained as uncertain candidates.

Tested radii: 0.25 and 0.50 m; vote fractions: 0.2, 0.3, 0.4 (6 conditions).
Display selection: radius 0.25 m / fraction 0.3. It adds 6,173 candidate cells,
15.4325 square meters: 3,796 previously unknown, 97 roadway, 2,280 sidewalk.
Increasing the radius to 0.5 m at the same fraction adds only six more cells.
This conservative display selection is not an accuracy optimum established by
ground truth. Yellow-green distinguishes candidates from original green grass.

Important limits: the green gate can miss dry/brown grass; projected overhanging
vegetation and green reflections can still be false positives. Existing grass
seeds are trusted provisionally, not independently validated. Candidate masks
need comparison with the user's actual missing area. Locations with no RGB or
no accepted laser-visible samples remain unclassified; vegetation behind a
blocking object is not invented as map surface.

## Reproduce and inspect

```bash
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 python3 bash/startup_bash/experiment_theta_grass_extent.py \
  maps_waypoints/maps/theta_semantic_tactile_context_map_20261020 /tmp/grass_extent_reproduction
python3 -m pytest -q bash/startup_bash/tests/test_theta_grass_extent.py
```

All trial class-ID PNGs, addition masks, baseline labels, selected label proposal,
selected per-cell observational evidence, comparisons, gallery and input hashes
are retained in the output folder. `selected.classes.png` is a proposal, not a
new active semantic map. Its candidate mask is the authoritative distinction
between original classes and proposed additions. The reference RGB and full
observation votes are already versioned and are not duplicated here.

Six tests passed: distance limit, observation evidence, tactile/unobserved
barriers, native visibility requirement, incompatible pavement color and weak
grass fraction. Every candidate output was additionally checked for observed
support, original-class protection, color/evidence gates and unchanged input
hashes. The phone gallery was checked at 390 px: all images decoded, HTTP 200,
no horizontal overflow. These checks establish implementation behavior, not
semantic accuracy. `source_versions.json`, `verification.json`, logs, hashes and
the phone screenshot preserve the experiment process.
