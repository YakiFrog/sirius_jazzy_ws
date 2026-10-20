# General road-texture quality: six windows, ten conditions, full-BAG rebuild

The request is to improve the entire texture map, rather than redraw crosswalk
markings. The selected method contains no white-line, tactile, grass or other
semantic detector. It applies the same observation-quality rule to every cell.
Crosswalks, tiles and shaded pavement are inspection/evaluation examples only.

Reference map: `maps_waypoints/maps/theta_semantic_tactile_context_map_20261020`.
New comparison map: `maps_waypoints/maps/theta_texture_quality_map_20261020`.
Map Select prefix: `theta_texture_quality_map_20261020/road_temporal`.
Phone gallery: `maps_waypoints/maps/theta_texture_quality_20261020/images.html`.
The unconfirmed grass-expansion proposals are not adopted in this map.

## Projection/fusion experiment

Load the original cached compressed camera bytes, checking their SHA256 against
the full-recording manifest. Read recorded TF and `/scan3` directly, without ROS
replay or publishing to a live map. Use the existing camera model, native planar
BEV/floor nodes, 1.2–4.8 m range, nearest scan within 0.15 s, strict recorded-TF
visibility, 0.10 m radial margin, and 0.25 m obstacle padding. The BEV calibration
has size 512, extent 10 m and camera height 1.135 m. Native sampling stride is 2;
the map grid is unchanged at 0.05 m. No time offset, intrinsic or pose optimization
is performed in this experiment.

Alternative inverse sampling projects each fixed world-cell center directly
into the two calibrated fisheyes. It removes intermediate BEV/scatter sampling,
but keeps the same camera model, local planar assumption and scan gate. Sampling
maps are unit-checked against the production maps at identical floor points,
including a nonzero mount rotation. Front/back blending remains six degrees.
This alternative was evaluated but not selected for the full rebuild.

Each camera frame is reduced to one weighted RGB observation per map cell.
The original summed point weight is retained for EMA, while the maximum native
quality within that cell measures view quality without inflating it by density.
This is explicitly a per-frame/cell implementation; thresholds near a cell's
subsample boundary may differ slightly from the older point-level live node.

Six centers `[900,1500,2700,3000,4500,5100]`, each ±48 camera indices at step 2,
give 294 unique observations. They include crosswalks, tiles, shadow, a nearly
stationary interval and turning. Every fifth observation in each window is
held out: 234 train observations, 60 held-out observations. The same frames,
poses, grids and visibility inputs are used across conditions.

Ten conditions are fully specified in `variants.json`:

- Native scatter EMA baseline, native scatter with quality protection, and the
  same protected scatter with an additional lens-angle quality factor.
- Direct cell-center EMA, protected direct sampling with/without that factor,
  best-observation direct sampling, and three color-consistency fusion trials.

The lens-angle factor is `0.15+0.85*cos(theta)^2`. Color-consistency trials reject
photometrically incompatible weaker views and reset incompatible history when
a better view arrives. Their RGB consistency was worse than the simpler selected
protection method; they are retained as rejected variants, not enabled globally.

Selected method: **native scatter + rear weight 0.5 + relative-quality threshold
0.5 + EMA decay 0.7**, without lens-angle or color-consistency gating. Per-cell
best quality is tracked; an incoming observation must be at least half of the
best quality seen there. Weak observations still fill previously unseen cells.
Rear weight is a recording-condition prior, not a moving-object probability.
All areas use this same rule; no marking-specific coordinates enter fusion.

## Evaluation and interpretation

For each projection type, independently build a reference from held-out frames
with quality >=0.12, support from at least three held-out times, and temporal
median RGB. Stable color evaluation requires mean-channel MAD <=12. All variants
are compared on common observed cells. Two references expose projection-model
bias: a direct sampler should not be declared geometrically better merely
because a direct-sampled reference favors it.

The separate boundary proxy uses Canny edges, common support, MAD <=35, and an
eroded support mask. It reports symmetric nearest-edge distance, capped at ten
pixels, multiplied by grid resolution. Segment distances are averaged equally;
RGB errors are weighted by evaluated cell count. These are image-consistency
proxies, not surveyed position error, ATE/RPE, crosswalk straightness, or true
map distortion measurements. No Canny result is used by the selected algorithm.

| Hold-out reference | Baseline RGB error | Selected RGB error | RGB reduction | Baseline edge proxy [m] | Selected edge proxy [m] |
| --- | ---: | ---: | ---: | ---: | ---: |
| Native scatter | 9.008 | 6.198 | 31.19% | 0.1405 | 0.0798 |
| Direct sampling | 9.479 | 6.882 | 27.41% | 0.1760 | 0.0838 |

All per-window results, reference support counts and other conditions are in
`metrics.json` / `metrics.csv`. The selection used these same windows, so they
are exploratory validation, not an independent final test set. Six correlated
windows from one BAG are not six independent data collections. There is no
independent-BAG generalization claim.

An initial comparison completed numeric evaluation but failed while serializing
NumPy viewport integers. After fixing metadata serialization, seven existing
rasters were explicitly reused unchanged and three factor-separation conditions
were added. Both the initial log and successful final comparison log are kept.
This recovery does not introduce or discard observations.

## Full-recording rebuild and verification

Apply the selected method to all **5,956** native observations in camera order,
with zero skips. Every frame's recorded translation, quaternion and native floor
point count is checked against the original full-map manifest. Also build a
native EMA baseline using those identical 5,956 observations to separate policy
effects from the older asynchronous RGB-map generation.

- Grid, resolution, world viewports, semantic class-ID file and semantic vote
  evidence are unchanged. Original RGB alpha is byte-identical. No coordinate
  warp or deletion of map geometry is performed.
- The matched full-run baseline and selected run have identical RGB observation
  support. Where neither has an accepted native sample, retain original RGB:
  12,963 fallback cells. There are zero newly observed RGB cells in this run.
- `baseline.texture.png` preserves the matched full-recording control; the old
  map is retained separately. Review images show both the old/current comparison
  and the controlled same-frames comparison.
- All native observations use the original projection and recorded trajectory.
  This improves fusion-related blur and irregular contours, not all errors from
  camera calibration, terrain relief, rolling motion or SLAM drift. Those may
  require additional measured calibration or geometry/time corrections.
- Seven unit tests passed for camera-map parity, quality protection, stale-color
  rejection/reset in experimental modes, weak-view support, density-independent
  quality and fixed-cell support. Final semantic/overlay rasters are checked
  against actual output labels/RGB. Both phone galleries were checked at 390 px:
  all images decoded, HTTP 200, no horizontal overflow.

## Reproduction and saved data

From the workspace root (ROS Jazzy environment):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=74 OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python3 bash/startup_bash/collect_theta_texture_observations.py \
  maps_waypoints/maps/theta_sam3_yellow_full_20261019 \
  maps_waypoints/maps/theta_semantic_tactile_context_map_20261020 /tmp/texture_observations
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python3 bash/startup_bash/compare_theta_texture_quality.py /tmp/texture_observations /tmp/texture_comparison
ROS_DOMAIN_ID=74 OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python3 bash/startup_bash/rebuild_theta_texture.py \
  maps_waypoints/maps/theta_sam3_yellow_full_20261019 \
  maps_waypoints/maps/theta_semantic_tactile_context_map_20261020 /tmp/texture_full_map \
  --variant scatter_protected
python3 bash/startup_bash/render_theta_semantic_map.py /tmp/texture_full_map
python3 -m pytest -q bash/startup_bash/test_theta_texture_quality.py
```

The reconstruction CLI is not tied to a crosswalk, grass, map coordinate or frame
interval. Another recording can use its own cached camera manifest/calibration
and native reference-map metadata; the camera calibration and projection inputs
must be valid for that recording. Existing output folders are not overwritten.

`observations_part*.zip` retain all 294 projected native/direct input packets,
including colors, weights, quality, angular/directional factors and cell indices.
`comparison_artifacts.zip` retains the observation manifest, all trial rasters,
hold-out references, single-observation diagnostic images and review images.
Extract all archives into a study folder to reproduce the local comparison
without rereading the BAG. Gallery links use the recorded HTTP-server aliases;
the small `review/` gallery links to files in the repository directly.

Every archived file's SHA256 and ZIP CRC was checked. `source_versions.json`
records root/navigation revisions, exact working-source hashes and library
versions. Camera hashes, exact timestamps/scans and poses are preserved in the
observation manifest and final `texture_frames.json`. The original camera bytes
and BAG provenance were already archived by earlier full-recording studies and
are referenced rather than duplicated. The final map folder is directly
versioned, including matched baseline, unchanged labels/evidence, RGB, metadata,
verification and all standard phone views. No live mapping/Nav2 server is switched.
