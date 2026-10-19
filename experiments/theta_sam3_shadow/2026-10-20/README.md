# Sequential shadow-tactile experiments

The user identified a dark section in the otherwise satisfactory full-recording
map. The physical yellow tactile strip is visible in the source photos but was
often absent from SAM3 output. This study keeps the good full-map result and
adds only validated tactile candidates from enhanced images. The final map
texture is byte-identical to the preceding full map.

## Sequence and inputs

The exploratory set has eight camera frames, each with front/rear views:
shadow2500,2650,2800,2950; gray-pavement controls2100,5100; normal examples900,3300.
Every original upload JPEG is verified against the immutable full-recording
cache. Camera geometry, projection and score cutoff0.5 are fixed. The standard
resolution is512; the1008 trial is explicitly recorded and was not selected.

Twenty-three conditions were tried sequentially, with exact enhanced upload
JPEGs, original server-class PNGs, assigned-class PNGs, scores and settings kept:

1. Gamma0.65 and Lab-L CLAHE2.
2. Stronger gamma0.45/0.30 and CLAHE4.
3. Shadow white balance plus gamma0.65/0.45.
4. Resolution1008 with original/gamma/white-balance inputs.
5. Plain tactile-paving wording with original/gamma/white-balance inputs.
6. Auxiliary yellow-only inference with original/gamma/white-balance inputs.
7. Auxiliary line-type wording, retained as a rejected comparison.
8. Lower-half ground crops with gamma/white-balance inputs.
9. Auxiliary `yellow tactile paving in shadow`, and `tactile paving strip`.

The plain/strip wording often produced no tactile detections. Higher resolution
did not consistently help. Gamma alone recovered some views. White balance and
yellow-only inference recovered more, including a case where a valid yellow
proposal was suppressed by a higher-scoring sidewalk mask in multiclass output.
The line-type wording again colored gray tiles in raw proposals. The selected
combination was white balance + gamma0.45 + the shadow-specific yellow prompt,
queried separately and used only to propose tactile additions.

`pilot_evaluation_all.json` records all23 conditions /368 inference calls.
Its exploratory metrics concern single-frame detector proposals, not the
full-map temporal baseline. Per-run configs, frame versions, upload hashes,
prompts and resolution are preserved. Unknown sole target prompts are assigned
ID4 client-side; the backend registry is never modified. All runs enforce backend
settings restoration on exit. Latest restoration snapshots per output directory
are retained; individual inference metadata retains every actual setting.

## Selected enhancement and safeguards

White balance estimates a neutral median from image rows60%-95%, V8..100 and
saturation<120 (fallback V8..120). Gains are mean(channel medians)/channel median,
clipped0.65..1.8. The adjustment fades with `clip((110-V)/70,0,1)`. Then a common
BGR gain applies `255*(V/255)^0.45`, capped at6, with V=max(B,G,R). This changes
recognition inputs only; it never replaces the map texture or changes geometry.

Auxiliary inference uses `yellow tactile paving in shadow`, resolution512 and
threshold0.5. A candidate needs a component of at least16 pixels, with at least
20% yellow evidence (OpenCV H15..40,S>=45,V>=35) on the enhanced image. The
component's median original-image V must be<=90. The20% choice was compared to
35% and to reducing saturation; keeping saturation45 recovered the same useful
coverage while retaining the stronger gray rejection criterion. Original
grass/wall/floor pixels are protected. Other classes from auxiliary inference
are never adopted. Existing baseline tactile labels are retained.

Forward/reverse DIS tracking uses the same consistency, texture and photometric
guards as the existing method, applied to the enhanced images. Scores remain
detector scores; a previous roadway score is never inherited by a tactile
addition. Tracks do not expire merely because a few detector calls miss. These
are correlated tracked observations, not independent confidence estimates.

The component fraction/saturation are optional arguments to the existing helper
and tracker. Their normal defaults remain0.35/45. The existing live/default
SAM3 path has not been replaced with experimental extra inference. The selected
offline sequence below creates the shadow-completion map.

## Dense and continuous validation

The selected candidates were compared on136 camera frames /272 views: each of
the eight centers with every offset-8..8. This includes68 gray-control views and
68 normal-control views. Two auxiliary methods were compared: yellow-only and
shadow-specific yellow, both using the same white balance/gamma input.

| Method | Tactile coverage in8 shadow interior ROIs |
|---|---:|
| Existing full-recording temporal baseline |26.45%|
| White balance + yellow-only + tracking |68.25%|
| White balance + shadow wording + tracking |79.42%|
| Continuous701-frame shadow run |79.60%|

The ROIs are small provisional source-photo interior polygons, not complete or
expert/user-verified ground truth. An initially misplaced draft was corrected
against source photos before numerical comparison. Coverage is not segmentation
accuracy. Across the68 full gray-control views, **zero new tactile pixels** were
added. The shadow negative ROIs also show zero additions. The two normal positive
ROIs stay at67.33%;73 added pixels occur elsewhere in normal-control images.
Some individual rear/occluded views still have no valid detection. No universal
low-light robustness or cross-BAG accuracy claim is made.

The final continuous run covers frames2350..3050 inclusive:701 camera frames /
1402 auxiliary views, including frames not used for choosing the initial method.
Tracking is continuous across the section. This is within-recording validation,
not a separate held-out recording. `dense_evaluation*`, `region/dense_evaluation*`
and all per-frame masks/counts retain the exact numerators and conditions.

## Map update and checks

The full reference map uses all5956 frames. Native map generation is repeated
for the701 modified frames with the same recorded TF, scans, visibility margins,
calibration, sampling and quality weights. Old selected-frame votes are
subtracted from the full temporal vote grid and replaced with the new votes.
The subtraction minimum is0.0; no unexplained negative mass is clipped. Final
labels change only where the updated argmax supports a new ID4. No coordinate-
specific correction or unobserved line completion is used.

The result adds5256 tactile cells (13.14m2 at0.05m resolution):5219 formerly
sidewalk cells and37 formerly unknown cells. All existing grass, roadway and
tactile labels remain unchanged. The entire reference RGBA texture, including
its previous hole fills, remains byte-identical. Native XYZ/RGB/quality weights
are equal for old/new semantic inputs on all701 frames; recorded poses and point
counts also match the prior full-map run. No selected frame is skipped.

The directly tracked map is
`maps_waypoints/maps/theta_semantic_shadow_map_20261020/`.
Map Select prefix: `theta_semantic_shadow_map_20261020/road_temporal`.

Phone review (Tailscale required):

- `http://100.97.20.32:54369/shadow/images.html`: map/photo before-after and GIFs.
- `http://100.97.20.32:54369/shadow_map/images.html`: three map image types.

84 theta/class tests and the package build pass. The new regression test checks
that the weaker component fraction is opt-in and cannot seed a gray-only mask.
HTTP, image dimensions, mobile decoding and texture/label invariants are checked.

## Storage and reproduction

Archive ZIPs restore paths relative to STUDY. They retain all actual enhanced
network JPEG uploads, inference PNGs/JSON, crop color canvases, gate masks and
processed label/score PNGs. Source-view JPEGs and bulk derived overlay JPEGs are
not duplicated into Git: source uploads are already kept in the full-recording
cache with hashes, and overlays can be regenerated from saved classes/scores.
All these files remain in the local study folder. Selected comparison JPEGs,
GIFs and HTML/JSON are directly browseable. `map_input` is derived from archived
masks and the reference map; its input hashes and the native subset evidence are
retained. SHA256 manifests cover retained raw files and the final archive.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
# Exploratory examples; every method/config remains in pilot/:
python3 bash/startup_bash/experiment_theta_sam3_shadow.py FULL_STUDY STUDY/pilot
python3 bash/startup_bash/experiment_theta_sam3_shadow.py FULL_STUDY STUDY/pilot \
  --target-only --tactile-prompt 'yellow tactile paving in shadow' --methods shade_wb45
# Dense controls and continuous target region:
python3 bash/startup_bash/experiment_theta_sam3_shadow.py FULL_STUDY STUDY/dense \
  --radius 8 --target-only --tactile-prompt 'yellow tactile paving in shadow' --methods shade_wb45
python3 bash/startup_bash/analyze_theta_sam3_shadow.py STUDY --method shade_wb45
python3 bash/startup_bash/experiment_theta_sam3_shadow.py FULL_STUDY STUDY/region/dense \
  --centers 2700 --radius 350 --target-only \
  --tactile-prompt 'yellow tactile paving in shadow' --methods shade_wb45
cp STUDY/review_rois.json STUDY/region/review_rois.json
python3 bash/startup_bash/analyze_theta_sam3_shadow.py STUDY/region --method shade_wb45
python3 bash/startup_bash/patch_theta_shadow_map.py STUDY/region REFERENCE_MAP NEW_MAP
python3 bash/startup_bash/render_theta_semantic_map.py NEW_MAP
```

The frame selection above identifies this experiment's scene. The enhancement,
darkness/color gates, projection and tracking do not depend on these frame IDs
or on map XY coordinates. Different recordings still require their own
calibration and validation, especially if tactile material/lighting differs.
