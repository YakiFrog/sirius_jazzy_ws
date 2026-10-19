# Yellow tactile paving and visible-surface persistence

This experiment addresses two user observations: gray patterned sidewalk was
classified as tactile paving, and an actually visible strip disappeared during
short image-detector misses. It supersedes the earlier raw-history0.85s tactile
gap-fill default. Previous study artifacts and maps remain intact.

## Controlled inputs and detection change

The same ten centers and every offset-8..8 provide170 capture frames /340 front
and rear views from `real_both_mapping_20260920_113447_recovered`. Calibration,
640x480 perspective input, SAM3 internal resolution512, score cutoff0.5 and
per-class threshold0.5 remain fixed. Every uploaded JPEG SHA256 matches the
previous dense study. No panorama/resolution change is mixed into this trial.

The active prompt is `grass, yellow tactile paving, roadway, sidewalk`. Former
ambiguous tactile/line-type prompts are inactive. Shared class configuration and
fallbacks now register yellow tactile paving asID4; old aliases remain registered
for map/configuration compatibility. All other semantic IDs remain unchanged.

The experiment does not mutate the running backend's registry. Yellow is the
only unregistered prompt in the recorded registry, so nonzero-score unknown-ID
winners are assignedID4 client-side, with original server-class PNGs preserved.
Normal future mapping registers the new prompt using the shared configuration.
Backend settings are restored, verified in `yellow/restoration.json`.

## Tracking change

An explicit visual track replaces the short raw-detection history expiry. A new
track requires an above-cutoff SAM3 yellow-tactile component of at least16pixels,
with at least35% yellow appearance evidence (OpenCV HSVH15..40,S>=45,V>=35).
The component test preserves dark ridges/shadows inside a recognized yellow
block. Rejected gray tactile components become unknown, not assumed sidewalk.

Each subsequent frame warps the previous tracked mask with DIS bidirectional
flow. Current-to-previous/previous-to-current consistency, image differences,
local texture, frame boundaries and yellow component evidence must still pass.
Only matched pixels remain; dilation never bridges an occluder or border. IDs1/2/3
are protected. A track retains its detector score and is not counted as a fresh
detector confirmation. The tracker does not reset merely because SAM3 misses
several frames. Timestamp reversal, shape change or a gap between received
frames exceeding1.5s resets it; this is a capture discontinuity guard, not a
detector-miss timeout. Pixels are removed on lost image correspondence,
occlusion or leaving the view.

Production THETA uses the causal tracker by default. Offline mapping additionally
evaluates independent forward and reverse tracks and unions their valid tactile
masks. This is mask/optical-flow tracking, not SAM3's learned video predictor.
Map accumulation still weights individual frame outputs; correlated tracked
masks must not be interpreted as independent posterior probabilities.

## Results and limits

Provisional visual-check ROIs from the earlier study are reused unchanged:

| Method | Tactile coverage,6 positive ROIs | Tactile coloring,10 gray-pavement ROIs | Aligned temporal IoU |
|---|---:|---:|---:|
| Previous ambiguous prompts |1547/10250 (15.1%)|1770/38635 (4.58%)|52.80%|
| Yellow prompt only |6900/10250 (67.3%)|0/38635|66.89%|
| Yellow + causal tracking |7442/10250 (72.6%)|0/38635|91.04%|
| Yellow + offline bidirectional tracking |7556/10250 (73.7%)|0/38635|97.18%|

ROI polygons are small assistant-drawn interior checks, not expert/user-verified
ground truth. Coverage is not whole-image accuracy. Temporal IoU measures
consistency; it does not establish correct semantics. No universal accuracy
claim is made. Yellow-only semantics intentionally excludes non-yellow tactile
paving, and the appearance gate may fail under unusual lighting. Road/sidewalk
ambiguity elsewhere is not solved by this change.

The initial tracking log omitted class0 negative ROIs from its summary. The final
report fixes that selection using the identical saved masks and flow totals;
`yellow/evaluation.json` and the report log contain all6 positive/10 negative
ROIs. No detector/tracker output changes were needed for that reporting fix.

Front2103 previously mislabeled22437 gray-tile pixels; the yellow prompt and both
trackers label0. The same absence of false tactile detections holds across all17
front views of segment2100. Front3297 changes from0 to7382 detected tactile
pixels before tracking. Rear4494 changes from0 to6105. Rear4507 is visually out
of view/occluded by the robot and has no stale tactile track; the static example
is retained. Phone GIFs show the same17 capture frames with identical images.

## Generated map and phone review

`maps_waypoints/maps/theta_semantic_yellow_map_20261019/` contains regenerated
semantic-only, overlay and texture-only maps and same-bounds local crops. All170
frames match input hashes and recorded TF; none is skipped. The existing scan
visibility gate is unchanged. Baseline/new point XYZ, RGB and weights are equal
for every mapped frame, and the entire final texture equals the previous map.

The new map has4506 tactile cells at0.05m resolution:2987 previously non-tactile
cells become tactile,2556 previous tactile cells receive other/unknown labels.
Those counts alone are not accuracy measurements. Geometry and meaningful
points are not deleted. Semantic inference covers the ten evaluated intervals;
the full-route RGB background does not imply full-BAG semantic inference.

Phone URLs on the existing Tailscale-only server:

- `http://100.97.20.32:54369/yellow/images.html`: before/after JPEGs and GIFs.
- `http://100.97.20.32:54369/yellow_map/images.html`: three map image types.

Both galleries are checked at390px mobile viewport: all images decode, with no
horizontal overflow. HTTP endpoints return200.82 tests and the navigation
package build pass. Tests include persistence through13 detector misses,
occlusion removal/no resurrection, translation out of view, gray-tile rejection,
protected grass, timestamp/shape resets and stable class-ID configuration.

## Archive and reproduction

`raw/yellow_seg*.zip` preserves all new inference classes, original server
classes, scores, inference metadata, causal/bidirectional classes/scores and
overlays/masks. Extract to the original dense STUDY directory, then copy the
files from this archive's `yellow/` into `STUDY/yellow/`. Dense raw camera inputs
and original baseline predictions are already archived in
`experiments/theta_sam3_temporal/2026-10-19/raw/`; restore those first. BAG and
calibration provenance are in the earlier projection/temporal studies. This
archive avoids duplicating those immutable inputs. `raw_file_sha256.json` checks
every new source artifact. `SHA256SUMS` checks the archive. Logs, configuration,
input manifest, source revisions, evaluation and phone screenshots are retained.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
# Only this step invokes the existing SAM3 server:
python3 bash/startup_bash/collect_theta_sam3_yellow.py STUDY
# Recompute tracking/evaluation from saved predictions:
python3 bash/startup_bash/track_theta_sam3_yellow.py STUDY
# Or only rebuild the phone report/ROI summary from the saved evaluation:
python3 bash/startup_bash/track_theta_sam3_yellow.py STUDY --report-only
ROS_DOMAIN_ID=73 python3 bash/startup_bash/map_theta_sam3_tactile.py \
  STUDY EXISTING_TEXTURE NEW_MAP --semantic-dir yellow --allow-reclassification
python3 bash/startup_bash/render_theta_semantic_map.py NEW_MAP
```

Keep output folders separate from previous archived maps. Detector/class changes
require explicit `--allow-reclassification` in the map builder; the earlier
fill-only experiment still retains its stricter original-tactile assertions.
