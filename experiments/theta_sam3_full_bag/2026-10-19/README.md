# Full-recording yellow-tactile semantic mapping

The complete `real_both_mapping_20260920_113447_recovered` camera stream is
processed in capture order:5956 camera frames /11912 front-and-rear views,
without sampling or ROS replay dropping frames. This expands the preceding
170-frame/ten-interval experiment. The camera header timestamps are strictly
increasing, spanning1578.432386392s; intervals range0.196094799..1.195998247s.
All are within the existing1.5s tracker continuity guard.

## Fixed method

The validated perspective projection,640x480 views, SAM3 internal resolution512,
score cutoff0.5 and prompt `grass, yellow tactile paving, roadway, sidewalk`
remain unchanged. Original raw sparse-study predictions are reused only for
exact input matches. Other observations are inferred sequentially with server
frame-version matching. The backend class registry remains unchanged; its only
unknown prompt is assignedID4 client-side, retaining the original server map.
Backend settings are restored on exit. Input image hashes, uploaded-JPEG hashes,
settings, scores and class maps are retained.

Causal yellow tracking proceeds across the entire stream, without resetting at
arbitrary review interval boundaries. It maintains visible correspondences
through detector misses; it removes unmatched, occluded and out-of-view pixels.
Independent reverse tracking uses the complete capture sequence, followed by
the same forward/reverse union as the validated offline experiment. Production
ROS still uses causal tracking. These masks are correlated observations, not
independent posterior probabilities.

Map generation uses recorded map->base TF and nearest /scan3 within150ms,
strict laser visibility margin0.10m and obstacle padding0.25m. Ground sampling
keeps native XYZ, RGB and quality weights identical for raw versus tracked
semantics. Here the baseline means the **same yellow prompt without tracking**;
it does not mean the former ambiguous tactile prompt. The0.05m grid and existing
RGB texture are preserved. New RGB fills only formerly unobserved cells.
Semantic classifications may still be wrong; full-recording inference is not
full-image ground-truth validation or Nav2 cost validation.

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
python3 bash/startup_bash/map_theta_sam3_full_bag.py DENSE_STUDY FULL_STUDY
python3 bash/startup_bash/finalize_theta_sam3_full_tracks.py FULL_STUDY
ROS_DOMAIN_ID=73 python3 bash/startup_bash/map_theta_sam3_tactile.py \
  FULL_STUDY ORIGINAL_TEXTURE FULL_MAP --semantic-dir yellow --allow-reclassification
python3 bash/startup_bash/render_theta_semantic_map.py FULL_MAP
```

The full-BAG runner resumes inference caches, validates source image/upload
hashes, and reconstructs tracking in sequence. Source BAG and perspective JPEG
uploads remain local; the archive keeps inference/track PNGs and metadata plus
hashes rather than duplicating the original video into Git. Reconstruct uploads
from the recorded camera data, calibration and OpenCV JPEG quality90, then match
the recorded upload SHA256. The original bytes also remain in FULL_STUDY.

Partial maps and previous sparse experiments are preserved in separate folders.
The full map gallery opens on its **whole-route overview**, with semantic-only,
overlay and texture-only images at identical bounds. Local crops retain the
original ten review locations. Final counts, skips, validation and hashes are
recorded alongside the generated map and this experiment.

## Run adjustments recorded for reproduction

The first run was deliberately interrupted after about1295 frames to skip
optical-flow computation when a previous tactile track is empty. All5182 saved
causal label/score PNGs match after resuming, proving prefix output parity.
The initial progress/log/backend restoration and hashes are retained in
`runs/initial/`; no persisted SAM3 inference was discarded.

A review also caught a score-union issue inherited from the small experiment:
when reverse tracking replaces a roadway label, its tactile score must be used,
not the maximum of that score and the unrelated roadway score. The corrected
merge takes a maximum only when both directions say tactile. The small-study
archives remain immutable at their recorded revisions. The full run finalizer
checks every union, corrects the in-flight legacy scores before mapping, keeps
prior affected score PNGs, and verifies all tactile masks remain identical.
Unit coverage includes high-roadway-score replacement and protected grass.
The finalized masks/scores are the inputs used to generate the full map.

## Completed results

All5956 camera frames /11912 views completed inference and both tracking
directions. All5956 frames contributed to mapping; no TF/scan/visibility frame
was skipped. The map has1292913 classified0.05m cells, including76349 grass,
41801 yellow tactile,303410 roadway and871353 sidewalk cells. These are map
coverage counts, not accuracy measurements.33879 previously empty texture cells
were filled from the same visibility-checked observations; all preexisting
observed RGBA cells remain byte-identical. Native XYZ/RGB/weight equality was
checked on every mapped frame.83 tests and the package build pass.

The first full run's score correction affected2211 score images /1201026 pixels;
all tactile masks remained unchanged. The archival snapshot was refreshed after
finalization and all142944 inference/track PNG+JSON files pass ZIP CRC and SHA256
checks against finalized data. Prior affected scores and the prefix optimization
parity evidence are retained under runs/. No ROS replay, live map-viewer restart
or persistent backend registry change was needed.

Whole-route phone gallery:
http://100.97.20.32:54369/full_map/images.html

Map selector prefix:
`theta_semantic_yellow_full_map_20261019/road_temporal`

To restore saved predictions, extract raw/full_seg*.zip into FULL_STUDY. Copy
the archived manifest/calibration/classes files to FULL_STUDY. Source BAG camera
images and exact perspective upload JPEGs remain local with recorded hashes;
restoring the original camera bytes/calibration is necessary for recomputing
optical flow. Mapping itself reads images/TF/scans directly from the original BAG
and the saved semantic predictions. The full map and three-view exports are
tracked separately under maps_waypoints/maps/theta_semantic_yellow_full_map_20261019/.
