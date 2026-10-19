# SAM3 projection study — 2026-10-19

10 local intervals from `real_both_mapping_20260920_113447_recovered`, centers 300,900,1500,2100,2700,3300,3900,4500,5100,5650 (zero-based camera message index); offsets -16,-8,0,8,16. 50 unique original camera JPEGs, about eight seconds per interval. Samples span the recording systematically; they were not selected for successful detections.

## Findings and limits

The deployed node uses calibrated front/rear perspective views (640×480, HFOV120°), SAM3 internal square resolution512 and downstream mask score cutoff0.8. The experiment compared these against resolution1008, four perspective directions, full 2048×1024 equirectangular images at512/1008, and four overlapping 180°×90° ground-hemisphere equirectangular tiles at1008. Shared prompts: grass; three tactile aliases; roadway; sidewalk. Per-prompt server threshold0.5, derived client cutoffs0.8/0.65/0.5. Target-only methods use only grass and tactile aliases on the center frames (not all50).

Provisional ROI checks (six tactile polygons, one small grass polygon, ten plain-pavement negative polygons, expert-unverified, not complete ground truth): at cutoff0.8 tactile coverage is4.2% for baseline512,18.2% for perspective1008,0.0% for full equirectangular1008,62.9% for perspective1008 with the four-class prompt `grass, yellow tactile paving, roadway, sidewalk`. The candidate uses JPEG-identical inputs to perspective1008. Candidate improvement is uneven and some strips remain incomplete; yellow-specific text does not establish performance on other tactile colors. Full equirectangular1008 reaches26.5% at cutoff0.5, but a projection change alone is insufficient.

At cutoff0.8 the single grass ROI is0% baseline versus88.3% for perspective1008. This is only one small plot, not a general lawn evaluation. Many sample images contain bushes/trees rather than grass. Plain pavement negative ROIs contain no target labels for candidate1008, but these small negative regions do not establish whole-image precision. Lowering baseline cutoff to0.5 produces4.6% tactile false coloring in these negatives. Coverage counts are not accuracy, mIoU, recall, or a validated benchmark.

Single-prompt diagnostics show text dependence; removing roadway/sidewalk can expose tactile labels lost in highest-score class competition. This diagnostic is not a production recipe. `yellow tactile paving` single-prompt requests use re-encoded saved reference JPEGs (extra compression); only the final multiclass candidate has exact primary JPEG parity.

No production node/config/class registry or map was changed. Runtime SAM3 mode, prompt, resolution and thresholds were restored after each experiment; see each restoration.json. Existing map viewers were left running.

## Method safeguards

The server interprets aspect ratios>1.8 as stereo SBS and would crop the right half of a normal full panorama. Wide mono images were transported as [full image | identical full image]. The server's left image is the intended complete panorama, with no padding; returned class/score shape was asserted. Each exact upload JPEG, per-prompt mask scores, response version and settings were saved. Server highest-mask-score wins across prompts; score PNG is quantized mask confidence, not calibrated per-pixel probability. Every model input is internally resized to a square. BEV image comparisons use512² cells over10m, radius1.2–4.8m; no LiDAR visibility mask or global map fusion was used. Common-view comparisons reproject all methods to identical front/left/rear/right images, and aggregate valid overlap by summed class score. Four-direction methods can change competition in overlapping views. Class-colored area outside a shared FOV is not directly comparable.

Primary study760 inferences; five single-prompt probes100; final multiclass candidate20 (plus pilot, kept separately). Timing JSON includes HTTP, serialization and GPU contention, not isolated model latency; the experiment permits40s timeouts and does not validate the production2s timeout.

## Review and data layout

Open comparison.html in a browser. Choose interval, viewing direction, methods and thresholds independently. Inputs and dense inference artifacts are in raw ZIP archives; extract all ZIPs into this directory to reconstruct their exact relative paths, including panorama links. Reference/review images and metadata are directly available without extraction. SHA256SUMS covers all files in this archive except itself; raw_file_sha256.json additionally checks every original study file before packaging. Original BAG stays local (4.7GB); hashes are in provenance.json. No raw inference outputs were discarded.

## Reproduction

Use the ROS Jazzy workspace and the same calibrated dual-fisheye inputs; SAM3 server localhost8080 must be reserved for this sequential test (other publishers cause an explicit failure). Exact model/server hashes and source commits are in provenance.json. No external model package versions are automatically installed. Source entry points are in bash/startup_bash/:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
python3 bash/startup_bash/compare_theta_sam3_projection.py /PATH/TO/BAG --output /NEW/STUDY --calibration /THIS/ARCHIVE/calibration_used.yaml --classes /THIS/ARCHIVE/classes_used.yaml
```

Create reference views and reuse the archived provisional ROI annotations before the prompt probes and reporter with `python3 bash/startup_bash/prepare_theta_sam3_reference.py STUDY --rois /THIS/ARCHIVE/review_rois.json`. The ROI file was manually defined for these specific frames; it is not reusable ground truth for another BAG. Then run probe_theta_sam3_prompts.py STUDY, probe_theta_sam3_candidate.py STUDY, report_theta_sam3_projection.py STUDY, build_theta_sam3_report.py STUDY. Evaluation scripts use the archived review_rois.json; changing ROI selection changes the numbers. Source-level unit tests cover spherical reprojection, wrap, common-view identity, full-panorama transport, stale responses and output shapes. Completion logs and verification results are archived.
