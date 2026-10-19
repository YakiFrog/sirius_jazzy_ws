# SAM3 temporal consistency study — 2026-10-19

User-reviewed baseline: current calibrated front/rear perspective images, SAM3 square resolution512, mask-score cutoff0.5. Same ten centers as the projection study, now17 **consecutive** messages per center (offsets -8 through8, around4.3s total),170 unique camera images and340 per-view predictions.60 predictions reused byte-for-byte from the prior study;280 newly inferred. Original BAG remains local; prior source hashes are copied in provenance.json. Exact network JPEGs and output class/score PNGs are archived.

## References and relationship to implementation

- Baghbaderani et al., WACV2024, [Temporally-Consistent Video Semantic Segmentation With Bidirectional Occlusion-Guided Feature Propagation](https://openaccess.thecvf.com/content/WACV2024/html/Baghbaderani_Temporally-Consistent_Video_Semantic_Segmentation_With_Bidirectional_Occlusion-Guided_Feature_Propagation_WACV_2024_paper.html): bidirectional flow and occlusion-aware propagation motivate checking correspondences before combining observations. This paper learns feature propagation/attention; our mask-level DIS-flow voting is **not a reproduction of BOFP**, and its published gains do not transfer automatically.
- Carion et al., [SAM3: Segment Anything with Concepts](https://arxiv.org/abs/2511.16719): image detector plus memory-based video tracking, including detection revisits and temporal disambiguation. The deployed server uses only the image processor. Our experiment reuses that image model and adds mask-level correspondence; it does **not enable SAM3's video predictor or learned memory**.

## Algorithm

Process each front/rear stream separately, maintaining original timestamps. For target frame, match up to4 preceding images (causal), or4 on either side (offline bidirectional), at most1.5s apart. OpenCV DIS MEDIUM estimates current-to-neighbor and reverse flow. Exclude points outside image, with forward/backward error above1.5+0.03×flow magnitude pixels, grayscale mismatch>30, or local7×7 variance<9. Remaining votes receive Gaussian correspondence/photometric weights and exp(-|dt|/2s) age weights. A neighbor must have score>=0.5 and reliability>=0.25. Current observation has1.2×score vote.

Only existing road5/sidewalk6 labels with score>=0.5 are eligible for change. Require at least3 distinct agreeing neighboring frame indices, weighted winner share>=0.75, and mean winning mask confidence at least0.05 greater than current confidence. Distinct frames are correlated samples, not independent statistical trials. Mask confidence and vote share are not calibrated probabilities. Grass3/tactile4/unknown0/wall1/floor2 remain exactly unchanged; no geometry is edited. Corrections are not fed back into history. Runtime history resets on replay/time regression, image shape change, projection change or capture gap>1.5s.

Initial diagnostic omitted the confidence-gain guard. At frame306 front, this changed10,789 pixels, propagating road labels into a currently more confident sidewalk region. The final guard rejects that change. Both variants and all inputs are retained. This is an exploratory iteration prompted by visual review, not a preregistered or held-out benchmark.

## Observed result — limited improvement

On all340 views, final causal processing changes142 road/sidewalk pixels; final bidirectional changes612; the initial ungated bidirectional variant changes11,579. These counts indicate edits, not correct edits. On a common flow-valid road/sidewalk domain across320 adjacent-view pairs (11,189,048 pixel correspondences), label flip fractions are1.2306% baseline,1.2284% causal,1.2210% final bidirectional,1.0240% ungated. The ungated variant illustrates why greater stability is not proof of correctness.

Provisional RGB-labelled19 interior ROIs across7 central frames:86.3079% matching requested class,0.6452% opposite road/sidewalk,13.0470% unknown/other for **all four methods**. No central-ROI accuracy improvement was demonstrated. These are small assistant-labelled polygons, not expert-verified complete ground truth. Centers300/5100/5650 have ambiguous plaza/event-space road definitions and were excluded from semantic correctness ROI evaluation, while retained in processing, flip metrics and visual comparison. Some missing sidewalk labels are unknown, not swapped; current conservative algorithm intentionally does not fill unknowns. Persistently wrong labels cannot be corrected from repeated identical wrong evidence alone.

## Usage and deployment state

Open comparison.html. The phone version is served on the existing Tailscale-only HTTP review server under `/temporal/comparison.html`; the host must remain running. In the comparison, choose interval, frame offset, front/rear and causal/bidirectional/initial method. Original inputs, baseline and outputs share exact image coordinates. `road_review_rois.json` is manually specified for these particular images and cannot serve as ground truth for another BAG.

Baseline config score_min is changed to0.5 per user review. Temporal consistency is implemented in the ROS node but remains **opt-in**, because the experiment did not demonstrate semantic correctness improvement. It can be tested with:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sirius_navigation theta_offline_mapping.launch.py sam3:=true sam3_temporal:=true
```

This launch only starts the mapping nodes; supply the BAG replay as in the existing offline workflow. ROS uses past frames only; bidirectional future-frame results are currently an offline experiment. Dense input capture timestamps matter: skipped frames, inference cost or large replay gaps can leave fewer than3 usable neighbors, causing no correction. No live map/backend/viewer was restarted, and server prompt/thresholds/resolution/mode were restored. Runtime flow uses uncompressed perspective images whereas archived comparisons use decoded exact uploaded JPEGs; numerical parity with a live node has not been claimed. No end-to-end full-map accuracy gain is established.

## Reproduce and inspect artifacts

Extract every raw/*.zip into this directory to restore dense artifacts and browser image paths. Inputs and all class/score maps are retained in those ZIPs; metadata/HTML are directly available. SHA256SUMS checks archive files; raw_file_sha256.json checks the exact unpacked originals. Model/backend hashes are inherited from the prior experiment provenance; environment versions are recorded with configuration. Reproduction assumes the same checkpoint, server source, calibration, class registry and packages; it does not install model dependencies.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
python3 bash/startup_bash/collect_theta_sam3_temporal.py /PATH/TO/PRIOR/PROJECTION/STUDY /NEW/STUDY
cp /THIS/ARCHIVE/road_review_rois.json /NEW/STUDY/
python3 bash/startup_bash/refine_theta_sam3_temporal.py /NEW/STUDY --ungated
python3 bash/startup_bash/refine_theta_sam3_temporal.py /NEW/STUDY
python3 bash/startup_bash/report_theta_sam3_temporal.py /NEW/STUDY
```

`collect` reuses prior inference only after exact network JPEG equality; otherwise infers through reserved SAM3 server localhost8080, and restores its settings. The optional ROI overlay JPEGs are review aids; their polygons are completely specified by road_review_rois.json and can be redrawn with cv2.polylines on central uploads. The previous-study backlink assumes the original HTTP folder layout and is optional after standalone extraction. Tests cover translated correspondence, occlusions, insufficient votes, class protection, time discontinuities, raw-history retention, and confidence-priority rejection.65 theta tests and the package build pass; launch arguments and phone browser rendering were checked. No comparison artifact should be presented as validated general accuracy.
