# Tactile-mask gap restoration — 2026-10-19

This extends the ten-interval temporal experiment (root commit b27b0f8). It reuses all170 recorded camera images/340 calibrated front/rear SAM3 predictions, with resolution512 and cutoff0.5. No new model inference, ROS replay or geometry modification was performed. The previous road/sidewalk trial intentionally left tactile IDs unchanged, explaining why its comparison still flickered on tactile paving.

## Logic and scope

Use original neighboring detections within0.85s (up to3 neighboring captured frames each way). DIS optical flow, forward/backward consistency, photometric and local-texture tests are inherited from the temporal study. Require reliability>=0.4 and mask score>=0.5, erode warped valid masks by1pixel, then require2 matching original observations and weighted mean confidence>=0.55. Offline bidirectional fill requires one past AND one future observation; causal fill requires2 past observations. Only missing tactile labels at current IDs0/4/5/6 can be filled. Existing tactile pixels remain unchanged; wall/floor/grass IDs1/2/3 remain unchanged. Store original masks, never restored predictions, in history, so filling cannot perpetuate itself indefinitely. Replay/time regression, large gaps and changed projection reset history.

Repeated raw false detections on ordinary gray paving also propagate in an unrestricted trial. Therefore the final new-fill pixels require a yellow appearance gate: OpenCV HSV H15..32 (30..64 degrees), S>=100/255, V>=35/255, with no closing across non-yellow pixels. New-fill connected components smaller than16pixels are excluded. This is a conservative appearance condition for this recording's yellow targets, **not a universal tactile classifier**. Pale/beige/white tactile surfaces may receive no new fill; their original SAM3 detections are retained. Color gating cannot fix an originally incorrect gray-pavement detection. Long gaps, uncertainty from occlusion, or surfaces never detected in the input remain unresolved. No semantic correctness is guaranteed just by temporal stability.

Three exploratory variants are preserved: unrestricted; yellow_loose (H15..40/S>=45/V>=35, closing3×3, no component filtering); final yellow (above). The initial loose yellow trial still added2102 wrong pixels to ordinary pavement at frame2103, motivating stricter saturation and component guards. The final trial adds zero pixels to frames2103 and2104 front, while retaining the original false detections there. These guard choices were revised after inspecting examples; this is not a held-out benchmark or preregistered study.

## Results and limitations

At front frame3297 and3299, the baseline has zero visible tactile pixels despite an RGB-visible yellow strip. Final offline bidirectional processing restores2381 and4371pixels respectively; causal processing restores929 and881. Visual review confirms additions on the strip but some holes/boundaries still remain. Pale tactile paving at rear4494 was not restored by the final strict gate, illustrating the color limitation.

Across320 flow-aligned adjacent-view pairs, tactile temporal IoU over all usable correspondences is52.804% baseline,53.525% causal,55.246% final bidirectional. Restricted to the same current-image yellow-appearance domain, it is57.078%,62.435%,75.158%. These metrics measure consistency, **not segmentation accuracy**; the union depends on predictions and greater coverage alone may affect IoU. The unrestricted diagnostic reaches61.884% whole-domain IoU while propagating false detections, illustrating this limitation. Inspect the exact counts/denominators in tactile/evaluation.json.

Reusing the previous provisional6 positive/10 plain-pavement negative ROIs on central frames: tactile coverage15.093% baseline,23.893% causal,15.093% final bidirectional; negative false coloring remains4.581% for all these methods. These are small, expert-unverified interior polygons, not whole-image ground truth. No general semantic accuracy gain, universal color handling, or full-map accuracy improvement is claimed.

## Review

The phone page is served on the existing Tailscale-only review server:
http://100.97.20.32:54369/temporal/tactile/comparison.html

Initial view: segment3300, front, offset-3 (=frame3297), tactile masks. Use the frame slider or playback and compare baseline with final bidirectional/causal/initial unrestricted. The photo and both masks are preloaded/decoded and swapped together; stale async requests are discarded. A bounded16-image cache avoids loading the whole recording into phone memory. Browser checks cover navigation to3299, rapid back-and-forth changes, mode changes, playback and stopping. A phone screenshot is included.

## Reconstruct and reproduce

Dense inputs and baseline inference are already preserved in experiments/theta_sam3_temporal/2026-10-19/raw/*.zip. To reconstruct the study, extract those archives to STUDY. Extract this archive's raw/*.zip to the **same STUDY root**; they restore tactile/, tactile_unrestricted/, tactile_yellow_loose/ and the baseline-derived tactile-mask PNGs. Copy each directly tracked variant directory's JSON/HTML/README into the corresponding STUDY variant folder. The report references baseline files via ../seg*/... from STUDY/tactile/comparison.html. Files before reconstruction are intentionally packaged, not individually browseable. Original BAG remains local, with hashes in prior provenance.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2
python3 bash/startup_bash/stabilize_theta_sam3_tactile.py STUDY --appearance unrestricted
python3 bash/startup_bash/stabilize_theta_sam3_tactile.py STUDY --appearance yellow_loose
python3 bash/startup_bash/stabilize_theta_sam3_tactile.py STUDY
python3 bash/startup_bash/report_theta_sam3_tactile.py STUDY
```

The reporter reuses review_rois.json from the original projection study, or from STUDY/tactile/review_rois.json when the recorded reference path does not exist. --html-only regenerates the page from existing evaluation.json and mask_counts.json without recomputing flow.

The ROS node has a separate causal option, enabled by default on the next mapping launch, with the same yellow gate and raw-history expiration:

```bash
ros2 launch sirius_navigation theta_offline_mapping.launch.py sam3:=true sam3_tactile_temporal:=true
```

It requires at least2 original matched observations in0.85s of capture time. If inference/replay drops too many frames, it deliberately leaves the current result unchanged. ROS flow uses uncompressed perspective views; the offline study uses decoded exact upload JPEGs, so bitwise live-node parity is not claimed. The tactile temporal switch defaults true; disable with sam3_tactile_temporal:=false. The road/sidewalk temporal switch remainsfalse, and the baseline cutoff remains0.5. No live map/backend/viewer was restarted. This is mask-level tracking; SAM3's learned video predictor is not activated.

72 theta tests and package build pass. Tests specifically cover short-gap recovery, required past/future support, stale/occluded evidence rejection, protecting existing detections/grass, new-fill color/component guards, and expiring raw history without recursive self-extension. SHA256SUMS covers archive files; raw_file_sha256.json checks every new source artifact. Original inference, rejected variants and the entire experiment process are preserved for later paper work.
