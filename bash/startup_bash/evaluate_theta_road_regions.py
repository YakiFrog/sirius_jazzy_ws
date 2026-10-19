"""Archive spatial-prior trials and provisional source-texture interior checks.

These exploratory ROIs are not an independently annotated test set. Their
coordinates are only for evaluation, never for generating the refined labels.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from render_theta_semantic_map import COLORS, frame

# Rectangles normalized to the original review viewports, drawn on RGB interiors.
INTERIORS = [
    (300, 6, (.68, .42, .85, .75)),
    (900, 6, (.32, .27, .50, .44)),
    (900, 5, (.48, .62, .62, .90)),
    (1500, 6, (.25, .25, .75, .60)),
    (2100, 6, (.30, .45, .70, .75)),
    (2700, 6, (.50, .44, .60, .70)),
    (3300, 6, (.59, .10, .70, .40)),
    (3300, 5, (.12, .48, .30, .80)),
    (3900, 6, (.65, .18, .85, .62)),
    (3900, 5, (.18, .30, .32, .48)),
    (4500, 6, (.28, .48, .50, .66)),
    (4500, 5, (.73, .28, .90, .40)),
    (5100, 6, (.50, .45, .70, .80)),
    (5650, 6, (.55, .47, .72, .65)),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    parser.add_argument('study', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--select', default='conservative/lambda0_8')
    args = parser.parse_args()
    base, study, out = args.reference, args.study, args.output
    before = cv2.imread(str(base / 'road_temporal.colored.pgm'), 0)
    texture = cv2.imread(str(base / 'road_temporal.texture.png'), -1)
    render = json.loads((base / 'rendering.json').read_text())
    rois = []
    for i, (center, cid, rectangle) in enumerate(INTERIORS):
        x0, y0, x1, y1 = render['views'][f'seg{center}']['pixel_bbox']
        left, top, right, bottom = rectangle
        bbox = [x0 + round(left * (x1-x0)), y0 + round(top * (y1-y0)),
                x0 + round(right * (x1-x0)), y0 + round(bottom * (y1-y0))]
        rois.append(dict(name=f'{center}_{cid}_{i}', center=center, expected_class=cid,
                         pixel_bbox=bbox, normalized_bbox=rectangle))
    (study / 'review_rois.json').write_text(json.dumps({
        'status': 'exploratory source-texture interior ROIs; no independent ground truth',
        'excluded_classes': [0, 1, 2, 3, 4], 'rois': rois}, indent=2))
    trials = {'baseline': base}
    trials.update({str(p.parent.relative_to(study)): p.parent for p in sorted(study.glob('**/road_temporal.colored.pgm'))})
    reports = {}
    for name, path in trials.items():
        labels = cv2.imread(str(path / 'road_temporal.colored.pgm'), 0)
        rows = []
        for roi in rois:
            x0, y0, x1, y1 = roi['pixel_bbox']
            fixed = before[y0:y1, x0:x1]
            valid = np.isin(fixed, (5, 6)) & (texture[y0:y1, x0:x1, 3] > 0)
            correct = (labels[y0:y1, x0:x1] == roi['expected_class']) & valid
            rows.append(dict(**roi, evaluated_cells=int(valid.sum()), correct_cells=int(correct.sum())))
        grouped = {}
        for cid in (5, 6):
            selected = [r for r in rows if r['expected_class'] == cid]
            total = sum(r['evaluated_cells'] for r in selected)
            correct = sum(r['correct_cells'] for r in selected)
            grouped[str(cid)] = dict(cells=total, correct=correct, fraction=correct/max(total, 1))
        reports[name] = dict(by_class=grouped, rois=rows)
    (study / 'roi_evaluation.json').write_text(json.dumps(reports, indent=2))
    selected = trials[args.select]
    out.mkdir(parents=True, exist_ok=True)
    for name in ('road_temporal.colored.pgm', 'road_temporal.colored.json', 'road_temporal.texture.png',
                 'manifest.json', 'calibration_used.yaml', 'semantic_coverage.png'):
        if (selected / name).exists():
            shutil.copy2(selected / name, out / name)
    cv2.imwrite(str(out / 'baseline.class_ids.png'), before)
    # Keep observational votes as evidence, without pretending they were new
    # network predictions produced by this postprocessing operation.
    shutil.copy2(base / 'map_evidence.npz', out / 'map_evidence.npz')
    meta = json.loads((out / 'manifest.json').read_text())
    meta['road_region_consistency'].update(
        selected_trial=args.select, source_votes_unchanged=True,
        vote_note='map_evidence contains original observations; output labels include a spatial prior',
        exploratory_roi_evaluation=reports[args.select]['by_class'])
    (out / 'manifest.json').write_text(json.dumps(meta, indent=2))
    labels = cv2.imread(str(out / 'road_temporal.colored.pgm'), 0)
    rgb = texture[..., :3].copy()
    rgb[texture[..., 3] == 0] = 248
    for center in sorted({r['center'] for r in rois}):
        x0, y0, x1, y1 = render['views'][f'seg{center}']['pixel_bbox']
        panes = []
        for title, lab in [('補正前', before), ('境界・連続性で補正後', labels), ('元の路面テクスチャ', None)]:
            crop = rgb[y0:y1, x0:x1].copy()
            if lab is not None:
                for cid, color in COLORS.items():
                    mask = lab[y0:y1, x0:x1] == cid
                    crop[mask] = np.rint(crop[mask] * .4 + np.asarray(color[::-1]) * .6).astype(np.uint8)
            panes.append(frame(crop, title, f'区間{center} ｜ 同じ範囲・縮尺', .05, width=900, show_legend=lab is not None))
        # Stack on phones; do not shrink three side-by-side panels to 130px.
        contact = Image.new('RGB', (900, sum(p.height for p in panes)), 'white')
        yy = 0
        for pane in panes:
            contact.paste(pane, (0, yy))
            yy += pane.height
        contact.save(study / f'before_after_{center}.jpg', quality=94, subsampling=0)
        source = rgb[y0:y1, x0:x1].copy()
        for roi in (r for r in rois if r['center'] == center):
            left, top, right, bottom = roi['pixel_bbox']
            cv2.rectangle(source, (left-x0, top-y0), (right-x0-1, bottom-y0-1),
                          (255, 70, 40) if roi['expected_class'] == 5 else (30, 190, 230), 1)
        frame(source, f'評価用の内部領域：{center}', '青枠：車道／黄枠：歩道（暫定・境界を除外）', .05, width=900,
              show_legend=False).save(study / f'rois_{center}.jpg', quality=94)
    verification = dict(
        selected_trial=args.select, total_camera_frames=meta['frame_count'],
        all_rgb_bytes_identical=(base / 'road_temporal.texture.png').read_bytes() == (out / 'road_temporal.texture.png').read_bytes(),
        protected_classes_unchanged=bool(np.array_equal(labels[~np.isin(before, (5, 6))], before[~np.isin(before, (5, 6))])),
        input_and_output_grid_equal=meta['grid'] == json.loads((base / 'manifest.json').read_text())['grid'],
        semantic_outside_texture_cells=int(((labels > 0) & (texture[..., 3] == 0)).sum()),
        roi_status='Exploratory same-recording interiors; not independent accuracy assessment.',
        roi_evaluation=reports[args.select]['by_class'],
        changed_cells=int((labels != before).sum()))
    assert verification['all_rgb_bytes_identical'] and verification['protected_classes_unchanged']
    assert verification['semantic_outside_texture_cells'] == 0
    (out / 'verification.json').write_text(json.dumps(verification, indent=2))
    (study / 'verification.json').write_text(json.dumps(verification, indent=2))
    print(json.dumps({name: r['by_class'] for name, r in reports.items()}, indent=2))


if __name__ == '__main__':
    main()
