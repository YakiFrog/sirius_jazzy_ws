"""Check a full road map and render comparisons in identical world coordinates.

This does not evaluate semantic accuracy or alter either map. Native observation
counts and recorded poses are compared independently of raster grid alignment.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from render_theta_semantic_map import COLORS, frame


def crop_world(image, grid, center, span_m=10., width=400, nearest=False):
    res = span_m/width
    x = center[0] - span_m/2 + (np.arange(width)+.5)*res
    y = center[1] + span_m/2 - (np.arange(width)+.5)*res
    u = (x-grid['origin'][0])/grid['resolution']-.5
    v = grid['height']-(y-grid['origin'][1])/grid['resolution']-.5
    u, v = np.meshgrid(u.astype(np.float32), v.astype(np.float32))
    return cv2.remap(image, u, v, cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT)


def visible_rgb(bgra):
    rgb = bgra[..., :3].copy()
    rgb[bgra[..., 3] == 0] = 248
    return rgb


def overlay(bgra, labels):
    rgb = visible_rgb(bgra)
    for cid, color in COLORS.items():
        mask = (labels == cid) & (bgra[..., 3] > 0)
        rgb[mask] = np.rint(rgb[mask]*.4 + np.asarray(color[::-1])*.6).astype(np.uint8)
    return rgb


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('new', type=Path); p.add_argument('reference', type=Path)
    p.add_argument('--centers', help='Optional review frame IDs only; default uniform route samples')
    a = p.parse_args()
    meta = json.loads((a.new/'manifest.json').read_text())
    oldmeta = json.loads((a.reference/'manifest.json').read_text())
    labels = cv2.imread(str(a.new/'road_temporal.colored.pgm'), 0)
    baseline = cv2.imread(str(a.new/'baseline.class_ids.png'), 0)
    texture = cv2.imread(str(a.new/'road_temporal.texture.png'), -1)
    oldlabels = cv2.imread(str(a.reference/'road_temporal.colored.pgm'), 0)
    oldtexture = cv2.imread(str(a.reference/'road_temporal.texture.png'), -1)
    assert labels.shape == texture.shape[:2] == baseline.shape
    assert labels.shape == (meta['grid']['height'], meta['grid']['width'])
    assert not ((labels > 0) & (texture[..., 3] == 0)).any()
    protected = ~np.isin(baseline, (5, 6))
    assert np.array_equal(labels[protected], baseline[protected])
    oldframes = {r['frame']: r for r in oldmeta['frames']}
    disagreements = []
    for row in meta['frames']:
        old = oldframes.get(row['frame'])
        if old is None or any(row[k] != old[k] for k in ('xyz', 'quaternion', 'ground_points')):
            disagreements.append(row['frame'])
    assert not disagreements, disagreements[:20]
    assert not meta['skipped'] and meta['frame_count'] == meta['input_camera_frames']
    with np.load(a.new/'map_evidence.npz') as evidence:
        coverage = evidence['sample_coverage'][::-1]
        assert np.array_equal(coverage > 0, texture[..., 3] > 0)
        assert not ((labels > 0) & (coverage == 0)).any()
    report = dict(full_bag=meta['full_bag'], camera_frames=meta['input_camera_frames'],
        mapped_frames=meta['frame_count'], skipped=meta['skipped'],
        native_point_counts_and_poses_equal_reference=True, geometry_comparison_frames=len(meta['frames']),
        protected_semantic_classes_preserved=True, all_semantics_inside_observed_rgb=True,
        rgb_support_equals_native_coverage=True, baseline_cache_frames=meta.get('reused_baseline_frames', 0),
        grid=meta['grid'], semantic_cells=meta['semantic_cells'], refinements=meta['refinements'],
        caveat='Agreement with the reference generation, not independent geometry or semantic ground truth.')
    if a.centers:
        indices = [int(i) for i in a.centers.split(',')]
    else:
        indices = [meta['frames'][i]['frame'] for i in np.linspace(0, len(meta['frames'])-1, 8, dtype=int)[1:-1]]
    rows = {r['frame']: r for r in meta['frames']}
    output = a.new/'comparison'; output.mkdir(exist_ok=True)
    gallery = '<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>全体地図の比較</title><style>body{font:16px system-ui;background:#13212d;color:white;margin:12px}img{width:100%;max-width:1000px;height:auto}a{color:#a8dfff}</style><h1>共通処理で生成した全体地図</h1><p>全5,956フレームの処理記録は生成記録を参照。比較画像は同じ世界座標・範囲・縮尺にそろえています。</p><a href="../images.html">全体図3種類</a>'
    gallery = gallery.replace('全5,956フレーム', f'全{meta["input_camera_frames"]:,}フレーム')
    reviews = []
    for index in indices:
        center = rows[index]['xyz'][:2]
        current = crop_world(texture, meta['grid'], center)
        prior = crop_world(oldtexture, oldmeta['grid'], center)
        newclass = crop_world(labels, meta['grid'], center, nearest=True)
        oldclass = crop_world(oldlabels, oldmeta['grid'], center, nearest=True)
        panes = []
        for title, data in [('前の地図：テクスチャ', visible_rgb(prior)), ('今回：テクスチャ', visible_rgb(current)),
                            ('前の地図：重ね合わせ', overlay(prior, oldclass)), ('今回：重ね合わせ', overlay(current, newclass))]:
            panes.append(frame(data, title, f'フレーム{index}付近／同じ10m四方', .025, width=1000))
        contact = Image.new('RGB', (1000, sum(im.height for im in panes)), 'white')
        top = 0
        for im in panes:
            contact.paste(im, (0, top)); top += im.height
        name = f'frame{index}.jpg'; contact.save(output/name, quality=93, subsampling=0)
        gallery += f'<h2>フレーム{index}付近</h2><a href="{name}"><img src="{name}" loading="lazy" alt="前後比較"></a>'
        reviews.append(dict(frame=index, world_center_m=center, span_m=10., path=name))
    (output/'images.html').write_text(gallery+'</html>')
    report['review_views'] = reviews
    (a.new/'verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('review_views', 'refinements')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
