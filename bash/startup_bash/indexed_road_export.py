"""Export and review the actual PGM/JSON representation, never a PNG sidecar display."""
import argparse
import hashlib
from html import escape
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation'))
from sirius_navigation.indexed_road_map import class_labels, encode_road_map, indexed_rgb


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def render_indexed_review(output, grid, metadata, prefix):
    output = Path(output)
    rgb, mask = indexed_rgb(grid, metadata)
    rgba = np.dstack([rgb, mask.astype(np.uint8)*255])
    Image.fromarray(rgba).save(output/'indexed.png')
    # Preview is a lossless decoding of the deliverable, not the source RGB.
    if not np.array_equal(np.asarray(Image.open(output/'indexed.png')), rgba):
        raise AssertionError('Indexed preview differs from PGM/JSON decoding')
    report = metadata.get('indexed_texture', {})
    html = '''<!doctype html><html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PGM＋JSONから復元した路面地図</title><style>body{font:15px system-ui;margin:0;background:#f4f6f5;color:#203038;line-height:1.8}main{max-width:1300px;margin:auto;padding:20px}h1{font-size:23px;line-height:1.5}a{color:#14694f}nav{display:flex;gap:16px;flex-wrap:wrap;margin:15px 0}img{display:block;width:100%;height:auto;image-rendering:pixelated;background:#fafafa;border:1px solid #d9e1db}.note{background:white;padding:14px;border:1px solid #d9e1db;border-radius:8px}small{color:#5a6a70}</style></head><body><main><h1>PGM＋JSONから復元した全体地図</h1><p class="note">認識した芝生・点字・車道・歩道は分類色。それ以外の観測済み路面は5段階のRGBをPGMのインデックスへ割り当てています。表示する色はJSONのパレットから復元します。PNGテクスチャの直接表示や半透明の色混合は行っていません。</p>'''
    html += f'<p>{grid.shape[1]:,} × {grid.shape[0]:,} セル / 解像度 {metadata["resolution"]*100:g} cm'
    if report:
        html += f' / 分類 {report["semantic_cells"]:,} セル / 減色RGB {report["texture_index_cells"]:,} セル'
    html += '</p><nav>'
    for path, title in [('indexed.png', '復元画像を拡大'), (prefix+'.colored.pgm', 'PGM本体'),
                        (prefix+'.colored.json', 'JSONパレット・分類'), ('verification.json', '検証記録')]:
        if (output/path).exists():
            html += f'<a href="{escape(path)}">{title}</a>'
    html += '</nav><img src="indexed.png" alt="PGMの各インデックスをJSONの色で復元した全体地図"><p><small>画像はPGM＋JSONの確認用に出力したPNGです。SAM3MapLoaderの入力には使いません。地図の各セルの色を確認できますが、RVizの視点や点の大きさは再現していません。</small></p></main></body></html>'
    (output/'images.html').write_text(html)


def export_existing(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    prefix = 'map' if (source/'map.colored.pgm').exists() else 'road_temporal'
    inputs = [source/(prefix+suffix) for suffix in ['.colored.pgm', '.colored.json', '.texture.png']]
    before = {p.name: sha(p) for p in inputs}
    old = cv2.imread(str(inputs[0]), cv2.IMREAD_UNCHANGED)
    texture = cv2.imread(str(inputs[2]), cv2.IMREAD_UNCHANGED)
    metadata = json.loads(inputs[1].read_text())
    if old is None or old.shape != (metadata['height'], metadata['width']):
        raise ValueError('PGM dimensions do not match JSON geometry')
    labels = class_labels(old)
    grid, palette, report = encode_road_map(labels, texture)
    # Quantized RGB IDs have no class cost. Class labels, world geometry and
    # semantic costs remain exactly those of the source map.
    metadata.update(palette=palette.tolist(), indexed_texture=report,
                    semantic_encoding='class_id')
    if not np.array_equal(class_labels(grid), labels):
        raise AssertionError('Reserved semantic IDs changed during export')
    if any(int(k) >= 7 for k in metadata.get('labels', {})):
        raise ValueError('Source assigns semantic meanings to appearance indices; explicit conversion required')
    output.mkdir(parents=True)
    if not cv2.imwrite(str(output/(prefix+'.colored.pgm')), grid):
        raise OSError('PGM save failed')
    (output/(prefix+'.colored.json')).write_text(json.dumps(metadata, indent=2))
    saved = cv2.imread(str(output/(prefix+'.colored.pgm')), cv2.IMREAD_UNCHANGED)
    if not np.array_equal(saved, grid):
        raise AssertionError('PGM round trip changed indices')
    source_meta = json.loads((source/'manifest.json').read_text())
    manifest = dict(status='complete', operation='PGM indexed-color export only',
                    derived_from=str(source), bag=source_meta.get('bag'),
                    frame_count=source_meta.get('frame_count'), full_bag=source_meta.get('full_bag'),
                    grid={k:metadata[k] for k in ['resolution','origin','width','height']},
                    indexed_texture=report, source_sha256=before,
                    exporter_sha256=sha(__file__),
                    codec_sha256=sha(Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation/sirius_navigation/indexed_road_map.py'))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    verification = dict(semantic_ids_unchanged=True, semantic_costs_unchanged=True,
                        world_grid_unchanged=True, pgm_roundtrip_exact=True,
                        source_sha256=before, output_pgm_sha256=sha(output/(prefix+'.colored.pgm')),
                        runtime_inputs=[prefix+'.colored.pgm', prefix+'.colored.json'],
                        texture_sidecar_in_output=False, **report)
    (output/'verification.json').write_text(json.dumps(verification, indent=2))
    render_indexed_review(output, saved, metadata, prefix)
    assert before == {p.name: sha(p) for p in inputs}
    (output/'completed.json').write_text(json.dumps(dict(status='complete', operation=manifest['operation'])))
    return verification


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    print(json.dumps(export_existing(args.source, args.output), indent=2))
