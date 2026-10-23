"""Export a brighter existing map without inference, reprojection or label edits.

Always start from raw fused RGB when available, avoiding cumulative brightening.
The output is a separate derived artifact, not a resumable SLAM/inference run.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

from generate_theta_road_map import render_review
from theta_road_pipeline import load_config
from theta_texture_tone import lift_texture, tone_report
from indexed_road_export import render_indexed_review
from sirius_navigation.indexed_road_map import class_labels, encode_road_map


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def brighten(source, output, tone):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    prefix = 'map' if (source/'map.colored.pgm').is_file() else 'road_temporal'
    raw = source/f'{prefix}.raw.texture.png'
    if not raw.exists():
        raw = source/f'{prefix}.texture.png'
    original = cv2.imread(str(raw), cv2.IMREAD_UNCHANGED)
    indexed_source = cv2.imread(str(source/f'{prefix}.colored.pgm'), 0)
    labels = class_labels(indexed_source)
    metadata = json.loads((source/f'{prefix}.colored.json').read_text())
    meta = json.loads((source/'manifest.json').read_text())
    if original is None or labels is None or original.shape[:2] != labels.shape:
        raise ValueError('Missing or inconsistent map images')
    toned = lift_texture(original, tone)
    report = tone_report(original, toned, tone)
    assert report['alpha_unchanged'] and report['unobserved_rgb_unchanged']
    assert report['highlights_unchanged'] and report['new_clipped_cells'] == 0
    output.mkdir(parents=True)
    exclude = {'manifest.json', 'config_used.yaml', f'{prefix}.texture.png',
        f'{prefix}.raw.texture.png', 'run_identity.json', 'parent_run_identity.json',
        'postprocess_identity.json', 'completed.json', 'verification.json',
        'http_verification.json', 'images.html', 'SHA256SUMS'}
    # Preserve native evidence/labels/metadata exactly; never copy old review claims.
    for path in source.iterdir():
        if path.is_file() and path.name not in exclude and path.suffix != '.jpg':
            shutil.copy2(path, output/path.name)
    shutil.copy2(raw, output/f'{prefix}.raw.texture.png')
    if not cv2.imwrite(str(output/f'{prefix}.texture.png'), toned):
        raise OSError('Failed to write texture')
    parent_identity = source/'run_identity.json'
    if not parent_identity.exists():
        parent_identity = source/'parent_run_identity.json'
    if parent_identity.exists():
        shutil.copy2(parent_identity, output/'parent_run_identity.json')
    identity = dict(operation='texture tone export only', parent_map=str(source),
        source_sha256={p.name: sha(p) for p in (raw, source/'manifest.json',
            source/f'{prefix}.colored.pgm', source/f'{prefix}.colored.json',
            source/'map_evidence.npz')},
        code_sha256={p.name: sha(p) for p in (Path(__file__),
            Path(__file__).with_name('theta_texture_tone.py'),
            Path(__file__).with_name('indexed_road_export.py'),
            Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation/sirius_navigation/indexed_road_map.py',
            Path(__file__).with_name('generate_theta_road_map.py'))}, tone=tone)
    config = yaml.safe_load((source/'config_used.yaml').read_text())
    config['texture']['tone'] = dict(tone)
    (output/'config_used.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    meta.update(config=config, texture_tone=report, derived_from=str(source),
        postprocess='texture tone only; no semantic inference or geometry changes')
    indexed, palette, indexed_report = encode_road_map(labels, toned)
    metadata.update(palette=palette.tolist(), indexed_texture=indexed_report)
    if not cv2.imwrite(str(output/f'{prefix}.colored.pgm'), indexed):
        raise OSError('Failed to write indexed PGM')
    (output/f'{prefix}.colored.json').write_text(json.dumps(metadata, indent=2))
    meta['indexed_texture'] = indexed_report
    # This hash describes the parent mapping run; do not present it as this export.
    if 'run_identity_sha256' in meta:
        meta['parent_run_identity_sha256'] = meta.pop('run_identity_sha256')
    render_review(output, labels, toned, meta['grid']['resolution'])
    render_indexed_review(output, indexed, metadata, prefix)
    (output/'manifest.json').write_text(json.dumps(meta, indent=2))
    (output/'postprocess_identity.json').write_text(json.dumps(identity, indent=2))
    preserved = ('map_evidence.npz',)
    assert all(sha(source/name) == sha(output/name) for name in preserved)
    assert np.array_equal(class_labels(indexed), labels)
    assert metadata.get('labels') == json.loads((source/f'{prefix}.colored.json').read_text()).get('labels')
    (output/'completed.json').write_text(json.dumps(dict(status='complete',
        operation=identity['operation'], frame_count=meta['frame_count'],
        semantic_ids_unchanged=True, semantic_costs_unchanged=True,
        vote_evidence_byte_identical=True, indexed_texture=indexed_report, texture_tone=report), indent=2))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    cfg = load_config(args.config) if args.config else load_config()
    print(json.dumps(brighten(args.source, args.output, cfg['texture']['tone']), indent=2))


if __name__ == '__main__':
    main()
