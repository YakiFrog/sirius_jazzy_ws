"""Rebuild all-frame RGB with an evaluated general observation fusion policy.

No semantic inference or geometry deformation. Keep the old texture where no
accepted native observation exists, and preserve the semantic raster verbatim.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np

from collect_theta_texture_observations import observations
from theta_texture_quality import TextureFusion,VARIANTS,feed_variant


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('reference',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--variant',choices=list(VARIANTS),default='scatter_protected');a=p.parse_args()
    if a.output.exists():raise RuntimeError('Output already exists; use a new output directory')
    a.output.mkdir(parents=True)
    source=json.loads((a.source/'manifest.json').read_text());meta=json.loads((a.reference/'manifest.json').read_text());grid=meta['grid'];w,h=grid['width'],grid['height']
    selected=VARIANTS[a.variant];baseline=VARIANTS['scatter_ema']
    if selected['source']!='native':raise ValueError('Full-run geometry-preserving release is limited to native projection variants')
    fusers={name:TextureFusion(w*h,cfg['mode'],cfg['relative'],cfg.get('tolerance',25)) for name,cfg in [('baseline',baseline),('selected',selected)]}
    rows=[]
    for packet,detail in observations(a.source,a.reference,range(source['camera_frames']),native=True,direct=False):
        feed_variant(fusers['baseline'],packet,baseline);feed_variant(fusers['selected'],packet,selected);rows.append(detail)
        if len(rows)%100==0:(a.output/'progress.json').write_text(json.dumps(dict(phase='full_rgb_fusion',frames=len(rows),total=source['camera_frames']),indent=2))
    assert len(rows)==source['camera_frames']==meta['frame_count'];assert len({r['index'] for r in rows})==len(rows)
    original=cv2.imread(str(a.reference/'road_temporal.texture.png'),-1);images={};masks={}
    for name,fuser in fusers.items():
        rgb,seen=fuser.result();rgb=rgb.reshape(h,w,3)[::-1];seen=seen.reshape(h,w)[::-1];merged=original.copy();merged[seen,:3]=rgb[seen];merged[seen,3]=255
        images[name]=merged;masks[name]=seen
    assert np.array_equal(masks['baseline'],masks['selected'])
    cv2.imwrite(str(a.output/'baseline.texture.png'),images['baseline']);cv2.imwrite(str(a.output/'road_temporal.texture.png'),images['selected'])
    cv2.imwrite(str(a.output/'fusion_support.png'),masks['selected'].astype(np.uint8)*255)
    for filename in ['road_temporal.colored.pgm','road_temporal.colored.json','calibration_used.yaml','map_evidence.npz','semantic_coverage.png']:
        shutil.copy2(a.reference/filename,a.output/filename)
    added=masks['selected']&(original[...,3]==0)
    settings=dict(variant=a.variant,parameters=selected,texture_decay=.7,camera_frames=len(rows),skipped_frames=0,
        reference_map=str(a.reference.resolve()),reference_texture_sha256=hashlib.sha256((a.reference/'road_temporal.texture.png').read_bytes()).hexdigest(),
        all_native_point_counts_and_recorded_poses_checked=True,observed_support_unchanged_between_fusion_conditions=True,
        new_observed_rgb_cells=int(added.sum()),old_rgb_fallback_cells=int(((original[...,3]>0)&~masks['selected']).sum()),
        semantic_labels_unchanged=True,world_grid_unchanged=True,geometry_warp=False,marking_specific_processing=False,
        note='Evaluated general observation fusion. Fixed camera model and recorded trajectory; residual projection/SLAM errors remain possible.')
    updated={**meta,'texture_quality':settings,'texture_source':'all native floor RGB observations from the recorded BAG',
        'existing_observed_rgb_unchanged':False,'entire_texture_unchanged':False}
    (a.output/'manifest.json').write_text(json.dumps(updated,indent=2));(a.output/'texture_frames.json').write_text(json.dumps(rows,indent=2))
    verification=dict(frames=len(rows),native_point_count_checks=len(rows),pose_checks=len(rows),skips=0,
        semantic_labels_byte_identical=(a.reference/'road_temporal.colored.pgm').read_bytes()==(a.output/'road_temporal.colored.pgm').read_bytes(),
        original_rgb_support_preserved=bool(np.all(images['selected'][...,3][original[...,3]>0]>0)),
        comparison_fusion_support_identical=True,grid=grid,new_observed_rgb_cells=int(added.sum()))
    assert verification['semantic_labels_byte_identical'] and verification['original_rgb_support_preserved']
    (a.output/'verification.json').write_text(json.dumps(verification,indent=2));(a.output/'progress.json').write_text(json.dumps(dict(phase='complete',frames=len(rows)),indent=2))
    print('FULL TEXTURE COMPLETE',json.dumps(settings),flush=True)


if __name__=='__main__':main()
