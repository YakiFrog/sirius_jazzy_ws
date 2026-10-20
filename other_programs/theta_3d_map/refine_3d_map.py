#!/usr/bin/env python3
"""Rebuild a map using an explicitly declared common planar indoor floor."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import numpy as np
import yaml

from clean_3d_map import load_poses, raw_scan_observations
from mapping_floor import refine_known_floor
from mapping_quality import trajectory_jump_quality
from mapping_color import initial_trajectory_source, read_trajectory
from mapping_dynamic import dufomap_filter, finishing_indices
from offline_3d_mapping import reduce_points, write_map, save_trajectory
from recolor_3d_map import recolor_new_output, read_ply


def rebuild(source, output, known_single_floor=False):
    if not known_single_floor:
        raise ValueError('Requires explicit --known-single-floor; not applicable to arbitrary terrain')
    source,output=Path(source).resolve(),Path(output).resolve()
    if output.exists():raise ValueError('Output must be a new directory')
    report=json.loads((source/'report.json').read_text())
    cache=Path(report['cleaning_source']['cache'])
    mount=np.asarray(report['resolved_base_from_lidar']);poses=load_poses(source,report)
    times=np.atleast_1d(np.genfromtxt(source/'trajectory.csv',delimiter=',',names=True))['stamp']
    chunks=[];colors=[]
    for i,pose in enumerate(poses):
        with np.load(cache/f'{i:06d}.npz') as frame:
            chunks.append(frame['xyz']@pose[:3,:3].T+pose[:3,3]);colors.append(frame['rgb'])
    old_xyz,old_rgb=reduce_points(np.concatenate(chunks),np.concatenate(colors),report['options']['voxel'])
    original,packed=read_ply(source/'map_3d_colored_unfiltered.ply')
    rgb=np.column_stack([(packed>>16)&255,(packed>>8)&255,packed&255])
    if (old_xyz.shape!=original.shape or not np.allclose(old_xyz,original,atol=2.e-5,rtol=0)
            or not np.array_equal(np.round(old_rgb).astype(np.uint8),rgb)):
        raise ValueError('Cache identity could not be verified against the source map')
    del chunks,old_xyz,old_rgb,original,packed,rgb
    print('Verified source/cache. Measuring individual scan floors.',flush=True)
    links=[(e['from'],e['to']) for e in report['edges'] if e.get('accepted')]
    new,refinement=refine_known_floor(cache,poses,times,mount,links)
    after=refinement['floor_quality_candidate']
    jumps=trajectory_jump_quality(new,times)
    level=np.asarray(refinement['frame_transform_old_map_to_new_map'])
    output.mkdir();(output/'.mapping-diagnostic').touch()
    for name in ('calibration.yaml','resolved_calibration.yaml','theta_calibration_used.yaml'):
        if (source/name).is_file():shutil.copyfile(source/name,output/name)
    save_trajectory(output/'trajectory.csv',times,new,mount)
    initial_source=initial_trajectory_source(source)
    initial_times,initial=read_trajectory(initial_source/'trajectory_initial.csv',mount)
    if not np.array_equal(initial_times,times):raise ValueError('Initial trajectory stamps differ')
    save_trajectory(output/'trajectory_initial.csv',times,[level@p for p in initial],mount)
    shutil.copyfile(source/'trajectory.csv',output/'trajectory_before_floor.csv')
    chunks=[]
    for i,pose in enumerate(new):
        with np.load(cache/f'{i:06d}.npz') as frame:chunks.append(frame['xyz']@pose[:3,:3].T+pose[:3,3])
    xyz,rgb=reduce_points(np.concatenate(chunks),np.concatenate(colors),report['options']['voxel'])
    del chunks,colors
    write_map(output/'map_3d_unfiltered.ply',xyz)
    write_map(output/'map_3d_colored_unfiltered.ply',xyz,rgb)
    refinement.update(source=str(source),cache=str(cache),cache_identity_verified=True,
        source_trajectory_sha256=hashlib.sha256((source/'trajectory.csv').read_bytes()).hexdigest())
    report.update(floor_refinement=refinement,floor_overlap_quality=after,trajectory_jump_quality=jumps,
        validation_status='needs_review',points=len(xyz),
        cleaning_source=dict(directory=str(source),cache=str(cache),
            geometry_policy='rebuilt from local scan returns at refined poses; prior masks not reused'),
        optimization_after_refinement='known-floor pose prior; original pose graph kept as historical evidence')
    report['optimization_before_floor_refinement']=report.pop('optimization',None)
    report['optimization']=dict(success=refinement['success'],cost=refinement['cost'],
        evaluations=refinement['evaluations'],method=refinement['model'])
    report['surface_refinement_before_floor']=report.pop('surface_refinement',None)
    for name in ('loop_residuals','optimization_first_pass'):
        if name in report:
            report[name+'_before_floor_refinement']=report.pop(name)
    report['source_pose_graph_evidence']='edges/loops describe the source graph; original measured factors were not reoptimized'
    refinement['implementation_sha256']={
        name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ('refine_3d_map.py','mapping_floor.py')}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    policy=Path(__file__).resolve().parents[2]/'params/offline_3d_mapping.yaml'
    defaults=yaml.safe_load(policy.read_text())['parameters']
    def frame_points(i):
        with np.load(cache/f'{i:06d}.npz') as frame:return frame['xyz'].copy()
    class Progress:
        def update(self,n,message):
            if n%400==0:print(message,flush=True)
    evidence={}
    keep,dynamic=dufomap_filter(xyz,frame_points,new,SimpleNamespace(**defaults),Progress(),
        times=times,observations=raw_scan_observations(output,report,evidence))
    dynamic['raw_scan_evidence']=evidence
    write_map(output/'map_3d.ply',xyz[keep]);write_map(output/'map_3d_colored.ply',xyz[keep],rgb[keep])
    write_map(output/'dynamic_candidates.ply',xyz[~keep],rgb[~keep])
    ids=finishing_indices(dynamic)
    write_map(output/'dynamic_fragments.ply',xyz[ids],rgb[ids])
    np.savez_compressed(output/'dynamic_mask.npz',keep=keep)
    shutil.copyfile(policy,output/'cleaning_policy.yaml')
    report.update(dynamic_filter=dict(enabled=True,**dynamic),points=int(keep.sum()))
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    recolor_new_output(output,Path(report['bag']),calibration_path=source/'theta_calibration_used.yaml')
    (output/'.mapping-diagnostic').unlink()
    print('Rebuilt map:',output,flush=True)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source');parser.add_argument('--output',required=True)
    parser.add_argument('--known-single-floor',action='store_true')
    args=parser.parse_args()
    rebuild(args.source,args.output,args.known_single_floor)
