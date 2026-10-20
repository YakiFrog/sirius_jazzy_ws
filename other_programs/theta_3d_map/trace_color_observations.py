#!/usr/bin/env python3
"""Trace the actual top color observations around a selected map coordinate.

Uses the same full-scene visibility, normals and image selection as recoloring.
Diagnostic products are hidden from ordinary map selection. No calibration fit.
"""
import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import yaml

from recolor_3d_map import recolor, read_ply
from sirius_navigation.theta_perspective import project_camera_frame_points


def run(source, output, xy, radius=.6):
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise ValueError('Trace output must be a new directory')
    xyz, rgb = read_ply(source/'map_3d_colored.ply')
    colors = np.column_stack([(rgb>>16)&255,(rgb>>8)&255,rgb&255])
    nearby = (np.linalg.norm(xyz[:,:2]-xy,axis=1)<radius) & (xyz[:,2]>.5) & (xyz[:,2]<1.8)
    local_ids = np.flatnonzero(nearby)
    if not len(local_ids):
        raise ValueError('No map points near requested coordinate/height')
    target = np.r_[xy,1.2]
    # Probe both the reported bright strip and the adjacent dark surface.
    probes = []
    for brightness, location in [('bright',target),('dark',target+[-.2,0,0]),('dark',target+[.2,0,0])]:
        candidates = local_ids[(colors[local_ids].mean(axis=1)>150) if brightness=='bright'
                               else (colors[local_ids].mean(axis=1)<50)]
        if len(candidates):
            probes.append(int(candidates[np.argmin(np.linalg.norm(xyz[candidates]-location,axis=1))]))
    probes = np.unique(probes)
    index = np.full(len(xyz),-1,int); index[local_ids] = np.arange(len(local_ids))
    views = yaml.safe_load((source/'colorization_policy.yaml').read_text())['fusion_views']
    scores = np.zeros((len(local_ids),views),np.float32)
    selected_frames = np.full(scores.shape,-1,int)
    selected_colors = np.zeros(scores.shape+(3,),np.uint8)
    frames = []
    output.mkdir(parents=True); (output/'.mapping-diagnostic').touch()
    calibration = yaml.safe_load((source/'theta_calibration_used.yaml').read_text())
    def observe(t, image, camera, ids, packed, confidence):
        frame = len(frames)
        frames.append(dict(stamp=t,camera=camera.tolist()))
        rows = index[ids]
        valid = (rows>=0)&(confidence>0)
        slots = np.argmin(scores[rows[valid]],axis=1)
        positions = np.flatnonzero(valid)
        better = confidence[positions]>scores[rows[positions],slots]
        positions,slots = positions[better],slots[better]
        rows = rows[positions]
        scores[rows,slots] = confidence[positions]
        selected_frames[rows,slots] = frame
        p = packed[positions]
        selected_colors[rows,slots] = np.column_stack([(p>>16)&255,(p>>8)&255,p&255])
        # Keep source images only when they become one of the probe's top views.
        if np.intersect1d(ids[positions],probes).size:
            camera_points = (xyz[probes]-camera[:3,3]) @ camera[:3,:3]
            uv, projected = project_camera_frame_points(camera_points,calibration,image.shape[:2])
            annotated = image.copy()
            record = []
            for k,(point,valid_projection) in enumerate(zip(uv,projected)):
                if not valid_projection: continue
                u,v = np.rint(point).astype(int)
                cv2.circle(annotated,(u,v),5,(0,0,255) if k==0 else (0,255,255),1)
                cv2.putText(annotated,f'P{k}',(u+7,v-7),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
                record.append(dict(probe=k,uv=point.tolist(),range_m=float(np.linalg.norm(camera_points[k]))))
            cv2.imwrite(str(output/f'view_{frame:04d}.jpg'),annotated)
            frames[-1]['projections'] = record
    work = output/'reproduced'
    recolor(source,work,policy_path=source/'colorization_policy.yaml',
            calibration_path=source/'theta_calibration_used.yaml',observation_callback=observe)
    _, reproduced = read_ply(work/'map_3d_colored.ply')
    if not np.array_equal(reproduced,rgb):
        raise ValueError('Reproduced RGB differs; do not attribute these traces to the selected map')
    used = np.unique(selected_frames[index[probes]])
    for image in output.glob('view_*.jpg'):
        if int(image.stem.split('_')[1]) not in used: image.unlink()
    details = []
    for point in probes:
        row = index[point]
        views = []
        for slot in np.argsort(scores[row])[::-1]:
            frame = int(selected_frames[row,slot])
            if frame<0: continue
            views.append(dict(frame=frame,confidence=float(scores[row,slot]),
                rgb=selected_colors[row,slot].tolist(),**frames[frame]))
        details.append(dict(point_id=int(point),xyz=xyz[point].tolist(),rgb=colors[point].tolist(),views=views))
    np.savez_compressed(output/'trace.npz',point_ids=local_ids,xyz=xyz[local_ids],rgb=colors[local_ids],
                        scores=scores,frames=selected_frames,view_rgb=selected_colors)
    result = dict(source=str(source),xy=list(xy),radius_m=radius,roi_points=len(local_ids),
        rgb_reproduced_exactly=True,probes=details,frames=frames,
        limitation='No ground-truth pixel correspondence; diagnostic trace, not calibration')
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    shutil.rmtree(work)
    print(f'Trace saved: {output}; {len(local_ids)} ROI points; RGB reproduction exact')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source'); parser.add_argument('--output',required=True)
    parser.add_argument('--xy',nargs=2,type=float,required=True)
    parser.add_argument('--radius',type=float,default=.6)
    args=parser.parse_args()
    run(args.source,args.output,args.xy,args.radius)
