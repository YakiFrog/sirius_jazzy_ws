"""Deterministic local RGB-map comparison using the production THETA nodes.

Uses recorded TF and synchronized scans, fixed canvases, identical training
frames, and held-out observations for a temporally stable RGB reference.
It performs no SAM3 inference and does not change the running ROS graph.
"""
from pathlib import Path
import argparse
import copy
import json
import warnings

import cv2
import numpy as np
import rclpy
import rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from sirius_navigation.theta_bev_node import ThetaBevNode
from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
from sirius_navigation.theta_indexed_map_node import (
    ThetaIndexedMapNode, quaternion_to_matrix, accumulate_weighted,
)
from sirius_navigation.theta_road_weights import directional_weights
from sirius_navigation.theta_perspective import build_perspective_remap

VARIANTS = [
    ('equal', 1., 0., 0.), ('rear050', .5, 0., 0.),
    ('rear020', .2, 0., 0.), ('rear005', .05, 0., 0.),
    ('equal_keep050', 1., .5, 0.), ('equal_keep080', 1., .8, 0.),
    ('rear050_keep050', .5, .5, 0.), ('rear020_keep050', .2, .5, 0.),
    ('rear020_keep080', .2, .8, 0.), ('equal_pad025', 1., 0., .25),
]
DTYPE = np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<f4'),
                  ('weight','<f4'),('semantic_id','u1')])
CAMERA = '/theta/dual_fisheye/image_raw/compressed'


class Capture:
    msg = None
    def publish(self, msg):
        self.msg = msg


def reference_from_holdout(frames, node):
    held = [f for f in frames if f['heldout']]
    stack = np.full((len(held), node.height, node.width, 3), np.nan, np.float32)
    for j, frame in enumerate(held):
        points = np.frombuffer(frame['clouds'][0.][0].data, DTYPE)
        gx = ((points['x']-node.origin[0])/node.res).astype(int)
        gy = ((points['y']-node.origin[1])/node.res).astype(int)
        keep = points['weight'] >= .12
        rgb = points['rgb'].copy().view(np.uint32)
        colors = np.column_stack([(rgb>>16)&255, (rgb>>8)&255, rgb&255])
        uy, ux, weight, value = accumulate_weighted(
            gy[keep], gx[keep], node.width, points['weight'][keep], colors[keep])
        stack[j, uy, ux] = value/weight[:,None]
    support = np.isfinite(stack[...,0]).sum(axis=0)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        median = np.nanmedian(stack, axis=0)
        mad = np.nanmedian(np.abs(stack-median), axis=0).mean(axis=-1)
    stable = (support >= 3) & (mad <= 12)
    return np.nan_to_num(median).astype(np.uint8), stable


def render(image, valid):
    out = image[:,:,::-1].copy()
    out[~valid] = 225
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--centers', default='1500,3000,4500')
    parser.add_argument('--half-window', type=int, default=64)
    parser.add_argument('--frame-step', type=int, default=2)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='Regenerate this experiment directory')
    args = parser.parse_args()
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=args.resume)
    centers = [int(v) for v in args.centers.split(',')]
    wanted = {i for c in centers for i in range(max(0,c-args.half_window),
                                              c+args.half_window+1,args.frame_step)}
    rclpy.init(args=['--ros-args', '-p', f'calibration:={args.calibration.resolve()}',
                    '-p','publish_raw:=false','-p','stride:=4',
                    '-p','lidar_strict_visibility:=true','-p','lidar_margin_m:=0.10'])
    buffer = Buffer(cache_time=Duration(seconds=4096))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag.resolve()), storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf','/tf_static','/scan3',CAMERA]))
    scans=[]; images={}; index=0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ('/tf','/tf_static'):
            for tf in deserialize_message(data, TFMessage).transforms:
                if topic == '/tf_static': buffer.set_transform_static(tf,'bag')
                else: buffer.set_transform(tf,'bag')
        elif topic == '/scan3':
            scan=deserialize_message(data,LaserScan)
            scans.append((scan.header.stamp.sec*10**9+scan.header.stamp.nanosec,scan))
        else:
            if index in wanted: images[index]=deserialize_message(data,CompressedImage)
            index+=1
    scans.sort(key=lambda x:x[0]); scan_times=np.array([s[0] for s in scans],np.int64)
    print(f'Loaded {len(images)} images, {len(scans)} scans and recorded TF', flush=True)
    bev=ThetaBevNode(); bev.buffer=buffer; bev.pub=Capture()
    ground=ThetaGroundCloudNode(); ground.visibility_tf=buffer; ground.lidar_gate=True
    ground.pub=Capture(); ground.semantic_pub=None
    cache={}; skipped=[]
    for i in sorted(images):
        msg=images[i]; ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        k=int(np.searchsorted(scan_times,ns)); near=min(
            (j for j in (k-1,k) if 0<=j<len(scans)),key=lambda j:abs(int(scan_times[j])-ns))
        if abs(int(scan_times[near])-ns)>150000000:
            skipped.append(i); continue
        ground.strict_scans={int(scan_times[near]):scans[near][1]}
        try:
            tf=buffer.lookup_transform('map','sirius3/base_footprint',msg.header.stamp)
        except Exception:
            skipped.append(i); continue
        bev.receive(msg); bmsg=bev.pub.msg
        image=np.frombuffer(bmsg.data,np.uint8).reshape(bmsg.height,bmsg.width,3)
        q,t=tf.transform.rotation,tf.transform.translation
        rotation=quaternion_to_matrix(q.x,q.y,q.z,q.w)
        frame={'index':i,'stamp_ns':ns,'pose':[t.x,t.y,t.z],
               'yaw':float(np.arctan2(rotation[1,0],rotation[0,0])), 'clouds':{}}
        for padding in (0.,.25):
            ground.lidar_padding=padding; ground.pub.msg=None
            ground._emit(bmsg.header.stamp,bmsg.header.frame_id,image,None,
                         publish_main=True,publish_semantic=False)
            cloud=ground.pub.msg
            if cloud is None: continue
            points=np.frombuffer(cloud.data,DTYPE).copy()
            xy=np.column_stack([points['x'],points['y']])
            world=np.column_stack([points['x'],points['y'],points['z']])@rotation.T+[t.x,t.y,t.z]
            points['x'],points['y'],points['z']=world.T
            cloud.header.frame_id='map';cloud.data=points.tobytes()
            frame['clouds'][padding]=(cloud,xy)
        if 0. in frame['clouds']: cache[i]=frame
        else: skipped.append(i)
        if len(cache)%30==0: print(f'Projected {len(cache)}/{len(images)}',flush=True)
    results={}; manifests={}
    for center in centers:
        frames=[cache[i] for i in sorted(cache) if abs(i-center)<=args.half_window]
        if len(frames)<10: raise RuntimeError(f'Too few matched frames at {center}')
        for j,f in enumerate(frames): f['heldout']=j%5==0
        arrays=[np.frombuffer(f['clouds'][0.][0].data,DTYPE) for f in frames]
        lo=np.array([min(p['x'].min() for p in arrays),min(p['y'].min() for p in arrays)])
        hi=np.array([max(p['x'].max() for p in arrays),max(p['y'].max() for p in arrays)])
        origin=np.floor((lo-2)/.05)*.05; size=np.ceil((hi+2-origin)/.05).astype(int)
        predicted={}; seen={}; panels=[]
        for name,rear,protection,padding in VARIANTS:
            node=ThetaIndexedMapNode();node.fill_hole_m=0;node.min_relative_weight=protection
            node._alloc_canvas(*origin,*size,.05)
            node._transform_to_map=lambda x,y,z,header: np.column_stack([x,y,z])
            initial=(node.width,node.height,tuple(node.origin))
            if name=='equal': reference,stable=reference_from_holdout(frames,node)
            for f in frames:
                if f['heldout'] or padding not in f['clouds']: continue
                cloud,xy=f['clouds'][padding]
                if rear!=1:
                    cloud=copy.copy(cloud);points=np.frombuffer(cloud.data,DTYPE).copy()
                    points['weight']*=directional_weights(xy[:,0],xy[:,1],rear)
                    cloud.data=points.tobytes()
                node._cloud_callback(cloud)
            assert initial==(node.width,node.height,tuple(node.origin)), 'Canvas changed across conditions'
            prefix=out/f'seg{center}_{name}'/'road';prefix.parent.mkdir(exist_ok=args.resume)
            node._save_callback(String(data=str(prefix)+'.colored'))
            obs=node.texture_weight>0
            rgb=np.clip(np.rint(node.texture_sum/np.maximum(node.texture_weight,1e-9)[...,None]),0,255).astype(np.uint8)
            predicted[name]=rgb;seen[name]=obs
            if name=='equal':
                cv2.imwrite(str(out/f'seg{center}_reference.png'),np.dstack([
                    reference[:,:,::-1],stable.astype(np.uint8)*255])[::-1])
            midpoint=min(frames,key=lambda f:abs(f['index']-center))['pose']
            cx,cy=np.rint((np.array(midpoint[:2])-origin)/.05).astype(int)
            view=render(rgb,obs)
            # Fixed world-coordinate crop, same for every candidate.
            padded=cv2.copyMakeBorder(view,120,120,120,120,cv2.BORDER_CONSTANT,value=(225,225,225))
            crop=padded[cy:cy+240,cx:cx+240][::-1]
            tile=cv2.resize(crop,(600,600),interpolation=cv2.INTER_NEAREST)
            tile=cv2.copyMakeBorder(tile,38,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245))
            cv2.putText(tile,f'{name}: rear={rear:g} protect={protection:g}',(10,25),
                        cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1,cv2.LINE_AA)
            panels.append(tile)
            node.destroy_node()
            print(f'Segment {center}: {name} saved',flush=True)
        common=stable & np.logical_and.reduce(list(seen.values()))
        rows={}
        for name,rear,protection,padding in VARIANTS:
            error=np.abs(predicted[name].astype(float)-reference).mean(axis=-1)
            rows[name]={'rear_weight':rear,'min_relative_weight':protection,'obstacle_padding_m':padding,
                        'observed_cells':int(seen[name].sum()),
                        'stable_reference_coverage':float((seen[name]&stable).sum()/max(1,stable.sum())),
                        'evaluation_cells':int(common.sum()),
                        'mean_rgb_error':float(error[common].mean()) if common.any() else None,
                        'median_rgb_error':float(np.median(error[common])) if common.any() else None}
        results[str(center)]=rows
        manifests[str(center)]={'training_indices':[f['index'] for f in frames if not f['heldout']],
            'heldout_indices':[f['index'] for f in frames if f['heldout']],
            'first_stamp_ns':frames[0]['stamp_ns'],'last_stamp_ns':frames[-1]['stamp_ns'],
            'stable_reference_cells':int(stable.sum()),'origin':origin.tolist(),'size':size.tolist(),
            'path_length_m':float(np.linalg.norm(np.diff(np.array([f['pose'][:2] for f in frames]),axis=0),axis=1).sum()),
            'yaw_span_degrees':float(np.rad2deg(np.ptp(np.unwrap([f['yaw'] for f in frames])))),
            'poses':[{'index':f['index'],'stamp_ns':f['stamp_ns'],'xyz':f['pose'],'yaw':f['yaw']} for f in frames]}
        cv2.imwrite(str(out/f'seg{center}_comparison.jpg'),np.vstack([
            np.hstack(panels[j:j+2]) for j in range(0,len(panels),2)]))
        raw=cv2.imdecode(np.frombuffer(images[center].data,np.uint8),cv2.IMREAD_COLOR)
        views=[]
        for yaw in (0,180):
            mx,my=build_perspective_remap(bev.calibration,yaw,raw.shape[:2],(640,480),120)
            view=cv2.remap(raw,mx,my,cv2.INTER_LINEAR)
            cv2.putText(view,'front' if yaw==0 else 'rear',(10,28),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,255),2)
            views.append(view)
        cv2.imwrite(str(out/f'seg{center}_camera.jpg'),np.hstack(views))
        (out/'metrics.json').write_text(json.dumps(results,indent=2))
    manifest={'bag':str(args.bag.resolve()),'calibration':str(args.calibration.resolve()),
              'segments':manifests,'skipped_indices':skipped,'variants':VARIANTS,
              'resolution':.05,'texture_decay':.7,'stride':4,'sam3':False,
              'reference':'heldout frames, >=3 timestamps, mean per-channel MAD <=12, quality >=.12'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    bev.destroy_node();ground.destroy_node();rclpy.shutdown()
    print('COMPLETE',out,flush=True)


if __name__=='__main__': main()
