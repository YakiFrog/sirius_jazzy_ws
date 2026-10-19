"""Paired SAM3 projection/resolution experiment on ten recorded local intervals.

No ROS replay, publisher, global map change or persistent SAM3 registry update.
Full class/score PNGs, exact uploaded JPEGs and response metadata are retained.
"""
from pathlib import Path
import argparse
import json
import time
import hashlib
import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from tf2_msgs.msg import TFMessage
import yaml
from sirius_navigation.theta_perspective import build_perspective_remap,build_bev_to_view_maps
from sirius_navigation.theta_bev_projection import build_blend_maps
from sirius_navigation.theta_equirectangular import (
    build_equirectangular_remap,build_bev_to_equirectangular,ground_tile,
)
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient

CAMERA='/theta/dual_fisheye/image_raw/compressed'
THRESHOLDS=(.8,.65,.5)
METHODS=[('perspective2_512',512,'p2',False),('perspective2_1008',1008,'p2',False),
         ('perspective4_1008',1008,'p4',False),('equirect_full_512',512,'erp',False),
         ('equirect_full_1008',1008,'erp',False),('equirect_ground4_1008',1008,'tiles',False),
         ('perspective2_targets_1008',1008,'p2',True),('equirect_ground4_targets_1008',1008,'tiles',True)]
COLORS={3:(0,255,0),4:(0,255,255),5:(255,0,0),6:(128,128,128)}


def overlay(image,classes,alpha=.45):
    result=image.copy()
    for cid,color in COLORS.items():
        keep=classes==cid
        result[keep]=(result[keep]*(1-alpha)+np.array(color)*alpha).astype(np.uint8)
    return result


def project_labels(samples,threshold,shape):
    votes=np.zeros((*shape,7),np.float32)
    support=np.zeros(shape,bool)
    for classes,scores,u,v,valid,wrap in samples:
        rows,cols=np.nonzero(valid)
        uu=np.rint(u[rows,cols]).astype(int);vv=np.rint(v[rows,cols]).astype(int)
        uu=uu%classes.shape[1] if wrap else np.clip(uu,0,classes.shape[1]-1)
        vv=np.clip(vv,0,classes.shape[0]-1)
        ids=classes[vv,uu];score=scores[vv,uu]/255.
        keep=(ids>=3)&(ids<=6)&(score>=threshold)
        np.add.at(votes,(rows[keep],cols[keep],ids[keep]),score[keep])
        support[rows,cols]=True
    label=np.argmax(votes,axis=-1).astype(np.uint8)
    return label,support


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--calibration',required=True,type=Path);p.add_argument('--classes',required=True,type=Path)
    p.add_argument('--centers',default='300,900,1500,2100,2700,3300,3900,4500,5100,5650')
    p.add_argument('--server',default='http://localhost:8080');p.add_argument('--resume',action='store_true')
    args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    centers=[int(x) for x in args.centers.split(',')];offsets=(-16,-8,0,8,16)
    wanted={c+d for c in centers for d in offsets}
    cal=yaml.safe_load(args.calibration.read_text());classes=yaml.safe_load(args.classes.read_text())
    # A 512-square BEV is used only for this image/ground comparison (1.95cm cells).
    cal['bev_size']=512;size=cal['bev_size']
    inputs=out/'inputs';inputs.mkdir(exist_ok=True)
    frames={};reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag.resolve()),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[CAMERA,'/tf_static']))
    index=0;mount_tf=None
    while reader.has_next():
        topic,data,bag_ns=reader.read_next()
        if topic=='/tf_static':
            for tf in deserialize_message(data,TFMessage).transforms:
                if tf.child_frame_id=='sirius3/theta_link':
                    q=tf.transform.rotation;t=tf.transform.translation
                    if max(abs(q.x),abs(q.y),abs(q.z))>1e-6:raise RuntimeError('Nonzero mount needs explicit TF composition')
                    cal['camera_position']=[t.x,t.y,t.z]
                    mount_tf={'parent':tf.header.frame_id,'child':tf.child_frame_id,
                              'translation':[t.x,t.y,t.z],'quaternion':[q.x,q.y,q.z,q.w]}
        else:
            if index in wanted:
                msg=deserialize_message(data,CompressedImage);raw=bytes(msg.data)
                (inputs/f'frame_{index:04d}.jpg').write_bytes(raw)
                frames[index]={'index':index,'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                               'bag_ns':bag_ns,'image_sha256':hashlib.sha256(raw).hexdigest()}
            index+=1
    if set(frames)!=wanted:raise RuntimeError('Missing selected input images')
    (out/'calibration_used.yaml').write_text(yaml.safe_dump(cal))
    (out/'classes_used.yaml').write_text(yaml.safe_dump(classes))
    manifest={'bag':str(args.bag.resolve()),'camera_topic':CAMERA,'centers':centers,'offsets':offsets,
              'frames':frames,'mount_tf':mount_tf,'thresholds':THRESHOLDS,'methods':METHODS,
              'bev_size':size,'bev_extent_m':cal['bev_extent_m'],'ground_radius_m':[1.2,4.8],
              'inference_timeout_seconds':40,'client_jpeg_quality':90,'registry_modified':False,
              'note':'Counts measure classified area, not segmentation accuracy. Model score threshold != pixel probability.'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    views={};matches={};erpmap=None;bevmap=None;stats={};timings=[]
    session=Sam3ExperimentClient(args.server,timeout=40.)
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2))
            (out/'registry_before.json').write_text(json.dumps(session.registry_before,indent=2))
            for prompt,spec in classes['classes'].items():
                if session.registry_before['ids'].get(prompt)!=spec['id']:
                    raise RuntimeError(f'Server class registry differs: {prompt}')
            for frame_index in sorted(frames):
                center=min(centers,key=lambda c:abs(c-frame_index));frame_dir=out/f'seg{center}'/f'frame{frame_index}'
                frame_dir.mkdir(parents=True,exist_ok=True)
                fisheye=cv2.imread(str(inputs/f'frame_{frame_index:04d}.jpg'))
                if erpmap is None:
                    erpmap=build_equirectangular_remap(cal,fisheye.shape[:2])
                    eu,ev,evalid=build_bev_to_equirectangular(cal,size)
                    bevmap=build_blend_maps(cal,fisheye.shape[1],fisheye.shape[0])
                    for yaw in (0,90,180,270):
                        views[yaw]=build_perspective_remap(cal,yaw,fisheye.shape[:2],(640,480),120)
                        matches[yaw]=build_bev_to_view_maps(cal,yaw,size,(640,480),120,1.2,4.8)
                pano=cv2.remap(fisheye,*erpmap,cv2.INTER_LINEAR)
                cv2.imwrite(str(frame_dir/'panorama.jpg'),pano,[cv2.IMWRITE_JPEG_QUALITY,92])
                a,b,c,d,blend=bevmap
                bev=np.clip(cv2.remap(fisheye,a,b,cv2.INTER_LINEAR).astype(float)*blend[...,None]+
                            cv2.remap(fisheye,c,d,cv2.INTER_LINEAR).astype(float)*(1-blend[...,None]),0,255).astype(np.uint8)
                cv2.imwrite(str(frame_dir/'ground_rgb.jpg'),bev)
                stats[str(frame_index)]={}
                for name,res,kind,targets in METHODS:
                    if targets and frame_index!=center:continue
                    prompt=classes['prompt']
                    if targets:prompt=', '.join(k for k,spec in classes['classes'].items() if spec['id'] in (3,4))
                    session.configure(prompt,classes['class_thresholds'],resolution=res)
                    batch=[]
                    if kind in ('p2','p4'):
                        for yaw in ((0,180) if kind=='p2' else (0,90,180,270)):
                            image=cv2.remap(fisheye,*views[yaw],cv2.INTER_LINEAR)
                            u,v,valid=matches[yaw];batch.append((f'yaw{yaw}',image,u,v,valid,False))
                    elif kind=='erp':batch=[('full',pano,eu,ev,evalid,True)]
                    else:
                        for yaw in (0,90,180,270):
                            image,u,v,valid=ground_tile(pano,eu,ev,evalid,yaw)
                            batch.append((f'yaw{yaw}',image,u,v,valid,False))
                    dest=frame_dir/name;dest.mkdir(exist_ok=True);samples=[];metas=[]
                    for view_name,image,u,v,valid,wrap in batch:
                        meta_file=dest/(view_name+'.json')
                        if args.resume and meta_file.exists():
                            cls=cv2.imread(str(dest/(view_name+'.classes.png')),0)
                            score=cv2.imread(str(dest/(view_name+'.scores.png')),0)
                            meta=json.loads(meta_file.read_text())
                            if cls is None or score is None:raise RuntimeError('Incomplete inference cache')
                        else:
                            cls,score,meta=session.infer(image)
                            (dest/(view_name+'.upload.jpg')).write_bytes(session.last_upload)
                            cv2.imwrite(str(dest/(view_name+'.classes.png')),cls)
                            cv2.imwrite(str(dest/(view_name+'.scores.png')),score)
                            cv2.imwrite(str(dest/(view_name+'.overlay.jpg')),overlay(image,np.where(score>=204,cls,0)))
                            meta_file.write_text(json.dumps(meta,indent=2))
                        samples.append((cls,score,u,v,valid,wrap));metas.append(meta)
                    method_stats={}
                    for cutoff in THRESHOLDS:
                        label,support=project_labels(samples,cutoff,(size,size))
                        cv2.imwrite(str(dest/f'ground_{cutoff:.2f}.png'),label)
                        cv2.imwrite(str(dest/f'ground_{cutoff:.2f}.overlay.jpg'),overlay(bev,label))
                        counts={str(cid):int((label==cid).sum()) for cid in (3,4,5,6)}
                        # P2 common field isolates classification from additional side coverage.
                        common=matches[0][2]|matches[180][2]
                        method_stats[str(cutoff)]={'class_pixels':counts,
                            'common_class_pixels':{str(cid):int(((label==cid)&common).sum()) for cid in (3,4,5,6)},
                            'view_supported_pixels':int(support.sum()),'common_supported_pixels':int((support&common).sum()),
                            'sampled_area_per_pixel_m2':(cal['bev_extent_m']/size)**2}
                    stats[str(frame_index)][name]=method_stats
                    timings.append({'frame':frame_index,'method':name,'views':len(batch),
                                    'seconds':sum(meta['wall_seconds'] for meta in metas),
                                    'view_timeout_2s_exceeded':sum(meta['wall_seconds']>2 for meta in metas)})
                    (out/'metrics.json').write_text(json.dumps(stats,indent=2))
                    (out/'timings.json').write_text(json.dumps(timings,indent=2))
                    print(f'frame {frame_index} / segment {center}: {name} done',flush=True)
            print('INFERENCE COMPLETE',flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'completed.json').write_text(json.dumps({'frames':len(frames),'segments':len(centers),'complete':True},indent=2))


if __name__=='__main__':main()
