"""Infer every recorded camera frame, then track yellow tactile paving both ways.

Sequential, resumable SAM3 inference without ROS replay/frame dropping. The
existing map builder consumes the resulting manifest and cached image maps.
"""
import argparse,hashlib,json,os,shutil,time
from pathlib import Path
import cv2,numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
from sirius_navigation.theta_perspective import build_perspective_remap
from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency,merge_tactile_tracks


def write_json(path,data):
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(data,indent=2));temporary.replace(path)


def png(path,data):
    if not cv2.imwrite(str(path),data):raise RuntimeError('Failed to save '+str(path))


def link(source,target):
    if not target.exists():os.link(source,target)


def folder(root,manifest,index):
    center=min(manifest['centers'],key=lambda c:abs(c-index))
    return root/'yellow'/f'seg{center}'/f'frame{index}'


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('reference',type=Path);parser.add_argument('output',type=Path);a=parser.parse_args()
    ref=a.reference;root=a.output;root.mkdir(parents=True,exist_ok=True);yellow=root/'yellow';yellow.mkdir(exist_ok=True);inputs=root/'inputs';inputs.mkdir(exist_ok=True);cv2.setNumThreads(2)
    source=json.loads((ref/'manifest.json').read_text());started=time.time();frames={}
    for name in ('calibration_used.yaml',):shutil.copy2(ref/name,root/name)
    shutil.copy2(Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation/config/sam3_classes.yaml',root/'classes_used.yaml')
    cal=yaml.safe_load((root/'calibration_used.yaml').read_text());prompt='grass, yellow tactile paving, roadway, sidewalk'
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=source['bag'],storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=[source['camera_topic']]))
    index=0
    while reader.has_next():
        _,data,ns=reader.read_next();msg=deserialize_message(data,CompressedImage);raw=bytes(msg.data);digest=hashlib.sha256(raw).hexdigest();dest=inputs/f'{index}.jpg'
        if str(index) in source['frames']:
            assert digest==source['frames'][str(index)]['sha256'];link(ref/'inputs'/f'{index}.jpg',dest)
        elif not dest.exists():dest.write_bytes(raw)
        else:assert hashlib.sha256(dest.read_bytes()).hexdigest()==digest
        frames[str(index)]={'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'bag_ns':ns,'sha256':digest};index+=1
    manifest={**source,'frames':frames,'full_bag':True,'camera_frames':index,'observations':index*2,'prompt':prompt,'reference':str(ref.resolve()),'coverage':'all camera frames; no interval sampling or frame dropping'}
    manifest['selection']='all_camera_frames'
    for key in ('offsets','reused_primary_offsets'):manifest.pop(key,None)
    write_json(root/'manifest.json',manifest);indices=sorted(int(i) for i in frames)
    for center in manifest['centers']:
        (yellow/f'seg{center}').mkdir(exist_ok=True)
        p=root/f'seg{center}'
        if not p.exists():p.symlink_to(Path('yellow')/f'seg{center}',target_is_directory=True)
    print('Extracted all',index,'camera frames',flush=True)
    stats={'full_bag':True,'total_frames':index,'total_observations':index*2,'fresh_inferences':0,'reused_verified_study':0,'cached_this_run':0,'started_unix':started}
    trackers={yaw:CausalTactileConsistency() for yaw in (0,180)};maps={};session=Sam3ExperimentClient()
    try:
        with session:
            write_json(root/'server_before.json',session.before);write_json(root/'registry_before.json',session.registry_before)
            unknown=set(prompt.split(', '))-set(session.registry_before['ids']);assert unknown in (set(),{'yellow tactile paving'})
            if not unknown:assert session.registry_before['ids']['yellow tactile paving']==4
            for name,cid in [('grass',3),('roadway',5),('sidewalk',6)]:assert session.registry_before['ids'][name]==cid
            session.configure(prompt,{p:.5 for p in prompt.split(', ')},threshold=1.,resolution=512)
            for completed,i in enumerate(indices,1):
                dest=folder(root,manifest,i);dest.mkdir(exist_ok=True);raw=None
                for yaw in (0,180):
                    stem=f'yaw{yaw}.';meta_file=dest/(stem+'json')
                    if not meta_file.exists() and str(i) in source['frames']:
                        center=min(source['centers'],key=lambda c:abs(c-i));old=ref/'yellow'/f'seg{center}'/f'frame{i}';original=ref/f'seg{center}'/f'frame{i}'
                        for ext in ('classes.png','scores.png','json','server_classes.png'):link(old/(stem+ext),dest/(stem+ext))
                        link(original/(stem+'upload.jpg'),dest/(stem+'upload.jpg'));stats['reused_verified_study']+=1
                    elif meta_file.exists():stats['cached_this_run']+=1
                    else:
                        if raw is None:raw=cv2.imread(str(inputs/f'{i}.jpg'))
                        if yaw not in maps:maps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
                        image=cv2.remap(raw,*maps[yaw],cv2.INTER_LINEAR);cls,score,meta=session.infer(image);ids=cls.copy()
                        if unknown:ids[(cls==0)&(score>0)]=4
                        png(dest/(stem+'server_classes.png'),cls);png(dest/(stem+'classes.png'),ids);png(dest/(stem+'scores.png'),score)
                        (dest/(stem+'upload.jpg')).write_bytes(session.last_upload);meta['only_unknown_prompt_assigned_to_4']=sorted(unknown);write_json(meta_file,meta);stats['fresh_inferences']+=1
                    meta=json.loads(meta_file.read_text());view_bytes=(dest/(stem+'upload.jpg')).read_bytes();assert hashlib.sha256(view_bytes).hexdigest()==meta['uploaded_jpeg_sha256']
                    image=cv2.imdecode(np.frombuffer(view_bytes,np.uint8),cv2.IMREAD_COLOR);ids=cv2.imread(str(dest/(stem+'classes.png')),0);score=cv2.imread(str(dest/(stem+'scores.png')),0);assert image is not None and ids is not None and score is not None
                    fixed,confidence,track_stats=trackers[yaw].apply(image,ids,score,frames[str(i)]['stamp_ns']/1e9)
                    png(dest/(stem+'causal.classes.png'),fixed);png(dest/(stem+'causal.scores.png'),confidence);write_json(dest/(stem+'causal.json'),track_stats)
                if completed%25==0 or completed==len(indices):
                    progress={**stats,'phase':'inference_and_causal_tracking','completed_frames':completed,'last_frame':i,'elapsed_sec':time.time()-started};write_json(root/'progress.json',progress);print(json.dumps(progress),flush=True)
    finally:
        if hasattr(session,'restoration'):write_json(root/'restoration.json',session.restoration)
    write_json(root/'inference_completed.json',{**stats,'complete':True,'elapsed_sec':time.time()-started})
    for yaw in (0,180):
        tracker=CausalTactileConsistency()
        for completed,i in enumerate(reversed(indices),1):
            dest=folder(root,manifest,i);stem=f'yaw{yaw}.';image=cv2.imread(str(dest/(stem+'upload.jpg')));ids=cv2.imread(str(dest/(stem+'classes.png')),0);scores=cv2.imread(str(dest/(stem+'scores.png')),0)
            reverse,rs,ts=tracker.apply(image,ids,scores,-frames[str(i)]['stamp_ns']/1e9)
            forward=cv2.imread(str(dest/(stem+'causal.classes.png')),0);fs=cv2.imread(str(dest/(stem+'causal.scores.png')),0);combined,confidence=merge_tactile_tracks(ids,forward,fs,reverse,rs)
            for method,cls,score in [('reverse',reverse,rs),('bidirectional',combined,confidence)]:png(dest/(stem+method+'.classes.png'),cls);png(dest/(stem+method+'.scores.png'),score)
            write_json(dest/(stem+'reverse.json'),ts)
            if completed%100==0 or completed==len(indices):
                progress={**stats,'phase':'reverse_tracking','yaw':yaw,'completed_frames':completed,'last_frame':i,'elapsed_sec':time.time()-started};write_json(root/'progress.json',progress);print(json.dumps(progress),flush=True)
    write_json(root/'completed.json',{**stats,'complete':True,'full_camera_coverage':True,'bidirectional_observations':len(indices)*2,'elapsed_sec':time.time()-started})


if __name__=='__main__':main()
