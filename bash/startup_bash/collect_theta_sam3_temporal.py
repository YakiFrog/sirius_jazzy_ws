"""Collect dense front/rear baseline SAM3 observations around ten existing centers."""
import argparse,hashlib,json,shutil
from pathlib import Path
import cv2,numpy as np,yaml,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from sirius_navigation.theta_perspective import build_perspective_remap
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    ref=a.reference;out=a.output;out.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((ref/'manifest.json').read_text());cal=yaml.safe_load((ref/'calibration_used.yaml').read_text());classes=yaml.safe_load((ref/'classes_used.yaml').read_text())
    wanted={c+d for c in manifest['centers'] for d in range(-8,9)};frames={};inputs=out/'inputs';inputs.mkdir(exist_ok=True)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=manifest['bag'],storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=[manifest['camera_topic']]))
    i=0
    while reader.has_next():
        _,data,ns=reader.read_next()
        if i in wanted:
            msg=deserialize_message(data,CompressedImage);raw=bytes(msg.data);(inputs/f'{i}.jpg').write_bytes(raw)
            frames[str(i)]={'stamp_ns':msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,'bag_ns':ns,'sha256':hashlib.sha256(raw).hexdigest()}
        i+=1
    assert len(frames)==len(wanted)
    config={'centers':manifest['centers'],'offsets':list(range(-8,9)),'bag':manifest['bag'],'camera_topic':manifest['camera_topic'],'frames':frames,'resolution':512,'score_min':.5,'prompt':classes['prompt'],'reference':str(ref.resolve()),'reused_primary_offsets':[-8,0,8]}
    (out/'manifest.json').write_text(json.dumps(config,indent=2));shutil.copy2(ref/'calibration_used.yaml',out/'calibration_used.yaml');shutil.copy2(ref/'classes_used.yaml',out/'classes_used.yaml')
    remaps={};session=Sam3ExperimentClient();fresh=0;reused=0
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2));(out/'registry_before.json').write_text(json.dumps(session.registry_before,indent=2))
            session.configure(classes['prompt'],classes['class_thresholds'],threshold=1.,resolution=512)
            for c in manifest['centers']:
                for i in range(c-8,c+9):
                    raw=cv2.imread(str(inputs/f'{i}.jpg'));dest=out/f'seg{c}'/f'frame{i}';dest.mkdir(parents=True,exist_ok=True)
                    for yaw in (0,180):
                        if yaw not in remaps:remaps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
                        image=cv2.remap(raw,*remaps[yaw],cv2.INTER_LINEAR);ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
                        baseline=ref/f'seg{c}'/f'frame{i}'/'perspective2_512'
                        if (dest/f'yaw{yaw}.json').exists():continue
                        if (baseline/f'yaw{yaw}.upload.jpg').exists():
                            assert (baseline/f'yaw{yaw}.upload.jpg').read_bytes()==jpeg.tobytes()
                            for ext in ('upload.jpg','classes.png','scores.png','json'):shutil.copy2(baseline/f'yaw{yaw}.{ext}',dest/f'yaw{yaw}.{ext}')
                            reused+=1
                        else:
                            cls,score,meta=session.infer(image)
                            cv2.imwrite(str(dest/f'yaw{yaw}.classes.png'),cls);cv2.imwrite(str(dest/f'yaw{yaw}.scores.png'),score)
                            (dest/f'yaw{yaw}.upload.jpg').write_bytes(session.last_upload);(dest/f'yaw{yaw}.json').write_text(json.dumps(meta,indent=2));fresh+=1
                print('Segment',c,'complete; fresh',fresh,'reused',reused,flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'completed.json').write_text(json.dumps({'complete':True,'unique_frames':len(frames),'new_inferences_this_run':fresh,'reused_inferences_this_run':reused},indent=2))


if __name__=='__main__':main()
