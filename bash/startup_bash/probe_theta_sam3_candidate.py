"""Test a four-class yellow-tactile candidate on exactly the original center pixels."""
from pathlib import Path
import argparse,json,hashlib
import cv2,numpy as np,yaml
from sirius_navigation.theta_perspective import build_perspective_remap
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
from compare_theta_sam3_projection import overlay


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('study',type=Path)
    args=parser.parse_args();p=args.study;manifest=json.loads((p/'manifest.json').read_text())
    cal=yaml.safe_load((p/'calibration_used.yaml').read_text());out=p/'candidate';out.mkdir(exist_ok=True)
    prompt='grass, yellow tactile paving, roadway, sidewalk'
    session=Sam3ExperimentClient();maps={}
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2))
            assert 'yellow tactile paving' not in session.registry_before['ids']
            session.configure(prompt,{x.strip():.5 for x in prompt.split(',')},threshold=.5,resolution=1008)
            for center in manifest['centers']:
                raw=cv2.imread(str(p/'inputs'/f'frame_{center:04d}.jpg'))
                for yaw in (0,180):
                    if yaw not in maps:maps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
                    image=cv2.remap(raw,*maps[yaw],cv2.INTER_LINEAR)
                    cls,score,meta=session.infer(image)
                    # The only unregistered prompt in this batch is yellow tactile paving.
                    corrected=cls.copy();corrected[(cls==0)&(score>0)]=4
                    expected=p/f'seg{center}'/f'frame{center}'/'perspective2_1008'/f'yaw{yaw}.upload.jpg'
                    assert hashlib.sha256(expected.read_bytes()).hexdigest()==meta['uploaded_jpeg_sha256']
                    dest=out/f'seg{center}';dest.mkdir(exist_ok=True)
                    for suffix,data in [('server_classes',cls),('classes',corrected),('scores',score)]:cv2.imwrite(str(dest/f'yaw{yaw}.{suffix}.png'),data)
                    (dest/f'yaw{yaw}.upload.jpg').write_bytes(session.last_upload)
                    meta['class_assignment']='Only nonzero-score unknown-ID winners are yellow tactile paving -> 4; registry unchanged'
                    (dest/f'yaw{yaw}.json').write_text(json.dumps(meta,indent=2))
                    for t in (.8,.65,.5):cv2.imwrite(str(dest/f'yaw{yaw}.{t:.2f}.overlay.jpg'),overlay(image,np.where(score/255.>=t,corrected,0).astype(np.uint8)))
                print('Candidate',center,'done',flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'completed.json').write_text(json.dumps({'complete':True,'prompt':prompt,'centers':manifest['centers'],
        'input_jpeg_identical_to':'perspective2_1008','class_id_assigned_client_side':4},indent=2))


if __name__=='__main__':main()
