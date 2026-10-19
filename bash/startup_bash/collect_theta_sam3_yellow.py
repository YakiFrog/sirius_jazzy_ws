"""Re-evaluate the same 340 exact JPEG inputs with yellow tactile paving at512."""
import argparse,hashlib,json
from pathlib import Path
import cv2,yaml
from sirius_navigation.theta_perspective import build_perspective_remap
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);a=p.parse_args()
    root=a.study;out=root/'yellow';out.mkdir(exist_ok=True)
    manifest=json.loads((root/'manifest.json').read_text());cal=yaml.safe_load((root/'calibration_used.yaml').read_text())
    prompt='grass, yellow tactile paving, roadway, sidewalk';session=Sam3ExperimentClient();maps={}
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2))
            (out/'registry_before.json').write_text(json.dumps(session.registry_before,indent=2))
            unknown=set(prompt.split(', '))-set(session.registry_before['ids'])
            assert unknown in (set(),{'yellow tactile paving'}),unknown
            session.configure(prompt,{name:.5 for name in prompt.split(', ')},threshold=1.,resolution=512)
            for center in manifest['centers']:
                for index in range(center-8,center+9):
                    raw=cv2.imread(str(root/'inputs'/f'{index}.jpg'));dest=out/f'seg{center}'/f'frame{index}';dest.mkdir(parents=True,exist_ok=True)
                    for yaw in (0,180):
                        if (dest/f'yaw{yaw}.json').exists():continue
                        if yaw not in maps:maps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
                        image=cv2.remap(raw,*maps[yaw],cv2.INTER_LINEAR)
                        cls,score,meta=session.infer(image);corrected=cls.copy()
                        if unknown:corrected[(cls==0)&(score>0)]=4
                        expected=root/f'seg{center}'/f'frame{index}'/f'yaw{yaw}.upload.jpg'
                        assert hashlib.sha256(expected.read_bytes()).hexdigest()==meta['uploaded_jpeg_sha256']
                        for ext,data in [('server_classes',cls),('classes',corrected),('scores',score)]:cv2.imwrite(str(dest/f'yaw{yaw}.{ext}.png'),data)
                        meta['only_unknown_prompt_assigned_to_4']=sorted(unknown)
                        (dest/f'yaw{yaw}.json').write_text(json.dumps(meta,indent=2))
                print('Yellow segment',center,'complete',flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'completed.json').write_text(json.dumps({'complete':True,'observations':340,'prompt':prompt,'resolution':512,'score_min':.5,'exact_jpeg_equal_to_baseline':True},indent=2))


if __name__=='__main__':main()
