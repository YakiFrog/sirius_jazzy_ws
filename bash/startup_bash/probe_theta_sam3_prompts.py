"""Single-prompt diagnostics on the same center frames; no registry modification."""
from pathlib import Path
import argparse,json,time
import cv2,numpy as np
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
from compare_theta_sam3_projection import overlay

PROBES=[('tactile_plain','tactile paving',4),('tactile_line','line-type tactile paving',4),
        ('tactile_yellow','yellow tactile paving',4),('grass_plain','grass',3),('grass_lawn','lawn',3)]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path)
    p.add_argument('--wait',action='store_true');args=p.parse_args();study=args.study
    while not (study/'completed.json').exists():
        if not args.wait:raise RuntimeError('Primary study has not completed')
        time.sleep(2)
    manifest=json.loads((study/'manifest.json').read_text());out=study/'prompt_probe';out.mkdir(exist_ok=True)
    session=Sam3ExperimentClient(timeout=40.);results={}
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2))
            for center in manifest['centers']:
                results[str(center)]={}
                for yaw in (0,180):
                    image=cv2.imread(str(study/'reference'/f'seg{center}'/f'yaw{yaw}.jpg'))
                    results[str(center)][str(yaw)]={}
                    for name,prompt,cid in PROBES:
                        session.configure(prompt,{prompt:.5},threshold=.5,resolution=1008)
                        cls,score,meta=session.infer(image)
                        # A sole unregistered text prompt still has valid scored masks;
                        # assign its known experimental class locally, never on the server.
                        assigned=np.where(score>0,cid,0).astype(np.uint8)
                        dest=out/f'seg{center}'/f'yaw{yaw}';dest.mkdir(parents=True,exist_ok=True)
                        cv2.imwrite(str(dest/f'{name}.server_classes.png'),cls)
                        cv2.imwrite(str(dest/f'{name}.classes.png'),assigned)
                        cv2.imwrite(str(dest/f'{name}.scores.png'),score)
                        (dest/f'{name}.upload.jpg').write_bytes(session.last_upload)
                        meta.update({'prompt':prompt,'assigned_class_id':cid,'assignment':'client-side from a sole prompt score map'})
                        (dest/f'{name}.json').write_text(json.dumps(meta,indent=2))
                        for cut in (.8,.65,.5):
                            label=np.where(score/255.>=cut,assigned,0).astype(np.uint8)
                            cv2.imwrite(str(dest/f'{name}.{cut:.2f}.overlay.jpg'),overlay(image,label))
                        results[str(center)][str(yaw)][name]={'class_id':cid,'mask_scores':meta['class_scores'],
                            'pixels_at_threshold':{str(t):int((score/255.>=t).sum()) for t in (.8,.65,.5)}}
                (out/'metrics.json').write_text(json.dumps(results,indent=2));print('Prompt probe',center,'done',flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'completed.json').write_text(json.dumps({'complete':True,'centers':manifest['centers'],'probes':PROBES},indent=2))


if __name__=='__main__':main()
