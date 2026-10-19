"""Sequential photometric SAM3 trials using original calibrated camera views.

No map geometry, source texture, prompt, score threshold or resolution changes.
All server settings are restored. Predicted coverage is not ground-truth accuracy.
"""
import argparse,hashlib,json
from pathlib import Path
import cv2,numpy as np,yaml
from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
from sirius_navigation.theta_perspective import build_perspective_remap
from sirius_navigation.theta_semantic_temporal import yellow_tactile_seeds
from compare_theta_sam3_projection import overlay


def enhance(image,method):
    if method.startswith(('hi_','shape_','solo_','line_','crop_','shade_','strip_')):return enhance(image,method.split('_',1)[1])
    if method=='identity':return image.copy()
    if method.startswith('wb'):
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV);v=hsv[...,2].astype(np.float32)
        region=np.zeros(v.shape,bool);region[int(v.shape[0]*.6):int(v.shape[0]*.95)]=True
        neutral=region&(v>=8)&(v<=100)&(hsv[...,1]<120)
        if neutral.sum()<128:neutral=region&(v>=8)&(v<=120)
        balanced=image.copy()
        if neutral.sum()>=128:
            median=np.maximum(np.median(image[neutral],axis=0),1.)
            gains=np.clip(np.mean(median)/median,.65,1.8)
            shade=np.clip((110.-v)/70.,0,1)
            balanced=np.clip(np.rint(image*(1+shade[...,None]*(gains-1))),0,255).astype(np.uint8)
        return enhance(balanced,'gamma'+method[2:])
    if method.startswith('gamma'):
        exponent=float(method[5:])/100.;v=image.max(axis=2).astype(np.float32)
        target=255.*np.power(v/255.,exponent)
        gain=np.minimum(target/np.maximum(v,1),6.)
        return np.clip(np.rint(image.astype(np.float32)*gain[...,None]),0,255).astype(np.uint8)
    if method.startswith('clahe'):
        lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
        lab[...,0]=cv2.createCLAHE(clipLimit=float(method[5:]),tileGridSize=(8,8)).apply(lab[...,0])
        return cv2.cvtColor(lab,cv2.COLOR_LAB2BGR)
    raise ValueError(method)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--centers',default='2500,2650,2800,2950,2100,5100,900,3300');p.add_argument('--radius',type=int,default=0);p.add_argument('--methods',default='gamma65,clahe2');p.add_argument('--resolution',type=int,default=512);p.add_argument('--tactile-prompt',default='yellow tactile paving');p.add_argument('--target-only',action='store_true');a=p.parse_args()
    src=a.source;out=a.output;out.mkdir(parents=True,exist_ok=True);manifest=json.loads((src/'manifest.json').read_text());cal=yaml.safe_load((src/'calibration_used.yaml').read_text());centers=[int(c) for c in a.centers.split(',')];methods=a.methods.split(',');indices=sorted({c+d for c in centers for d in range(-a.radius,a.radius+1)});maps={};cv2.setNumThreads(2)
    prompt=(a.tactile_prompt if a.target_only else f'grass, {a.tactile_prompt}, roadway, sidewalk');config={'source':str(src.resolve()),'centers':centers,'radius':a.radius,'frames':indices,'methods':methods,'prompt':prompt,'resolution':a.resolution,'score_min':.5,'no_geometry_or_texture_changes':True,'gamma_gain_cap':6.,'clahe_tile_grid':[8,8]};(out/'config.json').write_text(json.dumps(config,indent=2));(out/('config_'+ '_'.join(methods)+'.json')).write_text(json.dumps(config,indent=2));session=Sam3ExperimentClient();counts=[]
    try:
        with session:
            (out/'server_before.json').write_text(json.dumps(session.before,indent=2));(out/'registry_before.json').write_text(json.dumps(session.registry_before,indent=2));unknown=set(prompt.split(', '))-set(session.registry_before['ids']);assert unknown in (set(),{a.tactile_prompt})
            session.configure(prompt,{x:.5 for x in prompt.split(', ')},threshold=1.,resolution=a.resolution)
            for index in indices:
                basecenter=min(manifest['centers'],key=lambda c:abs(c-index));base=src/'yellow'/f'seg{basecenter}'/f'frame{index}';dest=out/f'frame{index}';dest.mkdir(exist_ok=True);raw=cv2.imread(str(src/'inputs'/f'{index}.jpg'))
                for yaw in (0,180):
                    stem=f'yaw{yaw}.'
                    if yaw not in maps:maps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
                    image=cv2.remap(raw,*maps[yaw],cv2.INTER_LINEAR);ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90]);assert ok
                    assert jpeg.tobytes()==(base/(stem+'upload.jpg')).read_bytes()
                    (dest/(stem+'original.jpg')).write_bytes(jpeg.tobytes());bgr=cv2.imdecode(jpeg,cv2.IMREAD_COLOR)
                    ids=cv2.imread(str(base/(stem+'classes.png')),0);score=cv2.imread(str(base/(stem+'scores.png')),0)
                    for ext,data in [('baseline.classes.png',ids),('baseline.scores.png',score)]:cv2.imwrite(str(dest/(stem+ext)),data)
                    for method in methods:
                        target=dest/(stem+method+'.')
                        path=lambda ext:Path(str(target)+ext)
                        boosted=enhance(image,method)
                        if not path('json').exists():
                            cls,scores,meta=session.infer(boosted[240:] if method.startswith('crop_') else boosted);assigned=cls.copy()
                            if unknown:assigned[(cls==0)&(scores>0)]=4
                            if method.startswith('crop_'):
                                color=boosted.copy();color[240:]=cv2.imdecode(np.frombuffer(session.last_upload,np.uint8),cv2.IMREAD_COLOR)[:,:640];cv2.imwrite(str(path('color.png')),color)
                                cls=np.pad(cls,((240,0),(0,0)));assigned=np.pad(assigned,((240,0),(0,0)));scores=np.pad(scores,((240,0),(0,0)));meta['crop_bbox_in_full_view']=[0,240,640,480]
                            for ext,data in [('server_classes.png',cls),('classes.png',assigned),('scores.png',scores)]:assert cv2.imwrite(str(path(ext)),data)
                            path('upload.jpg').write_bytes(session.last_upload);meta['only_unknown_prompt_assigned_to_4']=sorted(unknown);meta['original_jpeg_sha256']=hashlib.sha256(jpeg.tobytes()).hexdigest();path('json').write_text(json.dumps(meta,indent=2))
                        cached=json.loads(path('json').read_text());assert cached['settings']['prompt']==prompt and cached['settings']['resolution']==a.resolution
                        assert cached['original_jpeg_sha256']==hashlib.sha256(jpeg.tobytes()).hexdigest()
                        ids=cv2.imread(str(path('classes.png')),0);scores=cv2.imread(str(path('scores.png')),0);decoded=cv2.imread(str(path('color.png') if path('color.png').exists() else path('upload.jpg')))
                        oldgate=yellow_tactile_seeds(bgr,ids,scores);adapted=yellow_tactile_seeds(decoded,ids,scores);visible=np.where(scores/255.>=.5,ids,0).astype(np.uint8)
                        cv2.imwrite(str(path('overlay.jpg')),overlay(bgr,visible));cv2.imwrite(str(path('old_gate.png')),(oldgate*255).astype(np.uint8));cv2.imwrite(str(path('adapted_gate.png')),(adapted*255).astype(np.uint8))
                        counts.append({'frame':index,'yaw':yaw,'method':method,'raw_tactile':int((visible==4).sum()),'original_image_gate':int(oldgate.sum()),'enhanced_image_gate':int(adapted.sum())})
                print('Shadow trial',index,methods,flush=True)
    finally:
        if hasattr(session,'restoration'):(out/'restoration.json').write_text(json.dumps(session.restoration,indent=2))
    (out/'counts.json').write_text(json.dumps(counts,indent=2));(out/('counts_'+ '_'.join(methods)+'.json')).write_text(json.dumps(counts,indent=2));(out/'completed.json').write_text(json.dumps({'complete':True,'frames':len(indices),'views':len(indices)*2,'methods':methods},indent=2));print("Completed",len(counts),"view/method observations",flush=True)


if __name__=='__main__':main()
