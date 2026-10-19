"""Common-view overlays and provisional ROI checks for the paired SAM3 study."""
from pathlib import Path
import argparse,json
import cv2,numpy as np
from compare_theta_sam3_projection import METHODS,THRESHOLDS,overlay,project_labels
from sirius_navigation.theta_sam3_evaluation import reference_rays,sample_view_maps

EVAL_METHODS=METHODS+[("perspective2_yellow_1008",1008,"p2",False)]


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('study',type=Path)
    args=parser.parse_args();study=args.study;manifest=json.loads((study/'manifest.json').read_text())
    annotations=json.loads((study/'review_rois.json').read_text())
    checks=[];probe_checks=[];map_cache={}
    for center in manifest['centers']:
        frame_dir=study/f'seg{center}'/f'frame{center}'
        for name,res,kind,targets in EVAL_METHODS:
            view_names=['yaw0','yaw180'] if kind=='p2' else ['yaw0','yaw90','yaw180','yaw270'] if kind in ('p4','tiles') else ['full']
            source=study/'candidate'/f'seg{center}' if name=='perspective2_yellow_1008' else frame_dir/name
            saved=[]
            for view in view_names:
                cls=cv2.imread(str(source/(view+'.classes.png')),0)
                score=cv2.imread(str(source/(view+'.scores.png')),0)
                if cls is None or score is None:raise RuntimeError('Missing inference: '+str(frame_dir/name/view))
                saved.append((view,cls,score))
            dest=study/'review'/f'seg{center}'/name;dest.mkdir(parents=True,exist_ok=True)
            for yaw in (0,90,180,270):
                image=cv2.imread(str(study/'reference'/f'seg{center}'/f'yaw{yaw}.jpg'))
                samples=[]
                for view,cls,score in saved:
                    model_yaw=int(view[3:]) if view!='full' else 0
                    key=(yaw,kind,model_yaw)
                    if key not in map_cache:
                        u,v,valid,wrap=sample_view_maps(reference_rays(yaw),kind,model_yaw)
                        map_cache[key]=(u.astype(np.float32),v.astype(np.float32),valid,wrap)
                    u,v,valid,wrap=map_cache[key];samples.append((cls,score,u,v,valid,wrap))
                for threshold in THRESHOLDS:
                    label,support=project_labels(samples,threshold,image.shape[:2])
                    cv2.imwrite(str(dest/f'yaw{yaw}_{threshold:.2f}.classes.png'),label)
                    cv2.imwrite(str(dest/f'yaw{yaw}_{threshold:.2f}.overlay.jpg'),overlay(image,label))
                    for roi in annotations['rois']:
                        if roi['segment']!=center or roi['yaw']!=yaw:continue
                        mask=np.zeros(label.shape,np.uint8);cv2.fillPoly(mask,[np.array(roi['polygon'],np.int32)],1);mask=mask.astype(bool)
                        ids=[roi['class_id']] if roi['class_id'] else [3,4]
                        for cid in ids:
                            checks.append({'roi':roi['id'],'segment':center,'yaw':yaw,'method':name,'threshold':threshold,
                                'kind':'positive' if roi['class_id'] else 'negative','class_id':cid,'pixels':int(mask.sum()),
                                'predicted_pixels':int(((label==cid)&mask).sum()),'supported_pixels':int((support&mask).sum())})
        # Single-prompt diagnostics use exactly the stored common camera views.
        for roi in annotations['rois']:
            if roi['segment']!=center or roi['yaw'] not in (0,180):continue
            ids=[roi['class_id']] if roi['class_id'] else [3,4]
            for probe,prompt,cid in json.loads((study/'prompt_probe/completed.json').read_text())['probes']:
                if cid not in ids:continue
                folder=study/'prompt_probe'/f'seg{center}'/f"yaw{roi['yaw']}"
                score=cv2.imread(str(folder/f'{probe}.scores.png'),0)
                mask=np.zeros(score.shape,np.uint8);cv2.fillPoly(mask,[np.array(roi['polygon'],np.int32)],1);mask=mask.astype(bool)
                for t in THRESHOLDS:
                    probe_checks.append({'roi':roi['id'],'segment':center,'probe':probe,'prompt':prompt,'threshold':t,
                        'kind':'positive' if roi['class_id'] else 'negative','class_id':cid,'pixels':int(mask.sum()),
                        'predicted_pixels':int(((score/255.>=t)&mask).sum())})
        print('Common-view review',center,'complete',flush=True)
    (study/'roi_checks.json').write_text(json.dumps(checks,indent=2))
    (study/'prompt_roi_checks.json').write_text(json.dumps(probe_checks,indent=2))
    aggregate={}
    for name,_,_,_ in EVAL_METHODS:
        aggregate[name]={}
        for t in THRESHOLDS:
            agg={}
            for cid in (3,4):
                bykind={}
                for kind in ('positive','negative'):
                    rows=[r for r in checks if r['method']==name and r['threshold']==t and r['class_id']==cid and r['kind']==kind]
                    denominator=sum(r['pixels'] for r in rows)
                    bykind[kind]={'roi_count':len(rows),'pixels':denominator,
                                  'labelled_fraction':sum(r['predicted_pixels'] for r in rows)/denominator if denominator else None}
                agg[str(cid)]=bykind
            aggregate[name][str(t)]=agg
    probe_agg={}
    for name,prompt,cid in json.loads((study/'prompt_probe/completed.json').read_text())['probes']:
        probe_agg[name]={}
        for t in THRESHOLDS:
            entry={}
            for kind in ('positive','negative'):
                rows=[r for r in probe_checks if r['probe']==name and r['threshold']==t and r['kind']==kind]
                n=sum(r['pixels'] for r in rows)
                entry[kind]={'roi_count':len(rows),'pixels':n,'labelled_fraction':sum(r['predicted_pixels'] for r in rows)/n if n else None}
            probe_agg[name][str(t)]=entry
    (study/'roi_summary.json').write_text(json.dumps(aggregate,indent=2))
    (study/'prompt_roi_summary.json').write_text(json.dumps(probe_agg,indent=2))
    print('REVIEW COMPLETE',flush=True)


if __name__=='__main__':main()
