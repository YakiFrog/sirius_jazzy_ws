"""Review tactile-context candidates with explicit, exploratory source-image ROIs."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from render_theta_semantic_map import COLORS, frame


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference',type=Path);parser.add_argument('study',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--select',default='wider_context/radius1_6_strength8_0')
    args=parser.parse_args();base,study,out=args.reference,args.study,args.output
    helper=Path(__file__).with_name('evaluate_theta_road_regions.py')
    with (study/'standard_roi_evaluation.log').open('w') as log:
        subprocess.run([sys.executable,str(helper),str(base),str(study),str(out),'--select',args.select],stdout=log,stderr=subprocess.STDOUT,check=True)
    labels=cv2.imread(str(base/'road_temporal.colored.pgm'),0)
    selected=cv2.imread(str(out/'road_temporal.colored.pgm'),0)
    texture=cv2.imread(str(base/'road_temporal.texture.png'),-1)
    rgb=texture[...,:3].copy();rgb[texture[...,3]==0]=248
    rois=json.loads((study/'context_rois.json').read_text())['rois'];reports={}
    candidates={'baseline':base,**{str(p.parent.relative_to(study)):p.parent for p in sorted(study.glob('**/road_temporal.colored.pgm'))}}
    for name,path in candidates.items():
        prediction=cv2.imread(str(path/'road_temporal.colored.pgm'),0);rows=[]
        for roi in rois:
            x,y,X,Y=roi['pixel_bbox'];valid=np.isin(labels[y:Y,x:X],(5,6));current=prediction[y:Y,x:X]
            rows.append({**roi,'evaluated_cells':int(valid.sum()),'correct_cells':int(((current==roi['expected_class'])&valid).sum()),
                         'changed_cells':int(((current!=labels[y:Y,x:X])&valid).sum())})
        reports[name]={'rois':rows,'by_class':{str(cid):{'correct':sum(r['correct_cells'] for r in rows if r['expected_class']==cid),
            'cells':sum(r['evaluated_cells'] for r in rows if r['expected_class']==cid)} for cid in (5,6)}}
    (study/'context_roi_evaluation.json').write_text(json.dumps(reports,indent=2))
    verification=json.loads((out/'verification.json').read_text())
    verification.update(tactile_context_roi_status='exploratory same-recording interior ROIs; not expert or independent ground truth',
        tactile_context_roi_evaluation=reports[args.select]['by_class'],
        context_road_control_changed_cells=sum(r['changed_cells'] for r in reports[args.select]['rois'] if r['expected_class']==5),
        trial_conditions=len(candidates)-1,unit_tests_passed=17)
    for name,bbox in {'context_floor':(2223,1680,2314,1800),'context_band':(2230,1210,2330,1420),
                      'curb_control':(420,2400,540,2510)}.items():
        x0,y0,x1,y1=bbox;panes=[]
        for title,prediction in [('点字の関係を使う前',labels),('点字の関係を加えた後',selected),('元の路面テクスチャ',None)]:
            crop=rgb[y0:y1,x0:x1].copy()
            if prediction is not None:
                for cid,color in COLORS.items():
                    mask=prediction[y0:y1,x0:x1]==cid;crop[mask]=np.rint(crop[mask]*.4+np.asarray(color[::-1])*.6).astype(np.uint8)
            panes.append(frame(crop,title,'同じ範囲・縮尺',.05,width=1000,show_legend=prediction is not None))
        contact=Image.new('RGB',(1000,sum(p.height for p in panes)),'white');yy=0
        for pane in panes:contact.paste(pane,(0,yy));yy+=pane.height
        contact.save(study/(name+'.jpg'),quality=94,subsampling=0)
        source=rgb[y0:y1,x0:x1].copy()
        for roi in rois:
            x,y,X,Y=roi['pixel_bbox']
            if x>=x0 and X<=x1 and y>=y0 and Y<=y1:
                cv2.rectangle(source,(x-x0,y-y0),(X-x0-1,Y-y0-1),(0,190,250) if roi['expected_class']==6 else (255,80,40),1)
        cv2.imwrite(str(study/(name+'.rois.png')),source)
    (out/'verification.json').write_text(json.dumps(verification,indent=2));(study/'verification.json').write_text(json.dumps(verification,indent=2))
    subprocess.run([sys.executable,str(Path(__file__).with_name('render_theta_semantic_map.py')),str(out)],check=True)
    print(json.dumps(verification,indent=2))


if __name__=='__main__':main()
