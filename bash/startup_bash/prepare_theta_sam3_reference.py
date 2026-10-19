"""Rebuild common camera views and display provisional polygons for a saved study.

Pass --rois with an existing annotation JSON; this does not generate ground truth.
"""
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np,yaml
from sirius_navigation.theta_perspective import build_perspective_remap


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);p.add_argument('--rois',type=Path)
    a=p.parse_args();s=a.study
    if a.rois and a.rois.resolve()!=(s/'review_rois.json').resolve():shutil.copy2(a.rois,s/'review_rois.json')
    rois=json.loads((s/'review_rois.json').read_text())['rois']
    manifest=json.loads((s/'manifest.json').read_text());cal=yaml.safe_load((s/'calibration_used.yaml').read_text())
    maps={};overview=[]
    for center in manifest['centers']:
        raw=cv2.imread(str(s/'inputs'/f'frame_{center:04d}.jpg'));dest=s/'reference'/f'seg{center}';dest.mkdir(parents=True,exist_ok=True)
        panels=[]
        for yaw in (0,90,180,270):
            if yaw not in maps:maps[yaw]=build_perspective_remap(cal,yaw,raw.shape[:2],(640,480),120)
            image=cv2.remap(raw,*maps[yaw],cv2.INTER_LINEAR);cv2.imwrite(str(dest/f'yaw{yaw}.jpg'),image)
            annotated=image.copy()
            for r in rois:
                if r['segment']==center and r['yaw']==yaw:
                    color={3:(0,255,0),4:(0,255,255),0:(255,0,255)}[r['class_id']]
                    cv2.polylines(annotated,[np.array(r['polygon'],np.int32)],True,color,2)
            cv2.imwrite(str(dest/f'yaw{yaw}_rois.jpg'),annotated)
            cv2.putText(image,f'{center} yaw {yaw}',(10,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2);panels.append(image)
        contact=np.concatenate([np.concatenate(panels[:2],axis=1),np.concatenate(panels[2:],axis=1)],axis=0)
        cv2.imwrite(str(dest/'sources.jpg'),contact);overview.append(cv2.resize(contact,(640,480)))
    cv2.imwrite(str(s/'reference/source_overview.jpg'),np.concatenate(overview,axis=0))


if __name__=='__main__':main()
