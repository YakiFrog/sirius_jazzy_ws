"""Restore brief tactile-mask gaps in the saved dense SAM3 comparison."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np
from sirius_navigation.theta_semantic_temporal import Observation,align_observation,stabilize_tactile,yellow_tactile_appearance
from compare_theta_sam3_projection import overlay


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('study',type=Path);parser.add_argument('--appearance',choices=['yellow','yellow_loose','unrestricted'],default='yellow');a=parser.parse_args();p=a.study
    manifest=json.loads((p/'manifest.json').read_text());out=p/('tactile' if a.appearance=='yellow' else 'tactile_'+a.appearance);out.mkdir(exist_ok=True);cv2.setNumThreads(2)
    config={'appearance_mode':a.appearance,'yellow_hsv_hue':[15,32 if a.appearance=='yellow' else 40],'yellow_hsv_min_s':100 if a.appearance=='yellow' else 45,'minimum_fill_component_pixels':16 if a.appearance=='yellow' else 0,'yellow_hsv_min_v':35,'score_min':.5,'internal_resolution':512,'max_gap_sec':.85,'neighbor_radius_frames':3,'min_support':2,'min_alignment_reliability':.4,'min_mean_mask_score':.55,'mask_interior_erosion_px':1,'original_detections_unchanged':True,'no_recursive_feedback':True,'protected_classes':[1,2,3],'future_required_offline':True,'cv2_version':cv2.__version__}
    (out/'config.json').write_text(json.dumps(config,indent=2));metrics=[]
    for c in manifest['centers']:
        observations=[]
        for i in range(c-8,c+9):
            f=p/f'seg{c}'/f'frame{i}'
            observations.append({yaw:Observation(cv2.imread(str(f/f'yaw{yaw}.upload.jpg')),cv2.imread(str(f/f'yaw{yaw}.classes.png'),0),cv2.imread(str(f/f'yaw{yaw}.scores.png'),0),manifest['frames'][str(i)]['stamp_ns']/1e9) for yaw in (0,180)})
        for i,views in enumerate(observations):
            dest=out/f'seg{c}'/f'frame{c-8+i}';dest.mkdir(parents=True,exist_ok=True)
            for yaw,current in views.items():
                start=time.monotonic();aligned=[];matches=[]
                for j in range(max(0,i-3),min(len(observations),i+4)):
                    old=observations[j][yaw];dt=old.stamp-current.stamp
                    if i==j or abs(dt)>.85:continue
                    ids,score,reliability=align_observation(current.image,old);aligned.append((ids,score,reliability,dt));matches.append({'source_frame':c-8+j,'dt':dt,'usable_fraction':float((reliability>=.4).mean())})
                for method in ('causal','bidirectional'):
                    ids,scores,stats=stabilize_tactile(current.labels,current.scores,aligned,bidirectional=method=='bidirectional',appearance_mask=(yellow_tactile_appearance(current.image) if a.appearance=='yellow' else yellow_tactile_appearance(current.image,45,40,True) if a.appearance=='yellow_loose' else None),min_fill_area=16 if a.appearance=='yellow' else 0)
                    cv2.imwrite(str(dest/f'yaw{yaw}.{method}.classes.png'),ids);cv2.imwrite(str(dest/f'yaw{yaw}.{method}.scores.png'),scores)
                    visible=np.where(scores/255.>=.5,ids,0).astype(np.uint8);cv2.imwrite(str(dest/f'yaw{yaw}.{method}.overlay.jpg'),overlay(current.image,visible))
                    original=(current.labels==4)&(current.scores/255.>=.5);assert np.all(visible[original]==4)
                    keep=np.isin(current.labels,(1,2,3));assert np.array_equal(ids[keep],current.labels[keep])
                    stats.update({'segment':c,'frame':c-8+i,'yaw':yaw,'method':method,'neighbors':matches,'wall_seconds':time.monotonic()-start});metrics.append(stats)
        (out/'metrics.json').write_text(json.dumps(metrics,indent=2));print('Tactile',c,'complete',flush=True)
    (out/'completed.json').write_text(json.dumps({'complete':True,'frames':170,'views':340,'methods':['causal','bidirectional']},indent=2))


if __name__=='__main__':main()
