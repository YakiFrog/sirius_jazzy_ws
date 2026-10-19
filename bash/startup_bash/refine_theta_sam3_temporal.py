"""Compare raw, causal and bidirectional road/sidewalk temporal consistency."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np
from sirius_navigation.theta_semantic_temporal import Observation,align_observation,fuse_road_labels
from compare_theta_sam3_projection import overlay


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);p.add_argument('--ungated',action='store_true',help='Reproduce initial diagnostic without confidence-gain guard');a=p.parse_args();root=a.study
    tag='_ungated' if a.ungated else '';gain=-1. if a.ungated else .05
    manifest=json.loads((root/'manifest.json').read_text());metrics=[];cv2.setNumThreads(2)
    config={'neighbor_radius_frames':4,'min_support':3,'agreement':.75,'score_min':.5,'max_neighbor_age_sec':1.5,'photometric_limit_gray':30,'fb_pixel_limit':1.5,'vote_decay_seconds':2.,'confidence_gain':gain,'protected_ids':[0,1,2,3,4],'flow':'OpenCV DIS MEDIUM, forward/backward consistency','cv2_version':cv2.__version__}
    (root/f'temporal_config{tag}.json').write_text(json.dumps(config,indent=2))
    for center in manifest['centers']:
        for yaw in (0,180):
            observations=[]
            for index in range(center-8,center+9):
                folder=root/f'seg{center}'/f'frame{index}';observations.append(Observation(cv2.imread(str(folder/f'yaw{yaw}.upload.jpg')),cv2.imread(str(folder/f'yaw{yaw}.classes.png'),0),cv2.imread(str(folder/f'yaw{yaw}.scores.png'),0),manifest['frames'][str(index)]['stamp_ns']/1e9))
            for i,current in enumerate(observations):
                started=time.monotonic();previous=[];both=[];alignment=[]
                for j in range(max(0,i-4),min(len(observations),i+5)):
                    dt=abs(current.stamp-observations[j].stamp)
                    if i==j or dt>1.5:continue
                    ids,conf,quality=align_observation(current.image,observations[j]);quality*=np.exp(-dt/2.)
                    item=(ids,conf,quality);both.append(item)
                    if j<i:previous.append(item)
                    alignment.append({'source_index':center-8+j,'dt_sec':dt,'usable_fraction':float((quality>=.25).mean())})
                folder=root/f'seg{center}'/f'frame{center-8+i}'
                base=np.where(current.scores/255.>=.5,current.labels,0).astype(np.uint8)
                cv2.imwrite(str(folder/f'yaw{yaw}.baseline.overlay.jpg'),overlay(current.image,base))
                for name,neighbors in [('causal',previous),('bidirectional',both)]:
                    labels,scores,stats=fuse_road_labels(current.labels,current.scores,neighbors,confidence_gain=gain)
                    cv2.imwrite(str(folder/f'yaw{yaw}.{name}{tag}.classes.png'),labels);cv2.imwrite(str(folder/f'yaw{yaw}.{name}{tag}.scores.png'),scores)
                    visible=np.where(scores/255.>=.5,labels,0).astype(np.uint8)
                    cv2.imwrite(str(folder/f'yaw{yaw}.{name}{tag}.overlay.jpg'),overlay(current.image,visible))
                    assert np.array_equal(labels[np.isin(current.labels,[0,1,2,3,4])],current.labels[np.isin(current.labels,[0,1,2,3,4])])
                    stats.update({'segment':center,'frame':center-8+i,'yaw':yaw,'method':name,'alignment':alignment,'pair_processing_seconds':time.monotonic()-started});metrics.append(stats)
                if center-8+i==center:cv2.imwrite(str(folder/f'yaw{yaw}.source.jpg'),current.image)
        (root/f'temporal_metrics{tag}.json').write_text(json.dumps(metrics,indent=2));print('Refined',center,'complete',flush=True)
    (root/f'temporal_completed{tag}.json').write_text(json.dumps({'complete':True,'frames':len(manifest['frames']),'methods':['baseline','causal','bidirectional']},indent=2))


if __name__=='__main__':main()
