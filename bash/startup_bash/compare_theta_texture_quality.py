"""Compare general texture algorithms on multiple fixed windows and holdouts.

Both scatter- and direct-sampled holdout references are reported to expose
reference-model bias. Neither is surveyed ground truth or a geometry guarantee.
"""
import argparse
import json
import warnings
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from theta_texture_quality import VARIANTS,TextureFusion,feed_variant
from render_theta_semantic_map import frame


def load_packet(path):
    with np.load(path) as a:return {source:{k[len(source)+1:]:a[k] for k in a.files if k.startswith(source+'_')} for source in ('native','direct')}


def reference(frames,source,shape,index_map):
    stack=np.full((len(frames),*shape,3),np.nan,np.float32)
    for j,(_,packet) in enumerate(frames):
        p=packet[source];idx=index_map(p['index']);valid=(idx>=0)&(p['q']>=.12)
        stack[j].reshape(-1,3)[idx[valid]]=p['bgr'][valid]
    support=np.isfinite(stack[...,0]).sum(0)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning)
        median=np.nanmedian(stack,axis=0);mad=np.nanmedian(np.abs(stack-median),axis=0).mean(axis=2)
    return np.nan_to_num(median).clip(0,255).astype(np.uint8),(support>=3)&(mad<=12),(support>=3)&(mad<=35)


def edge_distance(predicted,ref,mask,res):
    inside=cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8))>0
    first=(cv2.Canny(predicted,50,120)>0)&inside;second=(cv2.Canny(ref,50,120)>0)&inside
    if first.sum()<10 or second.sum()<10:return None
    d1=cv2.distanceTransform((~first).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    d2=cv2.distanceTransform((~second).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
    return float((np.minimum(d1[second],10).mean()+np.minimum(d2[first],10).mean())*.5*res)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('observations',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--reuse-existing',action='store_true',help='Explicitly reuse matching previously generated trial rasters');a=p.parse_args()
    cfg=json.loads((a.observations/'manifest.json').read_text());meta=json.loads((Path(cfg['reference'])/'manifest.json').read_text());grid=meta['grid'];res=grid['resolution'];width=grid['width'];height=grid['height'];origin=np.asarray(grid['origin']);out=a.output;out.mkdir(parents=True,exist_ok=True)
    details={f['index']:f for f in cfg['frames']};results={};viewports={}
    for center in cfg['centers']:
        ids=sorted(i for i in details if abs(i-center)<=cfg['half_window']);packets=[(i,load_packet(a.observations/f'frame{i}.npz')) for i in ids]
        poses=np.asarray([details[i]['pose'][:2] for i in ids]);lo=np.maximum(np.floor((poses.min(0)-5-origin)/res).astype(int),0);hi=np.minimum(np.ceil((poses.max(0)+5-origin)/res).astype(int),[width,height]);w,h=(hi-lo).tolist();shape=(h,w)
        def index_map(index):
            x=index%width-lo[0];y=index//width-lo[1];valid=(x>=0)&(x<w)&(y>=0)&(y<h)
            return np.where(valid,y*w+x,-1)
        held=[f for j,f in enumerate(packets) if j%5==0];train=[f for j,f in enumerate(packets) if j%5!=0]
        references={source:reference(held,source,shape,index_map) for source in ('native','direct')}
        predicted={};observed={};segment=out/f'seg{center}';segment.mkdir(exist_ok=True)
        for name,variant in VARIANTS.items():
            old=cv2.imread(str(segment/(name+'.png')),-1) if a.reuse_existing and (segment/(name+'.png')).exists() else None
            if old is not None:
                assert old.shape==(h,w,4);predicted[name]=old[::-1,:,:3].copy();observed[name]=old[::-1,:,3]>0
            else:
                fusion=TextureFusion(w*h,variant['mode'],variant['relative'],variant.get('tolerance',25))
                for _,packet in train:feed_variant(fusion,packet,variant,index_map)
                image,seen=fusion.result();predicted[name]=image.reshape(h,w,3);observed[name]=seen.reshape(h,w)
            cv2.imwrite(str(segment/(name+'.png')),np.dstack((predicted[name],observed[name].astype(np.uint8)*255))[::-1])
        common=np.logical_and.reduce(list(observed.values()));rows={}
        for name,image in predicted.items():
            row={'observed_cells':int(observed[name].sum()),'reference':{}}
            for source,(ref,stable,edge_stable) in references.items():
                mask=common&stable;error=np.abs(image.astype(np.float32)-ref).mean(2)
                row['reference'][source]={'evaluation_cells':int(mask.sum()),'mean_rgb_error':float(error[mask].mean()) if mask.any() else None,
                    'edge_distance_m':edge_distance(image,ref,common&edge_stable,res)}
            rows[name]=row
        results[str(center)]=rows
        # Center-frame diagnostics show sampling before temporal fusion.
        for source in ('native','direct'):
            packet=dict(packets)[center][source];idx=index_map(packet['index']);valid=idx>=0
            single=np.full((h*w,3),248,np.uint8);single[idx[valid]]=np.clip(packet['bgr'][valid],0,255).astype(np.uint8)
            cv2.imwrite(str(segment/('single_'+source+'.png')),single.reshape(h,w,3)[::-1])
            ref,stable,_=references[source];cv2.imwrite(str(segment/('holdout_'+source+'.png')),np.dstack((ref,stable.astype(np.uint8)*255))[::-1])
        midpoint=np.asarray(details[center]['pose'][:2]);cx,cy=np.rint((midpoint-origin)/res).astype(int)-lo;half=round(5/res)
        x0=max(0,cx-half);x1=min(w,cx+half);y0=max(0,cy-half);y1=min(h,cy+half)
        panels=[]
        titles={'scatter_ema':'従来の段階投影・合成','scatter_protected':'段階投影＋良質観測を優先','scatter_protected_lens':'段階投影＋距離とレンズ角を考慮','direct_ema':'セル中心へ直接投影','direct_protected_no_lens':'直接投影＋良質観測を優先','direct_protected':'直接投影＋距離とレンズ角を考慮','direct_best':'直接投影＋最良観測を選択','direct_robust065_25':'直接投影＋整合性合成 0.65/25','direct_robust080_25':'直接投影＋整合性合成 0.80/25','direct_robust065_40':'直接投影＋整合性合成 0.65/40'}
        for name in VARIANTS:
            image=predicted[name].copy();image[~observed[name]]=248
            pane=frame(image[y0:y1,x0:x1][::-1],titles[name],f'区間{center} ｜ 同じ範囲・縮尺',res,width=800,show_legend=False);pane.save(segment/(name+'.jpg'),quality=94,subsampling=0);panels.append(pane)
        contact=Image.new('RGB',(1600,max(p.height for p in panels)*int(np.ceil(len(panels)/2))),'white')
        for j,pane in enumerate(panels):contact.paste(pane,((j%2)*800,(j//2)*pane.height))
        contact.save(segment/'comparison.jpg',quality=93,subsampling=0)
        viewports[str(center)]=dict(global_bottom_bbox=[*map(int,lo),*map(int,hi)],crop=list(map(int,[x0,y0,x1,y1])),train_indices=[i for i,_ in train],heldout_indices=[i for i,_ in held])
        (out/'metrics.json').write_text(json.dumps(results,indent=2));print('Compared segment',center,flush=True)
    aggregate={}
    for name in VARIANTS:
        aggregate[name]={}
        for source in ('native','direct'):
            rows=[r[name]['reference'][source] for r in results.values()];n=sum(r['evaluation_cells'] for r in rows)
            aggregate[name][source]={'mean_rgb_error':sum(r['mean_rgb_error']*r['evaluation_cells'] for r in rows)/max(n,1),
                'mean_segment_edge_distance_m':float(np.mean([r['edge_distance_m'] for r in rows if r['edge_distance_m'] is not None])), 'evaluation_cells':n}
    (out/'aggregate.json').write_text(json.dumps(aggregate,indent=2));(out/'viewports.json').write_text(json.dumps(viewports,indent=2));(out/'variants.json').write_text(json.dumps(VARIANTS,indent=2));print(json.dumps(aggregate,indent=2))


if __name__=='__main__':main()
