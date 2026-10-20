"""Recover grass candidates from connected, observed, grass-supported neighbors.

An observational vote fraction is not a calibrated grass probability. This
experiment keeps original classes separate from proposed grass additions and
never expands texture/geometry or fills unobserved space.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from render_theta_semantic_map import COLORS, frame


def expand_grass(labels, texture, votes, coverage, resolution, radius_m,
                 *, fraction_min=.3, grass_weight_min=.2):
    """Four-neighbor growth, requiring local evidence at every traversed cell."""
    assert labels.shape == texture.shape[:2] == votes.shape[:2] == coverage.shape
    observed=(texture[...,3]>0)&(coverage>0)
    total=votes.sum(axis=2)
    fraction=np.divide(votes[...,0],total,out=np.zeros(total.shape,np.float32),where=total>0)
    hsv=cv2.cvtColor(texture[...,:3],cv2.COLOR_BGR2HSV)
    color=(hsv[...,0]>=25)&(hsv[...,0]<=90)&(hsv[...,1]>=40)&(hsv[...,2]>=12)
    seeds=(labels==3)&observed
    eligible=observed&np.isin(labels,(0,5,6))&color&(fraction>=fraction_min)&(votes[...,0]>=grass_weight_min)
    reached=seeds.copy();steps=max(0,int(np.floor(radius_m/resolution+1e-7)))
    kernel=cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    for _ in range(steps):
        reached=(cv2.dilate(reached.astype(np.uint8),kernel)>0)&(seeds|eligible)
    added=reached&eligible
    proposed=labels.copy();proposed[added]=3
    assert not (added&~observed).any()
    assert np.array_equal(proposed[~np.isin(labels,(0,5,6))],labels[~np.isin(labels,(0,5,6))])
    return proposed,added,dict(steps=steps,maximum_four_neighbor_distance_m=steps*resolution,
        added_cells=int(added.sum()),added_area_m2=float(added.sum()*resolution**2),
        original_class_counts={str(k):int((added&(labels==k)).sum()) for k in (0,5,6)})


def overlay(rgb,labels):
    image=rgb.copy()
    for cid,color in COLORS.items():
        mask=labels==cid;image[mask]=np.rint(image[mask]*.4+np.asarray(color[::-1])*.6).astype(np.uint8)
    return image


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--radii',type=float,nargs='+',default=[.25,.5]);p.add_argument('--fractions',type=float,nargs='+',default=[.2,.3,.4]);a=p.parse_args()
    root=a.output;root.mkdir(parents=True,exist_ok=True)
    labels=cv2.imread(str(a.reference/'road_temporal.colored.pgm'),0);texture=cv2.imread(str(a.reference/'road_temporal.texture.png'),-1)
    meta=json.loads((a.reference/'manifest.json').read_text());res=meta['grid']['resolution'];evidence=np.load(a.reference/'map_evidence.npz')
    votes=evidence['temporal_votes'][::-1];coverage=evidence['sample_coverage'][::-1]
    rgb=texture[...,:3].copy();rgb[texture[...,3]==0]=248;before=overlay(rgb,labels)
    records={};proposals={}
    for radius in a.radii:
        for fraction in a.fractions:
            name=f'radius{radius}_fraction{fraction}'.replace('.', '_');target=root/name;target.mkdir(exist_ok=True)
            proposed,added,stats=expand_grass(labels,texture,votes,coverage,res,radius,fraction_min=fraction)
            cv2.imwrite(str(target/'classes.png'),proposed);cv2.imwrite(str(target/'grass_candidates.png'),added.astype(np.uint8)*255)
            records[name]=dict(radius_m=radius,grass_vote_fraction_min=fraction,grass_weight_min=.2,**stats)
            proposals[name]=(proposed,added)
    selected='radius0_25_fraction0_3'
    if selected not in proposals:selected=next(iter(proposals))
    proposed,added=proposals[selected]
    cv2.imwrite(str(root/'baseline.classes.png'),labels)
    cv2.imwrite(str(root/'selected.classes.png'),proposed);cv2.imwrite(str(root/'selected.grass_candidates.png'),added.astype(np.uint8)*255)
    # Candidate display is distinct from the existing confirmed class-3 layer.
    candidate_view=before.copy();candidate_view[added]=np.rint(rgb[added]*.3+np.asarray([90,245,150])*.7).astype(np.uint8)
    cv2.imwrite(str(root/'candidate_overlay.png'),candidate_view)
    whole=frame(candidate_view,'芝生の拡張候補（黄緑）','濃い緑：既存の芝生／黄緑：追加候補／未観測へは拡張しない',res,width=1600,show_legend=False)
    whole.save(root/'overview.jpg',quality=94,subsampling=0)
    n,component,sizes,centroids=cv2.connectedComponentsWithStats(added.astype(np.uint8),8)
    chosen=[];centers=[]
    for idx in np.argsort(sizes[1:,cv2.CC_STAT_AREA])[::-1]+1:
        cx,cy=centroids[idx]
        if any(np.hypot(cx-x,cy-y)*res<5 for x,y in centers):continue
        centers.append((cx,cy));chosen.append(int(idx))
        if len(chosen)==6:break
    crops=[]
    for rank,idx in enumerate(chosen,1):
        x,y,w,h,_=sizes[idx];pad=round(1.5/res);x0=max(0,x-pad);y0=max(0,y-pad);x1=min(labels.shape[1],x+w+pad);y1=min(labels.shape[0],y+h+pad)
        panes=[]
        for title,image in [('元のセマンティック',before),('芝生の追加候補を表示',candidate_view),('元の路面テクスチャ',rgb)]:
            panes.append(frame(image[y0:y1,x0:x1],title,'黄緑は候補／同じ範囲・縮尺',res,width=1000,show_legend=False))
        contact=Image.new('RGB',(1000,sum(im.height for im in panes)),'white');yy=0
        for im in panes:contact.paste(im,(0,yy));yy+=im.height
        contact.save(root/f'crop{rank}.jpg',quality=94,subsampling=0)
        crops.append(dict(name=f'crop{rank}',pixel_bbox=[int(x0),int(y0),int(x1),int(y1)],
                          world_center_m=[float(meta['grid']['origin'][0]+centroids[idx,0]*res),float(meta['grid']['origin'][1]+(labels.shape[0]-centroids[idx,1])*res)]))
    report=dict(reference=str(a.reference.resolve()),selected=selected,trials=records,crops=crops,
        status='candidate-only exploratory comparison; current map unchanged; target location not specified',
        input_sha256={name:hashlib.sha256((a.reference/name).read_bytes()).hexdigest() for name in ('road_temporal.colored.pgm','road_temporal.texture.png','map_evidence.npz')},
        settings=dict(hsv_hue_range=[25,90],hsv_saturation_min=40,hsv_value_min=12,connectivity=4,
                      required_support='observed RGB and native sample coverage; grass votes at every traversed cell',
                      protected_classes=[1,2,3,4],texture_unchanged=True,geometry_unchanged=True,
                      vote_fraction_is_calibrated_probability=False))
    (root/'evaluation.json').write_text(json.dumps(report,indent=2))
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>芝生の拡張候補</title><style>body{font:16px system-ui;margin:12px;line-height:1.65;background:#101820;color:#eff5f8}h1{font-size:24px}img{display:block;width:100%;max-width:1000px;margin:12px 0 28px}a{color:#8ed8ff}p{padding:12px;background:#253643}</style><h1>芝生の拡張候補</h1><p>濃い緑は既存の芝生、黄緑は追加候補です。芝生としての観測、既存芝生との接続、元画像の色を確認しています。未観測・LiDARで遮られた範囲へは広げません。現行地図は変更していません。具体的な不足箇所が未指定のため、これは候補の比較であり正解確認ではありません。</p>'''
    for crop in crops:html+=f'<h2>比較 {crop["name"][4:]}（上：元／中：候補／下：テクスチャ）</h2><a href="{crop["name"]}.jpg"><img src="{crop["name"]}.jpg" loading="lazy" alt="芝生候補の比較"></a>'
    html+='<h2>全体の候補</h2><a href="overview.jpg"><img src="overview.jpg" loading="lazy" alt="芝生候補の全体図"></a></html>'
    (root/'images.html').write_text(html);print(json.dumps({'selected':selected,'conditions':len(records),'result':records[selected]},indent=2))


if __name__=='__main__':main()
