"""Evaluate visible-surface tracking on the exact ten-interval yellow experiment."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency,Observation,warp_with_flow,merge_tactile_tracks
from compare_theta_sam3_projection import overlay

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
METHODS=('baseline','yellow','causal','bidirectional')


def load(root,center,index,yaw,method):
    folder=root/f'seg{center}'/f'frame{index}';stem=f'yaw{yaw}.'
    if method!='baseline':folder=root/'yellow'/f'seg{center}'/f'frame{index}'
    if method in ('causal','bidirectional'):stem+=method+'.'
    ids=cv2.imread(str(folder/(stem+'classes.png')),0);scores=cv2.imread(str(folder/(stem+'scores.png')),0)
    assert ids is not None and scores is not None
    return ids,scores


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);p.add_argument('--report-only',action='store_true');a=p.parse_args();root=a.study;out=root/'yellow';manifest=json.loads((root/'manifest.json').read_text());cv2.setNumThreads(2)
    saved=json.loads((out/'evaluation.json').read_text()) if a.report_only else None
    rows=saved['frames'] if saved else []
    for center in ([] if a.report_only else manifest['centers']):
        indices=list(range(center-8,center+9))
        for yaw in (0,180):
            sources={i:cv2.imread(str(root/f'seg{center}'/f'frame{i}'/f'yaw{yaw}.upload.jpg')) for i in indices}
            forward={};backward={}
            for reverse,target in [(False,forward),(True,backward)]:
                tracker=CausalTactileConsistency()
                for i in (indices[::-1] if reverse else indices):
                    ids,score=load(root,center,i,yaw,'yellow');stamp=manifest['frames'][str(i)]['stamp_ns']/1e9
                    target[i]=tracker.apply(sources[i],ids,score,-stamp if reverse else stamp)
            for i in indices:
                folder=out/f'seg{center}'/f'frame{i}';ids,scores=load(root,center,i,yaw,'yellow');f,fs,stats=forward[i];b,bs,_=backward[i]
                fused,ss=merge_tactile_tracks(ids,f,fs,b,bs)
                for method,cls,conf in [('causal',f,fs),('bidirectional',fused,ss)]:
                    cv2.imwrite(str(folder/f'yaw{yaw}.{method}.classes.png'),cls);cv2.imwrite(str(folder/f'yaw{yaw}.{method}.scores.png'),conf)
                counts={}
                for method in METHODS:
                    cls,conf=load(root,center,i,yaw,method);visible=np.where(conf/255.>=.5,cls,0).astype(np.uint8)
                    counts[method]=int((visible==4).sum())
                    cv2.imwrite(str(folder/f'yaw{yaw}.{method}.overlay.jpg'),overlay(sources[i],visible))
                    cv2.imwrite(str(folder/f'yaw{yaw}.{method}.tactile.png'),((visible==4)*255).astype(np.uint8))
                rows.append({'segment':center,'frame':i,'yaw':yaw,'tactile_pixels':counts,'tracking':stats})
        print('Tracked yellow',center,flush=True)
    # Reuse the same provisional central-frame ROIs from previous experiments.
    reference=Path(manifest['reference']);roi_file=reference/'review_rois.json'
    if not roi_file.exists():roi_file=out/'review_rois.json'
    rois=json.loads(roi_file.read_text());roi_rows=[]
    for roi in rois['rois']:
        if roi['class_id'] not in (0,4):continue
        mask=np.zeros((480,640),np.uint8);cv2.fillPoly(mask,[np.asarray(roi['polygon'],np.int32)],1);take=mask>0
        counts={}
        for method in METHODS:
            ids,score=load(root,roi['segment'],roi['frame'],roi['yaw'],method)
            counts[method]=int(((ids==4)&(score/255.>=.5)&take).sum())
        roi_rows.append({**roi,'pixels':int(take.sum()),'counts':counts})
    summary={}
    for kind in ('positive','negative'):
        subset=[r for r in roi_rows if (r['class_id']==0)==(kind=='negative')]
        summary[kind]={'roi_count':len(subset),'area':sum(r['pixels'] for r in subset),'methods':{m:sum(r['counts'][m] for r in subset) for m in METHODS}}
    # Same optical correspondences/domain for all methods, including gray false positives.
    consistency=saved['temporal_consistency'] if saved else {m:{'intersection':0,'union':0,'flip_pixels':0,'domain_pixels':0} for m in METHODS}
    for center in ([] if a.report_only else manifest['centers']):
        for yaw in (0,180):
            for i in range(center-7,center+9):
                current=cv2.imread(str(root/f'seg{center}'/f'frame{i}'/f'yaw{yaw}.upload.jpg'));previous=cv2.imread(str(root/f'seg{center}'/f'frame{i-1}'/f'yaw{yaw}.upload.jpg'))
                engine=cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM);cg=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY);pg=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY);back=engine.calc(cg,pg,None);front=engine.calc(pg,cg,None)
                for method in METHODS:
                    ids,score=load(root,center,i,yaw,method);old,oldscore=load(root,center,i-1,yaw,method)
                    warped,confidence,reliable=warp_with_flow(current,Observation(previous,old,oldscore,0),back,front)
                    good=reliable>=.4;now=(ids==4)&(score/255.>=.5);past=(warped==4)&(confidence>=.5);stats=consistency[method]
                    stats['intersection']+=int((now&past&good).sum());stats['union']+=int(((now|past)&good).sum());stats['flip_pixels']+=int(((now!=past)&good).sum());stats['domain_pixels']+=int(good.sum())
    for stats in consistency.values():stats['temporal_iou']=stats['intersection']/max(stats['union'],1)
    evaluation={'roi_summary':summary,'rois':roi_rows,'temporal_consistency':consistency,'frames':rows,'note':'ROI polygons are provisional non-expert visual checks. Temporal IoU is consistency, not accuracy. Same512 resolution, exact JPEGs, cutoff0.5.'}
    (out/'evaluation.json').write_text(json.dumps(evaluation,indent=2));(out/'review_rois.json').write_text(json.dumps(rois,indent=2))
    config={'prompt':'grass, yellow tactile paving, roadway, sidewalk','score_min':.5,'resolution':512,'seed_yellow_hue':[15,40],'seed_min_saturation':45,'seed_yellow_component_fraction':.35,'seed_min_component_pixels':16,'track_min_alignment_reliability':.4,'max_between_frame_gap_sec':1.5,'detector_miss_timeout_sec':None,'method':'Explicit visual track; each step requires fresh flow, appearance and photometric evidence. Original detector score retained; not counted as a fresh detector confirmation. Offline bidirectional = union of independently tracked directions. Gray tactile predictions become unknown, never assumed sidewalk.'}
    (out/'config.json').write_text(json.dumps(config,indent=2))
    # Phone-friendly exact-frame contact sheets and animations, no async viewer.
    font=ImageFont.truetype(FONT,22);links=[]
    for center,index,yaw in [(2100,2103,0),(3300,3297,0),(4500,4494,180),(4500,4507,180)]:
        image=cv2.imread(str(root/f'seg{center}'/f'frame{index}'/f'yaw{yaw}.upload.jpg'));sheet=Image.new('RGB',(1280,1040),'white');draw=ImageDraw.Draw(sheet)
        for col,(method,title) in enumerate([('baseline','従来：点字全般'),('bidirectional','変更後：黄色い点字＋追跡')]):
            draw.text((col*640+8,8),title,font=font,fill='black');sheet.paste(Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)),(col*640,40))
            rendered=cv2.imread(str(out/f'seg{center}'/f'frame{index}'/f'yaw{yaw}.{method}.overlay.jpg'));sheet.paste(Image.fromarray(cv2.cvtColor(rendered,cv2.COLOR_BGR2RGB)),(col*640,560))
            draw.text((col*640+8,526),f'frame {index} ／ yaw {yaw}',font=font,fill='black')
        name=f'example_{index}_{yaw}.jpg';sheet.save(out/name,quality=93);links.append((name,f'区間{center}・フレーム{index}'))
    for center,yaw in [(3300,0),(4500,180)]:
        animation=[]
        for index in range(center-8,center+9):
            sheet=Image.new('RGB',(960,400),'white');draw=ImageDraw.Draw(sheet)
            for col,method in enumerate(('baseline','bidirectional')):
                title='従来' if col==0 else '黄色い点字＋追跡';draw.text((col*480+8,5),f'{title}：{index}',font=font,fill='black')
                im=Image.open(out/f'seg{center}'/f'frame{index}'/f'yaw{yaw}.{method}.overlay.jpg').resize((480,360));sheet.paste(im,(col*480,40))
            animation.append(sheet)
        name=f'seg{center}_tracking.gif';animation[0].save(out/name,save_all=True,append_images=animation[1:],duration=300,loop=0);links.append((name,f'区間{center}・時系列'))
    html='<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>黄色い点字ブロック</title><style>body{font:17px system-ui;line-height:1.7;background:#11202b;color:white;margin:12px}img{width:100%;max-width:1280px;display:block}a{color:#8dd4ff}</style><h1>黄色い点字ブロックとして認識</h1><p>同じ入力画像・解像度512・しきい値0.5。左：従来、右：黄色の指定＋画像追跡。10区間170フレームを再評価。数フレーム認識が抜けても、画像上で対応が確認できる間は維持します。遮蔽・画角外・対応不良では解除します。</p>'
    for name,title in links:html+=f'<h2>{title}</h2><a href="{name}"><img src="{name}" loading="lazy"></a>'
    html+='<p>認識誤りは残り得ます。<a href="evaluation.json">評価の数値と条件</a>・<a href="config.json">補正条件</a></p>'
    (out/'images.html').write_text(html);print(json.dumps({'roi_summary':summary,'consistency':consistency},indent=2),flush=True)


if __name__=='__main__':main()
