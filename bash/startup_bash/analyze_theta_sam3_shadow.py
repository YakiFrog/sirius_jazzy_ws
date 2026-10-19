"""Validate shadow-only additions against the immutable full-recording baseline."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency,yellow_tactile_seeds,merge_tactile_tracks

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def baseline(root,manifest,index,yaw):
    c=min(manifest['centers'],key=lambda c:abs(c-index));f=root/'yellow'/f'seg{c}'/f'frame{index}';stem=f'yaw{yaw}.bidirectional.'
    return cv2.imread(str(f/(stem+'classes.png')),0),cv2.imread(str(f/(stem+'scores.png')),0)


def dark_components(image,mask,max_v=90):
    v=image.max(axis=2);n,cc,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8);keep=np.zeros(n,bool)
    for k in range(1,n):
        if stats[k,cv2.CC_STAT_AREA]>=16:keep[k]=np.median(v[cc==k])<=max_v
    return keep[cc]


def append_only(base,bs,extra,es):
    result=base.copy();score=bs.copy();take=(extra==4)&~np.isin(base,(1,2,3))
    score[take]=np.maximum(np.where(base[take]==4,bs[take],0),es[take]);result[take]=4
    np.testing.assert_array_equal(result[base==4],base[base==4]);assert not ((result!=base)&(result!=4)).any()
    return result,score


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);p.add_argument('--method',default='solo_wb45');a=p.parse_args();root=a.study;dense=root/'dense';config=json.loads((dense/'config.json').read_text());source=Path(config['source']);manifest=json.loads((source/'manifest.json').read_text());method=a.method;out=root/('processed' if method=='solo_wb45' else 'processed_'+method);out.mkdir(exist_ok=True);cv2.setNumThreads(2);rows=[]
    for center in config['centers']:
        indices=list(range(center-config['radius'],center+config['radius']+1))
        for yaw in (0,180):
            obs={};forward={};reverse={}
            for index in indices:
                folder=dense/f'frame{index}';stem=f'yaw{yaw}.';photo=cv2.imread(str(folder/(stem+'original.jpg')));light=cv2.imread(str(folder/(stem+method+'.upload.jpg')));ids=cv2.imread(str(folder/(stem+method+'.classes.png')),0);score=cv2.imread(str(folder/(stem+method+'.scores.png')),0);base,bs=baseline(source,manifest,index,yaw)
                mask=yellow_tactile_seeds(light,ids,score,yellow_fraction=.2);mask=dark_components(photo,mask)&~np.isin(base,(1,2,3));extra=np.where(mask,4,0).astype(np.uint8);sc=np.where(mask,score,0).astype(np.uint8)
                # Existing full-run tactile labels remain independent baseline
                # evidence; no other class from the extra inference is adopted.
                anchor=(base==4);extra[anchor]=4;sc[anchor]=np.maximum(sc[anchor],bs[anchor]);extra[np.isin(base,(1,2,3))]=base[np.isin(base,(1,2,3))]
                obs[index]=(photo,light,base,bs,extra,sc,manifest['frames'][str(index)]['stamp_ns']/1e9)
            for backward,target in [(False,forward),(True,reverse)]:
                tracker=CausalTactileConsistency(yellow_fraction=.2)
                for index in (indices[::-1] if backward else indices):
                    photo,light,base,bs,extra,sc,stamp=obs[index];target[index]=tracker.apply(light,extra,sc,-stamp if backward else stamp)
            for index in indices:
                photo,light,base,bs,extra,sc,stamp=obs[index];f,fs,_=forward[index];b,bs2,_=reverse[index];union,us=merge_tactile_tracks(base,f,fs,b,bs2);mask=dark_components(photo,union==4);union[~mask&~(base==4)]=0
                raw,rs=append_only(base,bs,extra,sc);tracked,ts=append_only(base,bs,union,us);dest=out/f'frame{index}';dest.mkdir(exist_ok=True);stats={}
                for name,ids,score in [('baseline',base,bs),('raw_add',raw,rs),('tracked',tracked,ts)]:
                    for ext,data in [('classes.png',ids),('scores.png',score)]:assert cv2.imwrite(str(dest/f'yaw{yaw}.{name}.{ext}'),data)
                    visible=(ids==4)&(score>=128);color=photo.copy();color[visible]=np.rint(color[visible]*.4+np.array([0,230,255])*.6).astype(np.uint8);cv2.imwrite(str(dest/f'yaw{yaw}.{name}.overlay.jpg'),color);stats[name]=int(visible.sum())
                rows.append({'center':center,'frame':index,'yaw':yaw,'pixels':stats,'added_pixels':int(((tracked==4)&(base!=4)).sum())})
        print('Shadow tracking',center,'complete',flush=True)
    (out/'mask_counts.json').write_text(json.dumps(rows,indent=2));rois=json.loads((root/'review_rois.json').read_text())['rois'];roi_rows=[]
    for roi in rois:
        if not (out/f'frame{roi["frame"]}').exists():continue
        mask=np.zeros((480,640),np.uint8);cv2.fillPoly(mask,[np.asarray(roi['polygon'],np.int32)],1);counts={}
        for name in ('baseline','raw_add','tracked'):
            f=out/f'frame{roi["frame"]}';ids=cv2.imread(str(f/f'yaw{roi["yaw"]}.{name}.classes.png'),0);score=cv2.imread(str(f/f'yaw{roi["yaw"]}.{name}.scores.png'),0);counts[name]=int(((ids==4)&(score>=128)&(mask>0)).sum())
        roi_rows.append({**roi,'area':int(mask.sum()),'counts':counts})
    summary={}
    for kind in ['shadow_positive','shadow_negative','control_negative','normal_positive']:
        subset=[q for q in roi_rows if q['kind']==kind];area=sum(q['area'] for q in subset);summary[kind]={'area':area,'methods':{m:sum(q['counts'][m] for q in subset)/max(area,1) for m in ['baseline','raw_add','tracked']}}
    controls=[q for q in rows if q['center'] in [2100,5100]];normal=[q for q in rows if q['center'] in [900,3300]]
    evaluation={'roi_summary':summary,'rois':roi_rows,'gray_control_views':len(controls),'normal_control_views':len(normal),'gray_control_added_pixels':sum(q['added_pixels'] for q in controls),'normal_control_added_pixels':sum(q['added_pixels'] for q in normal),'frames':len(config['frames']),'views':len(rows),'all_original_tactile_and_other_classes_preserved':True,'note':'Provisional interior ROIs, not full or expert ground truth. Additional detections only; normal inference labels preserved. No semantic accuracy claim from coverage alone.'};(root/'dense_evaluation.json').write_text(json.dumps(evaluation,indent=2));(root/('dense_evaluation_'+method+'.json')).write_text(json.dumps(evaluation,indent=2));print(json.dumps({k:v for k,v in evaluation.items() if k!='rois'},indent=2),flush=True)
    (out/'config.json').write_text(json.dumps({'auxiliary_method':method,'auxiliary_prompt':json.loads((dense/f'frame{config["frames"][0]}'/f'yaw0.{method}.json').read_text())['settings']['prompt'],'auxiliary_resolution':512,'score_min':.5,'minimum_yellow_fraction':.2,'minimum_saturation':45,'dark_component_median_original_max_v':90,'component_min_area':16,'protected_classes':[1,2,3],'original_labels_only_changed_to_4':True,'texture_or_geometry_changes':False},indent=2))
    font=ImageFont.truetype(FONT,21);links=[]
    for index,yaw in [(2500,0),(2500,180),(2650,0),(2800,0),(2950,180),(2100,0)]:
        if not (out/f'frame{index}').exists():continue
        sheet=Image.new('RGB',(1280,535),'white');draw=ImageDraw.Draw(sheet)
        for column,(name,title) in enumerate([('baseline','現在の地図生成ロジック'),('tracked','暗所の追加認識＋画像追跡')]):
            draw.text((column*640+10,8),f'{title} ｜ {index}/{yaw}',font=font,fill='#162d40');photo=Image.open(out/f'frame{index}'/f'yaw{yaw}.{name}.overlay.jpg');sheet.paste(photo,(column*640,50))
        filename=f'{method}_comparison_{index}_{yaw}.jpg';sheet.save(root/filename,quality=93);links.append((filename,f'フレーム{index}・方向{yaw}'))
    for center,yaw in [(2500,0),(2800,0)]:
        images=[]
        for index in range(center-8,center+9):
            sheet=Image.new('RGB',(960,400),'white');draw=ImageDraw.Draw(sheet)
            for column,name in enumerate(['baseline','tracked']):
                draw.text((column*480+8,4),('従来' if column==0 else '暗所補完')+f' {index}',font=font,fill='black');im=Image.open(out/f'frame{index}'/f'yaw{yaw}.{name}.overlay.jpg').resize((480,360));sheet.paste(im,(column*480,40))
            images.append(sheet)
        filename=f'{method}_shadow_{center}.gif';images[0].save(root/filename,save_all=True,append_images=images[1:],duration=300,loop=0);links.append((filename,f'区間{center}・連続17フレーム'))
    html='<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>日陰の点字認識の比較</title><style>body{font:17px system-ui;background:#11202b;color:white;line-height:1.7;margin:12px}img{display:block;width:100%;max-width:1280px}a{color:#8dd4ff}h1{font-size:26px}</style><h1>日陰の点字：順番に比較した結果</h1><p>左：現在の全区間生成ロジック。右：暗部・色かぶり補正した画像で、点字だけを別に追加認識。元画像と既存の分類は保持し、黄色と暗さを確認して追加しています。最も濃い日陰には未検出が残っています。</p>'
    for file,title in links:html+=f'<h2>{title}</h2><a href="{file}"><img src="{file}" loading="lazy"></a>'
    html+='<p><a href="dense_evaluation.json">数値と確認用領域</a> ／ <a href="'+out.name+'/config.json">処理条件</a></p>'
    (root/'images.html').write_text(html)


if __name__=='__main__':main()
