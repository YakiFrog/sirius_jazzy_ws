"""Evaluate tactile-gap restoration and build a synchronized phone comparison."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from sirius_navigation.theta_semantic_temporal import Observation,warp_with_flow,yellow_tactile_appearance

METHODS=('baseline','causal','bidirectional','unrestricted')


def paths(root,c,index,yaw,method):
    f=root/f'seg{c}'/f'frame{index}';stem=f'yaw{yaw}.'
    if method!='baseline':f=root/('tactile_unrestricted' if method=='unrestricted' else 'tactile')/f'seg{c}'/f'frame{index}';stem+=('bidirectional' if method=='unrestricted' else method)+'.'
    return f,stem


def load(root,c,index,yaw,method):
    f,stem=paths(root,c,index,yaw,method)
    ids=cv2.imread(str(f/(stem+'classes.png')),0);scores=cv2.imread(str(f/(stem+'scores.png')),0)
    if ids is None or scores is None:raise RuntimeError('Incomplete tactile study: '+str(f/stem))
    return ids,scores


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('study',type=Path);parser.add_argument('--html-only',action='store_true');a=parser.parse_args();root=a.study;out=root/'tactile'
    if a.html_only:
        build_html(out,json.loads((root/'manifest.json').read_text()),json.loads((out/'evaluation.json').read_text()),json.loads((out/'mask_counts.json').read_text()));return
    manifest=json.loads((root/'manifest.json').read_text());ref=Path(manifest['reference']);annotation_file=ref/'review_rois.json'
    if not annotation_file.exists():annotation_file=out/'review_rois.json'
    annotations=json.loads(annotation_file.read_text())
    rois=[r for r in annotations['rois'] if r['class_id'] in (0,4)];(out/'review_rois.json').write_text(json.dumps(annotations,indent=2))
    rows=[];cv2.setNumThreads(2);stats={m:{'flips':0,'stable_intersection':0,'temporal_union':0,'common_union':0,'valid_correspondences':0,'yellow_flips':0,'yellow_intersection':0,'yellow_union':0} for m in METHODS};cases=[]
    for c in manifest['centers']:
        for yaw in (0,180):
            previous=None
            for index in range(c-8,c+9):
                folder=root/f'seg{c}'/f'frame{index}';image=cv2.imread(str(folder/f'yaw{yaw}.upload.jpg'))
                masks={};labels={};scores={}
                for method in METHODS:
                    ids,sc=load(root,c,index,yaw,method);mask=(ids==4)&(sc/255.>=.5);masks[method]=mask;labels[method]=ids;scores[method]=sc
                    dest,stem=paths(root,c,index,yaw,method);cv2.imwrite(str(dest/(stem+'tactile.png')),mask.astype(np.uint8)*255)
                    if index==c:
                        for roi in rois:
                            if roi['segment']!=c or roi['yaw']!=yaw:continue
                            k=np.zeros(mask.shape,np.uint8);cv2.fillPoly(k,[np.array(roi['polygon'],np.int32)],1);k=k.astype(bool)
                            rows.append({'roi':roi['id'],'method':method,'kind':'positive' if roi['class_id']==4 else 'negative','pixels':int(k.sum()),'labelled_pixels':int((mask&k).sum())})
                cases.append({'segment':c,'frame':index,'yaw':yaw,'pixels':{m:int(mask.sum()) for m,mask in masks.items()}})
                if previous is not None:
                    engine=cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);pg=cv2.cvtColor(previous['image'],cv2.COLOR_BGR2GRAY)
                    backward=engine.calc(gray,pg,None);forward=engine.calc(pg,gray,None);warped={};valid=None
                    for method in METHODS:
                        ids,_,quality=warp_with_flow(image,Observation(previous['image'],previous['masks'][method].astype(np.uint8),np.full(gray.shape,255,np.uint8),0),backward,forward)
                        warped[method]=ids.astype(bool);valid=quality>=.4
                    union=np.zeros(gray.shape,bool)
                    for method in METHODS:union|=(masks[method]|warped[method])
                    domain=valid&union
                    for method in METHODS:
                        now=masks[method];old=warped[method];stats[method]['flips']+=int(((now!=old)&domain).sum());stats[method]['common_union']+=int(domain.sum());stats[method]['valid_correspondences']+=int(valid.sum())
                        stats[method]['stable_intersection']+=int((now&old&valid).sum());stats[method]['temporal_union']+=int(((now|old)&valid).sum())
                        yellow=valid&yellow_tactile_appearance(image)
                        stats[method]['yellow_flips']+=int(((now!=old)&yellow).sum());stats[method]['yellow_intersection']+=int((now&old&yellow).sum());stats[method]['yellow_union']+=int(((now|old)&yellow).sum())
                previous={'image':image,'masks':masks}
        print('Tactile evaluated',c,flush=True)
    roi_summary={}
    for method in METHODS:
        roi_summary[method]={}
        for kind in ('positive','negative'):
            r=[x for x in rows if x['method']==method and x['kind']==kind];n=sum(x['pixels'] for x in r)
            roi_summary[method][kind]={'roi_count':len(r),'pixels':n,'labelled_fraction':sum(x['labelled_pixels'] for x in r)/max(n,1)}
        stats[method]['flip_fraction_common_union']=stats[method]['flips']/max(1,stats[method]['common_union']);stats[method]['yellow_temporal_iou']=stats[method]['yellow_intersection']/max(1,stats[method]['yellow_union']);stats[method]['temporal_iou']=stats[method]['stable_intersection']/max(1,stats[method]['temporal_union'])
    summary={'roi_summary':roi_summary,'aligned_tactile_temporal_metrics':stats,'note':'Stability is not semantic accuracy. ROI labels are provisional, 6 tactile and10 normal pavement interiors. The unrestricted diagnostic can propagate false pavement labels.'}
    (out/'evaluation.json').write_text(json.dumps(summary,indent=2));(out/'roi_checks.json').write_text(json.dumps(rows,indent=2));(out/'mask_counts.json').write_text(json.dumps(cases,indent=2))
    build_html(out,manifest,summary,cases)


def build_html(out,manifest,summary,cases):
    compact={'centers':manifest['centers'],'summary':summary,'cases':cases}
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SAM3 点字ブロックのちらつき比較</title><style>body{font:16px system-ui;background:#101820;color:#edf2f7;margin:14px;line-height:1.6}h1{font-size:24px}select,input,button{font-size:16px;padding:7px;margin:3px;max-width:100%}a{color:#84d5ff}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}img{width:100%;display:block}figure{margin:0;min-width:0}.note{background:#243442;padding:12px}table{border-collapse:collapse;font-size:14px}td,th{border:1px solid #526274;padding:6px}#stats{overflow:auto}#source{max-width:640px}@media(max-width:800px){.grid{grid-template-columns:1fr}}#state{min-height:2em}</style>
<h1>点字ブロックの認識抜けを時系列で補う</h1><p>現行SAM3・解像度512・閾値0.5を維持。10区間・170フレーム・前後画像で比較。</p>
<div class="note">前後で認識した同じ点字ブロックを画像の動きに沿って追い、0.85秒以内の欠落を補完します。対応が不確かな場所、遮蔽や対応不良が疑われる場所、別クラスの芝生は除外します。元の認識を消さず、新たな補完だけを黄色い表面に限定しています。淡い色や別色の点字・長い欠落・最初から全く認識されない場所は残ります。</div>
<p><label>区間<select id="seg"></select></label><label>向き<select id="yaw"><option value="0">前方</option><option value="180">後方</option></select></label></p><p><button id="play">再生</button><input type="range" id="offset" min="-8" max="8" value="-3"><span id="frame"></span></p>
<p><label>比較方法<select id="method"><option value="bidirectional">前後で補完（黄色限定）</option><option value="causal">過去から補完（黄色限定）</option><option value="unrestricted">初期試験・色の条件なし</option></select></label><label>表示<select id="display"><option value="overlay">写真＋認識</option><option value="mask">点字マスクだけ</option></select></label></p>
<p id="state">読み込み中…</p><figure><figcaption>元画像</figcaption><img id="source"></figure><div class="grid"><figure><figcaption>現行・閾値0.5</figcaption><img id="base"></figure><figure><figcaption>時系列補完後</figcaption><img id="refined"></figure></div><p id="counts"></p><p>写真の黄色と認識色が似ているため、「点字マスクだけ」でも確認できます。画像は3枚がそろってから同時に更新します。</p><p><a href="../comparison.html">車道・歩道の試験に戻る</a> · <a href="README.md">手法・制約・再実行</a> · <a href="evaluation.json">評価JSON</a></p><h2>確認用の数値</h2><p>中心画像の点字6領域、通常舗装10領域は暫定の確認用アノテーションです。着色率は認識精度ではありません。時系列IoUも安定性の指標であり、誤認識が正しくなったことを意味しません。</p><div id="stats"></div>
<script>const D=DATA,$=id=>document.getElementById(id);D.centers.forEach(c=>$('seg').add(new Option(c,c)));$('seg').value=3300;$('display').value='mask';let token=0,playing=false,timer=null;
const imageCache=new Map();function ready(url){if(!imageCache.has(url)){imageCache.set(url,new Promise((ok,bad)=>{const im=new Image();im.onload=async()=>{try{if(im.decode)await im.decode();ok(im)}catch(e){bad(e)}};im.onerror=()=>{imageCache.delete(url);bad(new Error(url))};im.src=url}));while(imageCache.size>16)imageCache.delete(imageCache.keys().next().value)}return imageCache.get(url)}
async function update(){const mine=++token,c=Number($('seg').value),i=c+Number($('offset').value),y=Number($('yaw').value),m=$('method').value,mask=$('display').value==='mask';const raw=`../seg${c}/frame${i}/yaw${y}`,target=`${m==='unrestricted'?'../tactile_unrestricted/':'./'}seg${c}/frame${i}/yaw${y}.${m==='unrestricted'?'bidirectional':m}`;const urls=[raw+'.upload.jpg',raw+(mask?'.tactile.png':'.baseline.overlay.jpg'),target+(mask?'.tactile.png':'.overlay.jpg')];$('state').textContent='読み込み中…';try{const loaded=await Promise.all(urls.map(ready));if(mine!==token)return false;['source','base','refined'].forEach((id,k)=>{const old=$(id),next=loaded[k];next.id=id;if(old!==next)old.replaceWith(next)});$('frame').textContent=i+'（'+$('offset').value+'）';const r=D.cases.find(x=>x.segment===c&&x.frame===i&&x.yaw===y);$('counts').textContent=`点字ラベル：現行 ${r.pixels.baseline.toLocaleString()} → 補完後 ${r.pixels[m].toLocaleString()} 画素（面積は正しさではありません）`;$('state').textContent='表示中：'+i;history.replaceState(null,'',`#${c},${y},${i-c},${m},${mask?'mask':'overlay'}`);return true}catch(e){if(mine===token)$('state').textContent='画像を読み込めません。Tailscale接続を確認してください。';return false}}
function stop(){playing=false;clearTimeout(timer);$('play').textContent='再生'}async function tick(){if(!playing)return;$('offset').value=Number($('offset').value)>=8?-8:Number($('offset').value)+1;const ok=await update();if(!ok){stop();return}if(playing)timer=setTimeout(tick,600)}$('play').onclick=()=>{if(playing){stop();return}playing=true;$('play').textContent='停止';tick()};for(const id of ['seg','yaw','offset','method','display'])$(id).oninput=()=>{stop();update()};
if(location.hash){const [c,y,o,m,view]=location.hash.slice(1).split(',');if(D.centers.includes(Number(c))&&['0','180'].includes(y)&&Number(o)>=-8&&Number(o)<=8){$('seg').value=c;$('yaw').value=y;$('offset').value=o;if(['causal','bidirectional','unrestricted'].includes(m))$('method').value=m;if(view==='mask')$('display').value='mask'}}
let table='<table><tr><th>方法</th><th>点字6領域の着色率</th><th>通常舗装10領域の誤着色</th><th>全域の時系列IoU</th><th>黄色部分の時系列IoU</th></tr>';for(const m of ['baseline','causal','bidirectional','unrestricted']){const r=D.summary.roi_summary[m],t=D.summary.aligned_tactile_temporal_metrics[m],pct=x=>(x*100).toFixed(1)+'%';table+=`<tr><td>${m}</td><td>${pct(r.positive.labelled_fraction)}</td><td>${pct(r.negative.labelled_fraction)}</td><td>${pct(t.temporal_iou)}</td><td>${pct(t.yellow_temporal_iou)}</td></tr>`;}$('stats').innerHTML=table+'</table>';update();</script></html>'''
    (out/'comparison.html').write_text(html.replace('DATA',json.dumps(compact)));print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
