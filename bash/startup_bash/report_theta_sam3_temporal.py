"""Evaluate provisional road ROIs and create a phone-friendly temporal comparison."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from sirius_navigation.theta_semantic_temporal import Observation,warp_with_flow

METHODS=['baseline','causal','bidirectional','bidirectional_ungated']


def load(folder,yaw,method):
    stem=f'yaw{yaw}.'+('' if method=='baseline' else method+'.')
    return cv2.imread(str(folder/(stem+'classes.png')),0),cv2.imread(str(folder/(stem+'scores.png')),0)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('study',type=Path);a=parser.parse_args();p=a.study
    m=json.loads((p/'manifest.json').read_text());rois=json.loads((p/'road_review_rois.json').read_text())['rois'];roi_rows=[]
    for r in rois:
        d=p/f"seg{r['segment']}"/f"frame{r['segment']}"
        for method in METHODS:
            ids,scores=load(d,r['yaw'],method);ids=np.where(scores/255.>=.5,ids,0)
            mask=np.zeros(ids.shape,np.uint8);cv2.fillPoly(mask,[np.array(r['polygon'],np.int32)],1);keep=mask.astype(bool)
            roi_rows.append({'roi':r['id'],'class_id':r['class_id'],'method':method,'pixels':int(keep.sum()),'correct':int((ids[keep]==r['class_id']).sum()),'opposite':int((ids[keep]==11-r['class_id']).sum()),'unknown_or_other':int((~np.isin(ids[keep],(5,6))).sum())})
    flip={method:{'pixels':0,'flips':0} for method in METHODS};cv2.setNumThreads(2)
    for c in m['centers']:
        for yaw in (0,180):
            for index in range(c-7,c+9):
                d=p/f'seg{c}'/f'frame{index}';old=p/f'seg{c}'/f'frame{index-1}'
                image=cv2.imread(str(d/f'yaw{yaw}.upload.jpg'));previous=cv2.imread(str(old/f'yaw{yaw}.upload.jpg'))
                engine=cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);oldgray=cv2.cvtColor(previous,cv2.COLOR_BGR2GRAY)
                backward=engine.calc(gray,oldgray,None);forward=engine.calc(oldgray,gray,None)
                raw,score=load(d,yaw,'baseline');prev,ps=load(old,yaw,'baseline')
                aligned,conf,quality=warp_with_flow(image,Observation(previous,prev,ps,0),backward,forward)
                eligible=np.isin(raw,(5,6))&(score/255.>=.5)&np.isin(aligned,(5,6))&(conf>=.5)&(quality>=.25)
                for method in METHODS:
                    now,_=load(d,yaw,method);before,bs=load(old,yaw,method)
                    warped,_,_=warp_with_flow(image,Observation(previous,before,bs,0),backward,forward)
                    flip[method]['pixels']+=int(eligible.sum());flip[method]['flips']+=int(((now!=warped)&eligible).sum())
        print('Evaluated',c,flush=True)
    for value in flip.values():value['flip_fraction']=value['flips']/max(1,value['pixels'])
    roi_summary={}
    for method in METHODS:
        rows=[r for r in roi_rows if r['method']==method];n=sum(r['pixels'] for r in rows)
        roi_summary[method]={'pixels':n,'correct_fraction':sum(r['correct'] for r in rows)/n,'opposite_fraction':sum(r['opposite'] for r in rows)/n,'unknown_or_other_fraction':sum(r['unknown_or_other'] for r in rows)/n}
    metrics=json.loads((p/'temporal_metrics.json').read_text());oldmetrics=json.loads((p/'temporal_metrics_ungated.json').read_text())
    summary={'roi_count':len(rois),'annotated_centers':7,'unannotated_ambiguous_centers':[300,5100,5650],'roi_summary':roi_summary,'aligned_flip_rates':flip,'note':'Provisional manually chosen ROI interiors. Flip reduction is stability, not correctness; no whole-image ground truth.'}
    (p/'road_roi_checks.json').write_text(json.dumps(roi_rows,indent=2));(p/'evaluation.json').write_text(json.dumps(summary,indent=2))
    data=json.dumps({'centers':m['centers'],'summary':summary,'metrics':metrics,'ungated':oldmetrics})
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SAM3 時系列比較</title><style>body{font:16px system-ui;background:#101820;color:#edf2f7;margin:16px;line-height:1.6}h1{font-size:24px}select,input,button{font-size:16px;padding:8px;margin:4px}a{color:#84d5ff}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}img{width:100%}figure{margin:0}.note{padding:12px;background:#243442}#stats{overflow:auto}table{border-collapse:collapse}th,td{padding:7px;border:1px solid #526274}#source{max-width:640px}@media(max-width:800px){.grid{grid-template-columns:1fr}body{margin:10px}}</style>
<p><a href="tactile/comparison.html">点字ブロックのちらつき比較へ</a></p><h1>SAM3 時系列整合：現行モデル・閾値0.5</h1><p>10区間 × 17連続フレーム × 前後。内部解像度512・既存プロンプトを維持。</p>
<div class="note">位置合わせ・遮蔽除外・複数観測の一致・信頼度差を条件に、車道5／歩道6だけを補正。点字・芝生・未知ラベルは変更しません。<br>重要：今回の中央画像19確認領域では、最終補正による正解率の改善は確認できませんでした。周辺フレームでの局所的な変更はありますが、持続する誤認識は時系列多数決だけでは直せません。初期版には誤認識の拡散例があり、信頼度の条件を追加しました。</div>
<p><label>区間<select id="seg"></select></label><label>向き<select id="yaw"><option value="0">前方</option><option value="180">後方</option></select></label></p>
<p><button id="play">再生</button><label>フレーム <input type="range" id="offset" min="-8" max="8" value="0"></label><span id="frame"></span></p>
<figure><figcaption>元画像</figcaption><img id="source"></figure>
<div class="grid"><figure><figcaption>現行・閾値0.5</figcaption><img id="base"></figure><figure><label>比較<select id="method"><option value="bidirectional">前後の時系列補正（最終版）</option><option value="causal">過去のみ（ROSノード試験用）</option><option value="bidirectional_ungated">初期版・信頼度差の条件なし</option></select></label><img id="refined"></figure></div><p id="counts"></p><p>黄：点字、緑：芝、青：車道、灰：歩道。灰色の歩道は元画像と見分けにくい点に注意してください。</p><p><a id="roi">中央画像の確認領域</a> · <a href="README.md">手法・論文・再実行方法</a> · <a href="evaluation.json">評価JSON</a> · <a href="../comparison.html">前回の投影比較</a></p>
<h2>評価の範囲</h2><p>画像だけでは車道／歩道の定義が曖昧な広場3区間はROI評価から除外。残る7区間19領域を暫定で付与。全画像の正解マスクではなく、モデル間の一般的な性能順位は示しません。時系列の入れ替わり率が下がっても、意味的な誤りが直った証拠にはなりません。</p><div id="stats"></div>
<script>const D=DATA;const $=id=>document.getElementById(id);D.centers.forEach(c=>$('seg').add(new Option(c,c)));$('seg').value=3300;let timer=null;
function update(){const c=Number($('seg').value),i=c+Number($('offset').value),y=Number($('yaw').value),method=$('method').value,path=`seg${c}/frame${i}/yaw${y}`;$('frame').textContent=i+'（中央から '+$('offset').value+'）';$('source').src=path+'.upload.jpg';$('base').src=path+'.baseline.overlay.jpg';$('refined').src=path+'.'+method+'.overlay.jpg';$('roi').href=`seg${c}/frame${c}/yaw${y}.rois.jpg`;const rows=method.endsWith('ungated')?D.ungated:D.metrics;const r=rows.find(x=>x.segment===c&&x.frame===i&&x.yaw===y&&x.method===method.replace('_ungated',''));$('counts').textContent='車道↔歩道を変更した画素：'+(r?r.changed_pixels:0);}
for(const id of ['seg','yaw','offset','method'])$(id).oninput=update;$('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;$('play').textContent='再生';return;}$('play').textContent='停止';timer=setInterval(()=>{$('offset').value=Number($('offset').value)>=8?-8:Number($('offset').value)+1;update()},600)};
let t='<table><tr><th>方法</th><th>19領域の一致率</th><th>車道／歩道の逆判定</th><th>対応点での時系列入替率</th></tr>';for(const method of ['baseline','causal','bidirectional','bidirectional_ungated']){const r=D.summary.roi_summary[method],f=D.summary.aligned_flip_rates[method],pct=n=>(n*100).toFixed(3)+'%';t+=`<tr><td>${method}</td><td>${pct(r.correct_fraction)}</td><td>${pct(r.opposite_fraction)}</td><td>${pct(f.flip_fraction)}</td></tr>`;}$('stats').innerHTML=t+'</table>';update();</script></html>'''
    (p/'comparison.html').write_text(html.replace('DATA',data));print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
