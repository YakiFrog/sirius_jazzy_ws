from pathlib import Path
import time,json,cv2,numpy as np
base=Path('/home/kotantu-desktop/sirius_jazzy_ws/maps_waypoints/maps')
out=base/'theta_road_visibility_20261019_padded025'
previous=base/'theta_road_visibility_20261019'
paths=[out/k/('road_'+k+'.texture.png') for k in ['baseline','visible']]
while not all(p.exists() for p in paths):time.sleep(2)
time.sleep(2)
stats={};images=[]
for key,title,prefix in [('baseline','No visibility filter',out/'baseline/road_baseline'),('previous','Previous visibility filter',previous/'visible/road_visible'),('padded','Obstacle padding: 25 cm',out/'visible/road_visible')]:
 image=cv2.imread(str(prefix)+'.texture.png',cv2.IMREAD_UNCHANGED)
 meta=json.loads(Path(str(prefix)+'.colored.json').read_text());label=cv2.imread(str(prefix)+'.colored.pgm',0)
 mask=image[:,:,3]>0
 stats[key]={'observed_cells':int(mask.sum()),'area_m2':round(float(mask.sum()*meta['resolution']**2),2),'resolution':meta['resolution'],'origin':meta['origin'],'size':[meta['width'],meta['height']],'tactile_semantic_cells':int((label==4).sum())}
 rgb=image[:,:,:3].copy();rgb[~mask]=225
 scale=1300/rgb.shape[0];rgb=cv2.resize(rgb,(round(rgb.shape[1]*scale),1300),interpolation=cv2.INTER_AREA)
 rgb=cv2.copyMakeBorder(rgb,40,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245))
 cv2.putText(rgb,title,(10,26),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1,cv2.LINE_AA);images.append(rgb)
width=max(p.shape[1] for p in images);images=[cv2.copyMakeBorder(p,0,0,0,width-p.shape[1],cv2.BORDER_CONSTANT,value=(225,225,225)) for p in images]
cv2.imwrite(str(out/'map_comparison.jpg'),np.hstack(images))
stats['padded_vs_current_baseline_area_ratio']=stats['padded']['observed_cells']/stats['baseline']['observed_cells']
stats['padded_vs_previous_area_ratio']=stats['padded']['observed_cells']/stats['previous']['observed_cells']
(out/'comparison_stats.json').write_text(json.dumps(stats,ensure_ascii=False,indent=2))
ratio=stats['padded_vs_current_baseline_area_ratio']*100;oldratio=stats['padded_vs_previous_area_ratio']*100
html='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>THETA路面 遮蔽余白25cm</title><style>body{font:16px/1.7 system-ui;background:#f4f5f6;color:#222;max-width:1800px;margin:32px auto;padding:0 20px}img{max-width:100%;background:white}pre{padding:16px;background:white;overflow:auto}a{color:#167}table{border-collapse:collapse;background:white}td,th{padding:8px 20px;border:1px solid #ddd}</style><h1>路面マッピング：遮蔽の周囲を25cm広く除外</h1><p>real_both_mapping_20260920_113447_recovered / 解像度5cm / SAM3は路面分類のみ。</p><p>LiDARの各検出点の周囲に半径25cmの円を置き、その円と視線が交差する範囲まで遮蔽を広げました。前回の距離方向の余裕10cmも維持しています。距離やビーム数に応じて余白が変わらないよう、メートル単位で計算しています。</p><p>今回の観測面積は通常版の<strong>'''+f'{ratio:.1f}%'+'''</strong>、前回の遮蔽除外版の<strong>'''+f'{oldratio:.1f}%'+'''</strong>。これは残像除去率ではありません。取り込まない観測が増えるため、一部の路面に欠けが増える可能性があります。</p><h2>全体：通常版・前回・25cm拡張版</h2><a href="map_comparison.jpg"><img src="map_comparison.jpg"></a><h2>同じ代表フレーム：前回・追加除外部分（赤）・25cm拡張版</h2><a href="observation_comparison.jpg"><img src="observation_comparison.jpg"></a><p>床の色は生成せず、別時刻で実際に見えた観測だけで蓄積します。LiDARが捉えない障害物は、余白を増やしても除外できない場合があります。</p><details><summary>保存結果の数値</summary><pre>'''+json.dumps(stats,ensure_ascii=False,indent=2)+'''</pre></details><p><a href="../theta_road_visibility_20261019/comparison.html">前回の比較</a> / <a href="visible/road_visible.texture.png">拡張版の路面RGB（原寸）</a> / <a href="trial_parameters.json">今回の設定</a></p></html>'''
(out/'comparison.html').write_text(html)
old=previous/'comparison.html';text=old.read_text();marker='<!-- padded025-result -->'
if marker not in text:
 text=text.replace('<h1>',marker+'<p style="background:#e4f0ff;padding:16px"><strong>更新：</strong><a href="../theta_road_visibility_20261019_padded025/comparison.html">遮蔽を25cm広く除外した再生成結果はこちら</a></p><h1>',1);old.write_text(text)
print(json.dumps(stats,ensure_ascii=False,indent=2),flush=True)
