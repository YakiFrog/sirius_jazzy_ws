from pathlib import Path
import time,json,cv2,numpy as np
out=Path('/home/kotantu-desktop/sirius_jazzy_ws/maps_waypoints/maps/theta_road_visibility_20261019')
paths=[out/k/('road_'+k+'.texture.png') for k in ['baseline','visible']]
while not all(p.exists() for p in paths):
 if (out/'nodes.log').exists() and 'process has died' in (out/'nodes.log').read_text():raise RuntimeError('mapping node died')
 time.sleep(2)
time.sleep(2)
rows=[];stats={}
for k,p in zip(['baseline','visible'],paths):
 image=cv2.imread(str(p),cv2.IMREAD_UNCHANGED);meta=json.loads(p.with_name('road_'+k+'.colored.json').read_text());label=cv2.imread(str(p.with_name('road_'+k+'.colored.pgm')),0)
 mask=image[:,:,3]>0
 stats[k]={'observed_cells':int(mask.sum()),'observed_area_m2':float(mask.sum()*meta['resolution']**2),'origin':meta['origin'],'size':[meta['width'],meta['height']],'semantic_cells':{str(c):int((label==c).sum()) for c in range(3,7)}}
 rgb=image[:,:,:3].copy();rgb[~mask]=225
 scale=1300/rgb.shape[0];rgb=cv2.resize(rgb,(round(rgb.shape[1]*scale),1300),interpolation=cv2.INTER_AREA);rgb=cv2.copyMakeBorder(rgb,40,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245));cv2.putText(rgb,k,(15,28),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,0,0),2);rows.append(rgb)
w=max(p.shape[1] for p in rows);rows=[cv2.copyMakeBorder(p,0,0,0,w-p.shape[1],cv2.BORDER_CONSTANT,value=(225,225,225)) for p in rows]
cv2.imwrite(str(out/'map_comparison.jpg'),np.hstack(rows));stats['coverage_ratio']=stats['visible']['observed_cells']/stats['baseline']['observed_cells']
(out/'comparison_stats.json').write_text(json.dumps(stats,indent=2));print(json.dumps(stats,indent=2),flush=True)
html='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>THETA路面 遮蔽除外比較</title><style>body{font:16px system-ui;background:#f4f5f6;color:#222;max-width:1500px;margin:32px auto;padding:0 20px}img{max-width:100%;background:white}pre{padding:16px;background:white}a{color:#167}</style><h1>THETA路面マッピング：LiDAR遮蔽除外の試験</h1><p>real_both_mapping_20260920_113447_recovered / 5 cm grid / 同じBAGを同時再生。</p><p>左：通常版。右：同期LiDARとTFで床の可視範囲に限定。人・車などの物体クラス検出は追加していません。両版とも未知領域の穴埋めは無効です。</p><p>2D LiDARの高さに当たらない物体や柵の隙間などは残るため、動的障害物の完全除去を保証する試験ではありません。再生速度4倍・入力点群stride 4の比較試験です。既存の地図は変更していません。</p><h2>全体の路面RGB</h2><img src="map_comparison.jpg"><h2>代表観測：通常・除外部分・採用部分</h2><img src="observation_comparison.jpg"><h2>保存結果</h2><pre>'''+json.dumps(stats,ensure_ascii=False,indent=2)+'''</pre><p>物体によって隠れた床は、別の観測から得られた場合のみ地図に入ります。未観測の床の色は生成しません。観測面積の保持率は残像除去率を意味しません。</p></html>'''
(out/'comparison.html').write_text(html)
