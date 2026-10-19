"""Build a standalone, interactive report for compare_theta_road_weights.py."""
from pathlib import Path
import argparse
import html
import json


def write_report(folder, recommended='rear050_keep050'):
    folder=Path(folder)
    metrics=json.loads((folder/'metrics.json').read_text())
    manifest=json.loads((folder/'manifest.json').read_text())
    names=list(next(iter(metrics.values())))
    aggregate={}
    for name in names:
        rows=[segment[name] for segment in metrics.values()]
        support=sum(row['evaluation_cells'] for row in rows)
        aggregate[name]={
            'mean_rgb_error':sum(row['mean_rgb_error']*row['evaluation_cells'] for row in rows)/support,
            'observed_cells':sum(row['observed_cells'] for row in rows),
            'rear_weight':rows[0]['rear_weight'],
            'min_relative_weight':rows[0]['min_relative_weight'],
            'obstacle_padding_m':rows[0]['obstacle_padding_m'],
        }
    baseline=aggregate['equal']; selection=aggregate[recommended]
    improvement=100*(1-selection['mean_rgb_error']/baseline['mean_rgb_error'])
    recommendation={'candidate':recommended,'parameters':{
        'rear_observation_weight':selection['rear_weight'],
        'direction_transition_degrees':30.,
        'min_relative_observation_weight':selection['min_relative_weight']},
        'evidence':{'segments':len(metrics),'conditions':len(names),
                    'reference_error_reduction_percent':improvement,
                    'observed_cells_vs_baseline':selection['observed_cells']/baseline['observed_cells']},
        'limitation':'Single BAG, local RGB comparison. Reference is not ground truth; no full-map/default change.'}
    (folder/'recommendation.json').write_text(json.dumps(recommendation,ensure_ascii=False,indent=2))
    (folder/'aggregate_metrics.json').write_text(json.dumps(aggregate,indent=2))
    table=''
    for name,d in aggregate.items():
        css=' class="selected"' if name==recommended else ''
        per=''.join(f'<td>{metrics[seg][name]["mean_rgb_error"]:.2f}</td>' for seg in metrics)
        table+=f'<tr{css}><th>{html.escape(name)}</th><td>{d["rear_weight"]:g}</td><td>{d["min_relative_weight"]:g}</td>'+per+f'<td>{d["mean_rgb_error"]:.2f}</td><td>{100*d["observed_cells"]/baseline["observed_cells"]:.1f}%</td></tr>'
    headings=''.join(f'<th>区間 {s}</th>' for s in metrics)
    sections=''
    for seg,info in manifest['segments'].items():
        duration=(info['last_stamp_ns']-info['first_stamp_ns'])/1e9
        sections+=f'''<details><summary>区間 {seg}：{duration:.1f}秒・移動距離 {info['path_length_m']:.2f}m・全条件の比較</summary><p>融合 {len(info['training_indices'])} 枚 / 評価用保留 {len(info['heldout_indices'])} 枚。左が撮影時の前方、右が後方。</p><img src="seg{seg}_camera.jpg" loading="lazy"><a href="seg{seg}_comparison.jpg"><img src="seg{seg}_comparison.jpg" loading="lazy"></a></details>'''
    notes_path=folder/'review_notes.json'
    notes=json.loads(notes_path.read_text()) if notes_path.exists() else ['残像と路面模様は原画像と合わせて確認してください。']
    review=''.join('<li>'+html.escape(note)+'</li>' for note in notes)
    data=json.dumps({'segments':list(metrics),'variants':names,'recommended':recommended},ensure_ascii=False)
    page='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>路面地図 前後重みの比較</title><style>
body{font:16px/1.7 system-ui,sans-serif;color:#24303d;background:#f4f6f8;max-width:1500px;margin:28px auto;padding:0 20px}h1{font-size:1.8rem}h2{font-size:1.25rem;margin-top:32px}a{color:#1765a0}img{max-width:100%}.card{background:white;border:1px solid #dce2e7;border-radius:10px;padding:18px;margin:18px 0}.controls{display:flex;gap:16px;flex-wrap:wrap}select{font:inherit;padding:6px;max-width:100%}label{display:block}#viewer{position:relative;background:#ddd;max-width:1100px;margin:16px auto;line-height:0;overflow:hidden}#a{width:100%;display:block;image-rendering:pixelated}#b{position:absolute;inset:0;width:100%;height:100%;image-rendering:pixelated;clip-path:inset(0 0 0 50%)}#divider{position:absolute;top:0;bottom:0;width:2px;background:#2580bd;left:50%;pointer-events:none}input[type=range]{width:100%}table{border-collapse:collapse;width:100%;background:white;font-size:14px}td,th{padding:8px;border:1px solid #dce2e7;text-align:right;white-space:nowrap}th:first-child{text-align:left}.selected{background:#e7f2ff}pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#eaf0f5;padding:14px}details{background:white;border:1px solid #dce2e7;padding:16px;margin:16px 0}summary{cursor:pointer;font-weight:600}.scroll{overflow-x:auto}.muted{color:#526170}
</style><h1>路面マッピング：前後の観測重みと品質保護の比較</h1><p>real_both_mapping_20260920_113447_recovered ／ @@SEGMENTS@@区間・@@CONDITIONS@@条件 ／ 解像度5cm</p>
<div class="card"><strong>常用の第一候補：前方1.0・後方0.5、最低品質比0.5</strong><p>過去に見えた最良の品質に対して、半分未満の品質の観測ではそのセルを更新しません。後方からしか見えない新しい場所は取り込めます。方向の切り替えは側方付近で滑らかにつなぎます。</p><p>評価用に保留した安定観測とのRGB誤差は、全区間の合計で <strong>@@BASE@@ → @@BEST@@（@@IMPROVE@@%低下）</strong>。観測済みセル数は通常版と同じです。この数値は残像の除去率ではありません。</p></div>
<h2>同じ座標・縮尺で切り替えて比較</h2><div class="card"><div class="controls"><label>区間<select id="segment"></select></label><label>左側 A<select id="caseA"></select></label><label>右側 B<select id="caseB"></select></label></div><div id="viewer"><img id="a" alt="比較A"><img id="b" alt="比較B"><div id="divider"></div></div><label>比較境界<input id="split" type="range" min="0" max="100" value="50"></label><p class="muted">画像は原寸PNGです。拡大して舗装の目地、点字ブロック、人物の色の引き伸ばしを確認してください。A/Bは各区間で同じ画像順・地図原点・画素位置です。</p><p><a id="linkA">Aの原寸画像</a> ／ <a id="linkB">Bの原寸画像</a></p></div>
<h2>同じ評価領域でのRGB誤差（小さいほど整合）</h2><p>各区間の画像を一定間隔で融合用と評価用に分け、保留画像を地図には融合していません。枚数は下の区間別欄に記載しています。保留観測で3時刻以上・色のばらつきが小さいセルだけを参照とし、全条件で観測できた共通領域を評価しています。参照は実測の正解地図ではありません。人物を検出して採点したものでもありません。</p><div class="scroll"><table><thead><tr><th>条件</th><th>後方重み</th><th>最低品質比</th>@@HEADINGS@@<th>加重平均</th><th>観測セル数比</th></tr></thead><tbody>@@TABLE@@</tbody></table></div>
<h2>見た目で確認できたこと</h2><ul>@@REVIEW@@</ul>
<h2>区間ごとの全条件と元写真</h2>@@SECTIONS@@
<h2>今後のBAGで指定できる設定</h2><pre>rear_observation_weight:=0.5
min_relative_observation_weight:=0.5</pre><p>theta_offline_mapping.launch.py に上記の引数を指定できます。座標やこのBAG専用の判定は含みません。後方に人が多い運用の候補として評価した設定で、別BAGでの再現性は未確認です。既定値と既存の全体地図は変更していません。</p><p>今回はRGBの比較に限定し、SAM3は実行していません。人物の検出ラベルは使っていません。</p><p><a href="manifest.json">入力・区間・全条件</a> ／ <a href="metrics.json">区間別の数値</a> ／ <a href="recommendation.json">推奨候補の設定</a></p>
<script>const data=@@DATA@@;const $=id=>document.getElementById(id);for(const s of data.segments){$('segment').add(new Option(s,s));}for(const id of ['caseA','caseB'])for(const v of data.variants){$(id).add(new Option(v,v));}$('caseA').value='equal';$('caseB').value=data.recommended;$('segment').value='4500';function update(){const s=$('segment').value;for(const [choice,img,link]of[['caseA','a','linkA'],['caseB','b','linkB']]){const path=`seg${s}_${$(choice).value}/road.texture.png`;$(img).src=path;$(link).href=path;}}for(const id of ['segment','caseA','caseB'])$(id).addEventListener('change',update);$('split').addEventListener('input',()=>{const v=$('split').value;$('b').style.clipPath=`inset(0 0 0 ${v}%)`;$('divider').style.left=v+'%';});update();</script></html>'''
    for key,value in {'BASE':f'{baseline["mean_rgb_error"]:.2f}','BEST':f'{selection["mean_rgb_error"]:.2f}',
                      'IMPROVE':f'{improvement:.1f}','HEADINGS':headings,'TABLE':table,
                      'SECTIONS':sections,'DATA':data,'REVIEW':review,
                      'SEGMENTS':str(len(metrics)),'CONDITIONS':str(len(names))}.items():
        page=page.replace('@@'+key+'@@',value)
    (folder/'comparison.html').write_text(page)
    return recommendation


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--recommended',default='rear050_keep050')
    args=parser.parse_args()
    print(json.dumps(write_report(args.folder,args.recommended),ensure_ascii=False,indent=2))
