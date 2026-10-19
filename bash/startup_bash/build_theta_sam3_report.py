"""Build a self-contained local HTML comparison shell (images remain adjacent)."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);a=p.parse_args();s=a.study
    manifest=json.loads((s/'manifest.json').read_text());summary=json.loads((s/'roi_summary.json').read_text())
    data=json.dumps({'centers':manifest['centers'],'summary':summary},ensure_ascii=False)
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><title>SAM3 入力投影比較・10区間</title>
<style>body{font:16px system-ui;background:#101820;color:#edf2f7;margin:24px;line-height:1.65}h1{font-size:26px}select{font-size:15px;padding:6px;margin:5px;background:#fff;color:#182530}a{color:#84d5ff}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}img{width:100%;background:#000}figure{margin:0}figcaption{font-weight:bold}table{border-collapse:collapse;font-size:14px}td,th{border:1px solid #506273;padding:6px;text-align:right}th:first-child,td:first-child{text-align:left}.note{background:#243442;padding:14px;margin:14px 0}.legend{color:#ffdf44}button{padding:8px;cursor:pointer}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>
<h1>SAM3 セマンティック：入力投影・解像度・プロンプトの比較</h1>
<p>2026-10-19 / real_both_mapping_20260920_113447_recovered / 10区間 × 5フレーム。比較画面は各区間の中央フレームです。</p>
<div class="note">正距円筒画像そのままへの変更では、今回の点字ブロックの認識漏れは解消しませんでした。内部解像度、点字ブロックのプロンプト、クラス間の競合、採用閾値も影響しています。<br>既定設定は変更していません。黄色＝点字ブロック、緑＝芝生、青＝車道、灰＝歩道。色の量は正確さを意味しません。</div>
<label>区間<select id="segment"></select></label><label>視線<select id="yaw"><option value="0">前 0°</option><option value="180">後 180°</option><option value="90">左 90°</option><option value="270">右 270°</option></select></label>
<div class="grid"><figure><figcaption>元写真（共通投影）</figcaption><img id="source"></figure><figure><figcaption>暫定の確認領域（黄：点字、緑：芝、紫：通常舗装）</figcaption><img id="roi"></figure></div>
<div class="grid"><figure><label>A<select id="a"></select></label><select id="ta"><option>.80</option><option>.65</option><option>.50</option></select><img id="ia"></figure><figure><label>B<select id="b"></select></label><select id="tb"><option>.80</option><option selected>.65</option><option>.50</option></select><img id="ib"></figure></div>
<p><a id="panorama">正距円筒の元画像を開く</a> · <a href="reference/source_overview.jpg">全区間の写真一覧</a> · <a href="README.md">実験条件と再実行方法</a> · <a href="roi_checks.json">ROIごとの数値</a></p>
<h2>暫定ROI内の着色率</h2><p>人手の正解マスクとの精度評価ではありません。確認用に置いた点字6領域・芝生1領域で、そのクラスが着いた画素の割合です。通常舗装の10領域は対象クラスの誤着色を確認します。芝生は1領域のみで一般化できません。領域は専門家未確認・完全なアノテーションではありません。</p>
<div style="overflow:auto"><table id="stats"></table></div>
<p>全周画像は横長をステレオと判定するサーバを避けるため、同じ全周画像を左右に連結して送信しています。モデルには完全な全周画像1枚が入り、返却サイズも確認しました。モデル内部では全入力を正方形へリサイズします。スコアはマスクの信頼度で、画素ごとの校正済み確率ではありません。</p>
<script>const D=DATA;const names={perspective2_512:'現行 前後 / 512',perspective2_1008:'前後 / 1008',perspective4_1008:'前後左右 / 1008',equirect_full_512:'全周 正距円筒 / 512',equirect_full_1008:'全周 正距円筒 / 1008',equirect_ground4_1008:'正距円筒 地面4分割 / 1008',perspective2_targets_1008:'前後 対象2クラス / 1008',equirect_ground4_targets_1008:'正距円筒 地面4分割 対象2クラス',perspective2_yellow_1008:'前後 yellow tactile paving / 1008'};
const $=id=>document.getElementById(id);D.centers.forEach(c=>$('segment').add(new Option(c,c)));for(const id of ['a','b'])Object.keys(D.summary).forEach(m=>$(id).add(new Option(names[m],m)));$('a').value='perspective2_512';$('b').value='perspective2_yellow_1008';$('segment').value='3300';
function update(){const c=$('segment').value,y=$('yaw').value;$('source').src=`reference/seg${c}/yaw${y}.jpg`;$('roi').src=`reference/seg${c}/yaw${y}_rois.jpg`;$('roi').onerror=()=>{$('roi').onerror=null;$('roi').src=$('source').src};for(const k of ['a','b'])$('i'+k).src=`review/seg${c}/${$(k).value}/yaw${y}_${Number($('t'+k).value).toFixed(2)}.overlay.jpg`;$('panorama').href=`seg${c}/frame${c}/panorama.jpg`;}
for(const id of ['a','b','ta','tb','segment','yaw'])$(id).onchange=update;
let rows='<tr><th>方式</th><th>閾値</th><th>点字 6 ROI</th><th>芝 1 ROI</th><th>通常舗装の点字誤着色</th></tr>';for(const [m,ts] of Object.entries(D.summary))for(const [t,v] of Object.entries(ts)){const pct=x=>(100*x).toFixed(1)+'%';rows+=`<tr><td>${names[m]}</td><td>${t}</td><td>${pct(v['4'].positive.labelled_fraction)}</td><td>${pct(v['3'].positive.labelled_fraction)}</td><td>${pct(v['4'].negative.labelled_fraction)}</td></tr>`;}$('stats').innerHTML=rows;update();</script></html>'''
    (s/'comparison.html').write_text(html.replace('DATA',data))


if __name__=='__main__':main()
