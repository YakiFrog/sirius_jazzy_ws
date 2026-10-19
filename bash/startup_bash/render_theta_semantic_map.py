"""Render semantic-only/overlay/texture maps with identical extents for phones."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont

COLORS={3:(30,174,65),4:(255,218,28),5:(50,95,230),6:(153,163,176)}
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def frame(image,title,subtitle,res,width=1200,show_legend=True):
    h,w=image.shape[:2];scale=width/w;im=Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)).resize((width,round(h*scale)),Image.Resampling.NEAREST)
    result=Image.new('RGB',(width,im.height+156),'#ffffff');result.paste(im,(0,105));draw=ImageDraw.Draw(result)
    draw.text((20,10),title,font=ImageFont.truetype(FONT,32),fill='#172e43');draw.text((20,58),subtitle,font=ImageFont.truetype(FONT,20),fill='#516173')
    start=20
    for cid,label in ([(3,'芝生'),(4,'点字'),(5,'車道'),(6,'歩道')] if show_legend else []):
        draw.rectangle((start,im.height+120,start+17,im.height+137),fill=COLORS[cid]);draw.text((start+24,im.height+112),label,font=ImageFont.truetype(FONT,19),fill='#263b4b');start+=108
    meters=2 if w*res<=20 else 20;length=round(meters/res*scale);x=width-length-30;y=im.height+138
    draw.line((x,y,x+length,y),fill='#172e43',width=3);draw.line((x,y-5,x,y+5),fill='#172e43',width=2);draw.line((x+length,y-5,x+length,y+5),fill='#172e43',width=2);draw.text((x,y-29),f'{meters} m',font=ImageFont.truetype(FONT,19),fill='#172e43')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('map',type=Path);a=p.parse_args();root=a.map
    meta=json.loads((root/'manifest.json').read_text());res=meta['grid']['resolution'];origin=np.asarray(meta['grid']['origin']);h=meta['grid']['height'];w=meta['grid']['width']
    labels=cv2.imread(str(root/'road_temporal.colored.pgm'),0);texture=cv2.imread(str(root/'road_temporal.texture.png'),-1)
    assert labels.shape==texture.shape[:2]==(h,w)
    valid=texture[...,3]>0;rgb=texture[...,:3].copy();rgb[~valid]=248
    fullsem=np.full(rgb.shape,248,np.uint8);over=rgb.copy()
    for cid,color in COLORS.items():
        k=labels==cid;bgr=np.asarray(color[::-1]);fullsem[k]=bgr;over[k]=np.rint(rgb[k]*.4+bgr*.6).astype(np.uint8)
    # Evidence of RGB/semantic support is retained separately from display backgrounds.
    cv2.imwrite(str(root/'semantic_only.png'),fullsem);cv2.imwrite(str(root/'overlay.png'),over);cv2.imwrite(str(root/'texture_only.png'),rgb)
    cv2.imwrite(str(root/'semantic_layer_rgba.png'),np.dstack([fullsem,(labels>0).astype(np.uint8)*255]))
    groups={'overview':(0,0,w,h)}
    for center in sorted({f['segment'] for f in meta['frames']}):
        positions=np.asarray([f['xyz'][:2] for f in meta['frames'] if f['segment']==center]);lo=np.floor((positions.min(axis=0)-5-origin)/res).astype(int);hi=np.ceil((positions.max(axis=0)+5-origin)/res).astype(int)
        lo=np.maximum(lo,0);hi=np.minimum(hi,[w,h]);groups[f'seg{center}']=(int(lo[0]),int(h-hi[1]),int(hi[0]),int(h-lo[1]))
    detail={};titles={'semantic':'① セマンティック対象のみ','overlay':'② 路面テクスチャ ＋ セマンティック','texture':'③ 路面テクスチャのみ'}
    for name,(x0,y0,x1,y1) in groups.items():
        target=root/name;target.mkdir(exist_ok=True);images=[]
        subtitle=('検証済み10区間の分類／全区間の路面テクスチャ' if name=='overview' else f'検証区間 {name[3:]} ｜ '+('黄色い点字＋画像追跡' if meta.get('semantic_dir')=='yellow' else '前後の時系列補完')+' ｜ 同じ範囲・縮尺')
        for kind,source in [('semantic',fullsem),('overlay',over),('texture',rgb)]:
            image=source[y0:y1,x0:x1];rendered=frame(image,titles[kind],subtitle,res,1200 if name!='overview' else 1600,show_legend=kind!='texture');rendered.save(target/(kind+'.jpg'),quality=95,subsampling=0);images.append(rendered)
        contact=Image.new('RGB',(images[0].width,sum(x.height for x in images)),'white');yy=0
        for image in images:contact.paste(image,(0,yy));yy+=image.height
        contact.save(target/'three_views.jpg',quality=94,subsampling=0);detail[name]={'pixel_bbox':[x0,y0,x1,y1],'world_bbox_m':[float(origin[0]+x0*res),float(origin[1]+(h-y1)*res),float(origin[0]+x1*res),float(origin[1]+(h-y0)*res)]}
    (root/'rendering.json').write_text(json.dumps({'display_palette_rgb':COLORS,'class_ids_unchanged':True,'views':detail,'existing_observed_rgb_unchanged':True,'texture_holes_filled_cells':meta['texture_holes_filled_cells'],'semantic_frames':meta['frame_count'],'semantic_coverage':'ten evaluated intervals only'},indent=2))
    html='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>路面セマンティック地図・3種類</title><style>body{font:16px system-ui;margin:12px;background:#101820;color:#eef4f8;line-height:1.6}h1{font-size:24px}img{display:block;width:100%;max-width:1200px;margin:8px 0 24px}a{color:#8ed8ff}nav{margin:12px 0}figure{margin:0}.note{padding:12px;background:#253643}</style><h1>路面地図：3種類の画像</h1><div class="note">点字の補完を検証した区間3300の拡大図です。3枚とも同じ範囲・縮尺です。背景には既存の全区間RGB地図を使用し、未観測の穴は今回の観測で補っています。新しいセマンティックは検証済み10区間・170フレームから作成しました。未分類は白。認識誤りや欠けは残っています。</div><nav><a href="seg3300/three_views.jpg">拡大図3枚を1画像で開く</a> · <a href="#overview">全体図へ</a> · <a href="manifest.json">生成記録</a></nav>'''
    for name in ['seg3300','overview']:
        html+=f'<h2 id="{name}">{"改善区間3300" if name=="seg3300" else "全体図（セマンティックは検証した部分のみ）"}</h2>'
        for kind,title in titles.items():html+=f'<figure><figcaption>{title}</figcaption><a href="{name}/{kind}.jpg"><img src="{name}/{kind}.jpg" alt="{title}" loading="lazy"></a></figure>'
    html+='<h2>ほかの検証区間</h2><nav>'+ ' · '.join(f'<a href="seg{c}/three_views.jpg">{c}</a>' for c in sorted({f['segment'] for f in meta['frames']}))+'</nav></html>'
    if meta.get('semantic_dir')=='yellow':html=html.replace('点字の補完を検証した区間3300','黄色い点字の認識・追跡を検証した区間3300')
    (root/'images.html').write_text(html);print('RENDER COMPLETE',len(groups),'viewports',flush=True)


if __name__=='__main__':main()
