'use strict';
const TF_VIEW={data:null,busy:false,selected:null,positions:new Map(),shape:'',view:null,drag:null,autoFit:true,layout:localStorage.getItem('sirius-tf-layout')==='horizontal'?'horizontal':'hierarchy'};
const TF_NS='http://www.w3.org/2000/svg';
function updateTFHealth(data){
  const nav=document.querySelector('.nav-item[data-page="tf"]'),badge=document.getElementById('tf-error-badge');
  if(!badge)return;
  const panel=document.getElementById('tf-errors');
  if(!data.available){
    badge.hidden=false;badge.textContent='?';badge.dataset.state='unknown';
    nav.title='TF Tree · 監視データ未取得（現在の競合状態は未確認）';nav.setAttribute('aria-label',nav.title);
    panel.hidden=true;return;
  }
  const conflicts=data.conflicts||[],cycles=data.cycles||[],frames=new Set([...conflicts,...cycles]);
  const ready=data.readiness&&data.readiness.state==='ok';
  badge.hidden=!frames.size&&!ready;badge.textContent=ready?'✓':frames.size>99?'99+':String(frames.size);badge.dataset.state=ready?'ok':'error';
  nav.title=ready?'TF Tree · '+(data.readiness.summary||'シリウス準備OK'):frames.size?'TF Tree · 構成エラー '+frames.size+'フレーム（競合・循環）':'TF Tree';nav.setAttribute('aria-label',nav.title);
  panel.hidden=!frames.size;
  const incoming=data.edges.filter(e=>conflicts.includes(e.child)&&!e.stale);
  const shape=JSON.stringify([conflicts,cycles,incoming.map(e=>[e.parent,e.child,e.publisher,e.static])]);
  if(shape===TF_VIEW.errorShape)return;TF_VIEW.errorShape=shape;panel.replaceChildren();
  if(!frames.size)return;
  const title=document.createElement('strong');title.textContent='TF構成エラー · '+frames.size+'フレーム';panel.append(title);
  const hint=document.createElement('p');hint.textContent='同じ子フレームへの重複配信や循環は、自己位置推定を不安定にします。配信元を確認してください。';panel.append(hint);
  for(const frame of frames){
    const row=document.createElement('div'),button=document.createElement('button'),details=document.createElement('span');row.className='tf-error-row';
    button.textContent=frame;button.onclick=()=>selectTF(frame);
    details.textContent=(conflicts.includes(frame)?'競合: '+incoming.filter(e=>e.child===frame).map(e=>e.parent+' → '+frame+' / '+e.publisher).join(' ・ '):'TF経路が循環しています');
    row.append(button,details);panel.append(row);
  }
}
function tfElement(tag,attributes={},text){const el=document.createElementNS(TF_NS,tag);for(const [k,v] of Object.entries(attributes))el.setAttribute(k,v);if(text!==undefined)el.textContent=text;return el;}
function layoutTF(data){
  const nodes=data.frames,children=new Map(nodes.map(f=>[f,[]])),seen=new Set(),forest=[];
  for(const edge of data.edges)if(!children.get(edge.parent).includes(edge.child))children.get(edge.parent).push(edge.child);
  const tree=new Map(nodes.map(f=>[f,[]])),depth=new Map();
  for(const root of [...data.roots,...nodes]){
    if(seen.has(root))continue;forest.push(root);seen.add(root);depth.set(root,0);
    const queue=[root];for(let i=0;i<queue.length;i++)for(const child of children.get(queue[i]).sort()){
      if(seen.has(child))continue;seen.add(child);depth.set(child,depth.get(queue[i])+1);tree.get(queue[i]).push(child);queue.push(child);
    }
  }
  const positions=new Map();let y=42;
  if(TF_VIEW.layout==='horizontal'){
    const widths=[];for(const frame of nodes){const d=depth.get(frame);widths[d]=Math.max(widths[d]||0,Math.max(175,Math.min(360,frame.length*8.2+45)));}
    const columns=[32];for(let d=1;d<widths.length;d++)columns[d]=columns[d-1]+widths[d-1]+95;
    function placeHorizontal(frame){const children=tree.get(frame);let cy;if(children.length){children.forEach(placeHorizontal);cy=(positions.get(children[0]).y+positions.get(children.at(-1)).y)/2;}else{cy=y;y+=78;}positions.set(frame,{x:columns[depth.get(frame)],y:cy,width:Math.max(175,Math.min(360,frame.length*8.2+45))});}
    forest.forEach(root=>{placeHorizontal(root);y+=35;});return positions;
  }
  function place(frame){positions.set(frame,{x:32+depth.get(frame)*34,y,width:Math.max(175,Math.min(360,frame.length*8.2+45))});y+=52;tree.get(frame).forEach(place);}
  forest.forEach(root=>{place(root);y+=20;});return positions;
}
function setTFLayout(layout){
  if(!['hierarchy','horizontal'].includes(layout))return;
  TF_VIEW.layout=layout;localStorage.setItem('sirius-tf-layout',layout);TF_VIEW.view=null;TF_VIEW.autoFit=true;TF_VIEW.shape='';
  document.querySelectorAll('[data-tf-layout]').forEach(button=>{const active=button.dataset.tfLayout===layout;button.classList.toggle('active',active);button.setAttribute('aria-pressed',String(active));});
  if(TF_VIEW.data)renderTF(TF_VIEW.data);
}
function setTFView(){if(TF_VIEW.view){const v=TF_VIEW.view;document.getElementById('tf-svg').setAttribute('viewBox',[v.x,v.y,v.w,v.h].join(' '));}}
function fitTF(){
  if(!TF_VIEW.positions.size)return;const svg=document.getElementById('tf-svg');const box=svg.getBoundingClientRect();
  const points=[...TF_VIEW.positions.values()];let w=Math.max(...points.map(p=>p.x+p.width))+110,h=Math.max(...points.map(p=>p.y))+70;const contentW=w,contentH=h;
  const ratio=(box.width||800)/Math.max(100,(box.height||500)-35);if(w/h<ratio)w=h*ratio;else h=w/ratio;
  TF_VIEW.view={x:(contentW-w)/2,y:(contentH-h)/2,w,h};TF_VIEW.autoFit=true;setTFView();
}
function zoomTF(factor,point){
  const v=TF_VIEW.view;if(!v)return;const next=v.w*factor;if(next<80||next>30000)return;
  TF_VIEW.autoFit=false;const cx=point?point.x:v.x+v.w/2,cy=point?point.y:v.y+v.h/2;
  TF_VIEW.view={x:cx-(cx-v.x)*factor,y:cy-(cy-v.y)*factor,w:v.w*factor,h:v.h*factor};setTFView();
}
function selectTF(frame){TF_VIEW.selected=frame;document.querySelectorAll('.tf-node').forEach(n=>n.classList.toggle('selected',n.dataset.frame===frame));renderTFDetails();}
function searchTF(query){
  if(!TF_VIEW.data)return;query=query.trim().toLowerCase();if(!query)return;
  const frame=TF_VIEW.data.frames.find(f=>f.toLowerCase()===query)||TF_VIEW.data.frames.find(f=>f.toLowerCase().includes(query));
  if(!frame)return;selectTF(frame);const p=TF_VIEW.positions.get(frame),v=TF_VIEW.view;if(!p||!v)return;
  TF_VIEW.autoFit=false;v.x=p.x+p.width/2-v.w/2;v.y=p.y-v.h/2;setTFView();
}
function buildTFTree(data){
  const svg=document.getElementById('tf-svg');svg.replaceChildren();TF_VIEW.positions=layoutTF(data);
  if(TF_VIEW.layout==='horizontal'){
    const defs=tfElement('defs');for(const [name,color] of [['dynamic','var(--accent)'],['static','var(--mint)'],['stale','var(--faint)'],['conflict','var(--red)']]){
      const marker=tfElement('marker',{id:'tf-arrow-'+name,viewBox:'0 0 10 10',refX:9,refY:5,markerWidth:5,markerHeight:5,orient:'auto'});marker.append(tfElement('path',{d:'M 0 0 L 10 5 L 0 10 z',fill:color}));defs.append(marker);
    }svg.append(defs);
  }
  for(const [i,edge] of data.edges.entries()){
    const a=TF_VIEW.positions.get(edge.parent),b=TF_VIEW.positions.get(edge.child);
    const group=tfElement('g',{'data-edge':i,class:'tf-edge'});
    if(TF_VIEW.layout==='horizontal'){
      const x=a.x+a.width,m=(x+b.x)/2;
      group.append(tfElement('path',{d:`M${x},${a.y} C${m},${a.y} ${m},${b.y} ${b.x-3},${b.y}`}));
      group.append(tfElement('text',{x:(x+b.x)/2,y:(a.y+b.y)/2-10,'text-anchor':'middle'}));
    }else{
      group.append(tfElement('path',{d:`M${a.x+15},${a.y+20} V${b.y} H${b.x}`}));
      group.append(tfElement('text',{x:b.x+b.width+14,y:b.y+4,'text-anchor':'start'}));
    }
    group.append(tfElement('title',{},edge.parent+' → '+edge.child));svg.append(group);
  }
  for(const [frame,p] of TF_VIEW.positions){
    const group=tfElement('g',{class:'tf-node',transform:`translate(${p.x},${p.y-20})`,tabindex:0,role:'button','aria-label':frame});group.dataset.frame=frame;
    group.append(tfElement('rect',{width:p.width,height:40,rx:8}));group.append(tfElement('circle',{cx:15,cy:20,r:3}));
    const max=Math.floor((p.width-35)/8.2),label=frame.length>max?frame.slice(0,max-1)+'…':frame;
    group.append(tfElement('text',{x:27,y:24},label));group.append(tfElement('title',{},frame));
    group.onclick=e=>{e.stopPropagation();selectTF(frame);};group.onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();selectTF(frame);}};svg.append(group);
  }
  const options=document.getElementById('tf-frames');options.replaceChildren();for(const frame of data.frames){const option=document.createElement('option');option.value=frame;options.append(option);}
  if(!TF_VIEW.view||TF_VIEW.autoFit)fitTF();
}
function renderTF(data){
  TF_VIEW.data=data;const shape=TF_VIEW.layout+JSON.stringify(data.edges.map(e=>[e.parent,e.child,e.static,e.publisher_gid]));
  if(shape!==TF_VIEW.shape){TF_VIEW.shape=shape;buildTFTree(data);}
  const stale=data.edges.filter(e=>e.stale).length,statics=data.edges.filter(e=>e.static).length;
  const summary=document.getElementById('tf-counts');summary.replaceChildren();
  for(const text of [data.frames.length+' frames',data.edges.length+' transforms','静的 '+statics,'動的 '+(data.edges.length-statics),'Domain '+data.domain]){const span=document.createElement('span');span.textContent=text;summary.append(span);}
  const warnings=[];
  if(stale)warnings.push(stale+'件の動的TFが3秒以上未受信');
  if(data.conflicts.length)warnings.push('配信の競合: '+data.conflicts.join(', '));
  if(data.cycles.length)warnings.push('循環: '+data.cycles.join(', '));
  if(data.at_capacity)warnings.push('表示可能な変換数の上限に達しました');
  const banner=document.getElementById('tf-warning');banner.hidden=!warnings.length;banner.textContent=warnings.join(' / ');
  const empty=document.getElementById('tf-empty');empty.hidden=data.edges.length>0;empty.textContent='TFがまだありません。Domain '+data.domain+' の /tf・/tf_static を待っています。';
  data.edges.forEach((edge,i)=>{const group=document.querySelector('[data-edge="'+i+'"]');if(!group)return;
    const conflict=!edge.stale&&(data.conflicts.includes(edge.child)||(data.cycles.includes(edge.parent)&&data.cycles.includes(edge.child)));
    group.setAttribute('class','tf-edge'+(edge.static?' static':'')+(edge.stale?' stale':'')+(conflict?' conflict':''));
    if(TF_VIEW.layout==='horizontal')group.querySelector('path').setAttribute('marker-end','url(#tf-arrow-'+(conflict?'conflict':edge.stale?'stale':edge.static?'static':'dynamic')+')');
    group.querySelector('text').textContent=edge.static?'static':edge.stale?'停止中':edge.rate_hz===null?'測定中':edge.rate_hz.toFixed(1)+' Hz';
  });
  document.querySelectorAll('.tf-node').forEach(group=>{group.classList.toggle('warning',data.conflicts.includes(group.dataset.frame)||data.cycles.includes(group.dataset.frame));group.classList.toggle('selected',group.dataset.frame===TF_VIEW.selected);});
  if(TF_VIEW.selected&&!data.frames.includes(TF_VIEW.selected))TF_VIEW.selected=null;
  renderTFDetails();
}
function renderTFDetails(){
  const box=document.getElementById('tf-inspector'),data=TF_VIEW.data,frame=TF_VIEW.selected;box.replaceChildren();
  const heading=document.createElement('h2');heading.textContent=frame||'フレームの詳細';box.append(heading);
  if(!frame||!data){const help=document.createElement('p');help.className='tf-help';help.textContent='フレームを選択すると、変換値と配信状態を表示します。';box.append(help);return;}
  const children=[...new Set(data.edges.filter(e=>e.parent===frame).map(e=>e.child))];
  if(children.length){const links=document.createElement('div');links.className='tf-links';for(const child of children){const b=document.createElement('button');b.textContent=child;b.onclick=()=>selectTF(child);links.append(b);}box.append(links);}
  const incoming=data.edges.filter(e=>e.child===frame);
  if(!incoming.length){const text=document.createElement('p');text.className='tf-help';text.textContent='ルートフレーム · 親フレームは未受信です。';box.append(text);}
  for(const edge of incoming){
    const section=document.createElement('section');section.className='tf-detail';const title=document.createElement('h3');title.textContent=edge.parent+' → '+frame;section.append(title);
    const q=edge.rotation,[x,y,z,w]=q,deg=180/Math.PI;
    const rpy=[Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x)))),Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))].map(v=>(v*deg).toFixed(2)+'°');
    const state=edge.stale?'更新停止':data.conflicts.includes(frame)?'⚠ TF競合':data.cycles.includes(frame)?'⚠ 循環':edge.static?'保持中':'受信中';
    const fields=[['種別',edge.static?'静的TF':'動的TF'],['配信元',edge.publisher],['受信頻度',edge.static?'—（静的）':edge.rate_hz===null?'測定中':edge.rate_hz.toFixed(2)+' Hz'],['最終受信',edge.age_seconds.toFixed(1)+'秒前'],['状態',state],['XYZ (m)',edge.translation.map(v=>v.toFixed(4)).join(' / ')],['RPY',rpy.join(' / ')],['Q xyzw',q.map(v=>v.toFixed(5)).join(' / ')],['TF時刻',edge.stamp.toFixed(6)]];
    const dl=document.createElement('dl');for(const [key,value] of fields){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=key;dd.textContent=value;dl.append(dt,dd);}section.append(dl);box.append(section);
  }
}
async function pollTF(){
  if(TF_VIEW.busy)return;TF_VIEW.busy=true;const status=document.getElementById('tf-status');
  try{
    const data=await api('/api/tf');
    updateTFHealth(data);
    status.classList.toggle('stale',!data.available);
    status.textContent=data.available?'LIVE · '+new Date(data.timestamp*1000).toLocaleTimeString('ja-JP',{hour12:false}):data.error?'TF受信エラー':'TF受信を開始しています…';
    if(data.available){if((window.isPageVisible?isPageVisible('tf'):document.body.dataset.page==='tf'))renderTF(data);}
    else{const empty=document.getElementById('tf-empty');if(!TF_VIEW.data){empty.hidden=false;empty.textContent=data.error||'ROSのTFを受信しています…';}status.title=data.error||'';}
  }catch(error){updateTFHealth({available:false});status.classList.add('stale');status.textContent='接続を再試行しています';}
  finally{TF_VIEW.busy=false;}
}
(function(){
 const svg=document.getElementById('tf-svg');if(!svg)return;
 const badge=document.createElement('span');badge.id='tf-error-badge';badge.hidden=true;badge.setAttribute('aria-hidden','true');document.querySelector('.nav-item[data-page="tf"]').append(badge);
 const errors=document.createElement('section');errors.id='tf-errors';errors.className='tf-errors';errors.hidden=true;errors.setAttribute('role','alert');document.getElementById('tf-warning').before(errors);
 const modes=document.createElement('div');modes.className='segmented tf-layout-switch';modes.setAttribute('role','group');modes.setAttribute('aria-label','TFツリー表示形式');
 for(const [layout,label] of [['hierarchy','階層'],['horizontal','横並び（ROS風）']]){const button=document.createElement('button');button.textContent=label;button.dataset.tfLayout=layout;button.onclick=()=>setTFLayout(layout);modes.append(button);}
 document.querySelector('.tf-tools').prepend(modes);setTFLayout(TF_VIEW.layout);
 svg.addEventListener('wheel',e=>{if(!TF_VIEW.view)return;e.preventDefault();const rect=svg.getBoundingClientRect(),v=TF_VIEW.view;zoomTF(e.deltaY>0?1.12:1/1.12,{x:v.x+(e.clientX-rect.left)/rect.width*v.w,y:v.y+(e.clientY-rect.top)/rect.height*v.h});},{passive:false});
 svg.addEventListener('pointerdown',e=>{if(e.button!==0||e.target.closest('.tf-node')||!TF_VIEW.view)return;TF_VIEW.drag={...TF_VIEW.view,px:e.clientX,py:e.clientY};svg.setPointerCapture(e.pointerId);});
 svg.addEventListener('pointermove',e=>{const d=TF_VIEW.drag;if(!d)return;TF_VIEW.autoFit=false;const rect=svg.getBoundingClientRect();TF_VIEW.view={x:d.x-(e.clientX-d.px)*d.w/rect.width,y:d.y-(e.clientY-d.py)*d.h/rect.height,w:d.w,h:d.h};setTFView();});
 svg.addEventListener('pointerup',()=>TF_VIEW.drag=null);svg.addEventListener('pointercancel',()=>TF_VIEW.drag=null);
})();
