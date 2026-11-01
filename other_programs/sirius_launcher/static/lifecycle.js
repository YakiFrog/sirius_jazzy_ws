'use strict';
(()=>{
 const style=document.createElement('link');style.rel='stylesheet';style.href='/static/lifecycle.css?v=20261027c';document.head.append(style);
 const nav=document.createElement('button');nav.className='nav-item';nav.dataset.page='lifecycle';nav.type='button';nav.title='Nav2ライフサイクル';nav.setAttribute('aria-label',nav.title);nav.innerHTML=ICONS.refresh+'<span>Lifecycle</span>';nav.onclick=()=>setPage('lifecycle');document.querySelector('.nav-item[data-page="tf"]').after(nav);
 const page=document.createElement('section');page.id='page-lifecycle';page.className='page';page.setAttribute('aria-label','Nav2ライフサイクル');
 page.innerHTML='<header class="lc-heading"><div><h1>Nav2 Lifecycle</h1><p>ノードのつながりと状態を確認</p></div><button type="button" class="hbtn" id="lc-refresh">更新</button></header><div class="lc-summary" id="lc-summary" role="status">状態を取得中…</div><div id="lc-job" class="lc-job" role="status" hidden></div><section class="lc-overview" aria-labelledby="lc-overview-title"><header><h2 id="lc-overview-title">自律移動のつながり</h2><span>概略図・構成により変わります</span></header><div class="lc-flows" id="lc-flows"></div><p class="lc-overview-note">線の動きはイメージです。実際の通信を示すものではありません。稼働と受信は別で、障害物センサーはTopicsで確認。</p></section><section class="lc-transition" aria-label="ノードの状態遷移"><strong>状態遷移</strong><span>未設定 <small>― 設定 →</small> 待機 <small>― 有効化 →</small> 稼働</span><span class="lc-transition-back">逆：無効化・設定解除</span></section><details class="lc-details" id="lc-details"><summary>全ノードの詳細・操作 <small>状態を確認してから操作</small></summary><p class="lc-note">有効化すると既存ゴールの走行が再開する場合があります。依存ノードは自動で操作しません。ウェイポイント・ドッキング等は用途と管理設定により必要性が変わります。</p><div class="lc-grid" id="lc-nodes"></div></details><details class="lc-history"><summary>状態の変化・操作履歴</summary><ol id="lc-history"></ol><p class="lc-note">状態履歴は監視開始後の最新60件です。監視前の起動失敗は過去ログも確認してください。</p></details>';
 document.querySelector('.workspace').append(page);
 if(PAGE==='lifecycle')setPage('lifecycle');
 const stateNames={0:'状態不明',1:'未設定',2:'待機',3:'稼働',4:'終了',10:'設定中',11:'設定解除中',12:'終了処理中',13:'有効化中',14:'無効化中',15:'エラー処理中'};
 const actions={configure:{id:1,label:'設定',from:1},activate:{id:3,label:'有効化',from:2},deactivate:{id:4,label:'無効化',from:3},cleanup:{id:2,label:'設定解除',from:2}};
 let data=null,busy=false,pending=false,lastPoll=0,shape='',selectedLogs=new Set(),monitor=window.siriusNavigationSnapshot||null,monitorAt=Date.now();
 const text=(tag,value,cls)=>{const e=document.createElement(tag);e.textContent=value;if(cls)e.className=cls;return e;};
 const flows=document.getElementById('lc-flows');
 const body=document.createElement('div');body.className='lc-system-body';body.setAttribute('aria-label','自律移動の処理の概略図');
 const viewport=document.createElement('div');viewport.className='lc-system-viewport';viewport.append(body);flows.append(viewport);
 const hint=text('p','横にスクロールして全体を表示','lc-scroll-hint');flows.append(hint);
 const entries=[
  ['map_server','地図','map'],['@laser','2Dスキャン','scan'],['@tf','TF','tf'],
  ['amcl','AMCL','amcl'],['global_costmap/global_costmap','グローバル','global'],['local_costmap/local_costmap','ローカル','local'],
  ['bt_navigator','ゴール受付','goal'],['planner_server','経路計画','planner'],['controller_server','速度制御','controller'],
  ['velocity_smoother','速度平滑化','smoother'],['collision_monitor','衝突監視','collision'],['@mux','twist_mux','mux'],['@driver','駆動','driver']
 ];
 const inputGroup=document.createElement('div');inputGroup.className='lc-system-inputs';inputGroup.setAttribute('aria-label','自己位置推定の主な入力');body.append(inputGroup);
 const nodes={};
 for(const [key,label,slot] of entries){
  const node=document.createElement('button');node.type='button';node.className='lc-flow-node';node.dataset.key=key;node.dataset.slot=slot;node.dataset.tone='unknown';node.append(text('span',label,'lc-flow-name'),text('b','未取得','lc-flow-state'));
  if(!key.startsWith('@'))node.onclick=()=>{const details=document.getElementById('lc-details');details.open=true;const target=[...document.querySelectorAll('.lc-node')].find(card=>card.dataset.node===key);if(target){target.tabIndex=-1;target.focus({preventScroll:true});target.scrollIntoView({block:'center',behavior:'smooth'});}};
  else if(key==='@tf')node.onclick=()=>setPage('tf');else if(key==='@laser')node.onclick=()=>setPage('topics');else node.onclick=()=>document.getElementById('navigation-diagnostics-button')?.click();
  nodes[slot]=node;(slot==='map'||slot==='scan'||slot==='tf'?inputGroup:body).append(node);
 }
 const svg=document.createElementNS('http://www.w3.org/2000/svg','svg');svg.classList.add('lc-connectors');svg.setAttribute('aria-hidden','true');body.prepend(svg);
 const links=[[inputGroup,nodes.amcl],[nodes.amcl,nodes.global],[nodes.global,nodes.planner],[nodes.local,nodes.controller],...['goal','planner','controller','smoother','collision','mux'].map((slot,i)=>[nodes[slot],nodes[['planner','controller','smoother','collision','mux','driver'][i]]])];
 const diagram=[{body,svg,links,nodes:Object.values(nodes),inputGroup}];
 let drawScheduled=false;
 function scheduleConnections(){if(!drawScheduled){drawScheduled=true;requestAnimationFrame(drawConnections);}}
 function drawConnections(){drawScheduled=false;
  for(const {body,svg,links} of diagram){const root=body.getBoundingClientRect();if(!root.width||!root.height)continue;
   svg.setAttribute('viewBox','0 0 '+root.width+' '+root.height);
   for(const [index,[from,to]] of links.entries()){const a=from.getBoundingClientRect(),b=to.getBoundingClientRect();if(!a.width||!b.width)continue;
    const ax=a.left-root.left+a.width/2,ay=a.top-root.top+a.height/2,bx=b.left-root.left+b.width/2,by=b.top-root.top+b.height/2;
    const sameRow=a.top<b.bottom-1 && b.top<a.bottom-1;let x1,y1,x2,y2,d;
    if(sameRow){const dir=bx>=ax?1:-1;x1=ax+dir*a.width/2;y1=ay;x2=bx-dir*b.width/2;y2=by;const m=(x1+x2)/2;d=`M ${x1} ${y1} H ${m} V ${y2} H ${x2}`;}
    else {const dir=by>=ay?1:-1;x1=ax;y1=ay+dir*a.height/2;x2=bx;y2=by-dir*b.height/2;const m=(y1+y2)/2+(from.dataset.slot==='global'?-6:from.dataset.slot==='local'?6:0);d=`M ${x1} ${y1} V ${m} H ${x2} V ${y2}`;}
    let group=svg.children[index];
    if(!group){group=document.createElementNS('http://www.w3.org/2000/svg','g');for(const cls of ['lc-connector-base','lc-connector-pulse','lc-connector-head']){const path=document.createElementNS('http://www.w3.org/2000/svg','path');path.classList.add(cls);group.append(path);}svg.append(group);}
    const [path,pulse,head]=group.children;if(path.getAttribute('d')!==d){path.setAttribute('d',d);pulse.setAttribute('d',d);}
    const horizontal=sameRow?(bx>=ax?1:-1):0;const vertical=sameRow?0:(by>=ay?1:-1);const arrow=horizontal?`M ${x2-horizontal*5} ${y2-3} L ${x2} ${y2} L ${x2-horizontal*5} ${y2+3}`:`M ${x2-3} ${y2-vertical*5} L ${x2} ${y2} L ${x2+3} ${y2-vertical*5}`;if(head.getAttribute('d')!==arrow)head.setAttribute('d',arrow);

   }
  }
 }
 if(window.ResizeObserver){const observer=new ResizeObserver(scheduleConnections);for(const item of diagram){observer.observe(item.body);if(item.inputGroup)observer.observe(item.inputGroup);item.nodes.forEach(n=>observer.observe(n));}}
 else window.addEventListener('resize',scheduleConnections);
 function paintOverview(payload){
  const rows=payload.telemetry?.nodes||{},maps=payload.telemetry?.costmaps||{},live=!!payload.available;
  const counts={ok:0,waiting:0,transition:0,error:0,unknown:0};
  Object.keys(payload.nodes||{}).forEach(n=>counts[lifecycleState(rows[n],live).tone]++);
  const summary=document.getElementById('lc-summary'),summaryShape=JSON.stringify([counts,payload.domain,Object.keys(payload.nodes||{}).length]);
  if(summary._shape!==summaryShape){summary._shape=summaryShape;summary.replaceChildren();
   for(const [tone,label] of [['ok','稼働中'],['waiting','待機・未設定'],['transition','遷移中'],['error','エラー'],['unknown','未取得']]){
    if(counts[tone]===0 && Object.keys(payload.nodes||{}).length)continue;
    const chip=text('span',label+' '+counts[tone],'lc-count');chip.dataset.tone=tone;summary.append(chip);
   }
   summary.append(text('small','Domain '+payload.domain,'lc-domain'));
  }
  const extra=lifecycleGraphState(Date.now()-monitorAt<8000?monitor:null,payload.domain,navigationDiagnosticMapReady);
  for(const node of flows.querySelectorAll('.lc-flow-node')){
   const key=node.dataset.key;let state=key.startsWith('@')?extra[key.slice(1)]:lifecycleState(rows[key],live);
   node.dataset.tone=state.tone;let label=state.label,description=state.label;
   if(key==='@mux'&&label==='接続・解除確認')label='解除済み';
   if(key.includes('_costmap/')){const receipt=lifecycleCostmap(maps[key.startsWith('global')?'global':'local'],live).label;description+=' · '+receipt;label=label==='稼働中'&&receipt==='受信中'?'稼働・受信':label+' · '+receipt;}
   node.querySelector('.lc-flow-state').textContent=label;const name=key==='@laser'?'AMCLの2Dスキャン':node.querySelector('.lc-flow-name').textContent;node.setAttribute('aria-label',name+'：'+description+'。詳細を開く');
  }
  scheduleConnections();
 }
 window.addEventListener('sirius:navigation-state',event=>{monitor=event.detail;monitorAt=Date.now();if(data)paintOverview(data);});
 function render(payload){
  data=payload;
  const rows=payload.telemetry?.nodes||{},logs=payload.telemetry?.logs||{},maps=payload.telemetry?.costmaps||{};
  const jobs=payload.operations||[],job=jobs[0],inFlight=jobs.some(j=>j.status==='running');
  const live=!!payload.available;
  paintOverview(payload);
  const jobBox=document.getElementById('lc-job');
  jobBox.hidden=!job||(job.status==='done'&&Number.isFinite(job.finished)&&Date.now()/1000-job.finished>=15);
  if(job){jobBox.dataset.state=job.status;jobBox.textContent='/'+job.node+'：'+(actions[job.action]?.label||job.action)+' '+(job.status==='running'?'実行中（自動再送しません）':job.result?.ok?'目的状態を確認済み':job.result?.error||'結果を確認できません');}
  const container=document.getElementById('lc-nodes');
  // Rebuild only if the displayed state changed, preserving focus and open logs.
  const signature=JSON.stringify([live,payload.nodes,
   Object.entries(rows).map(([n,e])=>[n,e.fresh,e.value]),
   Object.entries(logs).map(([n,e])=>[n,e.fresh,e.value]),
   Object.entries(maps).map(([n,e])=>[n,e.fresh,e.value]),inFlight,pending]);
  if(signature!==shape){shape=signature;selectedLogs=new Set([...container.querySelectorAll('details[open]')].map(e=>e.dataset.node));container.replaceChildren();
   for(const [name,label] of Object.entries(payload.nodes||{})){
    const entry=rows[name]||{},value=entry.value||{},fresh=live&&entry.fresh,id=fresh?value.id:null;
    const card=document.createElement('article');card.className='lc-node';card.dataset.node=name;const nodeState=lifecycleState(entry,live);card.dataset.tone=nodeState.tone;
    const badge=text('b',nodeState.label,'lc-state');badge.dataset.tone=nodeState.tone;
    const header=document.createElement('header');header.append(text('h2',label),badge);card.append(header,text('code','/'+name));
    card.append(text('p',fresh?'現在：'+(value.label||'unknown')+' · 観測から '+Math.floor(entry.observed_seconds||0)+'秒':value.label?'最後の状態：'+value.label+'（現在は未確認）':'状態サービスの受信待ち','lc-state-detail'));
    if(name.includes('_costmap/')){
     const map=maps[name.startsWith('global')?'global':'local']||{},valid=live&&map.fresh&&map.value?.valid;
     const line=text('p',valid?'コストマップ受信中 · '+map.value.width+'×'+map.value.height:'コストマップ未受信・更新停止','lc-map');line.dataset.tone=valid?'ok':id===3?'warning':'unknown';card.append(line);
     card.append(text('p',name.startsWith('global')?'通常は「経路計画」の有効化に連動します。':'通常は「速度制御」の有効化に連動します。','lc-state-detail'));
    }
    const log=logs[name];if(log?.value){const details=document.createElement('details');details.className='lc-log';details.dataset.node=name;details.open=selectedLogs.has(name);details.append(text('summary',!log.fresh?'以前の通知（現在状態とは別です）':log.value.text.includes('transform')?'直近のTF関連ログ':'直近の通知'),text('p',log.value.text));card.append(details);}
    const buttons=document.createElement('div');buttons.className='lc-actions';
    for(const [action,definition] of Object.entries(actions)){
     const button=text('button',definition.label);button.type='button';button.disabled=pending||inFlight||id!==definition.from;
     button.title=button.disabled?'現在の状態では実行できません':'/'+name+' を'+definition.label;button.onclick=()=>send(name,action,definition.from);buttons.append(button);
    }
    card.append(buttons);container.append(card);
   }
  }
  for(const card of container.children){const entry=rows[card.dataset.node];if(live&&entry?.fresh)card.querySelector('.lc-state-detail').textContent='現在：'+(entry.value?.label||'unknown')+' · 観測から '+Math.floor(entry.observed_seconds||0)+'秒';}
  const history=document.getElementById('lc-history');history.replaceChildren();
  const records=[...(payload.telemetry?.history||[]).map(e=>({time:e.time,text:'/'+e.node+' → '+(stateNames[e.id]||e.label)})),
   ...jobs.map(e=>({time:e.started,text:'操作：/'+e.node+' '+(actions[e.action]?.label||e.action)+' · '+({running:'実行中',done:'確認済み',failed:'失敗・要確認'})[e.status]}))].sort((a,b)=>b.time-a.time).slice(0,60);
  for(const entry of records){history.append(text('li',new Date(entry.time*1000).toLocaleTimeString('ja-JP')+' '+entry.text));}
  if(!records.length)history.append(text('li','履歴はまだありません'));
 }
 async function refresh(force=false){
  if(busy||document.hidden||(!force&&!isPageVisible('lifecycle'))||(!force&&Date.now()-lastPoll<1500))return;
  busy=true;lastPoll=Date.now();
  try{const r=await fetch('/api/lifecycle',{cache:'no-store',signal:AbortSignal.timeout(5000)});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);render(d);}
  catch(error){shape='';monitor=null;if(data)render({...data,available:false});const summary=document.getElementById('lc-summary');summary._shape=null;summary.textContent='現在状態を取得できません：'+error.message;document.querySelectorAll('#lc-nodes button').forEach(b=>b.disabled=true);}
  finally{busy=false;}
 }
 async function send(node,action,state){
  if(pending)return;
  const label=actions[action].label;
  if(!confirm('/'+node+' を「'+label+'」しますか？\n\n有効化すると既存ゴールの走行が再開する可能性があります。Nav2管理により関連ノードにも影響する場合があります。\n緊急停止は変更しません。TF待ちなどで最大約110秒かかる場合があります。'))return;
  pending=true;shape='';if(data)render(data);
  try{const r=await fetch('/api/lifecycle/transition',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({node,action,expected_state:state}),signal:AbortSignal.timeout(6000)});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);if(data){data.operations=[d.operation,...(data.operations||[])];render(data);}}
  catch(error){const box=document.getElementById('lc-job');box.hidden=false;box.dataset.state='failed';box.textContent='要求結果は未確認です。再送せず状態を更新してください：'+error.message;}
  finally{pending=false;shape='';refresh(true);}
 }
 document.getElementById('lc-refresh').onclick=()=>refresh(true);
 document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
 setInterval(refresh,1000);refresh();
})();
