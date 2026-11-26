'use strict';
(()=>{
 const style=document.createElement('link');style.rel='stylesheet';style.href='/static/lifecycle.css?v=20261126-auto-activation';document.head.append(style);
 const nav=document.createElement('button');nav.className='nav-item';nav.dataset.page='lifecycle';nav.type='button';nav.title='Nav2ライフサイクル';nav.setAttribute('aria-label',nav.title);nav.innerHTML=ICONS.refresh+'<span>Lifecycle</span>';nav.onclick=()=>setPage('lifecycle');document.querySelector('.nav-item[data-page="tf"]').after(nav);
 const errorBadge=document.createElement('span');errorBadge.id='lifecycle-error-badge';errorBadge.hidden=true;errorBadge.setAttribute('aria-hidden','true');nav.append(errorBadge);
 function paintHealth(payload, blockers = lifecycleBlockers(payload)) {
  const health=lifecycleHealth(payload),count=health.errors.length;
  const blocked = blockers.length > 0;
  nav.toggleAttribute('data-lifecycle-alert', blocked);
  errorBadge.hidden=false;errorBadge.dataset.tone=blocked?'blocked':health.tone;
  errorBadge.textContent=count?(count>99?'99+':String(count)):blocked?'!':health.tone==='ok'?'✓':health.tone==='unknown'?'?':'…';
  const detail=blocked?'自律移動できません。'+blockers.map(node=>node.label+'：'+node.state).join('、'):
    health.tone==='unknown'?'状態未取得':health.tone==='waiting'?'待機・遷移中':count?'エラー '+count+' 件。'+health.errors.map(e=>'/'+e.node+' '+e.reason).join('。'):'主要ノード稼働・エラー検出なし';
  nav.title='Nav2 Lifecycle：'+detail;nav.setAttribute('aria-label',nav.title);
  const readiness = document.getElementById('lc-readiness');
  readiness.dataset.tone = blocked ? 'blocked' : health.tone;
  const reason = blocked ? blockers[0].label+'（'+blockers[0].name+'）が'+blockers[0].state+'です'+
    (blockers.length > 1 ? ' ／ ほか'+(blockers.length-1)+'件' : '') :
    health.tone==='ok'?'ライフサイクルの準備が完了しています':detail;
  const title = blocked ? '自律移動できません' : health.tone==='ok' ? '必須ノードは稼働中' :
    health.tone==='error' ? 'Lifecycleにエラーがあります' : '必須ノードの状態を確認中';
  const titleElement = readiness.querySelector('strong'), reasonElement = readiness.querySelector('p');
  if (titleElement.textContent !== title) titleElement.textContent = title;
  if (reasonElement.textContent !== reason) reasonElement.textContent = reason;
  readiness.title = blocked ? blockers.map(node=>node.label+'（'+node.name+'）：'+node.state).join('\n') : reason;
  window.dispatchEvent(new CustomEvent('sirius:lifecycle-blockers', {detail:blockers}));
 }
 const page=document.createElement('section');page.id='page-lifecycle';page.className='page';page.setAttribute('aria-label','Nav2ライフサイクル');
 page.innerHTML='<header class="lc-heading"><div><h1>Nav2 Lifecycle</h1><p>ノードのつながりと状態を確認</p></div><button type="button" class="hbtn" id="lc-refresh">更新</button></header><div class="lc-summary" id="lc-summary" role="status">状態を取得中…</div><div id="lc-job" class="lc-job" role="status" hidden></div><section class="lc-overview" aria-labelledby="lc-overview-title"><header><h2 id="lc-overview-title">自律移動のつながり</h2><span>概略図・構成により変わります</span></header><div class="lc-flows" id="lc-flows"></div><p class="lc-overview-note">線の動きはイメージです。実際の通信を示すものではありません。稼働と受信は別で、障害物センサーはTopicsで確認。</p></section><section class="lc-transition" aria-label="ノードの状態遷移"><strong>状態遷移</strong><span>未設定 <small>― 設定 →</small> 待機 <small>― 有効化 →</small> 稼働</span><span class="lc-transition-back">逆：無効化・設定解除</span></section><details class="lc-details" id="lc-details"><summary>全ノードの詳細・操作 <small>状態を確認してから操作</small></summary><p class="lc-note">有効化すると既存ゴールの走行が再開する場合があります。依存ノードは自動で操作しません。ウェイポイント・ドッキング等は用途と管理設定により必要性が変わります。</p><div class="lc-grid" id="lc-nodes"></div></details><details class="lc-history"><summary>状態の変化・操作履歴</summary><ol id="lc-history"></ol><p class="lc-note">状態履歴は監視開始後の最新60件です。監視前の起動失敗は過去ログも確認してください。</p></details>';
 document.querySelector('.workspace').append(page);
 page.querySelector('.lc-heading').insertAdjacentHTML('afterend', `
  <div class="lc-readiness-row">
   <section id="lc-readiness" class="lc-readiness-card" role="status" data-tone="unknown">
    <span class="lc-readiness-icon" aria-hidden="true"></span>
    <div><strong>必須ノードの状態を確認中</strong><p>状態を取得中…</p></div>
   </section>
   <section class="lc-auto-card" aria-label="自動有効化">
    <div><strong id="lc-auto-title">自動有効化</strong><p id="lc-auto-description">必須ノードの待機が続いたら有効化します</p></div>
    <button type="button" id="lc-auto-toggle" role="switch" aria-checked="false" aria-labelledby="lc-auto-title" aria-describedby="lc-auto-description" disabled>
     <span aria-hidden="true" class="lc-switch-track"></span><span id="lc-auto-state" aria-hidden="true">—</span>
    </button>
    <p id="lc-auto-error" role="status" hidden></p>
   </section>
  </div>`);
 const startup=document.createElement('section');startup.id='lc-startup';startup.hidden=true;startup.setAttribute('aria-label','起動時のTF待機状況');startup.setAttribute('role','status');document.getElementById('lc-summary').after(startup);
 function paintStartup(payload) {
  const notices=lifecycleStartup(payload),signature=JSON.stringify(notices);
  if(startup._shape===signature)return;
  startup._shape=signature;startup.hidden=notices.length===0;startup.replaceChildren();
  for(const notice of notices){
   const row=document.createElement('article');row.className='lc-startup-notice';row.dataset.tone=notice.tone;
   const title=text('strong',(notice.node.startsWith('global')?'グローバル':'ローカル')+'コストマップ · '+notice.label);
   row.append(title,text('p',notice.tone==='error'?'初期位置を設定し、TF Treeで接続を確認した後、対象ノードの有効化を再試行してください。':'地図とロボットをつなぐTFを待っています。初期位置とTF Treeを確認してください。'));
   row.append(text('small','経過時間は監視上の値です。TF待機上限の既定値は60秒です。'));
   const detail=document.createElement('details');detail.append(text('summary','理由・元のログ'),text('p',notice.reason));row.append(detail);
   const button=text('button','/'+notice.parent+' の操作を確認','hbtn');button.type='button';button.onclick=()=>{
    document.getElementById('lc-details').open=true;
    const card=[...document.querySelectorAll('.lc-node')].find(item=>item.dataset.node===notice.parent);
    if(card){card.tabIndex=-1;card.focus({preventScroll:true});card.scrollIntoView({block:'center',behavior:'smooth'});}
   };row.append(button);startup.append(row);
  }
 }
 if(PAGE==='lifecycle')setPage('lifecycle');
 const stateNames={0:'状態不明',1:'未設定',2:'待機',3:'稼働',4:'終了',10:'設定中',11:'設定解除中',12:'終了処理中',13:'有効化中',14:'無効化中',15:'エラー処理中'};
 const actions={configure:{id:1,label:'設定',from:1},activate:{id:3,label:'有効化',from:2},deactivate:{id:4,label:'無効化',from:3},cleanup:{id:2,label:'設定解除',from:2}};
 let data=null,busy=false,pending=false,autoPending=false,lastPoll=0,shape='',selectedLogs=new Set(),monitor=window.siriusNavigationSnapshot||null,monitorAt=Date.now();
 const autoToggle = document.getElementById('lc-auto-toggle'), autoError = document.getElementById('lc-auto-error');
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
   const stopped = new Set([inputGroup,...Object.values(nodes)].filter(node =>
    node.hasAttribute('data-blocked') || node === inputGroup && node.querySelector('[data-blocked]')));
   const downstream = new Set(stopped);
   svg.setAttribute('viewBox','0 0 '+root.width+' '+root.height);
   for(const [index,[from,to]] of links.entries()){const a=from.getBoundingClientRect(),b=to.getBoundingClientRect();if(!a.width||!b.width)continue;
    const ax=a.left-root.left+a.width/2,ay=a.top-root.top+a.height/2,bx=b.left-root.left+b.width/2,by=b.top-root.top+b.height/2;
    const sameRow=a.top<b.bottom-1 && b.top<a.bottom-1;let x1,y1,x2,y2,d;
    if(sameRow){const dir=bx>=ax?1:-1;x1=ax+dir*a.width/2;y1=ay;x2=bx-dir*b.width/2;y2=by;const m=(x1+x2)/2;d=`M ${x1} ${y1} H ${m} V ${y2} H ${x2}`;}
    else {const dir=by>=ay?1:-1;x1=ax;y1=ay+dir*a.height/2;x2=bx;y2=by-dir*b.height/2;const m=(y1+y2)/2+(from.dataset.slot==='global'?-6:from.dataset.slot==='local'?6:0);d=`M ${x1} ${y1} V ${m} H ${x2} V ${y2}`;}
    let group=svg.children[index];
    if(!group){group=document.createElementNS('http://www.w3.org/2000/svg','g');for(const cls of ['lc-connector-base','lc-connector-pulse','lc-connector-head','lc-connector-stop']){const path=document.createElementNS('http://www.w3.org/2000/svg','path');path.classList.add(cls);group.append(path);}svg.append(group);}
    const [path,pulse,head,stop]=group.children;if(path.getAttribute('d')!==d){path.setAttribute('d',d);pulse.setAttribute('d',d);}
    const blocked = stopped.has(from), muted = downstream.has(from);
    group.dataset.flow = blocked ? 'blocked' : muted ? 'muted' : 'ready';
    if (muted) downstream.add(to);
    if (blocked) {
     const x = (x1+x2)/2, y = sameRow ? y1 : (y1+y2)/2+(from.dataset.slot==='global'?-6:from.dataset.slot==='local'?6:0);
     stop.setAttribute('d',`M ${x-3} ${y-3} l 6 6 M ${x-3} ${y+3} l 6 -6`);
    }
    const horizontal=sameRow?(bx>=ax?1:-1):0;const vertical=sameRow?0:(by>=ay?1:-1);const arrow=horizontal?`M ${x2-horizontal*5} ${y2-3} L ${x2} ${y2} L ${x2-horizontal*5} ${y2+3}`:`M ${x2-3} ${y2-vertical*5} L ${x2} ${y2} L ${x2+3} ${y2-vertical*5}`;if(head.getAttribute('d')!==arrow)head.setAttribute('d',arrow);

   }
  }
 }
 if(window.ResizeObserver){const observer=new ResizeObserver(scheduleConnections);for(const item of diagram){observer.observe(item.body);if(item.inputGroup)observer.observe(item.inputGroup);item.nodes.forEach(n=>observer.observe(n));}}
 else window.addEventListener('resize',scheduleConnections);
 function paintOverview(payload, blockers = lifecycleBlockers(payload)){
  const rows=payload.telemetry?.nodes||{},maps=payload.telemetry?.costmaps||{},live=!!payload.available;
  const counts={ok:0,waiting:0,unconfigured:0,transition:0,error:0,unknown:0};
  Object.keys(payload.nodes||{}).forEach(n=>{
   const state = lifecycleState(rows[n],live);
   counts[state.label==='未設定'?'unconfigured':state.tone]++;
  });
  const summary=document.getElementById('lc-summary'),summaryShape=JSON.stringify([counts,payload.domain,Object.keys(payload.nodes||{}).length]);
  if(summary._shape!==summaryShape){summary._shape=summaryShape;summary.replaceChildren();
   for(const [tone,label] of [['ok','稼働中'],['waiting','待機中'],['unconfigured','未設定'],['transition','遷移中'],['error','エラー'],['unknown','未取得']]){
    if(counts[tone]===0 && Object.keys(payload.nodes||{}).length)continue;
    const chip=text('span',label+' '+counts[tone],'lc-count');chip.dataset.tone=tone==='unconfigured'?'waiting':tone;summary.append(chip);
   }
   summary.append(text('small','Domain '+payload.domain,'lc-domain'));
  }
  const extra=lifecycleGraphState(Date.now()-monitorAt<8000?monitor:null,payload.domain,navigationDiagnosticMapReady);
  const blockedNodes = new Set(blockers.map(node => node.name));
  for(const node of flows.querySelectorAll('.lc-flow-node')){
   const key=node.dataset.key;let state=key.startsWith('@')?extra[key.slice(1)]:lifecycleState(rows[key],live);
   node.toggleAttribute('data-blocked', blockedNodes.has(key));
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
  const blockers = lifecycleBlockers(payload);
  paintHealth(payload, blockers);
  paintAuto(payload.auto_activation);
  paintStartup(payload);
  const rows=payload.telemetry?.nodes||{},logs=payload.telemetry?.logs||{},maps=payload.telemetry?.costmaps||{};
  const jobs=payload.operations||[],job=jobs[0],inFlight=jobs.some(j=>j.status==='running');
  const live=!!payload.available;
  paintOverview(payload, blockers);
  const jobBox=document.getElementById('lc-job');
  jobBox.hidden=!job||(job.status==='done'&&Number.isFinite(job.finished)&&Date.now()/1000-job.finished>=15);
  if(job){jobBox.dataset.state=job.status;jobBox.textContent=(job.source==='automatic'?'自動 · ':'')+'/'+job.node+'：'+(actions[job.action]?.label||job.action)+' '+(job.status==='running'?'実行中（自動再送しません）':job.result?.ok?'目的状態を確認済み':job.result?.error||'結果を確認できません');}
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
   ...jobs.map(e=>({time:e.started,text:(e.source==='automatic'?'自動操作：':'操作：')+'/'+e.node+' '+(actions[e.action]?.label||e.action)+' · '+({running:'実行中',done:'確認済み',failed:'失敗・要確認'})[e.status]}))].sort((a,b)=>b.time-a.time).slice(0,60);
  for(const entry of records){history.append(text('li',new Date(entry.time*1000).toLocaleTimeString('ja-JP')+' '+entry.text));}
  if(!records.length)history.append(text('li','履歴はまだありません'));
 }
 async function refresh(force=false){
  const interval=isPageVisible('lifecycle')?1500:5000;
  if(busy||document.hidden||(!force&&Date.now()-lastPoll<interval))return;
  busy=true;lastPoll=Date.now();
  try{const r=await fetch('/api/lifecycle',{cache:'no-store',signal:AbortSignal.timeout(5000)});const d=await r.json();if(!r.ok)throw Error(d.error||r.status);render(d);}
  catch(error){shape='';monitor=null;paintHealth({available:false});if(data)render({...data,available:false,auto_activation:null});const summary=document.getElementById('lc-summary');summary._shape=null;summary.textContent='現在状態を取得できません：'+error.message;document.querySelectorAll('#lc-nodes button').forEach(b=>b.disabled=true);}
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
 function paintAuto(mode) {
  const known = typeof mode?.enabled === 'boolean';
  autoToggle.disabled = !known || autoPending;
  autoToggle.setAttribute('aria-checked', String(mode?.enabled === true));
  document.getElementById('lc-auto-state').textContent = known ? mode.enabled ? 'ON' : 'OFF' : '—';
  document.getElementById('lc-auto-description').textContent = known
   ? '必須ノードが'+mode.wait_seconds+'秒以上待機したら有効化します' : '自動有効化の状態を確認中';
 }
 autoToggle.onclick = async () => {
  const mode = data?.auto_activation;
  if (autoPending || typeof mode?.enabled !== 'boolean') return;
  const enabled = !mode.enabled;
  if (enabled && !confirm('待機中の必須ノードを自動で有効化します。\n\n有効化後、既存ゴールの走行が再開する場合があります。自動有効化をONにしますか？')) return;
  autoPending = true;
  autoError.hidden = true;
  paintAuto(mode);
  try {
   const response = await fetch('/api/lifecycle/auto-activation', {method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled}),signal:AbortSignal.timeout(6000)});
   const result = await response.json();
   if (!response.ok) throw Error(result.error || response.status);
   if (data) data.auto_activation = result.auto_activation;
  } catch (error) {
   autoError.textContent = '設定結果を確認できません：'+error.message;
   autoError.hidden = false;
  } finally {
   autoPending = false;
   paintAuto(data?.auto_activation);
   refresh(true);
  }
 };
 document.getElementById('lc-refresh').onclick=()=>refresh(true);
 document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
 setInterval(refresh,1000);refresh();
})();
