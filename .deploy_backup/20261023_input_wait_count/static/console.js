'use strict';
async function closeTerm(n){if(!confirm('このターミナルを終了しますか？\n実行中のコマンドも終了します。'))return;await api('/api/term/close',{name:n});TERM.sessions=TERM.sessions.filter(x=>x!==n);delete TERM.offsets[n];if(TERM.active===n){TERM.active=TERM.sessions[TERM.sessions.length-1]||null;}renderTermTabs();document.getElementById('termout').textContent='';if(TERM.active)selectTerm(TERM.active);}
// Keep program controls in the static bundle so UI updates preserve live PTYs.
async function restartIt(n,el){if(OPS[n]||!confirm('「'+n+'」を一度停止して再起動しますか？'))return;OPS[n]='再起動中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/restart',{name:n,domain:dom()});}finally{await finishOp(n,'再起動: '+n);}}
async function toggle(n,b){if(OPS[n])return;const cur=!!(b&&b.dataset.running==='1');
 if(cur)return stopIt(n,b);
 OPS[n]='起動中…';if(b)b.classList.add('busy');refresh();
 try{const res=await api('/api/start',{name:n,domain:dom()});if(res.error){delete OPS[n];toast('起動できません: '+n+' ('+res.error+')',true);refresh();return;}await finishOp(n,'起動: '+n);}
 catch(e){delete OPS[n];toast('操作に失敗: '+n,true);refresh();}}
async function stopIt(n,el){if(OPS[n]||!confirm('「'+n+'」を停止しますか？'))return;OPS[n]='停止中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/stop',{name:n});}finally{await finishOp(n,'停止: '+n);}}

let PAGE='home', RESOURCE_DATA=null, RESOURCE_BUSY=false, REFRESH_BUSY=false;
let PROCESS_SCOPE='related', PROCESS_SORT='cpu_percent', PROCESS_ASC=false, PROCESS_LIMIT=40;
let LAST_GOOD_STATUS=null, LAST_RESOURCE_POLL=0;
const PAGE_LABELS={home:'Home',terminal:'Terminal',resources:'Monitor',visualization:'3D Visualization',tf:'TF Tree',settings:'設定',docs:'操作ガイド',files:'ファイル'};
window.isPageVisible=page=>!!(window.APP_SPLIT?.has(page)||PAGE===page);
const originalApi=api;
api=async function(path,body){
  const result=await originalApi(path,body);
  if(result.error==='auth')throw new Error('ログインが必要です');
  return result;
};
function setPage(page){
  if(!PAGE_LABELS[page])page='home';
  PAGE=page;document.body.dataset.page=page;
  document.querySelectorAll('.page').forEach(el=>el.classList.toggle('active',isPageVisible(el.id.slice(5))));
  document.querySelectorAll('.nav-item[data-page]').forEach(el=>{const selected=el.dataset.page===page;el.classList.toggle('active',selected);el.setAttribute('aria-current',selected?'page':'false');});
  document.getElementById('page-label').textContent=PAGE_LABELS[page];
  document.getElementById('term').classList.toggle('open',isPageVisible('terminal'));
  if(window.teleopPageChanged)teleopPageChanged(page);
  if(isPageVisible('terminal')){renderTermTabs();if(TERM.active)pollTerm();}
  if(isPageVisible('resources'))pollResources(true);
  if(isPageVisible('tf'))pollTF();
  if(isPageVisible('files')&&window.fileBrowserPageChanged)fileBrowserPageChanged();
  if(isPageVisible('docs')&&window.initDocumentation)initDocumentation();
  if(window.visualizationPageChanged&&(typeof VIZ==='undefined'||VIZ.pageVisible!==isPageVisible('visualization')))visualizationPageChanged(page);
  history.replaceState(null,'','#'+page);
  if(window.parent!==window){try{parent.postMessage({type:'sirius:page',page},location.origin);}catch(error){}}
  document.querySelector('.workspace').scrollTop=0;
}
function toggleTerm(){setPage(PAGE==='terminal'?'home':'terminal');}
function toggleAllPresets(){const open=document.body.classList.toggle('show-all-presets');document.getElementById('more-presets').textContent=open?'折りたたむ':'すべてのプリセットを見る';}
function setTheme(theme){
  theme=theme==='light'?'light':'dark';document.documentElement.dataset.theme=theme;
  localStorage.setItem('sirius-theme',theme);document.getElementById('theme-select').value=theme;
}
async function toggleFullscreen(){
  if(window.parent!==window&&typeof parent.siriusToggleFullscreen==='function'){await parent.siriusToggleFullscreen();return;}
  try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}
  catch(error){toast('ブラウザの F11 でもフルスクリーンにできます');}
}
window.addEventListener('message',event=>{
  if(event.origin!==location.origin||!event.data)return;
  if(event.data.type==='sirius:toast'&&event.data.message)toast(event.data.message,true);
});
function connection(ok){const el=document.getElementById('connection');el.classList.toggle('offline',!ok);el.querySelector('span').textContent=ok?'接続済み':'再接続中';}
function paintHomeHealth(status){
  const nav=document.querySelector('.nav-item[data-page="home"]');if(!nav)return;
  let badge=document.getElementById('home-error-badge');
  if(!badge){badge=document.createElement('span');badge.id='home-error-badge';badge.setAttribute('aria-hidden','true');nav.append(badge);}
  const count=Object.values(status).filter(state=>state&&state.error===true).length;
  badge.hidden=count===0;badge.textContent=count>99?'99+':String(count);
  nav.title=count?'Home：プロセスエラー '+count+' 件':'Home';
  nav.setAttribute('aria-label',count?'Home、プロセスエラー '+count+' 件':'Home');
}
function openProgramInput(name){setPage('terminal');selectTerm('program:'+name,true);}
function paintInputWaiting(status){
  const count=Object.values(status).filter(state=>state?.running&&state.input_waiting).length;
  const nav=document.querySelector('.nav-item[data-page="terminal"]');if(!nav)return;
  let badge=document.getElementById('terminal-wait-badge');
  if(!badge){badge=document.createElement('span');badge.id='terminal-wait-badge';badge.className='input-wait-nav-badge';badge.setAttribute('aria-hidden','true');nav.append(badge);}
  badge.hidden=count===0;badge.textContent=count>99?'99+':String(count);
  nav.title=count?'Terminal：入力待ち '+count+' 件':'Terminal';
  nav.setAttribute('aria-label',count?'Terminal、入力待ち '+count+' 件':'Terminal');
}
function paintSummary(status){
  LAST_GOOD_STATUS=Date.now();connection(true);
  paintHomeHealth(status);paintInputWaiting(status);
  const running=Object.entries(status).filter(([name,state])=>state.running);
  document.getElementById('footer-status').textContent=running.length+' programs running';
  const target=document.getElementById('terminal-program-list');target.replaceChildren();
  for(const [name,state] of running){
    const row=document.createElement('div');row.className='terminal-program-row';
    const button=document.createElement('button');button.className='terminal-program-open';button.innerHTML=ICONS.terminal;
    const text=document.createElement('span');text.textContent=name;button.append(text);
    button.title=name+' のコンソール';button.onclick=()=>selectTerm('program:'+name);
    const restart=document.createElement('button');restart.type='button';restart.className='terminal-program-restart';
    restart.textContent='↻';restart.title=name+' を再起動';restart.setAttribute('aria-label',name+'を再起動');
    restart.onclick=event=>{event.stopPropagation();restartIt(name,restart);};
    row.append(button,restart);target.append(row);
  }
  if(window.syncProgramTerminals)syncProgramTerminals(status);
  if(!running.length){const empty=document.createElement('p');empty.className='empty';empty.textContent='稼働中のプログラムはありません';target.append(empty);}
}
async function refresh(){
  if(REFRESH_BUSY||!DATA)return;REFRESH_BUSY=true;
  try{await refreshStatus();}catch(error){connection(false);document.getElementById('footer-status').textContent='ホストへの接続を再試行しています';}
  finally{REFRESH_BUSY=false;}
}
function bytes(value, decimals=1){
  if(value===null||value===undefined||!Number.isFinite(value))return '—';
  if(value>=1024**3)return (value/1024**3).toFixed(decimals)+' GiB';
  if(value>=1024**2)return (value/1024**2).toFixed(decimals)+' MiB';
  if(value>=1024)return (value/1024).toFixed(decimals)+' KiB';return Math.round(value)+' B';
}
function percent(value,decimals=1){return Number.isFinite(value)?value.toFixed(decimals)+'%':'—';}
function uptime(seconds){
  if(!Number.isFinite(seconds))return '—';
  seconds=Math.max(0,Math.floor(seconds));
  if(seconds<60)return seconds+'s';
  const days=Math.floor(seconds/86400),hours=Math.floor(seconds%86400/3600),minutes=Math.floor(seconds%3600/60);
  return days?days+'d '+hours+'h':hours?hours+'h '+minutes+'m':minutes+'m';
}
function metric(key,value,detail,amount){
  document.getElementById('metric-'+key).innerHTML=value;
  document.getElementById('detail-'+key).textContent=detail;
  const meter=document.getElementById('meter-'+key);meter.style.width=Math.min(100,Math.max(0,amount||0))+'%';
}
async function pollResources(force=false){
  if(RESOURCE_BUSY||(!force&&Date.now()-LAST_RESOURCE_POLL<(isPageVisible('resources')?1600:5000)))return;
  RESOURCE_BUSY=true;LAST_RESOURCE_POLL=Date.now();
  try{
    const data=await api('/api/resources'+(isPageVisible('resources')?'':'?summary=1'));
    if(data.hostname){document.getElementById('host-name').textContent=data.hostname;document.getElementById('settings-host').textContent=data.hostname;}
    if(!data.available){document.getElementById('sample-status').innerHTML='<i class="live-dot"></i>最初の測定を待っています';return;}
    RESOURCE_DATA=data;renderResources(data);
  }catch(error){document.getElementById('sample-status').classList.add('stale');document.getElementById('sample-status').textContent='接続を再試行しています';}
  finally{RESOURCE_BUSY=false;}
}
function renderResources(data){
  const memory=data.memory,disk=data.disk;
  metric('cpu',data.cpu_percent.toFixed(1)+'<small>%</small>',data.logical_cpus+' logical cores',data.cpu_percent);
  metric('memory',(memory.used/1024**3).toFixed(1)+'<small>/ '+(memory.total/1024**3).toFixed(1)+' GiB</small>',percent(data.memory_percent)+' 使用中 · '+bytes(memory.available)+' 空き',data.memory_percent);
  metric('disk',disk.percent.toFixed(1)+'<small>%</small>',bytes(disk.free)+' 空き / '+bytes(disk.total),disk.percent);
  const temps=(data.temperatures||[]).filter(t=>/coretemp|k10temp|cpu|package|core/i.test((t.sensor||'')+' '+t.name));
  const temp=temps.length?Math.max(...temps.map(t=>t.celsius)):null;
  metric('temperature',temp===null?'—':temp.toFixed(0)+'<small>°C</small>',temp===null?'このホストでは取得できません':'現在の最高コア温度',temp===null?0:temp);
  document.getElementById('mini-cpu').textContent=percent(data.cpu_percent,0);
  document.getElementById('mini-memory').textContent=percent(data.memory_percent,0);
  document.getElementById('core-count').textContent=data.logical_cpus+' cores';
  document.getElementById('load-average').textContent='Load '+data.load_average.map(n=>n.toFixed(2)).join(' / ');
  const cores=document.getElementById('cpu-cores');cores.replaceChildren();
  data.cpu_cores.forEach((value,i)=>{const core=document.createElement('div');core.className='core';core.innerHTML='<span>'+String(i+1).padStart(2,'0')+'</span><b>'+value.toFixed(0)+'%</b><i style="width:'+Math.min(value,100)+'%"></i>';cores.append(core);});
  const fresh=Date.now()/1000-data.timestamp<5;
  const stamp=new Date(data.timestamp*1000).toLocaleTimeString('ja-JP',{hour12:false});
  const sample=document.getElementById('sample-status');sample.classList.toggle('stale',!fresh);sample.innerHTML='<i class="live-dot"></i>'+(fresh?'LIVE · ':'更新停止 · ')+stamp;
  document.getElementById('cpu-chart-value').textContent=percent(data.cpu_percent);
  document.getElementById('memory-chart-value').textContent=percent(data.memory_percent);
  if(data.history){chart('cpu',data.history,'cpu_percent','var(--accent)');chart('memory',data.history,'memory_percent','var(--mint)');}
  document.getElementById('resource-system').textContent=data.hostname+' · '+data.platform+' · Uptime '+uptime(data.uptime_seconds)+' · Swap '+bytes(data.swap.used)+' / '+bytes(data.swap.total);
  const interfaces=(data.interfaces||[]).filter(n=>!/^tailscale|^docker|^veth|^br-/.test(n.name)).sort((a,b)=>(b.receive_bytes_per_second||0)-(a.receive_bytes_per_second||0));
  const net=interfaces[0];document.getElementById('resource-network').textContent=net?net.name+'  ↓ '+bytes(net.receive_bytes_per_second)+'/s  ↑ '+bytes(net.send_bytes_per_second)+'/s':'ネットワーク速度を取得できません';
  if(data.processes)renderProcesses();
}
function chart(id,history,key,color){
  const W=600,H=100,now=history.length?history[history.length-1].timestamp:0;
  const points=history.map(p=>[Math.max(0,Math.min(W,(p.timestamp-now+120)/120*W)),H-Math.max(0,Math.min(100,p[key]))/100*H]);
  const line=points.map((p,i)=>(i?'L':'M')+p[0].toFixed(2)+','+p[1].toFixed(2)).join(' ');
  const fill=points.length?line+' L '+points.at(-1)[0]+','+H+' L '+points[0][0]+','+H+' Z':'';
  let svg='<svg viewBox="0 0 600 110" preserveAspectRatio="none" role="img" aria-label="'+(id==='cpu'?'CPU':'メモリ')+'使用率の推移"><defs><linearGradient id="gradient-'+id+'" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stop-color="'+color+'" stop-opacity=".18"/><stop offset="100%" stop-color="'+color+'" stop-opacity="0"/></linearGradient></defs>';
  for(const y of [0,50,100])svg+='<line x1="0" y1="'+(y+3)+'" x2="600" y2="'+(y+3)+'" stroke="var(--line)" stroke-width=".7" stroke-dasharray="3 5"/>';
  if(points.length)svg+='<g transform="translate(0 3)"><path d="'+fill+'" fill="url(#gradient-'+id+')"/><path d="'+line+'" fill="none" stroke="'+color+'" stroke-width="1.7" vector-effect="non-scaling-stroke"/><circle cx="'+points.at(-1)[0]+'" cy="'+points.at(-1)[1]+'" r="2.4" fill="'+color+'"/></g>';
  document.getElementById(id+'-chart').innerHTML=svg+'</svg>';
}
function setProcessScope(scope){PROCESS_SCOPE=scope;PROCESS_LIMIT=40;document.querySelectorAll('[data-scope]').forEach(el=>el.classList.toggle('active',el.dataset.scope===scope));renderProcesses();}
function sortProcesses(key){PROCESS_ASC=PROCESS_SORT===key?!PROCESS_ASC:false;PROCESS_SORT=key;renderProcesses();}
function showMoreProcesses(){PROCESS_LIMIT+=40;renderProcesses();}
function renderProcesses(){
  if(!RESOURCE_DATA||!RESOURCE_DATA.processes)return;
  const query=document.getElementById('process-search').value.trim().toLowerCase();
  const hiddenCount=RESOURCE_DATA.processes.filter(p=>p.hidden).length;
  const hiddenCountEl=document.getElementById('process-hidden-count');
  if(hiddenCountEl)hiddenCountEl.textContent=hiddenCount?' '+hiddenCount:'';
  const rows=RESOURCE_DATA.processes.filter(p=>(
    PROCESS_SCOPE==='all'||(PROCESS_SCOPE==='related'&&p.related)||
    (PROCESS_SCOPE==='hidden'&&p.hidden)||(PROCESS_SCOPE==='unrelated'&&p.unrelated)
  )&&(!query||[p.name,p.alias,p.command,String(p.pid)].some(s=>(s||'').toLowerCase().includes(query))));
  rows.sort((a,b)=>((b[PROCESS_SORT]||0)-(a[PROCESS_SORT]||0))*(PROCESS_ASC?-1:1)||a.pid-b.pid);
  document.getElementById('sort-cpu').textContent='CPU'+(PROCESS_SORT==='cpu_percent'?(PROCESS_ASC?' ↑':' ↓'):'');
  document.getElementById('sort-memory').textContent='メモリ'+(PROCESS_SORT==='memory_bytes'?(PROCESS_ASC?' ↑':' ↓'):'');
  document.getElementById('process-count').textContent=rows.length;
  const body=document.getElementById('resource-processes');body.replaceChildren();
  for(const p of rows.slice(0,PROCESS_LIMIT)){
    const tr=document.createElement('tr');const title=document.createElement('td');
    const name=document.createElement('div');name.className='process-name';
    const badge=document.createElement('span');badge.textContent=(p.alias||p.name).slice(0,2).toUpperCase();
    const label=document.createElement('span');label.className='process-label';
    const b=document.createElement('b');b.textContent=p.alias||p.name;
    const small=document.createElement('small');
    small.textContent=(p.hidden?'隠れ · ':'')+(p.alias?p.name:(p.command||p.status));
    if(p.hidden)small.classList.add('process-hidden');
    label.append(b,small);name.append(badge,label);title.append(name);title.title=p.command+(p.hidden?'（Launcher管理外）':'');
    tr.append(title);
    for(const [value,cls] of [[p.pid,''],[percent(p.cpu_percent),'process-cpu'],[bytes(p.memory_bytes),''],[p.threads,''],[uptime(p.age_seconds),'']]){const cell=document.createElement('td');cell.textContent=value;cell.className=cls;tr.append(cell);}
    const action=document.createElement('td');const button=document.createElement('button');button.className='process-detail-btn';button.innerHTML=ICONS.chevron;button.title='PID '+p.pid+' の詳細';button.setAttribute('aria-label',button.title);button.onclick=()=>processDetail(p);action.append(button);tr.append(action);body.append(tr);
  }
  if(!rows.length){const tr=document.createElement('tr');const td=document.createElement('td');td.colSpan=7;td.className='table-empty';td.textContent=query?'一致するプロセスがありません':PROCESS_SCOPE==='hidden'?'管理外の隠れプロセスはありません':PROCESS_SCOPE==='unrelated'?'関連のないプロセスはありません':'対象のプロセスがありません';tr.append(td);body.append(tr);}
  document.getElementById('process-more').hidden=rows.length<=PROCESS_LIMIT;
}
function processDetail(p){
  let modal=document.getElementById('process-detail');if(modal)modal.remove();
  modal=document.createElement('div');modal.className='modal open';modal.id='process-detail';modal.setAttribute('role','dialog');modal.setAttribute('aria-label','プロセスの詳細');
  const box=document.createElement('div');box.className='modalbox';box.style.maxWidth='650px';
  const heading=document.createElement('h3');heading.textContent=(p.alias||p.name)+(p.hidden?'（隠れ）':'');
  const relation=p.hidden?(p.hidden_kind==='duplicate'?'隠れ（重複起動）':'隠れ（Launcher管理外）'):p.related?'Sirius / ROS':'関連なし';
  const details=document.createElement('dl');details.className='detail-list';
  for(const [label,value] of [['PID',p.pid],['関係',relation],['起動管理',p.managed?'管理下':p.hidden?'管理外':'—'],['CPU',percent(p.cpu_percent)+' / 1 core'],['メモリ',bytes(p.memory_bytes)],['ステータス',p.status],['稼働時間',uptime(p.age_seconds)]]){const dt=document.createElement('dt');dt.textContent=label;const dd=document.createElement('dd');dd.textContent=value;details.append(dt,dd);}
  const command=document.createElement('pre');command.className='detail-command';command.textContent=p.command||p.name;
  const actions=document.createElement('div');actions.className='detail-actions';
  const stop=document.createElement('button');stop.className='hbtn danger';stop.textContent='停止';stop.title='プロセスと子孫を停止';stop.onclick=()=>stopProcess(p,modal);actions.append(stop);
  if(p.alias&&!p.hidden){const log=document.createElement('button');log.className='hbtn';log.textContent='ログを確認';log.onclick=()=>{modal.remove();showLog(p.alias);};actions.append(log);}
  const close=document.createElement('button');close.className='hbtn';close.textContent='閉じる';close.onclick=()=>modal.remove();actions.append(close);box.append(heading,details,command,actions);modal.append(box);document.body.append(modal);
  modal.onclick=e=>{if(e.target===modal)modal.remove();};close.focus();
}
async function stopProcess(p,modal){
  const program=!p.hidden&&(p.session||p.alias);
  const target=program?('「'+program+'」'+(p.session?'（セッション）':'')):('PID '+p.pid+(p.alias?'（'+p.alias+'）':''));
  if(!confirm(target+' を停止しますか？（子孫も停止）'))return;
  try{
    const res=program?await api('/api/stop',{name:program}):await api('/api/ps/kill',{pid:p.pid});
    if(res.error){toast('停止できません: '+res.error,true);return;}
    if(modal)modal.remove();toast('停止: '+target);refresh();
    if(PAGE==='resources')setTimeout(()=>pollResources(true),800);
  }catch(e){toast('停止に失敗しました',true);}
}
renderTermTabs=function(){
  const bar=document.getElementById('termtabs');bar.replaceChildren();
  TERM.sessions.forEach((name,i)=>{const tab=document.createElement('button');tab.className='ttab'+(name===TERM.active?' active':'');tab.innerHTML=ICONS.terminal;
    const label=document.createElement('span');label.textContent=name.startsWith('shell_')?'Terminal '+String(i+1).padStart(2,'0'):name;tab.append(label);tab.onclick=()=>selectTerm(name);
    const close=document.createElement('span');close.className='x';close.textContent='×';close.title='セッションを終了';close.onclick=e=>{e.stopPropagation();closeTerm(name);};tab.append(close);bar.append(tab);});
  const interrupt=document.getElementById('term-interrupt');if(interrupt)interrupt.disabled=!TERM.active;
  if(!TERM.sessions.length){const label=document.createElement('span');label.className='muted';label.style.padding='5px';label.textContent='新しいターミナルを開くと、ここに表示されます';bar.append(label);}
  const add=document.createElement('button');add.className='ttab new-term';add.title='新しいターミナル';add.setAttribute('aria-label','新しいターミナル');add.innerHTML=ICONS.plus+'<span>新規</span>';add.onclick=()=>newTerm('');bar.append(add);
};

async function sendTermControl(){
  if(!TERM.active)return;
  const name=TERM.active,input=document.getElementById('termin'),value=input.value;
  try{
    const result=await api('/api/term/write',{name,data:String.fromCharCode(3)});
    if(!result.ok){toast(result.error||'割り込みを送信できません',true);return;}
    if(TERM.active===name&&input.value===value)input.value='';
    setTimeout(pollTerm,100);
  }catch(error){toast('割り込みを送信できませんでした',true);}
}
function termInput(e){
  if(e.isComposing||e.keyCode===229||e.key!=='Enter'||!TERM.active)return;
  e.preventDefault();const input=document.getElementById('termin'),value=input.value;
  input.value='';api('/api/term/write',{name:TERM.active,data:value+'\n'});setTimeout(pollTerm,150);
}
function initTerminalControls(){
  const output=document.getElementById('termout');output.tabIndex=0;output.setAttribute('aria-label','ターミナル出力');
  const button=document.createElement('button');button.id='term-interrupt';button.className='hbtn';button.textContent='Ctrl+C';
  button.title='実行中のコマンドに割り込みを送る';button.disabled=!TERM.active;button.onclick=sendTermControl;
  document.querySelector('#term .tin').append(button);

}
initTerminalControls();

function reloadConsole(){
  sessionStorage.setItem('sirius-active-terminal',TERM.active||'');
  if(window.parent!==window&&typeof parent.siriusReloadApp==='function'){parent.siriusReloadApp();return;}
  location.reload();
}
(function(){
  const brand=document.querySelector('.brand'),button=document.createElement('button');
  button.className=brand.className;button.innerHTML=brand.innerHTML;button.title='画面を再読み込み';button.setAttribute('aria-label','画面を再読み込み');button.onclick=reloadConsole;
  const badge=document.createElement('span');badge.className='brand-reload';badge.setAttribute('aria-hidden','true');badge.innerHTML=ICONS.refresh;button.append(badge);brand.replaceWith(button);
  const title=document.querySelector('.app-name'),reload=document.createElement('button');
  reload.type='button';reload.id='header-reload';reload.className='app-name header-reload';
  reload.title='ページ全体を再読み込み';reload.setAttribute('aria-label',reload.title);
  reload.append(...title.childNodes);reload.onclick=reloadConsole;title.replaceWith(reload);
})();
function initPanelCollapse(panel,button,key,openTitle,closeTitle){
  if(!panel||!button)return;
  function apply(collapsed){
    panel.classList.toggle('collapsed',collapsed);
    button.setAttribute('aria-expanded',String(!collapsed));
    button.title=collapsed?openTitle:closeTitle;
  }
  apply(localStorage.getItem(key)==='1');
  button.addEventListener('click',()=>{
    const collapsed=!panel.classList.contains('collapsed');
    apply(collapsed);
    localStorage.setItem(key,collapsed?'1':'0');
  });
}
initPanelCollapse(document.querySelector('.panel.presets'),document.getElementById('preset-toggle'),'sirius-presets-collapsed','クイックスタートを展開','クイックスタートを折りたたむ');
initPanelCollapse(document.querySelector('.sidebar'),document.getElementById('running-toggle'),'sirius-running-collapsed','稼働中を展開','稼働中を折りたたむ');
function initHomeSplit(){
  const layout=document.querySelector('.home-layout'),splitter=document.getElementById('gutter');
  if(!layout||!splitter)return;
  const MIN_MAIN=360,MIN_SIDE=220;
  function clampShare(value){
    const width=layout.clientWidth;
    if(!width)return value;
    const min=MIN_SIDE/width,max=Math.max(min,1-MIN_MAIN/width);
    return Math.min(max,Math.max(min,value));
  }
  let share=parseFloat(localStorage.getItem('sirius-home-side-share'));
  if(!Number.isFinite(share))share=.26;
  function apply(){if(layout.clientWidth)layout.style.setProperty('--home-side-share',(clampShare(share)*100).toFixed(2)+'%');}
  apply();
  let drag=false;
  splitter.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag=true;splitter.setPointerCapture(e.pointerId);e.preventDefault();document.body.classList.add('split-dragging');});
  splitter.addEventListener('pointermove',e=>{if(!drag)return;const rect=layout.getBoundingClientRect();if(!rect.width)return;share=clampShare((rect.right-e.clientX)/rect.width);apply();});
  function end(){if(!drag)return;drag=false;document.body.classList.remove('split-dragging');localStorage.setItem('sirius-home-side-share',String(share));}
  splitter.addEventListener('pointerup',end);splitter.addEventListener('pointercancel',end);
  splitter.addEventListener('keydown',e=>{const step=e.key==='ArrowLeft'?-.02:e.key==='ArrowRight'?.02:0;if(!step)return;e.preventDefault();share=clampShare(share+step);apply();localStorage.setItem('sirius-home-side-share',String(share));});
  window.addEventListener('resize',apply);
}
initHomeSplit();
function initHomeLayout(){
  const main=document.querySelector('.home-main'),presets=main.querySelector('.presets'),sidebar=document.querySelector('.sidebar');
  const side=document.createElement('div');side.className='home-side';sidebar.before(side);side.append(sidebar);
  const desktop=matchMedia('(min-width:821px)');
  function arrange(){if(desktop.matches)side.prepend(presets);else main.prepend(presets);}
  desktop.addEventListener('change',arrange);arrange();
}
initHomeLayout();
async function initConsole(){
  setTheme(localStorage.getItem('sirius-theme')||'dark');
  try{await load();document.getElementById('autostart-status').textContent=DATA.console&&DATA.console.autostart?'自動起動 有効':'自動起動 未登録';const terms=await api('/api/term/list');for(const term of terms.terms||[])if(term.alive&&!TERM.sessions.includes(term.name))TERM.sessions.push(term.name);if(TERM.sessions.length){const saved=sessionStorage.getItem('sirius-active-terminal');TERM.active=(TERM.sessions.includes(saved)||(saved?.startsWith('program:')&&DATA.groups.some(g=>g.items.some(item=>item.name===saved.slice(8)))))?saved:TERM.sessions[0];}setPage(location.hash.slice(1)||'home');}
  catch(error){connection(false);toast('ホストに接続できません。再読み込みで接続を確認してください。',true);}
  pollResources(true);
}
setInterval(()=>{document.getElementById('clock').textContent=new Date().toLocaleTimeString('ja-JP',{hour:'2-digit',minute:'2-digit',hour12:false});if(LAST_GOOD_STATUS&&Date.now()-LAST_GOOD_STATUS>7000)connection(false);},1000);
// TF health must update even while Home or another tab is open.
pollTF();setInterval(pollTF,1000);setInterval(refresh,2000);setInterval(()=>pollTerm(),200);setInterval(pollLog,700);setInterval(()=>pollResources(),2000);
window.addEventListener('hashchange',()=>setPage(location.hash.slice(1)));
document.addEventListener('keydown',e=>{if(e.key==='Escape'){document.querySelectorAll('.modal.open').forEach(m=>m.classList.remove('open'));closeLog();}});
initConsole();

// Battery telemetry remains visible on every console page.
(function(){const script=document.createElement('script');script.src='/static/battery.js';document.body.append(script);})();
