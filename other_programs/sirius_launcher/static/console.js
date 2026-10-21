'use strict';
async function closeTerm(n){if(!confirm('このターミナルを終了しますか？\n実行中のコマンドも終了します。'))return;await api('/api/term/close',{name:n});TERM.sessions=TERM.sessions.filter(x=>x!==n);delete TERM.offsets[n];if(TERM.active===n){TERM.active=TERM.sessions[TERM.sessions.length-1]||null;}renderTermTabs();document.getElementById('termout').textContent='';if(TERM.active)selectTerm(TERM.active);}
// Keep program controls in the static bundle so UI updates preserve live PTYs.
async function restartIt(n,el){if(OPS[n]||!confirm('「'+n+'」を一度停止して再起動しますか？'))return;OPS[n]='再起動中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/restart',{name:n,domain:dom()});}finally{await finishOp(n,'再起動: '+n);}}
async function toggle(n,b){if(OPS[n])return;const cur=!!(b&&b.dataset.running==='1');
 if(cur)return stopIt(n,b);
 OPS[n]='起動中…';if(b)b.classList.add('busy');refresh();
 try{const res=await api('/api/start',{name:n,domain:dom()});if(res.error){delete OPS[n];toast('起動できません: '+n+' ('+res.error+')',true);refresh();return;}await finishOp(n,'起動: '+n);await showLog(n);}
 catch(e){delete OPS[n];toast('操作に失敗: '+n,true);refresh();}}
async function stopIt(n,el){if(OPS[n]||!confirm('「'+n+'」を停止しますか？'))return;OPS[n]='停止中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/stop',{name:n});}finally{await finishOp(n,'停止: '+n);}}

let PAGE='home', RESOURCE_DATA=null, RESOURCE_BUSY=false, REFRESH_BUSY=false;
let PROCESS_SCOPE='related', PROCESS_SORT='cpu_percent', PROCESS_ASC=false, PROCESS_LIMIT=40;
let LAST_GOOD_STATUS=null, LAST_RESOURCE_POLL=0;
const PAGE_LABELS={home:'Home',terminal:'Terminal',resources:'Monitor',tf:'TF Tree',settings:'設定'};
const originalApi=api;
api=async function(path,body){
  const result=await originalApi(path,body);
  if(result.error==='auth')throw new Error('ログインが必要です');
  return result;
};
function setPage(page){
  if(!PAGE_LABELS[page])page='home';
  PAGE=page;document.body.dataset.page=page;
  document.querySelectorAll('.page').forEach(el=>el.classList.toggle('active',el.id==='page-'+page));
  document.querySelectorAll('.nav-item[data-page]').forEach(el=>{const selected=el.dataset.page===page;el.classList.toggle('active',selected);el.setAttribute('aria-current',selected?'page':'false');});
  document.getElementById('page-label').textContent=PAGE_LABELS[page];
  document.getElementById('term').classList.toggle('open',page==='terminal');
  if(page!=='home')toggleTeleop(false);
  if(page==='terminal'){renderTermTabs();if(TERM.active)pollTerm();}
  if(page==='resources')pollResources(true);
  if(page==='tf')pollTF();
  history.replaceState(null,'','#'+page);
  document.querySelector('.workspace').scrollTop=0;
}
function toggleTerm(){setPage(PAGE==='terminal'?'home':'terminal');}
function toggleAllPresets(){const open=document.body.classList.toggle('show-all-presets');document.getElementById('more-presets').textContent=open?'折りたたむ':'すべてのプリセットを見る';}
function setTheme(theme){
  theme=theme==='light'?'light':'dark';document.documentElement.dataset.theme=theme;
  localStorage.setItem('sirius-theme',theme);document.getElementById('theme-select').value=theme;
}
async function toggleFullscreen(){
  try{if(document.fullscreenElement)await document.exitFullscreen();else await document.documentElement.requestFullscreen();}
  catch(error){toast('ブラウザの F11 でもフルスクリーンにできます');}
}
function connection(ok){const el=document.getElementById('connection');el.classList.toggle('offline',!ok);el.querySelector('span').textContent=ok?'接続済み':'再接続中';}
function paintSummary(status){
  LAST_GOOD_STATUS=Date.now();connection(true);
  const running=Object.entries(status).filter(([name,state])=>state.running);
  document.getElementById('footer-status').textContent=running.length+' programs running';
  const target=document.getElementById('terminal-program-list');target.replaceChildren();
  for(const [name,state] of running){
    const button=document.createElement('button');button.innerHTML=ICONS.terminal;
    const text=document.createElement('span');text.textContent=name;button.append(text);
    button.title=name+' のログと入力';button.onclick=()=>showLog(name);target.append(button);
  }
  if(!running.length){const empty=document.createElement('p');empty.className='empty';empty.textContent='起動中のプログラムはありません';target.append(empty);}
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
  const days=Math.floor(seconds/86400),hours=Math.floor(seconds%86400/3600),minutes=Math.floor(seconds%3600/60);
  return days?days+'d '+hours+'h':hours?hours+'h '+minutes+'m':minutes+'m';
}
function metric(key,value,detail,amount){
  document.getElementById('metric-'+key).innerHTML=value;
  document.getElementById('detail-'+key).textContent=detail;
  const meter=document.getElementById('meter-'+key);meter.style.width=Math.min(100,Math.max(0,amount||0))+'%';
}
async function pollResources(force=false){
  if(RESOURCE_BUSY||(!force&&Date.now()-LAST_RESOURCE_POLL<(PAGE==='resources'?1600:5000)))return;
  RESOURCE_BUSY=true;LAST_RESOURCE_POLL=Date.now();
  try{
    const data=await api('/api/resources'+(PAGE==='resources'?'':'?summary=1'));
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
  const rows=RESOURCE_DATA.processes.filter(p=>(PROCESS_SCOPE==='all'||p.related)&&(!query||[p.name,p.alias,p.command,String(p.pid)].some(s=>(s||'').toLowerCase().includes(query))));
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
    const small=document.createElement('small');small.textContent=p.alias?p.name:p.command||p.status;
    label.append(b,small);name.append(badge,label);title.append(name);title.title=p.command;
    tr.append(title);
    for(const [value,cls] of [[p.pid,''],[percent(p.cpu_percent),'process-cpu'],[bytes(p.memory_bytes),''],[p.threads,''],[uptime(p.age_seconds),'']]){const cell=document.createElement('td');cell.textContent=value;cell.className=cls;tr.append(cell);}
    const action=document.createElement('td');const button=document.createElement('button');button.className='process-detail-btn';button.innerHTML=ICONS.chevron;button.title='PID '+p.pid+' の詳細';button.setAttribute('aria-label',button.title);button.onclick=()=>processDetail(p);action.append(button);tr.append(action);body.append(tr);
  }
  if(!rows.length){const tr=document.createElement('tr');const td=document.createElement('td');td.colSpan=7;td.className='table-empty';td.textContent=query?'一致するプロセスがありません':'対象のプロセスがありません';tr.append(td);body.append(tr);}
  document.getElementById('process-more').hidden=rows.length<=PROCESS_LIMIT;
}
function processDetail(p){
  let modal=document.getElementById('process-detail');if(modal)modal.remove();
  modal=document.createElement('div');modal.className='modal open';modal.id='process-detail';modal.setAttribute('role','dialog');modal.setAttribute('aria-label','プロセスの詳細');
  const box=document.createElement('div');box.className='modalbox';box.style.maxWidth='650px';
  const heading=document.createElement('h3');heading.textContent=p.alias||p.name;
  const details=document.createElement('dl');details.className='detail-list';
  for(const [label,value] of [['PID',p.pid],['CPU',percent(p.cpu_percent)+' / 1 core'],['メモリ',bytes(p.memory_bytes)],['ステータス',p.status],['稼働時間',uptime(p.age_seconds)]]){const dt=document.createElement('dt');dt.textContent=label;const dd=document.createElement('dd');dd.textContent=value;details.append(dt,dd);}
  const command=document.createElement('pre');command.className='detail-command';command.textContent=p.command||p.name;
  const actions=document.createElement('div');actions.className='detail-actions';
  if(p.alias){const log=document.createElement('button');log.className='hbtn';log.textContent='ログを確認';log.onclick=()=>{modal.remove();showLog(p.alias);};actions.append(log);}
  const close=document.createElement('button');close.className='hbtn';close.textContent='閉じる';close.onclick=()=>modal.remove();actions.append(close);box.append(heading,details,command,actions);modal.append(box);document.body.append(modal);
  modal.onclick=e=>{if(e.target===modal)modal.remove();};close.focus();
}
renderTermTabs=function(){
  const bar=document.getElementById('termtabs');bar.replaceChildren();
  TERM.sessions.forEach((name,i)=>{const tab=document.createElement('button');tab.className='ttab'+(name===TERM.active?' active':'');tab.innerHTML=ICONS.terminal;
    const label=document.createElement('span');label.textContent=name.startsWith('shell_')?'Terminal '+String(i+1).padStart(2,'0'):name;tab.append(label);tab.onclick=()=>selectTerm(name);
    const close=document.createElement('span');close.className='x';close.textContent='×';close.title='セッションを終了';close.onclick=e=>{e.stopPropagation();closeTerm(name);};tab.append(close);bar.append(tab);});
  const interrupt=document.getElementById('term-interrupt');if(interrupt)interrupt.disabled=!TERM.active;
  if(!TERM.sessions.length){const label=document.createElement('span');label.className='muted';label.style.padding='5px';label.textContent='新しいターミナルを開くと、ここに表示されます';bar.append(label);}
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
  document.querySelector('.terminal-workspace').addEventListener('keydown',e=>{
    if(e.defaultPrevented||e.isComposing||!e.ctrlKey||e.altKey||e.metaKey||e.key.toLowerCase()!=='c')return;
    const selectedInput=e.target.selectionStart!==undefined&&e.target.selectionStart!==e.target.selectionEnd;
    if(selectedInput||window.getSelection().toString())return;
    if(!TERM.active)return;e.preventDefault();sendTermControl();
  });
}
initTerminalControls();

function reloadConsole(){
  sessionStorage.setItem('sirius-active-terminal',TERM.active||'');
  location.reload();
}
(function(){
  const brand=document.querySelector('.brand'),button=document.createElement('button');
  button.className=brand.className;button.innerHTML=brand.innerHTML;button.title='画面を再読み込み';button.setAttribute('aria-label','画面を再読み込み');button.onclick=reloadConsole;
  const badge=document.createElement('span');badge.className='brand-reload';badge.setAttribute('aria-hidden','true');badge.innerHTML=ICONS.refresh;button.append(badge);brand.replaceWith(button);
})();
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
  try{await load();document.getElementById('autostart-status').textContent=DATA.console&&DATA.console.autostart?'自動起動 有効':'自動起動 未登録';const terms=await api('/api/term/list');for(const term of terms.terms||[])if(term.alive&&!TERM.sessions.includes(term.name))TERM.sessions.push(term.name);if(TERM.sessions.length){const saved=sessionStorage.getItem('sirius-active-terminal');TERM.active=TERM.sessions.includes(saved)?saved:TERM.sessions[0];}setPage(location.hash.slice(1)||'home');}
  catch(error){connection(false);toast('ホストに接続できません。再読み込みで接続を確認してください。',true);}
  pollResources(true);
}
setInterval(()=>{document.getElementById('clock').textContent=new Date().toLocaleTimeString('ja-JP',{hour:'2-digit',minute:'2-digit',hour12:false});if(LAST_GOOD_STATUS&&Date.now()-LAST_GOOD_STATUS>7000)connection(false);},1000);
setInterval(()=>{if(PAGE==='tf')pollTF();},1000);setInterval(refresh,2000);setInterval(pollTerm,700);setInterval(pollLog,700);setInterval(teleopPoll,2000);setInterval(()=>pollResources(),2000);
window.addEventListener('hashchange',()=>setPage(location.hash.slice(1)));
document.addEventListener('keydown',e=>{if(e.key==='Escape'){document.querySelectorAll('.modal.open').forEach(m=>m.classList.remove('open'));closeLog();}});
initConsole();
