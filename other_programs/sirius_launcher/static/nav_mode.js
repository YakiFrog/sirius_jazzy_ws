'use strict';
const NAV={modes:[],state:null,selected:null,busy:false,loading:null,pollBusy:false,modalOpen:false,returnFocus:null};

function navEsc(value){
  return String(value??'').replace(/[&<>"']/g,ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}
function navDomain(){try{return dom()||'';}catch(_){const el=document.getElementById('domain');return el?el.value:'';}}
function navStatusText(status){
  if(status==='ok')return '';
  if(status==='unavailable')return '状態を取得できません';
  if(status==='unknown_mode')return '不明なモード';
  return '状態未取得';
}
function navCurrent(){return NAV.state&&NAV.state.status==='ok'?NAV.state:null;}
function navFind(mode){return NAV.modes.find(item=>item.mode===mode)||null;}
function navSpeedText(speed){return Number.isFinite(Number(speed))?`${Number(speed).toFixed(2).replace(/0$/,'').replace(/\.$/,'')} m/s`:'—';}
function navStatusClass(status){
  if(status==='ok')return 'is-ok';
  if(status==='unavailable'||status==='unknown_mode')return 'is-unknown';
  return 'is-unknown';
}
function navSetStatusClass(el,status){
  if(!el)return;
  el.classList.remove('is-ok','is-unknown','is-applying','is-failure');
  if(status)el.classList.add(status);
}
function navPaint(){
  const state=NAV.state||{};
  const current=navCurrent();
  const status=state.status||'unknown';
  const label=current?current.label:navStatusText(status);
  const speed=current?navSpeedText(current.speed):'—';
  const header=document.getElementById('header-nav-mode');
  const headerLabel=document.getElementById('header-nav-mode-label');
  const headerSpeed=document.getElementById('header-nav-mode-speed');
  if(header&&headerLabel){
    headerLabel.textContent=current?current.label:(status==='unavailable'?'状態取得不可':'モード —');
    if(headerSpeed)headerSpeed.textContent=current?navSpeedText(current.speed):'';
    navSetStatusClass(header,NAV.busy?'is-applying':navStatusClass(status));
    header.title=current?`Nav2走行モード: ${current.label}（設定速度 ${navSpeedText(current.speed)}）`:'Nav2走行モードの状態を取得できません。クリックして確認';
    header.setAttribute('aria-label',`Nav2走行モード ${current?current.label:'状態未取得'}。変更パネルを開く`);
  }
  const card=document.getElementById('drive-nav-mode');
  if(card){
    const cardLabel=document.getElementById('drive-nav-label');
    const cardSpeed=document.getElementById('drive-nav-speed');
    const cardState=document.getElementById('drive-nav-state');
    if(cardLabel)cardLabel.textContent=current?current.label:navStatusText(status);
    if(cardSpeed)cardSpeed.textContent=current?`設定 ${navSpeedText(current.speed)}`:'—';
    if(cardState){
      cardState.textContent=current
        ?'設定速度です。実測速度ではありません。'
        :(status==='unavailable'?'状態を取得できません。モードを推測していません。':'状態未取得です。モードを推測していません。');
    }
    navSetStatusClass(card,NAV.busy?'is-applying':navStatusClass(status));
  }
  const currentLabel=document.getElementById('nav-mode-current-label');
  const currentSpeed=document.getElementById('nav-mode-current-speed');
  const currentStatus=document.getElementById('nav-mode-current-status');
  if(currentLabel){
    currentLabel.textContent=current?current.label:navStatusText(status);
    if(currentSpeed)currentSpeed.textContent=current?`設定速度 ${navSpeedText(current.speed)}`:'—';
    if(currentStatus){
      currentStatus.textContent=current
        ?'設定速度です。実測速度ではありません。'
        :(status==='unknown_mode'?`受信値: ${state.mode||'—'}`:'現在値を推測していません。');
    }
  }
  const list=document.getElementById('nav-mode-list');
  if(list&&NAV.modes.length){
    list.querySelectorAll('.nav-mode-option').forEach(button=>{
      const mode=button.dataset.mode;
      const isCurrent=current&&current.mode===mode;
      const isSelected=NAV.selected===mode;
      button.classList.toggle('is-current',!!isCurrent);
      button.setAttribute('aria-selected',String(isSelected));
    });
  }
  navPaintDetail();
  const apply=document.getElementById('nav-mode-apply');
  if(apply){
    apply.disabled=NAV.busy||!NAV.selected||!navFind(NAV.selected)||(current&&current.mode===NAV.selected);
    apply.textContent=NAV.busy?'変更中…':(current&&current.mode===NAV.selected?'現在のモード':'このモードに変更');
  }
}
function navPaintDetail(){
  const detail=document.getElementById('nav-mode-detail');
  if(!detail)return;
  const item=NAV.selected?navFind(NAV.selected):null;
  if(!item){
    detail.innerHTML='走行モードを選ぶと、説明と設定速度を確認できます。';
    return;
  }
  const current=navCurrent();
  const suffix=current&&current.mode===item.mode?'（現在のモード）':'';
  detail.innerHTML=`<strong>${navEsc(item.label)}${suffix}</strong> ${navSpeedText(item.speed)}<br>${navEsc(item.description||'')}`;
}
async function navLoadModes(force=false){
  if(NAV.modes.length&&!force)return NAV.modes;
  if(NAV.loading)return NAV.loading;
  NAV.loading=(async()=>{
    try{
      const response=await fetch('/api/navigation_modes',{cache:'no-store',signal:AbortSignal.timeout(15000)});
      const data=await response.json();
      if(!response.ok||!Array.isArray(data.modes))throw new Error(data.error||'モード一覧を取得できません');
      NAV.modes=data.modes;
      navRenderModes();
      return NAV.modes;
    }catch(error){
      NAV.modes=[];
      navRenderModes(error.message||'モード一覧を取得できません');
      return [];
    }finally{NAV.loading=null;}
  })();
  return NAV.loading;
}
function navRenderModes(error){
  const list=document.getElementById('nav-mode-list');
  if(!list)return;
  list.textContent='';
  if(error){
    const note=document.createElement('p');note.className='nav-mode-note';note.textContent=error;list.append(note);
    return;
  }
  if(!NAV.modes.length){
    const note=document.createElement('p');note.className='nav-mode-note';note.textContent='モード一覧を取得できません。';list.append(note);
    return;
  }
  for(const item of NAV.modes){
    const button=document.createElement('button');
    button.type='button';button.className='nav-mode-option';button.dataset.mode=item.mode;
    button.setAttribute('role','option');button.setAttribute('aria-selected',String(item.mode===NAV.selected));
    const title=document.createElement('span');title.className='nav-mode-option-title';
    const name=document.createElement('b');name.textContent=item.label;
    const speed=document.createElement('span');speed.className='nav-mode-option-speed';speed.textContent=navSpeedText(item.speed);
    title.append(name,speed);
    const desc=document.createElement('span');desc.className='nav-mode-option-desc';desc.textContent=item.description||'';
    button.append(title,desc);
    button.addEventListener('click',()=>{NAV.selected=item.mode;navPaint();});
    list.append(button);
  }
}
async function navPoll(force=false){
  if(NAV.pollBusy)return;
  NAV.pollBusy=true;
  try{
    const domain=navDomain();
    const query=domain?('?domain='+encodeURIComponent(domain)):'';
    const response=await fetch('/api/navigation_mode'+query,{cache:'no-store',signal:AbortSignal.timeout(8000)});
    const data=await response.json();
    if(!response.ok)throw new Error(data.error||'状態を取得できません');
    NAV.state=data;
    navPaint();
  }catch(error){
    NAV.state={status:'unavailable',mode:null,error:error.message||'状態を取得できません'};
    navPaint();
  }finally{NAV.pollBusy=false;}
}
async function navOpen(){
  NAV.returnFocus=document.activeElement;
  NAV.selected=null;
  NAV.modalOpen=true;
  const modal=document.getElementById('nav-mode-modal');
  if(!modal)return;
  modal.hidden=false;
  document.body.classList.add('nav-mode-open');
  navOpenFeedback('');
  const details=document.getElementById('nav-mode-details');if(details)details.hidden=true;
  await Promise.all([navLoadModes(),navPoll(true)]);
  const focus=document.getElementById('nav-mode-close')||document.getElementById('nav-mode-modal');
  if(focus)focus.focus({preventScroll:true});
}
function navClose(){
  const modal=document.getElementById('nav-mode-modal');
  NAV.modalOpen=false;
  if(modal)modal.hidden=true;
  document.body.classList.remove('nav-mode-open');
  if(NAV.returnFocus&&NAV.returnFocus.isConnected)NAV.returnFocus.focus({preventScroll:true});
}
function navOpenFeedback(text,kind){
  const feedback=document.getElementById('nav-mode-feedback');
  if(!feedback)return;
  feedback.textContent=text||'';
  feedback.className='nav-mode-feedback'+(kind?' is-'+kind:'');
}
async function navApply(){
  if(NAV.busy||!NAV.selected)return;
  const item=navFind(NAV.selected);
  if(!item)return;
  NAV.busy=true;navPaint();
  navOpenFeedback(`${item.label}へ変更しています…`,'applying');
  const details=document.getElementById('nav-mode-details');if(details){details.hidden=true;details.textContent='';}
  try{
    const response=await fetch('/api/navigation_mode',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({mode:NAV.selected,domain:navDomain()}),signal:AbortSignal.timeout(70000)});
    let data=null;try{data=await response.json();}catch(_){data={};}
    if(data&&data.state)NAV.state=data.state;
    if(!response.ok||!data.ok){
      navOpenFeedback(data.error||'モード変更に失敗しました','failure');
      const lines=[...(data.details||[]),...(data.errors||[])];
      if(lines.length&&details){details.textContent=lines.join('\n');details.hidden=false;}
      if(typeof toast==='function')toast('走行モード変更に失敗しました'+(data.error?'（'+data.error+'）':''),true);
      return;
    }
    let text=data.confirmed?`${item.label}、設定速度 ${navSpeedText(item.speed)} に変更しました。`:`${item.label}への変更処理は成功しました。状態通知を確認中です。`;
    if(data.audio&&!data.audio.ok)text+=' 音声通知のみ失敗しました。';
    else if(data.audio&&data.audio.skipped&&data.audio.reason==='same_mode')text+=' 現在と同じモードのため、音声通知は省略しました。';
    navOpenFeedback(text,'success');
    NAV.selected=null;
    if(typeof toast==='function')toast(data.confirmed?`走行モード: ${item.label}`:`走行モード変更: ${item.label}（状態確認中）`);
    await navPoll(true);
  }catch(error){
    navOpenFeedback('モード変更の結果を確認できませんでした。','failure');
    if(typeof toast==='function')toast('モード変更の結果を確認できませんでした',true);
  }finally{
    NAV.busy=false;navPaint();
  }
}
document.addEventListener('DOMContentLoaded',()=>{
  const openers=['header-nav-mode','drive-nav-change'];
  openers.forEach(id=>{const el=document.getElementById(id);if(el)el.addEventListener('click',navOpen);});
  const close=document.getElementById('nav-mode-close');if(close)close.addEventListener('click',navClose);
  const cancel=document.getElementById('nav-mode-cancel');if(cancel)cancel.addEventListener('click',navClose);
  const apply=document.getElementById('nav-mode-apply');if(apply)apply.addEventListener('click',navApply);
  const modal=document.getElementById('nav-mode-modal');
  if(modal)modal.addEventListener('click',event=>{if(event.target===modal)navClose();});
  document.addEventListener('keydown',event=>{if(event.key==='Escape'&&NAV.modalOpen){event.preventDefault();navClose();}});
  const domain=document.getElementById('domain');
  if(domain)domain.addEventListener('change',()=>{NAV.selected=null;navLoadModes();navPoll(true);});
  navLoadModes();
  navPoll();
  const visibility=()=>{if(document.visibilityState==='visible')navPoll();};
  document.addEventListener('visibilitychange',visibility);
  setInterval(()=>{if(document.visibilityState==='visible')navPoll();},3000);
});
