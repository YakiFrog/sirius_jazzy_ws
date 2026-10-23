'use strict';
const DRIVE={session:'',seq:0,enabled:false,wantEnabled:false,pendingArm:false,visible:false,heartbeatBusy:false,lastReply:0,state:null};
const DRIVE_KEYS={ArrowUp:65,ArrowDown:66,ArrowLeft:68,ArrowRight:67,',':44,'.':46};
function driveSession(){return crypto.randomUUID?crypto.randomUUID().replaceAll('-',''):Array.from(crypto.getRandomValues(new Uint8Array(20)),b=>b.toString(16).padStart(2,'0')).join('');}
function drivePaint(state){
  if(state)DRIVE.state=state;
  const s=DRIVE.state||{}, active=DRIVE.enabled;
  const panel=document.getElementById('drive-panel');if(!panel)return;
  panel.classList.toggle('driving',active);
  document.getElementById('header-teleop').classList.toggle('driving',active);
  document.getElementById('drive-mode').textContent=active?'キー操作 ON':'操作 OFF';
  const routeChoice=document.getElementById('drive-route-mode');
  const changingRoute=!s.enabled&&s.route_mode&&s.route_mode!==routeChoice.value;
  const blocked=s.blocked&&!(changingRoute&&s.blocked===s.route_blocked)?s.blocked:'';
  const status=document.getElementById('drive-status');
  status.textContent=s.error||blocked||(s.enabled&&!s.owned?'別の画面で操作中':active?'操作中':changingRoute?'有効化時に選択した接続を確認します':'操作は無効です');
  status.classList.toggle('drive-blocked',!!(s.error||blocked));
  document.getElementById('drive-arm').textContent=active?'操作を無効にする':'操作を有効にする';
  document.getElementById('drive-arm').disabled=DRIVE.pendingArm||(!active&&!!(blocked||s.enabled));
  document.querySelectorAll('[data-drive-key]').forEach(b=>b.disabled=!active);
  const number=(v,d)=>Number.isFinite(v)?v.toFixed(d):'0.'+'0'.repeat(d);
  for(const [id,key,d] of [['drive-linear','linear',2],['drive-angular','angular',3],['drive-target-linear','target_linear',2],['drive-target-angular','target_angular',3]]){
    document.getElementById(id).textContent=number(s[key],d);
  }
  document.getElementById('drive-topic').textContent=s.topic||'/cmd_vel';
  routeChoice.disabled=!!(s.enabled||DRIVE.pendingArm);
  if(s.enabled&&s.route_mode)routeChoice.value=s.route_mode;
}
async function driveRequest(action,key){
  if(!DRIVE.session)DRIVE.session=driveSession();
  const body={action,session:DRIVE.session,seq:++DRIVE.seq,domain:dom()};
  if(action==='arm')body.route_mode=document.getElementById('drive-route-mode').value;
  if(key!==undefined)body.key=key;
  const response=await fetch('/api/teleop',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body),signal:AbortSignal.timeout(900)});
  const state=await response.json();
  if(body.session!==DRIVE.session||body.seq<DRIVE.lastReply)return state;
  DRIVE.lastReply=body.seq;
  if(!response.ok||!state.ok){
    DRIVE.enabled=false;drivePaint({...state,error:state.error||'速度指令を確認できません'});
    throw new Error(state.error||'速度指令を確認できません');
  }
  DRIVE.enabled=!!(DRIVE.wantEnabled&&state.enabled&&state.owned);
  drivePaint(state);
  return state;
}
async function driveArm(){
  if(DRIVE.enabled){driveDisable();return;}
  if(DRIVE.pendingArm)return;
  DRIVE.session=driveSession();DRIVE.seq=0;DRIVE.lastReply=0;DRIVE.wantEnabled=true;DRIVE.pendingArm=true;drivePaint();
  try{await driveRequest('arm');document.getElementById('drive-panel').focus({preventScroll:true});}
  catch(error){driveDisable(error.message);}
  finally{DRIVE.pendingArm=false;drivePaint();}
}
function driveDisable(reason){
  const shouldSend=DRIVE.enabled||DRIVE.pendingArm;
  DRIVE.enabled=false;DRIVE.wantEnabled=false;DRIVE.pendingArm=false;
  drivePaint({...DRIVE.state,enabled:false,owned:false,linear:0,angular:0,target_linear:0,target_angular:0,
    reason:reason||'操作は無効です',error:reason||''});
  if(shouldSend)driveRequest('disable').catch(()=>{});
}
async function driveKey(key){
  if(!DRIVE.enabled)return;
  try{await driveRequest('key',key);}catch(error){driveDisable(error.message);}
}
async function driveStop(){
  DRIVE.wantEnabled=DRIVE.enabled&&!DRIVE.pendingArm;DRIVE.pendingArm=false;
  drivePaint({...DRIVE.state,linear:0,angular:0,target_linear:0,target_angular:0});
  try{await driveRequest('stop');}catch(error){driveDisable(error.message);}
}
async function drivePoll(){
  if(!DRIVE.visible||document.hidden)return;
  const sequence=DRIVE.seq;
  try{
    const response=await fetch('/api/teleop/status?session='+encodeURIComponent(DRIVE.session),{cache:'no-store',signal:AbortSignal.timeout(2000)});
    if(!response.ok)throw new Error('状態を確認できません');
    const state=await response.json();
    if(sequence!==DRIVE.seq)return;
    // Only a command acknowledgement can enable this browser.
    if(DRIVE.enabled&&(!state.enabled||!state.owned||state.blocked))DRIVE.enabled=false;
    drivePaint(state);
  }catch(error){driveDisable('接続を確認できません');}
}
const DRIVE_PANEL=document.getElementById('drive-panel');
const DRIVE_ANCHOR=document.createComment('docked teleop position');
DRIVE_PANEL.before(DRIVE_ANCHOR);
const DRIVE_DRAWER=document.createElement('aside');
DRIVE_DRAWER.id='drive-drawer';DRIVE_DRAWER.hidden=true;
DRIVE_DRAWER.setAttribute('role','dialog');DRIVE_DRAWER.setAttribute('aria-label','テレオペ');
document.body.append(DRIVE_DRAWER);
const TERMINAL_DRIVE_SLOT=document.createElement('aside');TERMINAL_DRIVE_SLOT.className='terminal-drive-slot';TERMINAL_DRIVE_SLOT.hidden=true;TERMINAL_DRIVE_SLOT.setAttribute('aria-label','ターミナルのテレオペ');document.querySelector('.terminal-layout').append(TERMINAL_DRIVE_SLOT);
const driveAppbar=document.querySelector('.appbar');
function driveDrawerPosition(){DRIVE_DRAWER.style.setProperty('--drive-top',Math.max(12,driveAppbar.getBoundingClientRect().bottom+12)+'px');}
new ResizeObserver(driveDrawerPosition).observe(driveAppbar);
window.addEventListener('resize',driveDrawerPosition);
let driveReturnFocus=null;
function teleopPageChanged(page){
  driveDisable();
  const header=document.getElementById('header-teleop');
  if(window.isPageVisible?isPageVisible('terminal'):page==='terminal'){
    DRIVE_DRAWER.hidden=true;TERMINAL_DRIVE_SLOT.hidden=false;TERMINAL_DRIVE_SLOT.append(DRIVE_PANEL);
    document.getElementById('viz-layout').classList.add('drive-undocked');document.getElementById('drive-close').hidden=true;
    header.setAttribute('aria-expanded','true');header.title='テレオペにフォーカス（Alt+T / Mac: Option+T）';
  }else if(DRIVE_PANEL.parentElement===TERMINAL_DRIVE_SLOT){
    TERMINAL_DRIVE_SLOT.hidden=true;DRIVE_ANCHOR.after(DRIVE_PANEL);document.getElementById('viz-layout').classList.remove('drive-undocked');
    header.setAttribute('aria-expanded','false');header.title='テレオペを開く（Alt+T / Mac: Option+T）';
  }
  DRIVE.visible=(window.isPageVisible?(isPageVisible('terminal')||isPageVisible('visualization')):(page==='terminal'||page==='visualization'))||!DRIVE_DRAWER.hidden;
  if(DRIVE.visible)drivePoll();
}
function toggleTeleop(force){
  if(window.isPageVisible?isPageVisible('terminal'):document.body.dataset.page==='terminal'){DRIVE_PANEL.focus({preventScroll:true});return;}
  const open=typeof force==='boolean'?force:DRIVE_DRAWER.hidden;
  if(open===!DRIVE_DRAWER.hidden){if(open)DRIVE_PANEL.focus({preventScroll:true});return;}
  driveDisable();
  const header=document.getElementById('header-teleop');
  if(open){
    driveReturnFocus=document.activeElement;
    driveDrawerPosition();
    DRIVE_DRAWER.hidden=false;DRIVE_DRAWER.append(DRIVE_PANEL);
  }else{
    DRIVE_ANCHOR.after(DRIVE_PANEL);DRIVE_DRAWER.hidden=true;
  }
  document.getElementById('viz-layout').classList.toggle('drive-undocked',open);
  document.getElementById('drive-close').hidden=!open;
  header.setAttribute('aria-expanded',String(open));
  header.title=(open?'テレオペを閉じる':'テレオペを開く')+'（Alt+T / Mac: Option+T）';
  DRIVE.visible=open||document.body.dataset.page==='visualization';
  if(open){DRIVE_PANEL.focus({preventScroll:true});drivePoll();}
  else if(driveReturnFocus&&driveReturnFocus.isConnected&&driveReturnFocus.getClientRects().length)driveReturnFocus.focus({preventScroll:true});
  else header.focus({preventScroll:true});
}
document.getElementById('drive-close').addEventListener('click',()=>toggleTeleop(false));
document.querySelectorAll('[data-drive-key]').forEach(button=>button.addEventListener('click',()=>driveKey(Number(button.dataset.driveKey))));
document.getElementById('drive-arm').addEventListener('click',driveArm);
document.getElementById('drive-route-mode').addEventListener('change',()=>drivePaint());
document.getElementById('drive-stop').addEventListener('click',driveStop);
function driveElectronicStop(active){
  driveDisable();
  if(!window.requestElectronicStop||!window.requestElectronicStop(active))toast('電子停止の状態・通信を確認してください',true);
}
document.getElementById('drive-estop-on').addEventListener('click',()=>driveElectronicStop(true));
document.getElementById('drive-estop-off').addEventListener('click',()=>driveElectronicStop(false));
document.addEventListener('keydown',event=>{
  if(!DRIVE.visible||event.isComposing||event.ctrlKey||event.metaKey||event.altKey||!event.target.closest('#drive-panel')||event.target.closest('input,textarea,select,[contenteditable]'))return;
  const key=event.code==='KeyQ'?'q':event.code==='KeyE'?'e':event.key.toLowerCase();
  if(key!=='q'&&key!=='e')return;event.preventDefault();event.stopPropagation();if(!event.repeat)driveElectronicStop(key==='q');
});
document.addEventListener('keydown',event=>{
  if(!DRIVE.visible||!DRIVE.enabled||event.isComposing||event.ctrlKey||event.metaKey||event.altKey)return;
  if(event.target.closest('input,textarea,select,[contenteditable]'))return;
  if(event.key===' '){event.preventDefault();if(!event.repeat)driveStop();return;}
  const key=DRIVE_KEYS[event.key];if(key===undefined)return;
  event.preventDefault();if(!event.repeat)driveKey(key);
});
window.addEventListener('blur',()=>driveDisable());
document.addEventListener('focusin',event=>{if(DRIVE.enabled&&event.target.closest('input,textarea,select,[contenteditable]'))driveDisable();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)driveDisable();});
document.getElementById('domain').addEventListener('change',()=>driveDisable());
window.addEventListener('pagehide',()=>{
  if(!DRIVE.session)return;
  navigator.sendBeacon('/api/teleop',new Blob([JSON.stringify({action:'disable',session:DRIVE.session,seq:++DRIVE.seq,domain:dom()})],{type:'application/json'}));
  DRIVE.enabled=false;DRIVE.wantEnabled=false;
});
setInterval(async()=>{
  if(!DRIVE.enabled||DRIVE.heartbeatBusy)return;
  DRIVE.heartbeatBusy=true;
  try{await driveRequest('heartbeat');}catch(error){driveDisable(error.message);}
  finally{DRIVE.heartbeatBusy=false;}
},250);
setInterval(drivePoll,1500);
drivePaint();

// The splitter stops control before handling arrow keys or pointer drags.
(function initDriveResize(){
  const layout=document.getElementById('viz-layout'),splitter=document.getElementById('drive-splitter'),panel=document.getElementById('drive-panel');
  const key='sirius-viz-teleop-width';
  let preferred=null,drag=null;
  try{const saved=localStorage.getItem(key);if(saved!==null&&Number.isFinite(Number(saved)))preferred=Number(saved);}catch(error){}
  function bounds(){
    const sidebar=document.getElementById('viz-sidebar');
    const left=sidebar&&getComputedStyle(sidebar).display!=='none'?sidebar.getBoundingClientRect().width+14:0;
    return {min:220,max:Math.max(220,Math.min(520,layout.clientWidth-left-14-280))};
  }
  function apply(){
    if(matchMedia('(max-width:900px)').matches)return;
    const range=bounds();
    if(preferred===null)layout.style.removeProperty('--drive-width');
    else layout.style.setProperty('--drive-width',Math.max(range.min,Math.min(range.max,preferred))+'px');
    splitter.setAttribute('aria-valuemin',String(range.min));splitter.setAttribute('aria-valuemax',String(Math.round(range.max)));
    splitter.setAttribute('aria-valuenow',String(Math.round(panel.getBoundingClientRect().width)));
  }
  function save(){try{if(preferred===null)localStorage.removeItem(key);else localStorage.setItem(key,String(preferred));}catch(error){}}
  function finish(event){
    if(!drag||event&&event.pointerId!==drag.pointerId)return;
    drag=null;document.body.classList.remove('drive-resizing');save();
  }
  splitter.addEventListener('pointerdown',event=>{
    if(event.button!==0)return;
    driveDisable();drag={pointerId:event.pointerId,x:event.clientX,width:panel.getBoundingClientRect().width};
    splitter.setPointerCapture(event.pointerId);document.body.classList.add('drive-resizing');event.preventDefault();
  });
  splitter.addEventListener('pointermove',event=>{
    if(!drag||event.pointerId!==drag.pointerId)return;
    const range=bounds();preferred=Math.max(range.min,Math.min(range.max,drag.width+drag.x-event.clientX));apply();
  });
  splitter.addEventListener('pointerup',finish);splitter.addEventListener('pointercancel',finish);splitter.addEventListener('lostpointercapture',finish);
  splitter.addEventListener('dblclick',()=>{driveDisable();preferred=null;save();apply();});
  splitter.addEventListener('keydown',event=>{
    if(event.key!=='ArrowLeft'&&event.key!=='ArrowRight'&&event.key!=='Home')return;
    event.preventDefault();event.stopImmediatePropagation();driveDisable();
    if(event.key==='Home')preferred=null;
    else{const range=bounds();preferred=Math.max(range.min,Math.min(range.max,panel.getBoundingClientRect().width+(event.key==='ArrowLeft'?20:-20)));}
    save();apply();
  });
  new ResizeObserver(()=>{if(!drag)apply();}).observe(layout);
  new MutationObserver(apply).observe(layout,{attributes:true,attributeFilter:['class']});
  window.addEventListener('resize',()=>{if(drag)finish();apply();});window.addEventListener('blur',()=>finish());
  apply();
})();
