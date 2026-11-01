'use strict';

// Presentation only: reuse existing read-only receivers and explicit action buttons.
function operationsSummary(nav, battery) {
  const ready = typeof navigationPreflightState === 'function' ? navigationPreflightState(nav) : {tone:'unknown',badge:'未確認',checks:[]};
  const current = key => nav?.available && nav.navigation?.stages?.[key]?.fresh ? nav.navigation.stages[key].value : null;
  const driver = battery?.driver || {};
  const driverFresh = driver.state === 'live' && Number.isFinite(driver.age_seconds) && driver.age_seconds < (driver.stale_after_seconds || 5);
  const reasons = [];
  if (driverFresh && driver.emergency_stop === true) reasons.push('電子緊急停止がONになっています');
  if (driverFresh && driver.physical_emergency_stop === true) reasons.push('物理緊急停止の押下を検出（電圧による推定）');
  if (nav?.available && nav.navigation?.mismatch) reasons.push('解除状態とmuxの停止ロックが食い違っています');
  else if (current('lock') === true && !reasons.length) reasons.push('指令選択器の停止ロックが有効です');
  if (current('driver') === true && !reasons.length) reasons.push('駆動側の緊急停止・保護停止が有効です');
  const blocked = ready.checks.filter(c => c.tone === 'blocked');
  for (const check of blocked) reasons.push(check.label+'：'+check.text);
  if (reasons.length) return {tone:'blocked',title:'自律移動できません',reason:reasons[0],reasons,
    stop:current('driver') === true || current('lock') === true || (driverFresh && (driver.emergency_stop === true || driver.physical_emergency_stop === true)) ? '停止中' : '準備不足',ready};
  if (ready.tone !== 'ok') return {tone:'unknown',title:'自律移動の状態を確認中',reason:ready.checks.find(c=>c.tone!=='ok')?.text || '診断データの受信を待っています',reasons:[],stop:'未確認',ready};
  if (current('driver') !== false || current('lock') !== false) return {tone:'unknown',title:'停止状態を確認中',reason:'駆動側・muxの解除状態を確認できません',reasons:[],stop:'未確認',ready};
  return {tone:'ok',title:'自律移動の準備OK',reason:current('output')?.moving ? '速度指令を受信中です' : 'ゴールの指定を待っています',reasons:[],stop:'停止要因なし',ready};
}

if (typeof module !== 'undefined') module.exports = {operationsSummary};
if (typeof document !== 'undefined') (() => {
  document.body.classList.add('operations-ui');
  const css=document.createElement('link');css.rel='stylesheet';css.href='/static/operations_home.css?v=20261027-keys';document.head.append(css);
  const bar=document.querySelector('.appbar'),identity=bar?.querySelector('.page-identity'),home=document.getElementById('page-home');
  if (!bar || !identity || !home) return;
  const face=document.createElement('section');face.className='mission-face';face.dataset.tone='unknown';face.setAttribute('aria-label','ロボットの走行状態');
  face.innerHTML='<button type="button" class="mission-face-preview" aria-label="SiriusFaceの操作を開く"><img id="mission-face-image" alt="SiriusFaceのウィンドウ" hidden><span id="mission-face-placeholder"><img src="/static/branding/sirius-icon.png" alt=""><small>顔 未取得</small></span></button><div class="mission-face-message" role="status" aria-live="polite"><strong id="mission-title">自律移動の状態を確認中</strong><span id="mission-reason">診断データの受信を待っています</span></div>';
  identity.after(face);
  const diagnostic=document.getElementById('navigation-diagnostics-button');if(diagnostic){face.append(diagnostic);diagnostic.title='走行診断の詳細を開く';}
  face.querySelector('button').onclick=()=>document.getElementById('face-panel-open')?.click();
  const heading=document.createElement('div');heading.className='operations-heading';heading.innerHTML='<h1>ホーム</h1><p>ロボットの状態と、よく使う操作</p><p class="screen-switch-help">NUCの画面切替：<span><kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>F</kbd> 顔／Launcher</span><span><kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>W</kbd> Launcherを表示</span></p>';
  const summary=document.createElement('section');summary.className='operations-summary panel';summary.setAttribute('aria-label','走行と電源の要約');
  summary.innerHTML='<div><small>現在の状態</small><strong id="operations-stop">未確認</strong></div><button type="button" id="operations-readiness"><small>解除前チェック</small><strong>未確認</strong></button><button type="button" id="operations-power"><small>電源出力</small><strong class="power-port-icons"><span data-power-port="ac" title="ACコンセント出力"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><rect x="4" y="2" width="16" height="20" rx="5"/><path d="M9 7v5m6-5v5"/><path d="M10 17a2 2 0 0 1 4 0"/></svg><span>—</span></span><span data-power-port="dc" title="シガーソケット出力"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><circle cx="12" cy="10" r="7"/><circle cx="12" cy="10" r="2"/><path d="M5 10h2m10 0h2M8 16v5h8v-5"/></svg><span>—</span></span></strong></button><p id="operations-note">診断データの受信を待っています</p>';
  home.prepend(summary);home.prepend(heading);
  summary.querySelector('#operations-readiness').onclick=()=>diagnostic?.click();
  summary.querySelector('#operations-power').onclick=()=>document.getElementById('battery-toggle')?.click();
  const programs=document.querySelector('.program-section');
  if(programs){const details=document.createElement('details');details.className='program-library panel';details.innerHTML='<summary><span>プログラム一覧</span><small>検索・個別起動</small></summary>';programs.before(details);details.append(programs);}
  // Preserve shortcuts/search: focusing a program field reveals its native controls.
  document.getElementById('cmdin')?.addEventListener('focus',()=>{const d=document.querySelector('.program-library');if(d)d.open=true;});
  let nav=window.siriusNavigationSnapshot,battery=window.siriusBatterySnapshot,lastNav=Date.now(),lastBattery=Date.now();
  function paint(){
    const now=Date.now(),n=now-lastNav<8000?nav:null,b=now-lastBattery<8000?battery:null,state=operationsSummary(n,b);
    face.dataset.tone=state.tone;document.getElementById('mission-title').textContent=state.title;
    document.getElementById('mission-reason').textContent=state.reason+(state.reasons.length>1?' ／ ほか'+(state.reasons.length-1)+'項目':'');
    face.title=state.reasons.join('\n')||state.reason;
    document.getElementById('operations-stop').textContent=state.stop;document.getElementById('operations-stop').dataset.tone=state.tone;
    const check=document.getElementById('operations-readiness');check.dataset.tone=state.ready.tone;check.querySelector('strong').textContent=state.ready.badge;
    document.getElementById('operations-note').textContent=state.ready.tone==='ok'?'緊急停止を除く準備を確認済み。駆動応答は試走で確認。':'準備不足の内訳は「解除前チェック」から確認できます。';
    const power=n?navigationPowerStates(n):{ac:{},dc:{}};
    for(const [key,label] of [['ac','ACコンセント出力'],['dc','シガーソケット出力']]){
      const port=document.querySelector('[data-power-port="'+key+'"]'),value=power[key].value;
      port.querySelector('span').textContent=value?.toUpperCase()||'—';
      port.dataset.tone=value==='on'?'ok':value==='off'?'blocked':'unknown';
      port.setAttribute('aria-label',label+'：'+(value?.toUpperCase()||'未確認'));
      port.title=label+'：'+(value?.toUpperCase()||'未確認')+(power[key].power==null?'':' · '+power[key].power.toFixed(0)+' W');
    }
    document.getElementById('operations-power').dataset.tone=power.ac.value==='off'||power.dc.value==='off'?'blocked':power.ac.value==='on'&&power.dc.value==='on'?'ok':'unknown';
  }
  window.addEventListener('sirius:navigation-state',e=>{nav=e.detail;lastNav=Date.now();paint();});
  window.addEventListener('sirius:battery-state',e=>{battery=e.detail;lastBattery=Date.now();paint();});
  // Low-bandwidth, on-demand window snapshots. Never display a cached image as live.
  let previewBusy=false,objectURL=null,lastImage=0;
  const image=document.getElementById('mission-face-image'),placeholder=document.getElementById('mission-face-placeholder');
  function clearPreview(){image.hidden=true;placeholder.hidden=false;if(objectURL){URL.revokeObjectURL(objectURL);objectURL=null;image.removeAttribute('src');}}
  async function preview(){
    if(previewBusy||document.hidden)return;
    const s=window.siriusFaceState;
    if(!s?.connected||!s?.window_visible){clearPreview();return;}
    previewBusy=true;
    try{const response=await fetch('/api/face/preview',{cache:'no-store',signal:AbortSignal.timeout(2500)});if(!response.ok)throw Error();const blob=await response.blob();if(blob.type!=='image/jpeg')throw Error();
      const url=URL.createObjectURL(blob);if(objectURL)URL.revokeObjectURL(objectURL);objectURL=url;image.src=url;image.hidden=false;placeholder.hidden=true;lastImage=Date.now();
    }catch(error){clearPreview();}finally{previewBusy=false;}
  }
  window.addEventListener('sirius:face-state',()=>{if(!window.siriusFaceState?.connected)clearPreview();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden)clearPreview();else{paint();preview();}});
  window.addEventListener('pagehide',clearPreview);
  setInterval(()=>{paint();if(lastImage&&Date.now()-lastImage>6000)clearPreview();},2000);
  setInterval(preview,250);paint();preview();
})();
