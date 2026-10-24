'use strict';
(()=>{
 function init(){
  const FACE_ICON='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="8.5"/><circle cx="9" cy="10" r="1" fill="currentColor" stroke="none"/><circle cx="15" cy="10" r="1" fill="currentColor" stroke="none"/><path d="M8.7 14.1c1.8 1.7 4.8 1.7 6.6 0"/></svg>';
  const EAR_ICON='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M6 8.5a6.5 6.5 0 1 1 13 0c0 6-6 6-6 10a3.5 3.5 0 1 1-7 0"/><path d="M15 8.5a2.5 2.5 0 0 0-5 0v1a2 2 0 1 0 0 4"/><circle cx="19" cy="4.5" r="1" fill="currentColor" stroke="none"/></svg>';
  const dialog=document.getElementById('face-dialog'),faceButton=document.getElementById('face-panel-open');if(!dialog||!faceButton)return;
  const $=id=>document.getElementById(id),labels={normal:'通常',left:'左折',right:'右折',turning:'旋回・ハザード',emergency:'非常停止表示',solid:'点灯',blink:'点滅',alternate:'左右交互',breathe:'呼吸',rainbow:'虹色',off:'消灯',auto:'自動表示へ戻す'};
  for(const child of [...dialog.children])if(!child.classList.contains('face-dialog-heading'))child.dataset.devicePane='face';
  const tabs=document.createElement('div');tabs.className='ear-device-tabs';tabs.setAttribute('role','tablist');tabs.setAttribute('aria-label','顔と耳の操作');tabs.innerHTML='<button type="button" role="tab" id="device-face-tab" aria-selected="true">'+FACE_ICON+'<span>顔</span></button><button type="button" role="tab" id="device-ear-tab" aria-selected="false">'+EAR_ICON+'<span>耳LED</span></button>';
  dialog.querySelector('.face-dialog-heading').after(tabs);dialog.classList.add('has-ear-tabs');
  const pane=document.createElement('section');pane.className='ear-main';pane.dataset.devicePane='ear';pane.hidden=true;
  pane.innerHTML='<div class="ear-status-box"><strong id="ear-status">Gatewayを確認中…</strong><div class="ear-connections"><span id="ear-left">左耳 —</span><span id="ear-right">右耳 —</span></div><p id="ear-current" class="ear-note"></p></div><div class="ear-options"><label>色<input type="color" id="ear-color" value="#00a0ff"></label><label>テスト時間<select id="ear-duration"><option value="5">5秒</option><option value="10" selected>10秒</option><option value="20">20秒</option><option value="30">30秒</option></select></label></div><div class="ear-presets" id="ear-presets"></div><button type="button" class="ear-auto" data-ear-pattern="auto">自動表示へ戻す</button><p id="ear-feedback" class="ear-note" role="status" aria-live="polite"></p><p class="ear-note">テスト時間が終わるとposの自動表示へ戻ります。非常停止表示ボタンはLEDの表示テストです。実際の非常停止はテレオペの電子停止ONを使います。</p>';
  dialog.append(pane);
  if(window.siriusFaceCollapse)window.siriusFaceCollapse(pane,'ear','耳LED');
  const cardButton=document.createElement('button');cardButton.type='button';cardButton.className='hbtn ear-card-button';cardButton.innerHTML=EAR_ICON+'<span>耳の操作</span>';faceButton.innerHTML=FACE_ICON+'<span>顔の操作</span>';faceButton.after(cardButton);document.querySelector('.face-card-heading h2').textContent='SiriusFace / Ear';
  const desktop=matchMedia('(min-width:980px)');let selected='face';
  function applyLayout(){const both=desktop.matches;dialog.classList.toggle('ear-side-by-side',both);for(const section of dialog.querySelectorAll('[data-device-pane]'))section.hidden=both?false:section.dataset.devicePane!==selected;$('device-face-tab').setAttribute('aria-selected',String(selected==='face'));$('device-ear-tab').setAttribute('aria-selected',String(selected==='ear'));$('face-dialog-title').textContent=both?'SiriusFace / Ear':selected==='ear'?'耳LEDの操作':'顔の操作';dialog.classList.toggle('ear-selected',!both&&selected==='ear');}
  function select(paneName){selected=paneName;applyLayout();if(paneName==='ear')poll(true);}
  faceButton.addEventListener('click',()=>select('face'));cardButton.onclick=()=>{faceButton.click();select('ear');};$('device-face-tab').onclick=()=>select('face');$('device-ear-tab').onclick=()=>select('ear');
  desktop.addEventListener('change',()=>{if(dialog.open)applyLayout();if(desktop.matches&&dialog.open)poll(true);});
  applyLayout();
  for(const[key,label]of Object.entries(labels)){if(key==='auto')continue;const button=document.createElement('button');button.type='button';button.dataset.earPattern=key;button.textContent=label;$('ear-presets').append(button);}
  const quick=document.createElement('section');quick.className='ear-teleop';quick.setAttribute('aria-label','耳LEDの表示テスト');quick.innerHTML='<div class="ear-teleop-heading"><b>耳LED・表示テスト</b><span id="ear-quick-state">確認中</span></div><div id="ear-quick-buttons" class="ear-quick-buttons"></div><p id="ear-quick-feedback" class="ear-note" role="status"></p><small>10秒後に自動表示へ戻ります。非常停止表示はLEDのみの操作です。</small>';
  const drive=document.getElementById('drive-panel');drive.querySelector('.drive-emergency').after(quick);
  for(const key of['left','right','turning','emergency','off','auto']){const button=document.createElement('button');button.type='button';button.dataset.earPattern=key;button.textContent=key==='auto'?'自動へ':labels[key];$('ear-quick-buttons').append(button);}
  let state=null,busy=false,polling=false,lastPoll=0;
  function text(id,value){if($(id).textContent!==value)$(id).textContent=value;}
  const signals={straight:'通常',off:'通常',left:'左折',right:'右折',hazard:'旋回・ハザード',stop:'非常停止'};
  function paint(){const s=state||{},available=s.gateway_available,ready=s.ready&&available,description=!available?'BLE Gatewayを起動し、耳LEDを有効にしてください':!s.web_test_supported?'BLE Gatewayの更新が必要です':!s.enabled?'Gatewayで耳LEDが無効です':!ready?'耳LEDへ接続中…':s.stop?'実機の非常停止表示を優先中':'左右の耳LEDに接続済み';
   text('ear-status',description);text('ear-quick-state',ready?'接続済み':'未接続');cardButton.classList.toggle('ear-ready',!!ready);
   for(const side of['left','right']){const connected=available&&s[side+'_connected'];text('ear-'+side,(side==='left'?'左耳':'右耳')+' '+(connected?'接続済み':'未接続'));$('ear-'+side).classList.toggle('ear-connected',!!connected);}
   text('ear-current',s.gateway_error?'接続エラー：'+s.gateway_error:available?(s.test_pattern?'テスト中：'+(labels[s.test_pattern]||s.test_pattern)+' ／ 残り約'+Math.ceil(s.test_remaining||0)+'秒':'自動表示：'+(signals[s.signal]||'通常')):'');
   for(const button of document.querySelectorAll('[data-ear-pattern]')){const key=button.dataset.earPattern;button.disabled=busy||(key==='auto'?!(available&&s.web_test_supported&&s.enabled):!ready||!!s.stop);button.setAttribute('aria-pressed',String(key==='auto'?!s.test_pattern:key===s.test_pattern));}
  }
  function feedback(value,error=false){for(const id of['ear-feedback','ear-quick-feedback']){text(id,value);$(id).classList.toggle('files-error',error);}}
  async function poll(force=false){if(polling||busy||document.hidden||(!force&&!isPageVisible('home')&&!dialog.open&&!DRIVE.visible))return;if(!force&&Date.now()-lastPoll<1800)return;polling=true;
   try{const response=await fetch('/api/ear/status?domain='+encodeURIComponent(dom()),{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('状態を取得できません');state=await response.json();lastPoll=Date.now();}catch(_){state={gateway_available:false,ready:false};}finally{polling=false;paint();}}
  async function send(pattern,fromQuick){if(busy)return;busy=true;feedback('送信中…');paint();const hex=$('ear-color').value,color=[1,3,5].map(i=>parseInt(hex.slice(i,i+2),16));
   try{const response=await fetch('/api/ear/test',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pattern,color,duration:fromQuick?10:Number($('ear-duration').value),domain:dom()}),signal:AbortSignal.timeout(5000)});const result=await response.json();if(!response.ok||!result.ok)throw Error(result.error==='auth'?'ログインが必要です':result.error||'送信を確認できません');state=result;feedback(pattern==='auto'?'自動表示へ戻しました':labels[pattern]+'のテストを開始しました');}
   catch(error){feedback(error.message,true);}finally{busy=false;paint();poll(true);}}
  document.querySelectorAll('[data-ear-pattern]').forEach(button=>button.onclick=()=>send(button.dataset.earPattern,!!button.closest('.ear-teleop')));
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll(true);});$('domain').addEventListener('change',()=>poll(true));paint();poll(true);setInterval(()=>poll(),2000);
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
