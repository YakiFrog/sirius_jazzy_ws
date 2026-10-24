'use strict';
(()=>{
 function init(){
  const FACE_ICON='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><circle cx="12" cy="12" r="8.5"/><circle cx="9" cy="10" r="1" fill="currentColor" stroke="none"/><circle cx="15" cy="10" r="1" fill="currentColor" stroke="none"/><path d="M8.7 14.1c1.8 1.7 4.8 1.7 6.6 0"/></svg>';
  const names={normal:'通常',happy:'笑顔',angry:'怒り',sad:'悲しい',surprised:'驚き',cat:'猫',wink:'ウインク',pien:'ぴえん',sleeping:'睡眠',battery:'バッテリー表示',failure:'故障',hurt:'痛がる'};
  const ACTION_PRESETS=[
   ['left','左を向く','左を向いて'],['right','右を向く','右を向いて'],
   ['forward','前進','前に進んで'],['back','後退','後ろに下がって'],
   ['turn','その場旋回','その場で旋回して'],['stop','停止','止まれ'],
   ['battery','バッテリー','バッテリー状況は'],['people','人数確認','周りに人が何人いる'],
   ['status','状態確認','ステータス状況は'],['intro','自己紹介','自己紹介して'],
   ['fast','はやく','はやくして'],['slow','ゆっくり','おそくして'],
   ['normal_speed','通常速度','普通の速度で']
  ];
  const card=document.createElement('section');card.className='panel face-card';card.setAttribute('aria-label','SiriusFace');
  card.innerHTML='<div class="face-card-heading"><span class="face-symbol" aria-hidden="true">'+FACE_ICON+'</span><div><h2>SiriusFace</h2><p id="face-card-summary">接続を確認中…</p></div><span id="face-card-light" class="face-light" aria-hidden="true"></span></div><div class="face-card-bottom"><span id="face-card-expression">表情 —</span><div class="face-card-actions"><button type="button" class="hbtn" id="face-panel-open">'+FACE_ICON+'<span>操作</span></button></div></div>';
  const screenButton=document.createElement('button');screenButton.type='button';screenButton.id='header-face';screenButton.className='appbar-icon';screenButton.title='NUCを顔画面に切り替え';screenButton.setAttribute('aria-label','NUCを顔画面に切り替え');screenButton.innerHTML='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><rect x="3" y="4" width="18" height="16" rx="5"/><path d="M8 9v3m8-3v3m-8 3q4 3 8 0"/></svg>';document.querySelector('.top-status')?.prepend(screenButton);
  const presets=document.querySelector('.presets');if(!presets)return;
  const arrange=()=>presets.before(card);arrange();matchMedia('(min-width:821px)').addEventListener('change',arrange);
  const dialog=document.createElement('dialog');dialog.className='face-dialog';dialog.id='face-dialog';dialog.setAttribute('aria-labelledby','face-dialog-title');
  dialog.innerHTML=`<header class="face-dialog-heading"><div><p class="face-eyebrow">SIRIUS FACE / EAR</p><h2 id="face-dialog-title">顔の操作</h2></div><button type="button" id="face-close" class="face-close" aria-label="顔の操作を閉じる">×</button></header>
   <div class="face-widgets" data-device-pane="face">
   <section class="face-status-area"><h3>接続・画面</h3><strong id="face-status-text">接続を確認中…</strong><div class="face-indicators"><span id="face-api-state">API —</span><span id="face-window-state">画面 —</span><span id="face-speech-state">音声 —</span></div><p id="face-shortcut-name"></p><div class="face-window-actions"><button type="button" id="face-launch">顔を起動</button><button type="button" id="face-focus">顔画面を表示</button><button type="button" id="face-launcher-focus">Launcher画面を表示</button><button type="button" id="face-refresh">状態を更新</button></div><p class="face-panel-note">NUCのモニター表示を切り替えます。顔画面の右上にマウスを寄せると戻るボタンが現れます。</p><p class="face-panel-note" id="face-switch-keys" hidden>NUCのキー：Ctrl + Alt + F で切り替え ／ Ctrl + Alt + W でLauncherへ</p></section>
   <section class="face-section face-expressions-section"><h3>表情・表示モード</h3><div id="face-expressions" class="face-expression-grid"></div><p class="face-panel-note">故障は表示演出です。「痛がる」は一時的な反応です。バッテリー表示はFace側の受信データを使い、未受信時は既定値になる場合があります。</p></section>
   <section class="face-section face-action-section"><h3>行動指示</h3><div class="face-action-mode" role="group" aria-label="行動指示の方式"><button type="button" data-action-mode="llm" aria-pressed="true">LLM経由</button><button type="button" data-action-mode="nav" aria-pressed="false">LLMなし（直接）</button></div><form class="face-action-form" id="face-action-form"><input id="face-action-text" maxlength="200" placeholder="例：前に進んで / 左を向いて / 座標(4,2)にいって" aria-label="行動指示"><button type="submit" id="face-action-send">送信</button></form><div class="face-action-presets" id="face-action-presets"></div><p class="face-action-feedback" id="face-action-feedback" role="status" aria-live="polite"></p><p class="face-panel-note" id="face-action-note">LLM経由：自由な文章をLLMが解釈して行動します。</p></section>
   <section class="face-section face-gaze-section"><h3>視線</h3><div class="face-gaze-grid" id="face-gaze"></div></section>
   <section class="face-section face-message-section"><h3>字幕・読み上げ</h3><textarea id="face-message" aria-label="字幕・読み上げ" maxlength="140" rows="2" placeholder="伝えたい短いメッセージ（140文字まで）"></textarea><div class="face-text-actions"><button type="button" id="face-subtitle">字幕を3秒表示</button><button type="button" id="face-speak">読み上げ</button><button type="button" id="face-clear-text">字幕を消す</button></div></section>
   <p class="face-feedback" id="face-feedback" role="status" aria-live="polite"></p><p class="face-panel-note">操作したときだけ、表情・視線・字幕を変更します。読み上げは現在の音声に割り込みます。</p>
   </div>`;
  document.body.append(dialog);
  const $=id=>document.getElementById(id);let state=null,busy=false,polling=false,lastPoll=0,actionMode='llm';
  try{actionMode=localStorage.getItem('sirius-face-action-mode')==='nav'?'nav':'llm';}catch(error){}
  function initCollapse(section,key,title){
   if(!section||section.dataset.collapsible==='1')return;
   section.dataset.collapsible='1';section.dataset.widgetKey=key;
   const heading=[...section.children].find(child=>child.matches('h3,h4,label'));
   if(heading){if(!title)title=heading.textContent.trim();heading.remove();}
   if(!title)return;
   const body=document.createElement('div');body.className='face-collapse-body';body.id='face-collapse-'+key;
   for(const child of [...section.children])body.append(child);
   const head=document.createElement('div');head.className='face-section-head';
   const button=document.createElement('button');button.type='button';button.className='face-collapse-toggle';button.setAttribute('aria-expanded','true');button.setAttribute('aria-controls',body.id);
   const label=document.createElement('span');label.textContent=title;button.append(label);
   button.insertAdjacentHTML('beforeend','<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="m6 9 6 6 6-6"/></svg>');
   head.append(button);
   if(section.parentElement?.classList.contains('face-widgets')){
     const handle=document.createElement('button');handle.type='button';handle.className='face-drag-handle';handle.title='ドラッグして移動';handle.setAttribute('aria-label',title+'をドラッグして移動');
     handle.innerHTML='<svg viewBox="0 0 10 18" fill="currentColor" aria-hidden="true"><circle cx="2.5" cy="3" r="1.25"/><circle cx="7.5" cy="3" r="1.25"/><circle cx="2.5" cy="9" r="1.25"/><circle cx="7.5" cy="9" r="1.25"/><circle cx="2.5" cy="15" r="1.25"/><circle cx="7.5" cy="15" r="1.25"/></svg>';
     let drag=null,autoScrollFrame=0;
     const moveTo=(x,y)=>{
       const target=document.elementFromPoint(x,y)?.closest('.face-widgets > .face-section, .face-widgets > .face-status-area');
       if(!target||target===section||target.parentElement!==section.parentElement)return;
       const rect=target.getBoundingClientRect();
       section.parentElement.insertBefore(section,y<rect.top+rect.height/2?target:target.nextSibling);
     };
     const autoScroll=()=>{
       autoScrollFrame=0;if(!drag||!drag.started)return;
       const dialogEl=document.getElementById('face-dialog');const rect=dialogEl.getBoundingClientRect();const edge=48;
       let delta=0;if(drag.y<rect.top+edge)delta=-12;else if(drag.y>rect.bottom-edge)delta=12;
       if(delta){dialogEl.scrollTop+=delta;moveTo(drag.x,drag.y);}
       autoScrollFrame=requestAnimationFrame(autoScroll);
     };
     handle.addEventListener('pointerdown',event=>{if(event.pointerType==='mouse'&&event.button!==0)return;event.preventDefault();event.stopPropagation();drag={id:event.pointerId,section,startX:event.clientX,startY:event.clientY,x:event.clientX,y:event.clientY,started:false};handle.setPointerCapture(event.pointerId);});
     handle.addEventListener('pointermove',event=>{
       if(!drag||event.pointerId!==drag.id)return;
       if(!drag.started&&Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)<7){drag.x=event.clientX;drag.y=event.clientY;return;}
       drag.x=event.clientX;drag.y=event.clientY;
       if(!drag.started){drag.started=true;section.classList.add('face-dragging');document.body.classList.add('face-dragging-active');}
       moveTo(event.clientX,event.clientY);
       if(!autoScrollFrame)autoScrollFrame=requestAnimationFrame(autoScroll);
     });
     const finish=event=>{
       if(!drag||(event&&event.pointerId!==drag.id))return;
       const started=drag.started;drag=null;if(autoScrollFrame){cancelAnimationFrame(autoScrollFrame);autoScrollFrame=0;}section.classList.remove('face-dragging');document.body.classList.remove('face-dragging-active');
       if(event&&handle.hasPointerCapture(event.pointerId))handle.releasePointerCapture(event.pointerId);
       if(started)saveWidgetOrder();
     };
     handle.addEventListener('pointerup',finish);handle.addEventListener('pointercancel',finish);handle.addEventListener('lostpointercapture',finish);
     head.append(handle);
   }
   section.prepend(head);section.append(body);
   let open=true;try{open=localStorage.getItem('sirius-face-collapse-'+key)!=='0';}catch(error){}
   function apply(){button.setAttribute('aria-expanded',String(open));body.hidden=!open;}
   button.onclick=()=>{open=!open;apply();try{localStorage.setItem('sirius-face-collapse-'+key,open?'1':'0');}catch(error){}};
   apply();
  }
  function saveWidgetOrder(){
   const container=document.querySelector('.face-widgets');if(!container)return;
   const order=[...container.children].filter(el=>el.dataset.widgetKey).map(el=>el.dataset.widgetKey);
   try{localStorage.setItem('sirius-face-widget-order',JSON.stringify(order));}catch(error){}
  }
  function restoreWidgetOrder(){
   const container=document.querySelector('.face-widgets');if(!container)return;
   let order=[];try{order=JSON.parse(localStorage.getItem('sirius-face-widget-order')||'[]');}catch(error){}
   if(!Array.isArray(order)||!order.length)return;
   const byKey=new Map([...container.children].filter(el=>el.dataset.widgetKey).map(el=>[el.dataset.widgetKey,el]));
   const sorted=order.map(key=>byKey.get(key)).filter(Boolean);
   const fixed=[...container.children].filter(el=>!el.dataset.widgetKey);
   const remaining=[...byKey.values()].filter(el=>!sorted.includes(el));
   container.append(...sorted,...remaining,...fixed);
  }
  window.siriusFaceCollapse=initCollapse;
  function message(text,bad=false){$('face-feedback').textContent=text;$('face-feedback').classList.toggle('error',bad);}
  function actionFeedback(text,bad=false){const el=$('face-action-feedback');if(!el)return;el.textContent=text;el.classList.toggle('error',bad);}
  function paintActionMode(){const ready=!!(state&&state.connected&&state.speech_ready);for(const button of document.querySelectorAll('[data-action-mode]')){button.setAttribute('aria-pressed',String(button.dataset.actionMode===actionMode));button.disabled=busy;}const input=$('face-action-text'),send=$('face-action-send');if(input)input.disabled=busy||!ready;if(send)send.disabled=busy||!ready;for(const button of document.querySelectorAll('[data-action-preset]'))button.disabled=busy||!ready;const note=$('face-action-note');if(note)note.textContent=!ready?'SiriusFaceの指示APIへ接続すると送信できます。':actionMode==='nav'?'LLMなし（直接）：ナビゲーション側の指示パーサーへ送ります。':'LLM経由：自由な文章をLLMが解釈して行動します。';}
  function paint(){if(!state)return;const s=state,summary=s.connected?'接続済み':s.error?'接続を確認できません':s.starting?'起動中…':s.running?'起動中・API待ち':'未起動';
   $('face-card-summary').textContent=summary;$('face-status-text').textContent=s.error||summary;$('face-card-light').classList.toggle('connected',s.connected);
   $('face-card-expression').textContent='表情 '+(s.connected?(names[s.expression]||s.expression||'—'):'—');
   for(const[id,on,label]of[['face-api-state',s.connected,'API'],['face-window-state',s.window_visible,'画面'],['face-speech-state',s.speech_ready,'音声']]){const el=$(id);el.textContent=label+' '+(on?'接続可':'未確認');el.classList.toggle('ready',on);}
   $('face-shortcut-name').textContent=s.shortcut_available?'デスクトップの「'+s.shortcut_name+'」を使用':'この端末には顔の起動ショートカットがありません';
   $('face-launch').disabled=busy||!s.shortcut_available||s.running||s.starting;$('face-launch').textContent=s.starting?'起動中…':s.running?'起動済み':'顔を起動';
   $('face-focus').disabled=busy||!s.window_visible;screenButton.disabled=busy||!s.window_visible;$('face-launcher-focus').disabled=busy||!s.launcher_visible;$('face-switch-keys').hidden=!s.switch_shortcuts;$('face-refresh').disabled=busy||polling;
   for(const b of $('face-expressions').children){b.disabled=busy||!s.connected;b.setAttribute('aria-pressed',String(s.connected&&b.dataset.expression===s.expression));}
   for(const b of $('face-gaze').children)b.disabled=busy||!s.connected||!s.window_visible;
   $('face-subtitle').disabled=busy||!s.connected;$('face-clear-text').disabled=busy||!s.connected;$('face-speak').disabled=busy||!s.connected||!s.speech_ready;
   paintActionMode();
  }
  async function poll(force=false){if(polling||busy||document.hidden||(!force&&!dialog.open&&!isPageVisible('home')))return;if(!force&&Date.now()-lastPoll<2500)return;polling=true;
   try{const response=await fetch('/api/face/status',{cache:'no-store',signal:AbortSignal.timeout(6500)});const data=await response.json();if(!response.ok)throw Error(data.error==='auth'?'Launcherへのログインが必要です':data.error||'状態を取得できません');state=data;lastPoll=Date.now();}
   catch(error){state={...(state||{}),connected:false,speech_ready:false,error:'Faceの状態を取得できません'};if(dialog.open)message(error.message,true);}
   finally{polling=false;paint();}
  }
  async function send(action,extra={},successMessage=null,onError=null){if(busy)return false;if(['focus','focus_launcher'].includes(action)&&typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();busy=true;message('送信中…');paint();try{const response=await fetch('/api/face/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(8000)});const result=await response.json();if(!response.ok||result.error)throw Error(result.error==='auth'?'Launcherへのログインが必要です':result.error||'操作できませんでした');message(successMessage||(result.already_running?'顔はすでに起動しています':result.starting?'顔の起動を開始しました':'送信しました'));return true;}
   catch(error){message(error.message,true);if(onError)onError(error.message);if(!dialog.open)dialog.showModal();return false;}finally{busy=false;paint();await poll(true);}
  }
  async function sendInstruction(mode,text){const value=String(text||'').trim();if(!value){message('行動指示を入力してください',true);actionFeedback('行動指示を入力してください',true);$('face-action-text').focus();return false;}const label=mode==='nav'?'ナビ指示（LLMなし）':'LLM指示';const done=label+'を送信しました：'+value;actionFeedback('送信中…');const ok=await send('instruction',{mode,text:value},done,error=>actionFeedback(error,true));if(ok){actionFeedback(done);$('face-action-text').value='';}return ok;}
  function setActionMode(mode){actionMode=mode==='nav'?'nav':'llm';try{localStorage.setItem('sirius-face-action-mode',actionMode);}catch(error){}paintActionMode();}
  for(const[key,label]of Object.entries(names)){const b=document.createElement('button');b.type='button';b.textContent=label;b.dataset.expression=key;b.onclick=()=>send('expression',{expression:key});$('face-expressions').append(b);}
  for(const[key,label]of [['up','↑'],['left','←'],['center','中央'],['right','→'],['down','↓']]){const b=document.createElement('button');b.type='button';b.textContent=label;b.dataset.direction=key;b.setAttribute('aria-label','視線を'+({up:'上',left:'左',center:'中央',right:'右',down:'下'}[key])+'へ');b.onclick=()=>send('gaze',{direction:key});$('face-gaze').append(b);}
  $('face-panel-open').onclick=()=>{if(typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();dialog.showModal();$('face-close').focus();poll(true);};$('face-close').onclick=()=>dialog.close();
  dialog.addEventListener('click',e=>{if(e.target!==dialog)return;const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();});
  screenButton.onclick=()=>send('focus');$('face-launcher-focus').onclick=()=>send('focus_launcher');$('face-launch').onclick=()=>send('launch');$('face-focus').onclick=()=>send('focus');$('face-refresh').onclick=()=>poll(true);
  const textAction=action=>{const text=$('face-message').value.trim();if(!text){message('メッセージを入力してください',true);$('face-message').focus();return;}if(action==='speak'&&/[\[\]]/.test(text)){message('読み上げには制御タグを使えません',true);return;}send(action,{text});};
  $('face-subtitle').onclick=()=>textAction('subtitle');$('face-speak').onclick=()=>textAction('speak');$('face-clear-text').onclick=()=>send('clear_text');
  for(const[key,label,instruction]of ACTION_PRESETS){const b=document.createElement('button');b.type='button';b.className='face-action-preset';b.dataset.actionPreset=key;b.textContent=label;b.title=instruction;b.onclick=()=>sendInstruction(actionMode,instruction);$('face-action-presets').append(b);}
  for(const b of document.querySelectorAll('[data-action-mode]'))b.onclick=()=>setActionMode(b.dataset.actionMode);
  $('face-action-form').onsubmit=e=>{e.preventDefault();sendInstruction(actionMode,$('face-action-text').value);};
  initCollapse(document.querySelector('.face-status-area'),'status','接続・画面');
  initCollapse(document.querySelector('.face-expressions-section'),'expressions','表情・表示モード');
  initCollapse(document.querySelector('.face-action-section'),'action','行動指示');
  initCollapse(document.querySelector('.face-gaze-section'),'gaze','視線');
  initCollapse(document.querySelector('.face-message-section'),'message','字幕・読み上げ');
  restoreWidgetOrder();
  dialog.addEventListener('keydown',e=>e.stopPropagation());state={connected:false,running:false,window_visible:false,shortcut_available:false,starting:false};paint();document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll(true);});poll();setInterval(()=>poll(),3000);
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
