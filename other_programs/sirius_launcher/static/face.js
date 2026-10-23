'use strict';
(()=>{
 function init(){
  const names={normal:'通常',happy:'笑顔',angry:'怒り',sad:'悲しい',surprised:'驚き',cat:'猫',wink:'ウインク',pien:'ぴえん',sleeping:'睡眠',battery:'バッテリー表示',failure:'故障',hurt:'痛がる'};
  const card=document.createElement('section');card.className='panel face-card';card.setAttribute('aria-label','SiriusFace');
  card.innerHTML='<div class="face-card-heading"><span class="face-symbol" aria-hidden="true">◕ ◕</span><div><h2>SiriusFace</h2><p id="face-card-summary">接続を確認中…</p></div><span id="face-card-light" class="face-light" aria-hidden="true"></span></div><div class="face-card-bottom"><span id="face-card-expression">表情 —</span><button type="button" class="hbtn" id="face-panel-open">操作</button></div>';
  const screenButton=document.createElement('button');screenButton.type='button';screenButton.id='header-face';screenButton.className='appbar-icon';screenButton.title='NUCを顔画面に切り替え';screenButton.setAttribute('aria-label','NUCを顔画面に切り替え');screenButton.innerHTML='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><rect x="3" y="4" width="18" height="16" rx="5"/><path d="M8 9v3m8-3v3m-8 3q4 3 8 0"/></svg>';document.querySelector('.top-status')?.prepend(screenButton);
  const presets=document.querySelector('.presets');if(!presets)return;
  const arrange=()=>presets.before(card);arrange();matchMedia('(min-width:821px)').addEventListener('change',arrange);
  const dialog=document.createElement('dialog');dialog.className='face-dialog';dialog.id='face-dialog';dialog.setAttribute('aria-labelledby','face-dialog-title');
  dialog.innerHTML=`<header class="face-dialog-heading"><div><p class="face-eyebrow">SIRIUS FACE</p><h2 id="face-dialog-title">顔の操作</h2></div><button type="button" id="face-close" class="face-close" aria-label="顔の操作を閉じる">×</button></header>
   <section class="face-status-area"><strong id="face-status-text">接続を確認中…</strong><div class="face-indicators"><span id="face-api-state">API —</span><span id="face-window-state">画面 —</span><span id="face-speech-state">音声 —</span></div><p id="face-shortcut-name"></p><div class="face-window-actions"><button type="button" id="face-launch">顔を起動</button><button type="button" id="face-focus">顔画面を表示</button><button type="button" id="face-launcher-focus">Launcher画面を表示</button><button type="button" id="face-refresh">状態を更新</button></div><p class="face-panel-note">NUCのモニター表示を切り替えます。顔画面の右上にマウスを寄せると戻るボタンが現れます。</p><p class="face-panel-note" id="face-switch-keys" hidden>NUCのキー：Ctrl + Alt + F で切り替え ／ Ctrl + Alt + W でLauncherへ</p></section>
   <section class="face-section face-expressions-section"><h3>表情・表示モード</h3><div id="face-expressions" class="face-expression-grid"></div><p class="face-panel-note">故障は表示演出です。「痛がる」は一時的な反応です。バッテリー表示はFace側の受信データを使い、未受信時は既定値になる場合があります。</p></section>
   <section class="face-section face-gaze-section"><h3>視線</h3><div class="face-gaze-grid" id="face-gaze"></div></section>
   <section class="face-section face-message-section"><label for="face-message">字幕・読み上げ</label><textarea id="face-message" maxlength="140" rows="2" placeholder="伝えたい短いメッセージ（140文字まで）"></textarea><div class="face-text-actions"><button type="button" id="face-subtitle">字幕を3秒表示</button><button type="button" id="face-speak">読み上げ</button><button type="button" id="face-clear-text">字幕を消す</button></div></section>
   <p class="face-feedback" id="face-feedback" role="status" aria-live="polite"></p><p class="face-panel-note">操作したときだけ、表情・視線・字幕を変更します。読み上げは現在の音声に割り込みます。</p>`;
  document.body.append(dialog);
  const $=id=>document.getElementById(id);let state=null,busy=false,polling=false,lastPoll=0;
  function message(text,bad=false){$('face-feedback').textContent=text;$('face-feedback').classList.toggle('error',bad);}
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
  }
  async function poll(force=false){if(polling||busy||document.hidden||(!force&&!dialog.open&&!isPageVisible('home')))return;if(!force&&Date.now()-lastPoll<2500)return;polling=true;
   try{const response=await fetch('/api/face/status',{cache:'no-store',signal:AbortSignal.timeout(6500)});const data=await response.json();if(!response.ok)throw Error(data.error==='auth'?'Launcherへのログインが必要です':data.error||'状態を取得できません');state=data;lastPoll=Date.now();}
   catch(error){state={...(state||{}),connected:false,speech_ready:false,error:'Faceの状態を取得できません'};if(dialog.open)message(error.message,true);}
   finally{polling=false;paint();}
  }
  async function send(action,extra={}){if(busy)return;if(['focus','focus_launcher'].includes(action)&&typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();busy=true;message('送信中…');paint();try{const response=await fetch('/api/face/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...extra}),signal:AbortSignal.timeout(8000)});const result=await response.json();if(!response.ok||result.error)throw Error(result.error==='auth'?'Launcherへのログインが必要です':result.error||'操作できませんでした');message(result.already_running?'顔はすでに起動しています':result.starting?'顔の起動を開始しました':'送信しました');}
   catch(error){message(error.message,true);if(!dialog.open)dialog.showModal();}finally{busy=false;paint();await poll(true);}
  }
  for(const[key,label]of Object.entries(names)){const b=document.createElement('button');b.type='button';b.textContent=label;b.dataset.expression=key;b.onclick=()=>send('expression',{expression:key});$('face-expressions').append(b);}
  for(const[key,label]of [['up','↑'],['left','←'],['center','中央'],['right','→'],['down','↓']]){const b=document.createElement('button');b.type='button';b.textContent=label;b.dataset.direction=key;b.setAttribute('aria-label','視線を'+({up:'上',left:'左',center:'中央',right:'右',down:'下'}[key])+'へ');b.onclick=()=>send('gaze',{direction:key});$('face-gaze').append(b);}
  $('face-panel-open').onclick=()=>{if(typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();dialog.showModal();$('face-close').focus();poll(true);};$('face-close').onclick=()=>dialog.close();
  dialog.addEventListener('click',e=>{if(e.target!==dialog)return;const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();});
  screenButton.onclick=()=>send('focus');$('face-launcher-focus').onclick=()=>send('focus_launcher');$('face-launch').onclick=()=>send('launch');$('face-focus').onclick=()=>send('focus');$('face-refresh').onclick=()=>poll(true);
  const textAction=action=>{const text=$('face-message').value.trim();if(!text){message('メッセージを入力してください',true);$('face-message').focus();return;}if(action==='speak'&&/[\[\]]/.test(text)){message('読み上げには制御タグを使えません',true);return;}send(action,{text});};
  $('face-subtitle').onclick=()=>textAction('subtitle');$('face-speak').onclick=()=>textAction('speak');$('face-clear-text').onclick=()=>send('clear_text');
  dialog.addEventListener('keydown',e=>e.stopPropagation());state={connected:false,running:false,window_visible:false,shortcut_available:false,starting:false};paint();document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll(true);});poll();setInterval(()=>poll(),3000);
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
