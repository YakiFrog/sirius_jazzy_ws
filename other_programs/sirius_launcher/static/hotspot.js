'use strict';
(()=>{
 function init(){
  const host=document.querySelector('.top-status'),settings=document.querySelector('#page-settings .settings-grid');
  if(!host||!settings)return;
  const chip=document.createElement('button');chip.id='hotspot-toggle';chip.type='button';chip.className='hotspot-chip';chip.dataset.state='unknown';chip.setAttribute('aria-haspopup','dialog');
  chip.innerHTML='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true"><path d="M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0m-11 4a6 6 0 0 1 8 0"/><circle cx="12" cy="20" r="1"/></svg><span>AP確認中</span>';
  host.prepend(chip);
  const body=prefix=>`<p class="hotspot-state" id="${prefix}-state" role="status">確認中…</p><div class="hotspot-connect"><div><dl class="hotspot-facts"><dt>Wi-Fi名</dt><dd id="${prefix}-ssid">—</dd><dt>接続用URL</dt><dd><a id="${prefix}-url" target="_blank" rel="noopener noreferrer">—</a></dd><dt>自動起動</dt><dd id="${prefix}-auto">—</dd></dl><button type="button" class="hbtn hotspot-copy" id="${prefix}-copy" disabled>URLをコピー</button><p id="${prefix}-feedback" class="hotspot-note" role="status"></p></div><figure class="hotspot-qr" id="${prefix}-qr-box" hidden><canvas id="${prefix}-qr" role="img" aria-label="アクセスポイント経由のLauncher URL"></canvas><figcaption>Launcherを開くQR</figcaption></figure></div><p class="hotspot-note" id="${prefix}-help">アクセスポイントの状態を確認しています。</p>`;
  const actions={start:'起動',restart:'再起動',stop:'停止'};
  const controls=prefix=>`<div class="hotspot-actions" id="${prefix}-actions" role="group" aria-label="アクセスポイントの操作" hidden>${Object.entries(actions).map(([action,label])=>`<button type="button" class="hbtn" id="${prefix}-${action}" disabled>${label}</button>`).join('')}</div><p class="hotspot-note" id="${prefix}-operation" role="status"></p>`;
  const card=document.createElement('article');card.className='surface settings-card hotspot-card';card.id='hotspot-card';card.innerHTML='<div class="section-heading"><h2>アクセスポイント</h2></div>'+body('hotspot-settings');settings.append(card);
  const dialog=document.createElement('dialog');dialog.id='hotspot-dialog';dialog.className='hotspot-dialog';dialog.setAttribute('aria-labelledby','hotspot-title');dialog.innerHTML='<header class="hotspot-heading"><h2 id="hotspot-title">アクセスポイント</h2><button type="button" class="hbtn hotspot-close" id="hotspot-close" aria-label="アクセスポイントの案内を閉じる">×</button></header>'+body('hotspot-panel');document.body.append(dialog);
  const $=id=>document.getElementById(id),set=(id,text)=>{if($(id).textContent!==text)$(id).textContent=text;};
  const panels=['hotspot-settings','hotspot-panel'];
  for(const prefix of panels){
   $(prefix+'-state').insertAdjacentHTML('afterend',controls(prefix));
   $(prefix+'-copy').insertAdjacentHTML('afterend',` <button type="button" class="hbtn hotspot-copy" id="${prefix}-login" hidden aria-expanded="false">AP用ログイン情報</button><p class="hotspot-note hotspot-login" id="${prefix}-credentials" role="status" hidden></p>`);
  }
  let state=null,busy=false,operating=false,lastUrl='';
  function draw(canvas,url){const QR=qrcodegen.QrCode,qr=QR.encodeText(url,QR.Ecc.MEDIUM),size=qr.size+8,scale=6;canvas.width=canvas.height=size*scale;const ctx=canvas.getContext('2d');ctx.fillStyle='#fff';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#000';for(let y=0;y<qr.size;y++)for(let x=0;x<qr.size;x++)if(qr.getModule(x,y))ctx.fillRect((x+4)*scale,(y+4)*scale,scale,scale);}
  function paint(){if(!state)return;const s=state,active=s.active,ready=active&&!!s.url,error=s.state==='error',label=active?'AP稼働':error?'AP未確認':s.state==='unconfigured'?'APなし':'AP停止';
   chip.dataset.state=active?'active':error?'unknown':'inactive';chip.querySelector('span').textContent=label;chip.title='アクセスポイント：'+(s.ssid||'未設定')+'（'+label+'）';chip.setAttribute('aria-label',chip.title+'・URLとQRを表示');
   const stateLabels={active:'稼働中',error:'状態を取得できません',unconfigured:'この端末には設定がありません',unavailable:'ドングルが未接続です',device_busy:'ドングルは別の接続に使用中です',starting:'接続処理中',no_address:'接続済み・IPアドレスなし'};
   for(const prefix of panels){set(prefix+'-state',stateLabels[s.state]||'停止中');$(prefix+'-state').dataset.active=String(active);set(prefix+'-ssid',s.ssid||'—');set(prefix+'-auto',s.ssid?(s.autostart?'ON':'OFF'):'—');const link=$(prefix+'-url');set(prefix+'-url',ready?s.url:'—');if(ready)link.href=s.url;else link.removeAttribute('href');$(prefix+'-copy').disabled=!ready;$(prefix+'-qr-box').hidden=!ready;
    $(prefix+'-actions').hidden=!s.can_control||!s.ssid;
    $(prefix+'-login').hidden=!s.can_control||!s.offline_login;
    for(const action of Object.keys(actions))$(prefix+'-'+action).disabled=operating||s.busy||!(action==='start'?s.can_start:s.can_stop);
    set(prefix+'-help',ready?'Wi-Fi「'+s.ssid+'」につないでから、URLまたはQRで開いてください。'+(s.offline_login?'AP用のログイン情報で入れます。':'インターネット接続は不要です。'):active?'Wi-Fiは稼働中です。接続用URLは準備できていません。':'稼働すると、この端末へ接続するURLとQRが表示されます。');
    if(ready&&s.url!==lastUrl)try{draw($(prefix+'-qr'),s.url);}catch(_){$(prefix+'-qr-box').hidden=true;}
   }
   lastUrl=ready?s.url:'';
  }
  async function poll(){if(busy||operating||document.hidden)return;busy=true;try{const response=await fetch('/api/hotspot/status',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('status unavailable');const latest=await response.json();if(!operating)state=latest;}catch(_){if(!operating)state={...(state||{}),active:false,state:'error',url:null,can_start:false,can_stop:false};}finally{busy=false;paint();}}
  const feedback=message=>panels.forEach(prefix=>set(prefix+'-operation',message));
  async function control(action){
   if(operating)return;
   if(action!=='start'&&!confirm(`アクセスポイントを${actions[action]}しますか？ 接続中の端末は切断されます。`))return;
   operating=true;feedback(`${actions[action]}中…`);paint();
   try{
    const response=await fetch('/api/hotspot/status',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action}),signal:AbortSignal.timeout(36000)});
    const latest=await response.json();
    if('active' in latest)state=latest;
    const errors={busy:'別の操作を実行中です。',unavailable:'現在の状態では操作できません。',access:'この接続からは操作できません。',start_failed:'起動に失敗しました。ドングルの接続を確認してください。',stop_failed:'停止に失敗しました。',timeout:'操作が時間内に完了しませんでした。'};
    feedback(response.ok?`${actions[action]}しました。`:errors[latest.error]||`${actions[action]}に失敗しました。`);
   }catch(_){feedback('結果を取得できませんでした。接続後に稼働状態を確認してください。');}
   finally{operating=false;paint();poll();}
  }
  for(const prefix of panels)for(const action of Object.keys(actions))$(prefix+'-'+action).onclick=()=>control(action);
  function hideLogin(prefix){
   $(prefix+'-credentials').hidden=true;
   set(prefix+'-credentials','');
   $(prefix+'-login').setAttribute('aria-expanded','false');
  }
  for(const prefix of panels)$(prefix+'-login').onclick=async()=>{
   const button=$(prefix+'-login'),details=$(prefix+'-credentials');
   if(!details.hidden){hideLogin(prefix);return;}
   button.setAttribute('aria-expanded','true');
   details.hidden=false;set(details.id,'取得中…');
   try{
    const response=await fetch('/api/hotspot/login',{cache:'no-store',signal:AbortSignal.timeout(5000)});
    if(!response.ok)throw Error('credentials unavailable');
    const {username,password}=await response.json();
    if(!details.hidden)set(details.id,`ユーザー名 ${username} ／ パスワード ${password}`);
   }catch(_){if(!details.hidden)set(details.id,'ログイン情報を取得できませんでした。');}
  };
  dialog.addEventListener('close',()=>hideLogin('hotspot-panel'));
  for(const prefix of['hotspot-settings','hotspot-panel'])$(prefix+'-copy').onclick=async()=>{if(!state?.active||!state.url)return;let copied=false;try{await navigator.clipboard.writeText(state.url);copied=true;}catch(_){const text=document.createElement('textarea');text.value=state.url;text.style.cssText='position:fixed;opacity:0';(dialog.open?dialog:document.body).append(text);text.select();try{copied=document.execCommand('copy');}catch(_){}text.remove();}set(prefix+'-feedback',copied?'コピーしました':'URLを長押し、または右クリックしてコピーしてください');};
  chip.onclick=()=>{if(typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();dialog.showModal();$('hotspot-close').focus();poll();};$('hotspot-close').onclick=()=>dialog.close();dialog.addEventListener('keydown',e=>e.stopPropagation());dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});poll();setInterval(poll,5000);
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
