'use strict';
(()=>{
 function init(){
  const host=document.querySelector('.top-status'),settings=document.querySelector('#page-settings .settings-grid');
  if(!host||!settings)return;
  const chip=document.createElement('button');chip.id='hotspot-toggle';chip.type='button';chip.className='hotspot-chip';chip.dataset.state='unknown';chip.setAttribute('aria-haspopup','dialog');
  chip.innerHTML='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" aria-hidden="true"><path d="M2 8a16 16 0 0 1 20 0M5 12a11 11 0 0 1 14 0m-11 4a6 6 0 0 1 8 0"/><circle cx="12" cy="20" r="1"/></svg><span>AP確認中</span>';
  host.prepend(chip);
  const body=prefix=>`<p class="hotspot-state" id="${prefix}-state" role="status">確認中…</p><div class="hotspot-connect"><div><dl class="hotspot-facts"><dt>Wi-Fi名</dt><dd id="${prefix}-ssid">—</dd><dt>接続用URL</dt><dd><a id="${prefix}-url" target="_blank" rel="noopener noreferrer">—</a></dd><dt>自動起動</dt><dd id="${prefix}-auto">—</dd></dl><button type="button" class="hbtn hotspot-copy" id="${prefix}-copy" disabled>URLをコピー</button><p id="${prefix}-feedback" class="hotspot-note" role="status"></p></div><figure class="hotspot-qr" id="${prefix}-qr-box" hidden><canvas id="${prefix}-qr" role="img" aria-label="アクセスポイント経由のLauncher URL"></canvas><figcaption>Launcherを開くQR</figcaption></figure></div><p class="hotspot-note" id="${prefix}-help">アクセスポイントの状態を確認しています。</p>`;
  const card=document.createElement('article');card.className='surface settings-card hotspot-card';card.id='hotspot-card';card.innerHTML='<div class="section-heading"><h2>アクセスポイント</h2></div>'+body('hotspot-settings');settings.append(card);
  const dialog=document.createElement('dialog');dialog.id='hotspot-dialog';dialog.className='hotspot-dialog';dialog.setAttribute('aria-labelledby','hotspot-title');dialog.innerHTML='<header class="hotspot-heading"><h2 id="hotspot-title">アクセスポイント</h2><button type="button" class="hbtn hotspot-close" id="hotspot-close" aria-label="アクセスポイントの案内を閉じる">×</button></header>'+body('hotspot-panel');document.body.append(dialog);
  const $=id=>document.getElementById(id),set=(id,text)=>{if($(id).textContent!==text)$(id).textContent=text;};
  let state=null,busy=false,lastUrl='';
  function draw(canvas,url){const QR=qrcodegen.QrCode,qr=QR.encodeText(url,QR.Ecc.MEDIUM),size=qr.size+8,scale=6;canvas.width=canvas.height=size*scale;const ctx=canvas.getContext('2d');ctx.fillStyle='#fff';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.fillStyle='#000';for(let y=0;y<qr.size;y++)for(let x=0;x<qr.size;x++)if(qr.getModule(x,y))ctx.fillRect((x+4)*scale,(y+4)*scale,scale,scale);}
  function paint(){if(!state)return;const s=state,active=s.active&&!!s.url,error=s.state==='error',label=active?'AP稼働':error?'AP未確認':s.state==='unconfigured'?'APなし':'AP停止';
   chip.dataset.state=active?'active':error?'unknown':'inactive';chip.querySelector('span').textContent=label;chip.title='アクセスポイント：'+(s.ssid||'未設定')+'（'+label+'）';chip.setAttribute('aria-label',chip.title+'・URLとQRを表示');
   for(const prefix of['hotspot-settings','hotspot-panel']){set(prefix+'-state',active?'稼働中':error?'状態を取得できません':s.state==='unconfigured'?'この端末には設定がありません':s.state==='unavailable'?'ドングルが未接続です':'停止中');$(prefix+'-state').dataset.active=String(active);set(prefix+'-ssid',s.ssid||'—');set(prefix+'-auto',s.ssid?(s.autostart?'有効':'無効'):'—');const link=$(prefix+'-url');set(prefix+'-url',active?s.url:'—');if(active)link.href=s.url;else link.removeAttribute('href');$(prefix+'-copy').disabled=!active;$(prefix+'-qr-box').hidden=!active;
    set(prefix+'-help',active?'先にWi-Fi「'+s.ssid+'」へ接続してから、URLまたはQRで開いてください。インターネット接続は不要です。':'稼働すると、この端末へ接続するURLとQRが表示されます。');
    if(active&&s.url!==lastUrl)try{draw($(prefix+'-qr'),s.url);}catch(_){$(prefix+'-qr-box').hidden=true;}
   }
   lastUrl=active?s.url:'';
  }
  async function poll(){if(busy||document.hidden)return;busy=true;try{const response=await fetch('/api/hotspot/status',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(!response.ok)throw Error('status unavailable');state=await response.json();}catch(_){state={...(state||{}),active:false,state:'error',url:null};}finally{busy=false;paint();}}
  for(const prefix of['hotspot-settings','hotspot-panel'])$(prefix+'-copy').onclick=async()=>{if(!state?.active||!state.url)return;let copied=false;try{await navigator.clipboard.writeText(state.url);copied=true;}catch(_){const text=document.createElement('textarea');text.value=state.url;text.style.cssText='position:fixed;opacity:0';(dialog.open?dialog:document.body).append(text);text.select();try{copied=document.execCommand('copy');}catch(_){}text.remove();}set(prefix+'-feedback',copied?'コピーしました':'URLを長押し、または右クリックしてコピーしてください');};
  chip.onclick=()=>{if(typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();dialog.showModal();$('hotspot-close').focus();poll();};$('hotspot-close').onclick=()=>dialog.close();dialog.addEventListener('keydown',e=>e.stopPropagation());dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});poll();setInterval(poll,5000);
 }
 if(document.readyState==='loading')document.addEventListener('DOMContentLoaded',init);else init();
})();
