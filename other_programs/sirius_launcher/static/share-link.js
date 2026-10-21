'use strict';
(function(){
  const grid=document.querySelector('#page-settings .settings-grid');
  if(!grid||document.getElementById('launcher-share-card'))return;
  let owner=window;
  try{if(parent!==window&&parent.location.origin===location.origin)owner=parent;}catch(_){}
  let shareUrl=owner.location.origin+'/#home';
  const card=document.createElement('article');card.id='launcher-share-card';card.className='surface settings-card launcher-share-card';
  card.innerHTML='<div class="section-heading"><h2>共有リンク</h2></div><p class="launcher-share-note" id="launcher-share-note">別の端末でSiriusLauncherを開くリンクです。</p><a id="launcher-share-link" target="_blank" rel="noopener noreferrer"></a><div class="launcher-share-actions"><button class="hbtn" type="button" id="launcher-share-copy">リンクをコピー</button><span id="launcher-share-status" role="status" aria-live="polite"></span></div>';
  grid.append(card);
  const link=document.getElementById('launcher-share-link'),note=document.getElementById('launcher-share-note'),status=document.getElementById('launcher-share-status');
  function display(){
    link.href=shareUrl;link.textContent=shareUrl;
    const host=new URL(shareUrl).hostname;
    note.textContent=host.endsWith('.ts.net')||/^100\./.test(host)?'同じTailscaleネットワークの端末で開けます。':host==='localhost'||host==='127.0.0.1'||host==='[::1]'?'このURLはこの端末専用です。他の端末への共有はネットワークのURLで開いてください。':'別の端末でSiriusLauncherを開くリンクです。';
  }
  display();
  const ready=(async()=>{
    try{
      const response=await fetch('/static/share-link.json',{cache:'no-store',signal:AbortSignal.timeout(3000)});
      if(!response.ok)return;
      const data=await response.json(),url=new URL(data.public_url);
      if(url.protocol!=='http:'&&url.protocol!=='https:')return;
      shareUrl=url.origin+'/#home';display();
    }catch(_){}
  })();
  document.getElementById('launcher-share-copy').onclick=async()=>{
    status.textContent='';await ready;
    try{
      if(!navigator.clipboard?.writeText)throw new Error('clipboard unavailable');
      await navigator.clipboard.writeText(shareUrl);status.textContent='コピーしました';return;
    }catch(_){}
    const text=document.createElement('textarea');text.value=shareUrl;text.setAttribute('readonly','');text.style.cssText='position:fixed;left:-9999px;top:0';document.body.append(text);text.select();
    try{status.textContent=document.execCommand('copy')?'コピーしました':'リンクを長押し、または右クリックしてコピーしてください';}
    catch(_){status.textContent='リンクを長押し、または右クリックしてコピーしてください';}
    finally{text.remove();document.getElementById('launcher-share-copy').focus();}
  };
})();
