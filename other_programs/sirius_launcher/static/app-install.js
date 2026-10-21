'use strict';
(function(){
  // Installation belongs to the outer document, which owns the browser tab.
  let host=window;
  try{if(parent!==window&&parent.location.origin===location.origin)host=parent;}catch(_){}
  if(!host.siriusAppInstall){
    const state={prompt:null,installed:host.matchMedia('(display-mode: standalone)').matches};
    const notify=()=>host.dispatchEvent(new host.Event('sirius:install-state'));
    host.addEventListener('beforeinstallprompt',event=>{event.preventDefault();state.prompt=event;notify();});
    host.addEventListener('appinstalled',()=>{state.installed=true;state.prompt=null;notify();});
    host.siriusAppInstall={state,async request(){
      if(!state.prompt)return false;
      const prompt=state.prompt;state.prompt=null;notify();
      await prompt.prompt();
      await prompt.userChoice;
      notify();return true;
    }};
  }
  const card=document.getElementById('theme-select')?.closest('article');
  if(!card||document.getElementById('app-install-row'))return;
  const row=document.createElement('div');row.id='app-install-row';row.className='setting-row';
  row.innerHTML='<span><b>アプリとして使う</b><small id="app-install-status">専用ウィンドウで起動できます。</small></span><button class="hbtn" type="button" id="app-install-button">手順を表示</button>';
  card.append(row);
  const dialog=document.createElement('dialog');dialog.className='app-install-dialog';dialog.setAttribute('aria-labelledby','app-install-title');
  dialog.innerHTML='<h2 id="app-install-title">SiriusLauncherをアプリに追加</h2><ol><li>Chrome右上の「⋮」を開きます。</li><li>「キャスト、保存、共有」または「保存と共有」を開きます。</li><li>「SiriusLauncherをインストール」または「ページをアプリとしてインストール」を選びます。</li></ol><p>追加後は、デスクトップやDockなどから専用ウィンドウで起動できます。</p><small>アドレスバーにインストールボタンが出ない場合も、Chromeメニューから追加できます。NUCへの接続は引き続き必要です。</small><footer><button class="hbtn" id="app-install-close" type="button">閉じる</button></footer>';
  document.body.append(dialog);
  const button=document.getElementById('app-install-button'),status=document.getElementById('app-install-status');
  function update(){const s=host.siriusAppInstall.state;button.textContent=s.installed?'追加済み':s.prompt?'インストール':'手順を表示';button.disabled=s.installed;status.textContent=s.installed?'アプリの専用ウィンドウで利用できます。':'専用ウィンドウで起動できます。';}
  button.onclick=async()=>{
    button.disabled=true;
    try{if(!await host.siriusAppInstall.request())dialog.showModal();}
    catch(_){dialog.showModal();}
    finally{update();}
  };
  document.getElementById('app-install-close').onclick=()=>dialog.close();
  host.addEventListener('sirius:install-state',update);
  window.addEventListener('pagehide',()=>host.removeEventListener('sirius:install-state',update),{once:true});
  update();
})();
