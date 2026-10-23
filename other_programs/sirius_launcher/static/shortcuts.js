'use strict';
(function initLauncherShortcuts(){
  // Use physical key codes: Option+number produces symbols on macOS.
  const pages=['home','terminal','resources','visualization','tf','topics','settings','docs','files'];
  const mac=/Mac|iPhone|iPad/.test(navigator.platform),modifier=mac?'Option':'Alt';
  function addHint(button,label){
    const badge=document.createElement('span');badge.className='shortcut-hint';
    badge.textContent=label;badge.setAttribute('aria-hidden','true');button.append(badge);
    button.classList.add('has-shortcut');
  }
  pages.forEach((page,index)=>{
    const button=document.querySelector('.nav-item[data-page="'+page+'"]');
    if(!button)return;
    const hint='（'+modifier+'+'+(index+1)+'）';
    const annotate=()=>{if(!button.title.endsWith(hint))button.title=(button.title||PAGE_LABELS[page]||page)+hint;};
    annotate();
    // TF health updates its tooltip as new diagnostics arrive.
    new MutationObserver(annotate).observe(button,{attributes:true,attributeFilter:['title']});
    button.setAttribute('aria-keyshortcuts','Alt+'+(index+1));
    addHint(button,String(index+1));
  });
  addHint(document.getElementById('header-teleop'),'T');
  const arm=document.getElementById('drive-arm');
  // Keep the hint separate: drivePaint replaces the button's text on every reply.
  const armHint=document.createElement('p');armHint.className='drive-arm-hint';
  armHint.textContent=modifier+'+Enter で操作の有効／無効を切り替え';arm.after(armHint);
  arm.setAttribute('aria-keyshortcuts','Alt+Enter');arm.title=armHint.textContent;
  document.querySelector('#header-teleop kbd').textContent=modifier+' T';
  document.querySelector('.footer-shortcut').textContent=modifier+'長押しでキー表示 · '+modifier+'+Enter 操作ON/OFF';
  function hints(show){document.body.classList.toggle('show-shortcuts',show);}
  document.addEventListener('keyup',event=>{if(!event.altKey)hints(false);},true);
  window.addEventListener('blur',()=>hints(false));
  document.addEventListener('visibilitychange',()=>{if(document.hidden)hints(false);});
  document.addEventListener('keydown',event=>{
    if(event.key==='Alt'||event.altKey)hints(true);
    if(event.defaultPrevented||event.isComposing||event.repeat)return;
    if(event.key==='Escape'&&!DRIVE_DRAWER.hidden&&!document.querySelector('.modal.open,.log.open,dialog[open]')){
      event.preventDefault();event.stopImmediatePropagation();toggleTeleop(false);return;
    }
    if(!event.altKey||event.ctrlKey||event.metaKey||event.shiftKey)return;
    const code=event.code||(/^[1-9]$/.test(event.key)?'Digit'+event.key:event.key.toLowerCase()==='t'?'KeyT':event.key==='Enter'?'Enter':'');
    const match=/^Digit([1-9])$/.exec(code);
    const armShortcut=code==='Enter'||code==='NumpadEnter';
    if(code!=='KeyT'&&!match&&!armShortcut)return;
    if(armShortcut&&document.querySelector('.modal.open,.log.open,dialog[open]'))return;
    event.preventDefault();event.stopImmediatePropagation();
    if(armShortcut){
      if(!DRIVE.enabled&&DRIVE_DRAWER.hidden)toggleTeleop(true);
      if(!arm.disabled)arm.click();
      else{DRIVE_PANEL.focus({preventScroll:true});toast(DRIVE.state?.blocked||'現在は操作を有効にできません',true);}
    }else if(code==='KeyT')toggleTeleop();
    else{
      setPage(pages[Number(match[1])-1]);
      document.querySelector('.nav-item[data-page="'+pages[Number(match[1])-1]+'"]').focus({preventScroll:true});
    }
  },true);
})();
