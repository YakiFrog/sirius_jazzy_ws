'use strict';
// Presentation only: retain the existing telemetry and explicit action handlers.
(()=>{
 const bar=document.querySelector('.appbar'),home=document.getElementById('page-home'),face=document.querySelector('.mission-face');
 if(!bar||!home||!face)return;
 document.body.classList.add('refined-ui');
 const faceImage=document.getElementById('mission-face-image');
 function syncFaceAspect(){if(!faceImage?.naturalWidth||!faceImage.naturalHeight)return;const ratio=String(faceImage.naturalWidth/faceImage.naturalHeight);if(document.body.style.getPropertyValue('--face-aspect')!==ratio)document.body.style.setProperty('--face-aspect',ratio);}
 if(faceImage){faceImage.addEventListener('load',syncFaceAspect);syncFaceAspect();}
 // The outer PWA shell reserves its safe area once, outside the iframe.
 try{if(parent!==window&&parent.document.documentElement.hasAttribute('data-safe-shell'))document.documentElement.dataset.safeShellChild='true';}catch(_){}
 const themeMeta=document.querySelector('meta[name="theme-color"]');
 function syncThemeChrome(){if(themeMeta)themeMeta.content=document.documentElement.dataset.theme==='light'?'#f2f5fa':'#0f151f';}
 new MutationObserver(syncThemeChrome).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});syncThemeChrome();
 const css=document.createElement('link');css.rel='stylesheet';css.href='/static/operations_refined.css?v=20261101-preview-colors';document.head.append(css);
 const diagnostic=document.getElementById('navigation-diagnostics-button'),runningHeading=home.querySelector('.sidebar>.section-heading');
 if(diagnostic&&runningHeading){runningHeading.querySelector('.live-dot')?.remove();runningHeading.append(diagnostic);}
 const top=bar.querySelector('.top-status'),power=document.getElementById('operations-power');
 if(power){power.classList.add('refined-power');power.title='ACコンセント・シガーソケット出力の状態';top.append(power);}
 const morePanel=document.getElementById('header-more-panel');
 const compactControls=matchMedia('(max-width:1180px)');
 function arrangeHeaderDetails(){if(power&&morePanel)(compactControls.matches?morePanel:top).append(power);}
 compactControls.addEventListener('change',arrangeHeaderDetails);arrangeHeaderDetails();
 const message=face.querySelector('.mission-face-message');
 if(message){const button=document.createElement('button');button.type='button';button.className=message.className;button.setAttribute('aria-label','走行状態の詳細を開く');button.setAttribute('aria-describedby','mission-title mission-reason');button.setAttribute('aria-live','polite');button.append(...message.childNodes);button.onclick=()=>diagnostic?.click();message.replaceWith(button);}
 const stops=document.createElement('div');stops.className='refined-stops';stops.setAttribute('aria-label','緊急停止の現在状態');
 const icon='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" aria-hidden="true"><rect x="4" y="4" width="16" height="16" rx="5"/><path d="M8 12h8"/></svg>';
 for(const [key,label] of [['electronic','電子停止'],['physical','物理停止']]){
  const button=document.createElement('button');button.type='button';button.className='refined-stop';button.dataset.stopKind=key;button.dataset.tone='unknown';button.innerHTML=icon+'<span>'+label+'</span><strong>未確認</strong>';
  button.onclick=()=>document.getElementById('emergency-stop-toggle')?.click();stops.append(button);
 }
 top.append(stops);
 function paintStops(snapshot){
  const d=snapshot?.driver||{},fresh=d.state==='live'&&Number.isFinite(d.age_seconds)&&d.age_seconds<(d.stale_after_seconds||5);
  for(const [key,field,label] of [['electronic','emergency_stop','電子停止'],['physical','physical_emergency_stop','物理停止']]){
   const button=stops.querySelector('[data-stop-kind='+key+']'),value=fresh?d[field]:null;
   const state=value===true?(key==='physical'?'押下推定':'ON'):value===false?(key==='physical'?'押下なし':'OFF'):'未確認';
   button.dataset.tone=value===true?'blocked':value===false?'ok':'unknown';button.querySelector('strong').textContent=state;
   button.title=label+'：'+state+(key==='physical'?'（駆動電圧からの推定）':'')+'。詳細を開く';button.setAttribute('aria-label',button.title);
  }
 }
 let battery=window.siriusBatterySnapshot,received=Date.now();paintStops(battery);
 window.addEventListener('sirius:battery-state',event=>{battery=event.detail;received=Date.now();paintStops(battery);});
 setInterval(()=>paintStops(Date.now()-received<8000?battery:null),2000);
 const mini=home.querySelector('.mini-health');
 if(mini){
  const system=document.createElement('section');system.className='refined-system panel';system.setAttribute('aria-label','システム使用状況');
  const heading=document.createElement('div');heading.className='section-heading';heading.innerHTML='<h2>システム</h2><span>HOST HEALTH</span>';system.append(heading,mini);home.querySelector('.home-side')?.append(system);
  const button=mini.querySelector('button'),cpu=document.getElementById('mini-cpu'),memory=document.getElementById('mini-memory');
  button.replaceChildren();
  for(const [label,value] of [['CPU',cpu],['メモリ',memory]]){const tile=document.createElement('span');tile.className='refined-metric';const name=document.createElement('small');name.textContent=label;tile.append(name,value);button.append(tile);}
  button.setAttribute('aria-label','CPU・メモリ使用率。リソースモニターを開く');
 }
 const filter=document.getElementById('preset-category-filter');
 if(filter){
  const tabs=document.createElement('div');tabs.className='refined-categories';tabs.setAttribute('role','group');tabs.setAttribute('aria-label','クイックスタートの用途');filter.closest('.preset-filter').append(tabs);
  function paintCategories(){const signature=JSON.stringify([...filter.options].map(o=>[o.value,o.textContent]));
   if(tabs.dataset.options!==signature){tabs.dataset.options=signature;tabs.replaceChildren();for(const option of filter.options){const b=document.createElement('button');b.type='button';b.dataset.category=option.value;b.textContent=option.value||'すべて';b.onclick=()=>{filter.value=option.value;filter.dispatchEvent(new Event('change',{bubbles:true}));paintCategories();};tabs.append(b);}}
   for(const b of tabs.children)b.setAttribute('aria-pressed',String(b.dataset.category===filter.value));
  }
  new MutationObserver(paintCategories).observe(filter,{childList:true,subtree:true});filter.addEventListener('change',paintCategories);paintCategories();
 }
})();
