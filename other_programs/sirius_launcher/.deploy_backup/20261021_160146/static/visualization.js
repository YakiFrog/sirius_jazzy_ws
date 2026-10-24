'use strict';
const VIZ_KEYS={
  viewer:'sirius-viz-viewer',
  bridge:'sirius-viz-bridge',
  auto:'sirius-viz-auto',
  layout:'sirius-viz-layout-initialized-v1',
  sidebar:'sirius-viz-sidebar-collapsed',
  workspace:'sirius-viz-workspace-initialized-v1'
};
const VIZ={inited:false,initPromise:null,workspaceReady:false,workspacePromise:null,frameLoaded:false,loading:false,frameState:'idle',bridgeState:'idle',defaults:null,settings:null,ws:null,loadTimer:null,probeTimer:null,loadToken:0,pendingLayoutInit:false,lastProbe:0,configError:''};
function vizGet(key){try{return localStorage.getItem(key);}catch(error){return null;}}
function vizSet(key,value){try{if(value===null)localStorage.removeItem(key);else localStorage.setItem(key,value);}catch(error){}}
function vizFallbackDefaults(){
  const host=location.hostname||'127.0.0.1';
  const secure=location.protocol==='https:';
  return {
    viewer_url:(secure?'https://':'http://')+host+':8091/',
    bridge_url:(secure?'wss://':'ws://')+host+':8766',
    layout_url:location.origin+'/api/visualization/layout'
  };
}
async function fetchVizDefaults(){
  const controller=new AbortController();
  const timer=setTimeout(()=>controller.abort(),4000);
  try{
    const response=await fetch('/api/visualization/config',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const data=await response.json();
    if(!data.viewer_url||!data.bridge_url)throw new Error('設定が不完全です');
    return data;
  }finally{clearTimeout(timer);}
}
function vizViewerUrl(value){
  let raw=String(value||'').trim();
  if(!raw)throw new Error('Viewer URLが空です');
  if(!/^[a-z][a-z0-9+.-]*:\/\//i.test(raw))raw=(location.protocol==='https:'?'https://':'http://')+raw;
  const url=new URL(raw);
  if(url.protocol!=='http:'&&url.protocol!=='https:')throw new Error('Viewer URLはhttp(s)で指定してください');
  return url;
}
function vizBridgeUrl(value){
  let raw=String(value||'').trim();
  if(!raw)throw new Error('Bridge URLが空です');
  if(!/^[a-z][a-z0-9+.-]*:\/\//i.test(raw))raw=(location.protocol==='https:'?'wss://':'ws://')+raw;
  const url=new URL(raw);
  if(url.protocol==='http:')url.protocol='ws:';
  if(url.protocol==='https:')url.protocol='wss:';
  if(url.protocol!=='ws:'&&url.protocol!=='wss:')throw new Error('Bridge URLはws(s)で指定してください');
  url.hash='';
  return url.href;
}
function vizCredit(){return new Date().toLocaleTimeString('ja-JP',{hour:'2-digit',minute:'2-digit',hour12:false});}
function setVizDot(id,state){
  const dot=document.getElementById(id);
  if(!dot)return;
  dot.className='viz-dot'+(state==='ready'?' ready':state==='failed'?' failed':state==='checking'?' checking':'');
}
function setVizFrameState(text,state){
  VIZ.frameState=state;
  const el=document.getElementById('viz-frame-state');
  if(el){el.textContent=text;el.className=state==='failed'?'bad':state==='ready'?'good':'';}
}
function setVizEmpty(title,text,retry){
  const titleEl=document.getElementById('viz-empty-title');
  const textEl=document.getElementById('viz-empty-text');
  const retryEl=document.getElementById('viz-retry');
  if(titleEl)titleEl.textContent=title;
  if(textEl)textEl.textContent=text;
  if(retryEl)retryEl.hidden=!retry;
}
function updateVizStatus(){
  const auto=VIZ.settings?VIZ.settings.auto:true;
  const viewer=VIZ.frameState==='ready'?'ready':VIZ.frameState==='failed'?'failed':VIZ.frameState==='loading'?'checking':'idle';
  const bridge=auto?VIZ.bridgeState:'idle';
  setVizDot('viz-viewer-dot',viewer);
  setVizDot('viz-bridge-dot',bridge);
  const viewerText={idle:'待機中',checking:'読み込み中',ready:'読み込み済み',failed:'読み込み失敗'}[viewer]||'待機中';
  const bridgeText=!auto?'自動接続オフ':{idle:'未確認',checking:'接続確認中',ready:'到達確認済み',failed:'未接続'}[bridge]||'未確認';
  const viewerEl=document.getElementById('viz-viewer-state');
  const bridgeEl=document.getElementById('viz-bridge-state');
  if(viewerEl)viewerEl.textContent=viewerText;
  if(bridgeEl)bridgeEl.textContent=bridgeText;
  let badge='準備中',badgeClass='',detail='表示設定を読み込んでいます。';
  if(VIZ.configError)detail=VIZ.configError;
  if(!auto){
    badge='手動接続';
    detail=viewer==='ready'?'Viewerを読み込みました。自動接続は無効です。Lichtblick内の接続画面からBridgeへ接続してください。':'Viewerを読み込んでいます。自動接続は無効です。';
  }else if(viewer==='failed'){
    badge='Viewerエラー';badgeClass='bad';
    detail='Viewerを読み込めませんでした。URLとLichtblickコンテナの状態を確認してください。';
  }else if(bridge==='failed'){
    badge='Bridge未接続';badgeClass='bad';
    detail=(VIZ.settings?VIZ.settings.bridge:'Bridge')+' に到達できません。foxglove_bridgeの起動とポートを確認してください。';
  }else if(viewer==='ready'&&bridge==='ready'){
    badge='表示準備完了';badgeClass='good';
    detail='Viewerの読み込みとBridgeの到達性を確認しました（'+vizCredit()+'）。Lichtblick内部の接続状態はiframe外から取得できません。';
  }else if(viewer==='loading'||bridge==='checking'){
    badge='接続確認中';
    detail='ViewerとBridgeの状態を確認しています。';
  }else if(viewer==='ready'){
    badge='Bridge確認待ち';
    detail='Viewerは読み込まれました。Bridgeの到達性を確認しています。';
  }
  const badgeEl=document.getElementById('viz-status-badge');
  const detailEl=document.getElementById('viz-status-detail');
  if(badgeEl){badgeEl.textContent=badge;badgeEl.className='viz-badge'+(badgeClass?' '+badgeClass:'');}
  if(detailEl)detailEl.textContent=detail;
}
function setVizBridgeState(state){
  VIZ.bridgeState=state;
  updateVizStatus();
}
function applyVizSidebar(collapsed){
  const layout=document.getElementById('viz-layout');
  if(!layout)return;
  layout.classList.toggle('viz-sidebar-collapsed',collapsed);
  const toggle=document.getElementById('viz-panel-toggle');
  if(toggle){toggle.setAttribute('aria-expanded',String(!collapsed));toggle.title=collapsed?'設定パネルを開く':'設定パネルを閉じる';}
  vizSet(VIZ_KEYS.sidebar,collapsed?'1':'0');
}
function toggleVizSidebar(){
  const layout=document.getElementById('viz-layout');
  if(!layout)return;
  applyVizSidebar(!layout.classList.contains('viz-sidebar-collapsed'));
}
function hydrateVizSettings(){
  if(!VIZ.settings)return;
  const viewer=document.getElementById('viz-viewer-url');
  const bridge=document.getElementById('viz-bridge-url');
  const auto=document.getElementById('viz-auto');
  if(viewer)viewer.value=VIZ.settings.viewer;
  if(bridge)bridge.value=VIZ.settings.bridge;
  if(auto)auto.checked=VIZ.settings.auto;
}
function readVizSettings(defaults){
  const autoRaw=vizGet(VIZ_KEYS.auto);
  const viewer=vizGet(VIZ_KEYS.viewer)||defaults.viewer_url;
  const bridge=vizGet(VIZ_KEYS.bridge)||defaults.bridge_url;
  const auto=autoRaw===null?true:autoRaw==='1';
  return {viewer,bridge,auto,layout_url:defaults.layout_url};
}
function ensureVizWorkspace(){
  if(VIZ.workspaceReady)return Promise.resolve();
  if(VIZ.workspacePromise)return VIZ.workspacePromise;
  let origin;
  try{origin=vizViewerUrl(VIZ.settings.viewer).origin;}
  catch(error){VIZ.workspaceReady=true;return Promise.resolve();}
  if(vizGet(VIZ_KEYS.workspace)===origin){VIZ.workspaceReady=true;return Promise.resolve();}
  VIZ.workspacePromise=new Promise(resolve=>{
    let done=false,frame=null;
    const finish=ok=>{
      if(done)return;
      done=true;
      clearTimeout(timer);
      window.removeEventListener('message',onMessage);
      if(frame)frame.remove();
      if(ok)vizSet(VIZ_KEYS.workspace,origin);
      VIZ.workspaceReady=true;
      VIZ.workspacePromise=null;
      resolve();
    };
    const onMessage=event=>{
      if(event.origin!==origin||!event.data||event.data.type!=='sirius:viz-workspace-ready')return;
      finish(true);
    };
    window.addEventListener('message',onMessage);
    frame=document.createElement('iframe');
    frame.title='Lichtblick view preparation';
    frame.tabIndex=-1;
    frame.setAttribute('aria-hidden','true');
    frame.style.cssText='position:fixed;width:1px;height:1px;left:-10px;top:-10px;opacity:0;pointer-events:none;border:0';
    frame.src=origin+'/sirius-start.html?ts='+Date.now();
    document.body.append(frame);
    const timer=setTimeout(()=>finish(false),2500);
  });
  return VIZ.workspacePromise;
}
function buildVizUrl(){
  const viewer=vizViewerUrl(VIZ.settings.viewer);
  const bridge=vizBridgeUrl(VIZ.settings.bridge);
  viewer.searchParams.delete('ds');
  viewer.searchParams.delete('ds.url');
  if(VIZ.settings.auto){
    viewer.searchParams.set('ds','foxglove-websocket');
    viewer.searchParams.set('ds.url',bridge);
  }
  const firstLayout=vizGet(VIZ_KEYS.layout)!=='1';
  VIZ.pendingLayoutInit=!!(firstLayout&&VIZ.settings.layout_url);
  if(VIZ.pendingLayoutInit)viewer.searchParams.set('layoutUrl',VIZ.settings.layout_url);
  return viewer.href;
}
function showVizFailure(message){
  VIZ.loading=false;
  VIZ.frameLoaded=false;
  clearTimeout(VIZ.loadTimer);
  setVizFrameState('読み込み失敗','failed');
  const wrap=document.getElementById('viz-frame-wrap');
  if(wrap){wrap.classList.remove('loaded');wrap.classList.add('failed');}
  setVizEmpty('3Dビューを読み込めません',message+' Viewer URLとLichtblickの起動状態を確認してください。',true);
  updateVizStatus();
}
function handleVizFrameLoad(token){
  if(token!==VIZ.loadToken||!VIZ.loading)return;
  VIZ.loading=false;
  VIZ.frameLoaded=true;
  clearTimeout(VIZ.loadTimer);
  const wrap=document.getElementById('viz-frame-wrap');
  if(wrap){wrap.classList.remove('failed');wrap.classList.add('loaded');}
  setVizFrameState('読み込み完了','ready');
  if(VIZ.pendingLayoutInit){
    vizSet(VIZ_KEYS.layout,'1');
    VIZ.pendingLayoutInit=false;
  }
  updateVizStatus();
}
async function probeVisualizationBridge(){
  clearTimeout(VIZ.probeTimer);
  if(VIZ.ws){try{VIZ.ws.close();}catch(error){}VIZ.ws=null;}
  if(!VIZ.settings){setVizBridgeState('idle');return;}
  if(!VIZ.settings.auto){setVizBridgeState('idle');return;}
  let url;
  try{url=vizBridgeUrl(VIZ.settings.bridge);}catch(error){setVizBridgeState('failed');return;}
  setVizBridgeState('checking');
  let ws;
  try{
    ws=new WebSocket(url,['foxglove.websocket.v1','foxglove.sdk.v1']);
  }catch(error){
    setVizBridgeState('failed');
    return;
  }
  VIZ.ws=ws;
  VIZ.probeTimer=setTimeout(()=>{
    if(VIZ.ws!==ws)return;
    try{ws.close();}catch(error){}
    VIZ.ws=null;
    setVizBridgeState('failed');
    VIZ.lastProbe=Date.now();
  },6000);
  ws.addEventListener('open',()=>{
    clearTimeout(VIZ.probeTimer);
    if(VIZ.ws!==ws)return;
    VIZ.lastProbe=Date.now();
    setVizBridgeState('ready');
    setTimeout(()=>{if(ws.readyState===WebSocket.OPEN)try{ws.close();}catch(error){}},80);
  });
  ws.addEventListener('error',()=>{
    clearTimeout(VIZ.probeTimer);
    if(VIZ.ws!==ws)return;
    try{ws.close();}catch(error){}
    VIZ.ws=null;
    VIZ.lastProbe=Date.now();
    setVizBridgeState('failed');
  });
  ws.addEventListener('close',()=>{
    clearTimeout(VIZ.probeTimer);
    if(VIZ.ws!==ws)return;
    VIZ.ws=null;
    if(VIZ.bridgeState==='checking'){
      VIZ.lastProbe=Date.now();
      setVizBridgeState('failed');
    }
  });
}
async function loadVisualizationFrame(){
  if(!VIZ.inited){await initVisualization();return;}
  if(VIZ.loading)return;
  const frame=document.getElementById('viz-frame');
  const wrap=document.getElementById('viz-frame-wrap');
  if(!frame||!wrap)return;
  VIZ.loading=true;
  VIZ.frameLoaded=false;
  VIZ.loadToken+=1;
  const token=VIZ.loadToken;
  clearTimeout(VIZ.loadTimer);
  if(wrap){wrap.classList.remove('loaded','failed');}
  setVizFrameState('準備中','loading');
  setVizEmpty('表示環境を準備しています','Lichtblickの初期表示を設定しています。',false);
  updateVizStatus();
  if(!VIZ.workspaceReady){
    await ensureVizWorkspace();
    if(!VIZ.loading)return;
  }
  let url;
  try{url=buildVizUrl();}
  catch(error){showVizFailure(error.message);return;}
  setVizFrameState('読み込み中','loading');
  setVizEmpty('3Dビューを起動しています','Lichtblickを読み込んでいます。初回は少し時間がかかります。',false);
  updateVizStatus();
  const openTab=document.getElementById('viz-open-tab');
  if(openTab)openTab.href=url;
  frame.onload=()=>handleVizFrameLoad(token);
  frame.src=url;
  VIZ.loadTimer=setTimeout(()=>{
    if(token!==VIZ.loadToken||!VIZ.loading)return;
    showVizFailure('Viewerの読み込みがタイムアウトしました。');
  },30000);
  probeVisualizationBridge();
}
async function initVisualization(){
  if(VIZ.inited){
    if(!VIZ.frameLoaded&&!VIZ.loading)await loadVisualizationFrame();
    else if(VIZ.settings&&VIZ.settings.auto&&Date.now()-VIZ.lastProbe>30000)probeVisualizationBridge();
    return;
  }
  if(VIZ.initPromise)return VIZ.initPromise;
  VIZ.initPromise=(async()=>{
    try{
      VIZ.defaults=await fetchVizDefaults();
      VIZ.configError='';
    }catch(error){
      VIZ.defaults=vizFallbackDefaults();
      VIZ.configError='表示設定APIに接続できないため、既定のURLを使用しています。';
    }
    VIZ.settings=readVizSettings(VIZ.defaults);
    VIZ.inited=true;
    const savedSidebar=vizGet(VIZ_KEYS.sidebar);
    applyVizSidebar(savedSidebar===null?matchMedia('(max-width:900px)').matches:savedSidebar==='1');
    hydrateVizSettings();
    updateVizStatus();
    await loadVisualizationFrame();
  })();
  try{await VIZ.initPromise;}finally{VIZ.initPromise=null;}
}
function reloadVisualization(){loadVisualizationFrame();}
function saveVisualizationSettings(event){
  if(event)event.preventDefault();
  if(!VIZ.defaults)return;
  try{
    const viewer=vizViewerUrl(document.getElementById('viz-viewer-url').value).href;
    const bridge=vizBridgeUrl(document.getElementById('viz-bridge-url').value);
    const auto=document.getElementById('viz-auto').checked;
    VIZ.settings={viewer,bridge,auto,layout_url:VIZ.defaults.layout_url};
    vizSet(VIZ_KEYS.viewer,viewer);
    vizSet(VIZ_KEYS.bridge,bridge);
    vizSet(VIZ_KEYS.auto,auto?'1':'0');
    hydrateVizSettings();
    loadVisualizationFrame();
    toast('3D表示の接続設定を保存しました');
  }catch(error){
    toast('接続設定を確認してください: '+error.message,true);
  }
}
function resetVisualizationSettings(){
  if(!VIZ.defaults)return;
  document.getElementById('viz-viewer-url').value=VIZ.defaults.viewer_url;
  document.getElementById('viz-bridge-url').value=VIZ.defaults.bridge_url;
  document.getElementById('viz-auto').checked=true;
  saveVisualizationSettings();
}
function resetVisualizationLayout(){
  if(!confirm('既定の3Dレイアウトを再読み込みします。Lichtblick内のレイアウト変更は失われます。'))return;
  vizSet(VIZ_KEYS.layout,null);
  vizSet(VIZ_KEYS.workspace,null);
  VIZ.workspaceReady=false;
  loadVisualizationFrame();
  toast('既定レイアウトを再読み込みします');
}
async function toggleVisualizationFullscreen(){
  const stage=document.getElementById('viz-stage');
  try{
    if(document.fullscreenElement){await document.exitFullscreen();return;}
    if(stage&&stage.requestFullscreen){await stage.requestFullscreen();return;}
  }catch(error){}
  if(window.parent!==window&&typeof parent.siriusToggleFullscreen==='function'){
    await parent.siriusToggleFullscreen();
    return;
  }
  toast('ブラウザの F11 でもフルスクリーンにできます');
}
