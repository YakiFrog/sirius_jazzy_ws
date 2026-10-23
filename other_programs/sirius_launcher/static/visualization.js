'use strict';
const VIZ_KEYS={
  viewer:'sirius-viz-viewer',
  bridge:'sirius-viz-bridge',
  auto:'sirius-viz-auto',
  refresh:'sirius-viz-refresh-minutes',
  layout:'sirius-viz-layout-initialized-v1',
  layoutSelected:'sirius-viz-layout-selected-v1',
  sidebar:'sirius-viz-sidebar-collapsed',
  workspace:'sirius-viz-workspace-initialized-v1'
};
const VIZ_REFRESH_DEFAULT_MINUTES=60;
const VIZ_REFRESH_OPTIONS=[0,30,60,180,360];
const VIZ_DATA_KEYS={mobileAuto:'sirius-viz-mobile-auto',pauseAway:'sirius-viz-pause-away'};
const VIZ={inited:false,initPromise:null,workspaceReady:false,workspacePromise:null,frameLoaded:false,loading:false,frameState:'idle',bridgeState:'idle',defaults:null,settings:null,ws:null,loadTimer:null,probeTimer:null,refreshTimer:null,loadToken:0,pendingLayoutInit:false,pendingLayoutApply:false,lastProbe:0,configError:'',layouts:[]};
function vizGet(key){try{return localStorage.getItem(key);}catch(error){return null;}}
function vizSet(key,value){try{if(value===null)localStorage.removeItem(key);else localStorage.setItem(key,value);}catch(error){}}
function vizIsMobile(){return !!(navigator.userAgentData?.mobile||/Android|iPhone|iPad|iPod/.test(navigator.userAgent)||(navigator.maxTouchPoints>0&&matchMedia('(pointer:coarse)').matches));}
function vizPauseAway(){return vizGet(VIZ_DATA_KEYS.pauseAway)!=='0';}
function vizCanStream(){return !!(VIZ.requested&&(!vizPauseAway()||(VIZ.pageVisible&&!document.hidden)));}
function paintVizDataPolicy(){
  document.getElementById('viz-mobile-auto').checked=vizGet(VIZ_DATA_KEYS.mobileAuto)==='1';
  document.getElementById('viz-pause-away').checked=vizPauseAway();
  document.getElementById('viz-network-toggle').textContent=VIZ.requested?'通信停止':'通信開始';
  document.getElementById('viz-network-start').hidden=vizCanStream();
}
function stopVisualizationCommunication(){
  VIZ.loadToken++;VIZ.loading=false;VIZ.frameLoaded=false;
  clearTimeout(VIZ.loadTimer);clearTimeout(VIZ.refreshTimer);clearTimeout(VIZ.probeTimer);
  VIZ.refreshTimer=null;
  if(VIZ.ws){const ws=VIZ.ws;VIZ.ws=null;try{ws.close();}catch(error){}}
  if(VIZ.cancelWorkspace){VIZ.cancelWorkspace();VIZ.cancelWorkspace=null;VIZ.workspaceReady=false;}
  const frame=document.getElementById('viz-frame');
  if(frame?.hasAttribute('src')){
    const blank=frame.cloneNode(false);blank.removeAttribute('src');
    frame.onload=null;frame.replaceWith(blank);
  }
  document.getElementById('viz-frame-wrap').classList.remove('loaded','failed');
  VIZ.bridgeState='idle';setVizFrameState('通信停止','idle');
  setVizEmpty('3Dデータの通信は停止中です','「通信開始」で点群・映像の受信を開始します。テレオペは通信停止中も利用できます。',false);
  paintVizDataPolicy();updateVizStatus();
}
function visualizationPageChanged(page){
  VIZ.pageVisible=window.isPageVisible?isPageVisible('visualization'):page==='visualization';
  if(!VIZ.pageVisible){if(vizPauseAway()){VIZ.requested=false;stopVisualizationCommunication();}return;}
  if(!VIZ.requested&&(!vizIsMobile()||vizGet(VIZ_DATA_KEYS.mobileAuto)==='1'))VIZ.requested=true;
  initVisualization();
}
function toggleVisualizationCommunication(){
  VIZ.requested=!VIZ.requested;
  if(!VIZ.requested)stopVisualizationCommunication();else{paintVizDataPolicy();initVisualization();}
}
function saveVizDataPolicy(){
  vizSet(VIZ_DATA_KEYS.mobileAuto,document.getElementById('viz-mobile-auto').checked?'1':'0');
  vizSet(VIZ_DATA_KEYS.pauseAway,document.getElementById('viz-pause-away').checked?'1':'0');
  if(vizPauseAway()&&(!VIZ.pageVisible||document.hidden)){VIZ.requested=false;stopVisualizationCommunication();}
  paintVizDataPolicy();toast('3D表示の通信設定を保存しました');
}
document.addEventListener('visibilitychange',()=>{
  if(document.hidden&&vizPauseAway()){VIZ.requested=false;stopVisualizationCommunication();}
  else if(!document.hidden&&VIZ.pageVisible)visualizationPageChanged('visualization');
});
paintVizDataPolicy();
function vizFallbackDefaults(){
  const host=location.hostname||'127.0.0.1';
  const secure=location.protocol==='https:';
  return {
    viewer_url:(secure?'https://':'http://')+host+(secure?':8443/':':8091/'),
    bridge_url:(secure?'wss://':'ws://')+host+(secure?':8767':':8766'),
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
function vizFallbackLayouts(){
  return [{
    id:'default',
    name:'標準',
    builtin:true,
    url:(VIZ.defaults&&VIZ.defaults.layout_url)||location.origin+'/api/visualization/layout'
  }];
}
function vizSelectedLayout(){
  return VIZ.layouts.find(item=>item.id===VIZ.settings.layout_id)||
    VIZ.layouts.find(item=>item.id==='default')||VIZ.layouts[0]||null;
}
function hydrateVizLayoutOptions(){
  const select=document.getElementById('viz-layout-select');
  if(!select||!VIZ.settings)return;
  select.replaceChildren();
  for(const entry of VIZ.layouts){
    const option=document.createElement('option');
    option.value=entry.id;
    option.textContent=entry.builtin?entry.name+'（既定）':entry.name;
    select.append(option);
  }
  select.value=VIZ.settings.layout_id;
}
async function refreshVizLayouts(){
  let layouts=VIZ.layouts.length?VIZ.layouts:vizFallbackLayouts();
  try{
    const response=await fetch('/api/visualization/layouts',{cache:'no-store'});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const data=await response.json();
    if(Array.isArray(data.layouts)&&data.layouts.length)layouts=data.layouts;
  }catch(error){}
  VIZ.layouts=layouts;
  let entry=layouts.find(item=>item.id===VIZ.settings.layout_id);
  if(!entry){
    entry=layouts.find(item=>item.id==='default')||layouts[0];
    VIZ.settings.layout_id=entry.id;
    vizSet(VIZ_KEYS.layoutSelected,entry.id);
  }
  VIZ.settings.layout_url=entry.url;
  hydrateVizLayoutOptions();
  return entry;
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
  if(!vizCanStream()){
    setVizDot('viz-viewer-dot','idle');setVizDot('viz-bridge-dot','idle');
    document.getElementById('viz-viewer-state').textContent='通信停止';
    document.getElementById('viz-bridge-state').textContent='未接続';
    document.getElementById('viz-status-badge').textContent='通信オフ';
    document.getElementById('viz-status-badge').className='viz-badge';
    document.getElementById('viz-status-detail').textContent='Viewerと点群・映像の接続を終了しています。Launcherの状態監視は継続します。';
    return;
  }
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
function vizRefreshMinutes(value){
  if(value===null||value===undefined||value==='')return VIZ_REFRESH_DEFAULT_MINUTES;
  const minutes=Number(value);
  return VIZ_REFRESH_OPTIONS.includes(minutes)?minutes:VIZ_REFRESH_DEFAULT_MINUTES;
}
function hydrateVizSettings(){
  if(!VIZ.settings)return;
  const viewer=document.getElementById('viz-viewer-url');
  const bridge=document.getElementById('viz-bridge-url');
  const auto=document.getElementById('viz-auto');
  const refresh=document.getElementById('viz-refresh');
  if(viewer)viewer.value=VIZ.settings.viewer;
  if(bridge)bridge.value=VIZ.settings.bridge;
  if(auto)auto.checked=VIZ.settings.auto;
  if(refresh)refresh.value=String(VIZ.settings.refresh_minutes);
}
function readVizSettings(defaults){
  const autoRaw=vizGet(VIZ_KEYS.auto);
  const viewer=vizGet(VIZ_KEYS.viewer)||defaults.viewer_url;
  const bridge=vizGet(VIZ_KEYS.bridge)||defaults.bridge_url;
  const auto=autoRaw===null?true:autoRaw==='1';
  const refreshRaw=vizGet(VIZ_KEYS.refresh);
  const refresh=refreshRaw===null?VIZ_REFRESH_DEFAULT_MINUTES:vizRefreshMinutes(refreshRaw);
  return {viewer,bridge,auto,refresh_minutes:refresh,layout_url:defaults.layout_url,
    layout_id:vizGet(VIZ_KEYS.layoutSelected)||'default'};
}
function ensureVizWorkspace(){
  if(!vizCanStream())return Promise.resolve();
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
      VIZ.cancelWorkspace=null;
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
    VIZ.cancelWorkspace=()=>finish(false);
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
  if(VIZ.pendingLayoutInit||VIZ.pendingLayoutApply){
    const layoutUrl=new URL(VIZ.settings.layout_url);
    layoutUrl.searchParams.set('_',String(Date.now()));
    viewer.searchParams.set('layoutUrl',layoutUrl.href);
  }
  return viewer.href;
}
function scheduleVizRefresh(){
  clearTimeout(VIZ.refreshTimer);
  VIZ.refreshTimer=null;
  if(!vizCanStream())return;
  const minutes=VIZ.settings?Number(VIZ.settings.refresh_minutes):0;
  if(!(minutes>0))return;
  VIZ.refreshTimer=setTimeout(()=>{
    VIZ.refreshTimer=null;
    if(VIZ.loading||!VIZ.frameLoaded){scheduleVizRefresh();return;}
    loadVisualizationFrame(true);
  },minutes*60000);
}
function showVizFailure(message){
  VIZ.loading=false;
  VIZ.frameLoaded=false;
  clearTimeout(VIZ.loadTimer);
  clearTimeout(VIZ.refreshTimer);
  VIZ.refreshTimer=null;
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
  VIZ.pendingLayoutApply=false;
  scheduleVizRefresh();
  updateVizStatus();
}
async function probeVisualizationBridge(){
  if(!vizCanStream())return;
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
async function loadVisualizationFrame(silent){
  if(!vizCanStream()){stopVisualizationCommunication();return;}
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
  if(!silent){
    wrap.classList.remove('loaded','failed');
    setVizFrameState('準備中','loading');
    setVizEmpty('表示環境を準備しています','Lichtblickの初期表示を設定しています。',false);
  }else{
    setVizFrameState('自動更新中','loading');
  }
  updateVizStatus();
  if(!VIZ.workspaceReady){
    await ensureVizWorkspace();
    if(!VIZ.loading||token!==VIZ.loadToken||!vizCanStream())return;
  }
  let url;
  try{url=buildVizUrl();}
  catch(error){showVizFailure(error.message);return;}
  if(!silent){
    setVizFrameState('読み込み中','loading');
    setVizEmpty('3Dビューを起動しています','Lichtblickを読み込んでいます。初回は少し時間がかかります。',false);
  }
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
    if(!vizCanStream()){stopVisualizationCommunication();return;}
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
    await refreshVizLayouts();
    VIZ.inited=true;
    const savedSidebar=vizGet(VIZ_KEYS.sidebar);
    applyVizSidebar(savedSidebar===null?matchMedia('(max-width:900px)').matches:savedSidebar==='1');
    hydrateVizSettings();
    updateVizStatus();
    paintVizDataPolicy();
    if(vizCanStream())await loadVisualizationFrame();else stopVisualizationCommunication();
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
    const refresh=vizRefreshMinutes(document.getElementById('viz-refresh').value);
    VIZ.settings={viewer,bridge,auto,refresh_minutes:refresh,layout_url:VIZ.settings.layout_url,
      layout_id:VIZ.settings.layout_id};
    vizSet(VIZ_KEYS.viewer,viewer);
    vizSet(VIZ_KEYS.bridge,bridge);
    vizSet(VIZ_KEYS.auto,auto?'1':'0');
    vizSet(VIZ_KEYS.refresh,String(refresh));
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
  document.getElementById('viz-refresh').value=String(VIZ_REFRESH_DEFAULT_MINUTES);
  saveVisualizationSettings();
}
function applyVisualizationLayout(layoutId){
  if(VIZ.loading){toast('表示の読み込み完了後に再度お試しください',true);return;}
  const id=layoutId||(document.getElementById('viz-layout-select')||{}).value;
  const entry=VIZ.layouts.find(item=>item.id===id);
  if(!entry){toast('レイアウトが見つかりません',true);return;}
  VIZ.settings.layout_id=entry.id;
  VIZ.settings.layout_url=entry.url;
  vizSet(VIZ_KEYS.layoutSelected,entry.id);
  VIZ.pendingLayoutApply=true;
  loadVisualizationFrame();
  toast('レイアウトを適用: '+entry.name);
}
function resetVisualizationLayout(){
  if(!confirm('既定の3Dレイアウトを再読み込みします。Lichtblick内のレイアウト変更は失われます。'))return;
  vizSet(VIZ_KEYS.layout,null);
  vizSet(VIZ_KEYS.layoutSelected,'default');
  vizSet(VIZ_KEYS.workspace,null);
  VIZ.workspaceReady=false;
  applyVisualizationLayout('default');
}
function triggerVisualizationLayoutImport(){
  const input=document.getElementById('viz-layout-file');
  if(input)input.click();
}
async function importVisualizationLayoutFile(input){
  const file=input.files&&input.files[0];
  input.value='';
  if(!file)return;
  if(file.size>2000000){toast('JSONファイルが大きすぎます（2MBまで）',true);return;}
  let layout;
  try{layout=JSON.parse(await file.text());}
  catch(error){toast('JSONを読み込めません: '+error.message,true);return;}
  if(!layout||typeof layout!=='object'||Array.isArray(layout)){
    toast('レイアウトJSONはオブジェクトである必要があります',true);return;
  }
  const suggested=file.name.replace(/\.json$/i,'')||'レイアウト';
  const name=prompt('保存するレイアウト名',suggested);
  if(name===null)return;
  const clean=name.trim();
  if(!clean){toast('レイアウト名を入力してください',true);return;}
  let data={};
  try{
    const response=await fetch('/api/visualization/layouts',{
      method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({name:clean,layout})
    });
    data=await response.json().catch(()=>({}));
    if(!response.ok)throw new Error(data.error||('HTTP '+response.status));
  }catch(error){toast('レイアウトを保存できません: '+error.message,true);return;}
  await refreshVizLayouts();
  if(data.layout&&data.layout.id){
    const select=document.getElementById('viz-layout-select');
    if(select)select.value=data.layout.id;
    applyVisualizationLayout(data.layout.id);
  }
}
async function exportVisualizationLayout(){
  const select=document.getElementById('viz-layout-select');
  const entry=VIZ.layouts.find(item=>item.id===(select?select.value:VIZ.settings.layout_id));
  if(!entry){toast('レイアウトが見つかりません',true);return;}
  try{
    const response=await fetch(entry.url,{cache:'no-store'});
    if(!response.ok)throw new Error('HTTP '+response.status);
    const text=await response.text();
    const blob=new Blob([text],{type:'application/json'});
    const link=document.createElement('a');
    link.href=URL.createObjectURL(blob);
    link.download=(entry.name||entry.id).replace(/[\\/:*?"<>|]+/g,'_')+'.json';
    document.body.append(link);link.click();link.remove();
    setTimeout(()=>URL.revokeObjectURL(link.href),1000);
    toast('レイアウトを書き出しました: '+entry.name);
  }catch(error){toast('レイアウトを書き出せません: '+error.message,true);}
}
async function deleteVisualizationLayout(){
  const select=document.getElementById('viz-layout-select');
  const entry=VIZ.layouts.find(item=>item.id===(select?select.value:VIZ.settings.layout_id));
  if(!entry)return;
  if(entry.builtin){toast('標準レイアウトは削除できません',true);return;}
  if(!confirm('「'+entry.name+'」を削除しますか？'))return;
  try{
    const response=await fetch('/api/visualization/layouts/'+encodeURIComponent(entry.id),{method:'DELETE'});
    const data=await response.json().catch(()=>({}));
    if(!response.ok)throw new Error(data.error||('HTTP '+response.status));
  }catch(error){toast('レイアウトを削除できません: '+error.message,true);return;}
  const wasCurrent=VIZ.settings.layout_id===entry.id;
  await refreshVizLayouts();
  if(wasCurrent)applyVisualizationLayout('default');
  else toast('レイアウトを削除しました: '+entry.name);
}
async function toggleVisualizationFullscreen(){
  const stage=document.getElementById('viz-layout');
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
