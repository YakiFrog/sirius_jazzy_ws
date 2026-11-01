'use strict';
// Graph status is evidence, not a drive-readiness decision.
function lifecycleState(entry, available) {
  if (!available || !entry?.fresh || !entry.value || !Number.isInteger(entry.value.id)) return {tone:'unknown',label:'未取得'};
  const id=entry.value.id;
  if(id===3)return {tone:'ok',label:'稼働中'};
  if(id===1)return {tone:'waiting',label:'未設定'};
  if(id===2)return {tone:'waiting',label:'待機中'};
  if(id===15)return {tone:'error',label:'エラー処理中'};
  if(id>=10&&id<=14)return {tone:'transition',label:'遷移中'};
  return {tone:'unknown',label:id===4?'終了':'状態不明'};
}
function lifecycleCostmap(entry, available) {
  if(!available||!entry?.fresh||!entry.value)return {tone:'unknown',label:'受信未確認'};
  return entry.value.valid ? {tone:'ok',label:'受信中'} : {tone:'error',label:'空・不正'};
}
function lifecycleGraphState(nav, domain, mapReady) {
  const valid=!!nav?.available && (!nav.domain || String(nav.domain)===String(domain));
  const pre=valid ? nav.navigation?.preflight||{} : {},stages=valid ? nav.navigation?.stages||{} : {};
  const current=key=>stages[key]?.fresh?stages[key].value:null;
  const tf=!valid||!nav.readiness ? {tone:'unknown',label:'未取得'} :
    nav.readiness.ready&&mapReady(nav) ? {tone:'ok',label:'接続確認'} : {tone:'waiting',label:'要確認'};
  const scan=pre.scan?.fresh&&pre.scan.value;
  const laser=!scan?{tone:'unknown',label:'受信未確認'}:
    scan.valid&&scan.samples>0&&scan.stamp_current?{tone:'ok',label:'受信中'}:{tone:'waiting',label:'要確認'};
  const route=pre.routing?.fresh&&pre.routing.value,lock=current('lock');
  const mux=lock===true?{tone:'error',label:'ロック中'}:
    route&&!route.connected?{tone:'waiting',label:'接続要確認'}:
    route?.connected&&lock===false?{tone:'ok',label:'接続・解除確認'}:{tone:'unknown',label:'未確認'};
  const stopped=current('driver'),communication=pre.communication?.fresh&&pre.communication.value===true;
  const driver=stopped===true?{tone:'error',label:'停止中'}:
    stopped===false&&communication?{tone:'ok',label:'応答あり'}:{tone:'unknown',label:'未確認'};
  return {tf,laser,mux,driver};
}
if(typeof module!=='undefined')module.exports={lifecycleState,lifecycleCostmap,lifecycleGraphState};
