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
function lifecycleHealth(payload, now = Date.now() / 1000) {
  if (!payload?.available) return {tone: 'unknown', errors: []};
  const rows = payload.telemetry?.nodes || {};
  const logs = payload.telemetry?.logs || {};
  const errors = [];
  let observed = 0;
  for (const [name, entry] of Object.entries(rows)) {
    if (!entry?.fresh || !Number.isInteger(entry.value?.id)) continue;
    observed++;
    const id = entry.value.id;
    let reason = id === 15 ? 'エラー処理中' : '';
    const log = logs[name];
    const unresolved = id !== 3 && Number.isFinite(log?.age) &&
      Number.isFinite(entry.observed_seconds) &&
      log.age <= entry.observed_seconds + 2;
    const recovered = id === 3 && Number.isFinite(log?.age) &&
      Number.isFinite(entry.observed_seconds) && log.age > entry.observed_seconds + 2;
    if (log?.value?.level >= 40 && !recovered && (log.fresh || unresolved)) {
      reason = log.value.text || 'エラーログを受信';
    }
    const operation = (payload.operations || []).filter(job => job.node === name)
      .sort((a, b) => b.started - a.started)[0];
    if (operation?.status === 'failed' && Number.isFinite(operation.finished) &&
        Number.isFinite(entry.observed_seconds) &&
        operation.finished >= now - entry.observed_seconds - 2) {
      reason = operation.result?.error || '操作に失敗';
    }
    if (reason) errors.push({node: name, reason});
  }
  if (errors.length) return {tone: 'error', errors};
  if (!observed) return {tone: 'unknown', errors};
  // These two nodes can stay inactive when their optional features are unused.
  const required = Object.keys(payload.nodes || rows)
    .filter(name => name !== 'docking_server' && name !== 'waypoint_follower');
  if (required.some(name => !rows[name]?.fresh || !Number.isInteger(rows[name].value?.id))) {
    return {tone: 'unknown', errors};
  }
  const waiting = required.some(name => rows[name].value.id !== 3) ||
    (payload.operations || []).some(job => job.status === 'running');
  return {tone: waiting ? 'waiting' : 'ok', errors};
}
function lifecycleStartup(payload) {
  if (!payload?.available) return [];
  const rows = payload.telemetry?.nodes || {};
  const logs = payload.telemetry?.logs || {};
  const errors = lifecycleHealth(payload).errors;
  const notices = [];
  for (const [name, parent] of [
    ['global_costmap/global_costmap', 'planner_server'],
    ['local_costmap/local_costmap', 'controller_server'],
  ]) {
    const entry = rows[name];
    if (!entry?.fresh) continue;
    const failure = errors.find(error => error.node === name);
    if (failure && /before timeout/i.test(failure.reason)) {
      notices.push({node: name, parent, tone: 'error', label: 'TF待機タイムアウト', reason: failure.reason});
    } else if (entry.value?.id === 13 && logs[name]?.fresh &&
               /transform/i.test(logs[name].value?.text || '')) {
      const seconds = Math.max(0, Math.floor(entry.observed_seconds || 0));
      notices.push({node: name, parent, tone: 'waiting', label: 'TF接続待ち ' + seconds + '秒経過', reason: logs[name].value.text});
    }
  }
  return notices;
}
if(typeof module!=='undefined')module.exports={lifecycleState,lifecycleCostmap,lifecycleGraphState,lifecycleHealth,lifecycleStartup};
