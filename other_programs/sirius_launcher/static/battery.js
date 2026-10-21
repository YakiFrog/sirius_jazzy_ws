'use strict';
(function(){
  if(document.getElementById('battery-toggle'))return;
  const host=document.querySelector('.top-status');if(!host)return;
  const style=document.createElement('link');style.rel='stylesheet';style.href='/static/battery.css';document.head.append(style);
  const toggle=document.createElement('button');toggle.id='battery-toggle';toggle.className='battery-chip';toggle.type='button';
  toggle.innerHTML='<svg viewBox="0 0 25 20" fill="none" aria-hidden="true"><rect x="1" y="4" width="20" height="12" rx="3" stroke="currentColor" stroke-width="1.5"/><path d="M24 8v4" stroke="currentColor" stroke-width="2" stroke-linecap="round"/><rect id="battery-level" x="4" y="7" width="0" height="6" rx="1" fill="currentColor"/></svg><span class="battery-chip-label">バッテリー</span><b id="battery-chip-value">—</b><span id="battery-chip-status">未取得</span>';
  const clock=document.getElementById('clock');if(clock&&clock.parentNode===host)host.insertBefore(toggle,clock);else host.append(toggle);
  const dialog=document.createElement('dialog');dialog.className='battery-dialog';dialog.id='battery-dialog';dialog.setAttribute('aria-labelledby','battery-title');
  dialog.innerHTML='<div class="battery-heading"><h2 id="battery-title">バッテリー・駆動電源</h2><button class="hbtn" id="battery-close" aria-label="バッテリー表示を閉じる">閉じる</button></div><p id="battery-state" class="battery-state">未取得</p><div class="battery-values"><section><small>残量</small><strong id="battery-percentage">— %</strong><div class="battery-meter"><i id="battery-percentage-bar"></i></div></section><section><small id="battery-voltage-label">電圧</small><strong id="battery-voltage">— V</strong></section></div><dl class="battery-facts"><dt>電流</dt><dd id="battery-current">— A</dd><dt>温度</dt><dd id="battery-temperature">— °C</dd><dt>内部電圧</dt><dd id="battery-internal">— V</dd><dt>充放電状態</dt><dd id="battery-charging">未取得</dd><dt>非常停止</dt><dd id="battery-estop">未取得</dd><dt>ドライバー通知</dt><dd id="battery-faults">未取得</dd><dt>最終受信</dt><dd id="battery-age">未取得</dd><dt>取得元</dt><dd id="battery-source">受信を待っています</dd></dl><p class="battery-help" id="battery-help">バッテリー情報の受信を待っています。</p>';
  document.body.append(dialog);toggle.onclick=()=>{dialog.showModal();document.getElementById('battery-close').focus();};
  document.getElementById('battery-close').onclick=()=>dialog.close();dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});
  const labels={waiting:'未取得',connecting:'接続待ち',invalid:'値未取得',absent:'未接続',stale:'更新停止',receiver_error:'受信エラー',connection_error:'通信待ち',auth:'ログインが必要です'};
  let last={},busy=false;
  const fmt=(v,unit,decimals=1)=>Number.isFinite(v)?v.toFixed(decimals)+' '+unit:'— '+unit;
  function render(data){
    last=data;const live=data.state==='live';let status=labels[data.state]||'受信中';
    if(live&&data.emergency_stop===true)status='非常停止中';else if(live&&data.fault_flags>0)status='ドライバー通知';else if(live&&data.charging)status=data.charging;
    const value=Number.isFinite(data.percentage)?fmt(data.percentage,'%',0):Number.isFinite(data.voltage)?fmt(data.voltage,'V'):'—';
    document.getElementById('battery-chip-value').textContent=value;document.getElementById('battery-chip-status').textContent=status;
    toggle.dataset.state=data.state||'waiting';toggle.classList.toggle('caution',live&&(data.emergency_stop===true||data.fault_flags>0));
    toggle.title='バッテリー・駆動電源: '+value+' / '+status;toggle.setAttribute('aria-label',toggle.title);
    const level=document.getElementById('battery-level');level.setAttribute('width',live&&Number.isFinite(data.percentage)?String(Math.min(100,Math.max(0,data.percentage))/100*14):'0');
    document.getElementById('battery-state').textContent=status;document.getElementById('battery-state').dataset.state=data.state;
    for(const [id,field,unit] of [['percentage','percentage','%'],['voltage','voltage','V'],['current','current','A'],['temperature','temperature','°C'],['internal','internal_voltage','V']])document.getElementById('battery-'+id).textContent=fmt(data[field],unit,field==='percentage'?0:1);
    document.getElementById('battery-percentage-bar').style.width=Number.isFinite(data.percentage)?Math.min(100,Math.max(0,data.percentage))+'%':'0';
    dialog.classList.toggle('battery-stale',!live);document.getElementById('battery-voltage-label').textContent=data.source_type==='RoboteqStatus'?'駆動電源電圧':'電圧';
    document.getElementById('battery-charging').textContent=data.charging||'未取得';
    document.getElementById('battery-estop').textContent=data.emergency_stop===true?'非常停止中':data.emergency_stop===false?'停止フラグなし':'未取得';
    document.getElementById('battery-faults').textContent=Number.isInteger(data.fault_flags)?'0x'+data.fault_flags.toString(16).toUpperCase():'未取得';
    document.getElementById('battery-age').textContent=Number.isFinite(data.age_seconds)?data.age_seconds.toFixed(1)+'秒前':'未取得';
    document.getElementById('battery-source').textContent=data.topic?(data.source_type+' · '+data.topic):(data.sources||[]).join(', ')||'受信を待っています';
    const help=document.getElementById('battery-help');
    help.textContent=data.receiver_error||((data.errors||[]).join(' / '))||(data.source_type==='RoboteqStatus'?'電圧はモータードライバー駆動側の値です。非常停止で電源が遮断されると、バッテリー本体の電圧とは異なる場合があります。この配信には残量％が含まれていません。':Number.isFinite(data.percentage)?'残量はバッテリー情報の配信値です。':'残量％が配信されると、ここに表示します。');
  }
  async function poll(){
    if(busy||document.hidden)return;busy=true;
    try{const response=await fetch('/api/battery',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(response.status===401){render({...last,state:'auth'});return;}if(!response.ok)throw new Error('HTTP '+response.status);render(await response.json());}
    catch(error){render({...last,state:'connection_error'});}finally{busy=false;}
  }
  render({state:'waiting'});poll();setInterval(poll,3000);document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});
})();
