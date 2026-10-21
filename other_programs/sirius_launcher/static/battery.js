'use strict';
(function(){
  if(document.getElementById('battery-toggle'))return;
  const host=document.querySelector('.top-status');if(!host)return;
  const style=document.createElement('link');style.rel='stylesheet';style.href='/static/battery.css';document.head.append(style);
  const toggle=document.createElement('button');toggle.id='battery-toggle';toggle.className='battery-chip';toggle.type='button';
  toggle.innerHTML='<svg viewBox="0 0 25 20" fill="none" aria-hidden="true"><rect x="1" y="4" width="20" height="12" rx="3" stroke="currentColor" stroke-width="1.5"/><path d="M24 8v4" stroke="currentColor" stroke-width="2" stroke-linecap="round"/><rect id="battery-level" x="4" y="7" width="0" height="6" rx="1" fill="currentColor"/></svg><span class="battery-chip-label">バッテリー</span><b id="battery-chip-value">—</b><span id="battery-chip-status">未取得</span>';
  const clock=document.getElementById('clock');if(clock&&clock.parentNode===host)host.insertBefore(toggle,clock);else host.append(toggle);
  const dialog=document.createElement('dialog');dialog.className='battery-dialog';dialog.id='battery-dialog';dialog.setAttribute('aria-labelledby','battery-title');
  dialog.innerHTML=`<div class="battery-heading"><h2 id="battery-title">バッテリー・駆動電源</h2><button class="hbtn" id="battery-close" aria-label="バッテリー表示を閉じる">閉じる</button></div>
    <p id="battery-state" class="battery-state">未取得</p>
    <div class="battery-values">
      <section id="battery-soc-card"><small>本体の残量</small><strong id="battery-percentage">— %</strong><div class="battery-meter"><i id="battery-percentage-bar"></i></div></section>
      <section id="battery-voltage-card"><small>Roboteq 駆動電圧</small><strong id="battery-voltage">— V</strong></section>
      <section id="battery-input-card"><small>Anker 入力合計</small><strong id="battery-power-input">— W</strong></section>
      <section id="battery-output-card"><small>Anker 出力合計</small><strong id="battery-power-output">— W</strong></section>
    </div>
    <div class="battery-details-grid">
    <section class="battery-source-group" id="battery-body-group">
      <div class="battery-source-heading"><h3>バッテリー本体</h3><span id="battery-body-state">未取得</span></div>
      <dl class="battery-facts"><dt>本体温度</dt><dd id="battery-temperature">— °C</dd><dt>充放電状態</dt><dd id="battery-charging">未取得</dd><dt>最終受信</dt><dd id="battery-age">未取得</dd><dt>取得元</dt><dd id="battery-source">未取得</dd></dl>
    </section>
    <section class="battery-source-group" id="battery-power-group">
      <div class="battery-source-heading"><h3>Anker 入出力・残り時間</h3><span id="battery-power-state">未取得</span></div>
      <dl class="battery-facts"><dt>推定稼働時間</dt><dd id="battery-runtime">未取得</dd><dt>充電完了まで</dt><dd id="battery-charge-time">—</dd><dt>充電完了予定</dt><dd id="battery-charge-at">—</dd></dl>
      <p class="battery-port-caption">ポート別電力（瞬時値）</p><dl class="battery-ports">
        <div><dt>AC入力</dt><dd id="battery-port-ac_input">— W</dd></div><div><dt>ソーラー入力</dt><dd id="battery-port-solar_input">— W</dd></div>
        <div><dt>AC出力</dt><dd id="battery-port-ac_output">— W</dd></div><div><dt>シガー出力</dt><dd id="battery-port-dc_output">— W</dd></div>
        <div><dt>USB-C1</dt><dd id="battery-port-usb_c1">— W</dd></div><div><dt>USB-C2</dt><dd id="battery-port-usb_c2">— W</dd></div>
        <div><dt>USB-C3</dt><dd id="battery-port-usb_c3">— W</dd></div><div><dt>USB-A1</dt><dd id="battery-port-usb_a1">— W</dd></div>
      </dl><p class="battery-detail-note" id="battery-time-note">残り時間は本体の推定値を表示します。</p>
      <dl class="battery-facts"><dt>最終受信</dt><dd id="battery-power-age">未取得</dd><dt>取得元</dt><dd id="battery-power-source">未取得</dd></dl>
    </section>
    <section class="battery-source-group" id="battery-driver-group">
      <div class="battery-source-heading"><h3>Roboteq モータードライバー</h3><span id="battery-driver-state">未取得</span></div>
      <dl class="battery-facts"><dt>MD電源電流</dt><dd id="battery-current">— A</dd><dt>MD温度</dt><dd id="battery-driver-temperature">— °C</dd><dt>MD内部電圧</dt><dd id="battery-internal">— V</dd><dt>非常停止</dt><dd id="battery-estop">未取得</dd><dt>ドライバー通知</dt><dd id="battery-faults">未取得</dd><dt>最終受信</dt><dd id="battery-driver-age">未取得</dd><dt>取得元</dt><dd id="battery-driver-source">未取得</dd></dl>
    </section>
    </div>
    <p class="battery-help" id="battery-help"></p>`;
  document.body.append(dialog);toggle.onclick=()=>{dialog.showModal();document.getElementById('battery-close').focus();};
  document.getElementById('battery-close').onclick=()=>dialog.close();dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});
  const labels={waiting:'未取得',connecting:'接続待ち',invalid:'値未取得',absent:'未接続',stale:'更新停止',receiver_error:'受信エラー',connection_error:'通信待ち',auth:'ログインが必要です'};
  let last={},busy=false;
  const text=(id,value)=>{document.getElementById(id).textContent=value;};
  const fmt=(v,unit,decimals=1)=>Number.isFinite(v)?v.toFixed(decimals)+' '+unit:'— '+unit;
  const watt=v=>fmt(v,'W',Number.isInteger(v)?0:1);
  function duration(hours){
    if(!Number.isFinite(hours)||hours<=0||hours>=6553.5)return '未取得';
    const minutes=Math.round(hours*60),h=Math.floor(minutes/60),m=minutes%60;
    return minutes===0?'1分未満':(h?h+'時間':'')+(m?m+'分':'');
  }
  function source(data,key,kind){
    const record=data[key]||(data.source_type===kind?data:{state:'waiting'});
    return ['connecting','receiver_error','connection_error','auth'].includes(data.state)?{...record,state:data.state}:record;
  }
  function status(record){
    if(record.state!=='live')return labels[record.state]||'未取得';
    if(record.emergency_stop===true)return '非常停止中';
    if(record.fault_flags>0)return 'ドライバー通知';
    return record.charging||'受信中';
  }
  function render(data){
    last=data;
    const battery=source(data,'battery','BatteryState'),driver=source(data,'driver','RoboteqStatus'),details=source(data,'details','SolixStatus');
    const primary=data.source_type==='RoboteqStatus'?driver:battery;
    const live=primary.state==='live',driverLive=driver.state==='live';
    const caution=driverLive&&(driver.emergency_stop===true||driver.fault_flags>0);
    let message=status(primary);
    if(primary!==driver&&caution)message+=' · '+status(driver);
    else if(primary!==driver&&driver.state==='stale')message+=' · MD更新停止';
    const value=Number.isFinite(primary.percentage)?fmt(primary.percentage,'%',0):Number.isFinite(primary.voltage)?fmt(primary.voltage,'V'):'—';
    text('battery-chip-value',value);text('battery-chip-status',message);
    document.querySelector('.battery-chip-label').textContent=primary===driver?'MD電源':'バッテリー';
    toggle.dataset.state=primary.state||'waiting';toggle.classList.toggle('caution',caution||driver.state==='stale');
    toggle.title='バッテリー・駆動電源: '+value+' / '+message;toggle.setAttribute('aria-label',toggle.title);
    document.getElementById('battery-level').setAttribute('width',live&&Number.isFinite(primary.percentage)?String(Math.min(100,Math.max(0,primary.percentage))/100*14):'0');
    text('battery-state',message);
    text('battery-percentage',fmt(battery.percentage,'%',0));text('battery-voltage',fmt(driver.voltage,'V'));
    text('battery-temperature',fmt(battery.temperature,'°C'));text('battery-current',fmt(driver.current,'A'));
    text('battery-driver-temperature',fmt(driver.temperature,'°C'));text('battery-internal',fmt(driver.internal_voltage,'V'));
    document.getElementById('battery-percentage-bar').style.width=Number.isFinite(battery.percentage)?Math.min(100,Math.max(0,battery.percentage))+'%':'0';
    text('battery-power-input',watt(details.input_power));text('battery-power-output',watt(details.output_power));
    for(const id of ['battery-input-card','battery-output-card'])document.getElementById(id).classList.toggle('is-stale',details.state!=='live');
    const unknownFlow=!['charging','discharging','idle'].includes(details.time_remaining_kind);
    text('battery-runtime',details.time_remaining_kind==='discharging'?duration(details.runtime_hours):unknownFlow?'未取得':'—');
    text('battery-charge-time',details.time_remaining_kind==='charging'?duration(details.charge_remaining_hours):unknownFlow?'未取得':'—');
    text('battery-charge-at',details.time_remaining_kind==='charging'?(Number.isFinite(details.charge_complete_at)?new Date(details.charge_complete_at*1000).toLocaleString('ja-JP',{month:'numeric',day:'numeric',hour:'2-digit',minute:'2-digit',timeZone:'Asia/Tokyo'})+' JST':'未取得'):'—');
    for(const key of ['ac_input','solar_input','ac_output','dc_output','usb_c1','usb_c2','usb_c3','usb_a1']){
      let value=watt((details.ports||{})[key]);
      const state=(details.output_states||{})[key==='ac_output'?'ac':key==='dc_output'?'dc':''];
      if(state==='on'||state==='off')value+=' · '+state.toUpperCase();
      text('battery-port-'+key,value);
    }
    const noTime=['charging','discharging'].includes(details.time_remaining_kind)&&!Number.isFinite(details.runtime_hours)&&!Number.isFinite(details.charge_remaining_hours);
    text('battery-time-note',(noTime?'本体の残り時間は未取得です。':'残り時間は本体の推定値です。')+'USBは配信されたポート電力を表示し、入出力方向は未取得です。');
    for(const [record,group,card,stateId,ageId,sourceId] of [
      [battery,'battery-body-group','battery-soc-card','battery-body-state','battery-age','battery-source'],
      [driver,'battery-driver-group','battery-voltage-card','battery-driver-state','battery-driver-age','battery-driver-source'],
      [details,'battery-power-group',null,'battery-power-state','battery-power-age','battery-power-source']]){
      document.getElementById(group).classList.toggle('is-stale',record.state!=='live');
      if(card)document.getElementById(card).classList.toggle('is-stale',record.state!=='live');
      text(stateId,status(record));text(ageId,Number.isFinite(record.age_seconds)?record.age_seconds.toFixed(1)+'秒前':'未取得');
      text(sourceId,record.topic?(record.source_type+' · '+record.topic):'受信を待っています');
    }
    if(details.device_mac)text('battery-power-source',(details.topic||'')+' · '+details.device_mac);
    text('battery-charging',battery.charging||'未取得');
    const lastKnown=driverLive?'':'（最終値）';
    text('battery-estop',driver.emergency_stop===true?'非常停止中'+lastKnown:driver.emergency_stop===false?'停止フラグなし'+lastKnown:'未取得');
    text('battery-faults',Number.isInteger(driver.fault_flags)?'0x'+driver.fault_flags.toString(16).toUpperCase()+lastKnown:'未取得');
    text('battery-help',data.receiver_error||((data.errors||[]).join(' / '))||'残量・本体温度はバッテリー、駆動電圧・電流・内部電圧・非常停止はRoboteqの取得値です。駆動電圧は、非常停止などによりバッテリー本体の電圧とは異なる場合があります。');
  }
  async function poll(){
    if(busy||document.hidden)return;busy=true;
    try{const response=await fetch('/api/battery',{cache:'no-store',signal:AbortSignal.timeout(5000)});if(response.status===401){render({...last,state:'auth'});return;}if(!response.ok)throw new Error('HTTP '+response.status);render(await response.json());}
    catch(error){render({...last,state:'connection_error'});}finally{busy=false;}
  }
  render({state:'waiting'});poll();setInterval(poll,3000);document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});
})();
