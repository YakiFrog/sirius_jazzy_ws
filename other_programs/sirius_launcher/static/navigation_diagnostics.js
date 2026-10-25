'use strict';

function navigationDiagnosticMapReady(data) {
  const edges = data.edges || [], reachable = new Set(['map']);
  for (let i = 0; i <= edges.length; i++) {
    let changed = false;
    for (const edge of edges) {
      if (edge.stale) continue;
      if (reachable.has(edge.parent) && !reachable.has(edge.child)) { reachable.add(edge.child); changed = true; }
      if (reachable.has(edge.child) && !reachable.has(edge.parent)) { reachable.add(edge.parent); changed = true; }
    }
    if (!changed) break;
  }
  return reachable.has('sirius3/base_footprint');
}

function navigationDiagnosticState(data) {
  const result = (tone, badge, title, detail) => ({tone, badge, title, detail});
  if (!data || !data.available || !data.navigation) return result('unknown', '未確認', '現在の走行状態は未確認', '診断データの受信を待っています。');
  const stages = data.navigation.stages || {};
  const current = key => stages[key] && stages[key].fresh ? stages[key].value : null;
  const moving = key => !!(current(key) && current(key).moving);
  if (data.navigation.mismatch) return result('blocked', '停止', '停止状態が食い違っています', 'Roboteqは解除済みですが、muxの停止ロックが残り、最終速度指令が出ていません。');
  if (current('driver') === true) return result('blocked', '停止', '緊急停止中：駆動側で停止', 'Roboteqの緊急停止が有効です。速度指令が出ていても駆動は停止します。');
  if (current('lock') === true) return result('blocked', '停止', '停止ロック中：muxで遮断', '指令選択器（twist_mux）の停止ロックが有効です。');
  if (data.readiness && data.readiness.state === 'error') return result('blocked', 'TFエラー', 'TF構成エラー', '座標変換の競合や循環があります。TFTreeで確認してください。');
  if (current('driver') !== false || current('lock') !== false) return result('unknown', '未確認', '停止状態を確認できません', 'Roboteq・muxの両方について、新しい解除状態を確認するまでOK表示にはしません。');
  if (moving('safe') && !moving('output')) return result('attention', '要確認', 'muxの手前で指令が止まっています', '安全監視通過後には移動指令がありますが、最終出力は停止指令、または未受信です。手動指令による上書きの可能性もあります。');
  if (moving('nav') && !moving('safe')) return result('attention', '要確認', '安全監視の段階で停止・受信待ち', 'Nav2は移動を指示していますが、安全監視を通過した移動指令を確認できません。');
  if (!data.readiness || !data.readiness.ready || !navigationDiagnosticMapReady(data)) return result('attention', '要確認', '自律移動の座標変換を確認できません', '停止は解除済みですが、mapからロボットまでのTF準備を確認できません。TFTreeで確認してください。');
  if (moving('output') && current('motion') === true) {
    return moving('nav') && moving('safe')
      ? result('ok', 'OK', '走行指令が通過・車輪回転を確認', '停止解除・TF・Nav2指令・最終出力・車輪Odomを確認しています。')
      : result('ok', 'OK', '最終指令と車輪回転を確認', '手動操作などの可能性もあります。Nav2による自律走行とは限りません。');
  }
  if (moving('output')) return result('attention', '要確認', '指令は通過・車輪回転は未確認', '最終速度指令は出ていますが、車輪Odomの動きを確認できません。');
  if (current('goal') === 6) return result('attention', '要確認', 'Nav2の直近ゴールが失敗', '直近の通知を確認してください。停止解除だけで経路計算が成功するとは限りません。');
  return result('idle', '待機', '停止要因は未検出・移動指令待ち', '停止解除とTFは確認済みです。現在は最終の移動指令がありません。ゴール待ち・計算中・到着後などを各段階で確認してください。');
}

function navigationPowerStates(data) {
  const battery = data && data.battery && data.battery.details || {};
  const live = !!(data && data.available && battery.state === 'live'
    && Number.isFinite(battery.age_seconds) && battery.age_seconds < (battery.stale_after_seconds || 5));
  const states = battery.output_states || {}, ports = battery.ports || {};
  const port = (key, field) => ({value:live && ['on','off'].includes(states[key]) ? states[key] : null,
    power:live && Number.isFinite(ports[field]) ? ports[field] : null});
  return {ac:port('ac','ac_output'), dc:port('dc','dc_output'), live,
    age:battery.age_seconds, mac:battery.device_mac};
}

function navigationPreflightState(data) {
  const checks = [];
  const add = (id, label, tone, text) => checks.push({id,label,tone,text});
  const navigation = data && data.available && data.navigation || {};
  const preflight = navigation.preflight || {}, nodes = preflight.nodes || {}, stages = navigation.stages || {};
  const current = key => stages[key] && stages[key].fresh ? stages[key].value : null;
  const names = {bt_navigator:'Nav2のゴール受付', planner_server:'経路計画', controller_server:'速度制御',
    amcl:'自己位置推定', map_server:'地図配信', velocity_smoother:'速度平滑化', collision_monitor:'衝突監視'};
  for (const [name,label] of Object.entries(names)) {
    const entry = nodes[name] || {}, value = entry.value;
    add(name,label,!entry.fresh || !value ? 'unknown' : value.active ? 'ok' : 'blocked',
      !entry.fresh || !value ? '状態未確認' : value.active ? '稼働中' : '非稼働：'+value.label);
  }
  const tf = data && data.available && data.readiness;
  add('tf','自己位置のTF',!tf ? 'unknown' : tf.ready && navigationDiagnosticMapReady(data) ? 'ok' : 'blocked',
    !tf ? '未確認' : tf.ready && navigationDiagnosticMapReady(data) ? 'map → ロボット接続' : 'TF未準備・要確認');
  const scan = preflight.scan || {}, laser = scan.value;
  add('scan','AMCLが使うスキャン',!scan.fresh || !laser ? 'unknown' : laser.valid && laser.samples > 0 && laser.stamp_current ? 'ok' : 'blocked',
    !scan.fresh || !laser ? (scan.topic || '使用トピック')+'：受信未確認' : scan.topic+(laser.valid && laser.samples > 0 && laser.stamp_current ? ' を受信' : '：データ・時刻を確認'));
  const communication = preflight.communication || {};
  add('communication','ドライバ通信',communication.fresh && communication.value === true ? 'ok' : 'unknown',
    communication.fresh && communication.value === true ? '通信応答あり' : '応答未確認（停止中の影響も確認）');
  const routing = preflight.routing || {};
  add('routing','現在のmux指令経路',!routing.fresh || !routing.value ? 'unknown' : routing.value.connected ? 'ok' : 'blocked',
    !routing.fresh || !routing.value ? '接続未確認' : routing.value.connected ? '配信元・購読先を確認' : '未接続：'+routing.value.missing.slice(0,2).join(' / ')+(routing.value.missing.length>2?' ほか'+(routing.value.missing.length-2)+'件':''));
  add('mux','mux状態の受信',stages.lock && stages.lock.fresh ? 'ok' : 'unknown',
    stages.lock && stages.lock.fresh ? '受信中（ロックON/OFFは除外）' : '未受信');
  const power = navigationPowerStates(data);
  for (const [key,label] of [['ac','AC電源出力'],['dc','シガー出力']]) {
    const port = power[key];
    add(key,label,port.value === 'on' ? 'ok' : port.value === 'off' ? 'blocked' : 'unknown',
      port.value ? port.value.toUpperCase()+(port.power === null ? '' : ' · '+port.power.toFixed(0)+' W') : '出力スイッチ未確認');
  }
  // Emergency stops are downstream of these checks and are deliberately not
  // failures here. Do not require a moving wheel or a post-mux output while held.
  const nav = current('nav'), safe = current('safe');
  if (nav && nav.moving && (!safe || !safe.moving)) add('safety_limit','安全監視の通過状況','attention',
    safe ? 'Nav2指令はありますが、安全監視後は停止指令です' : 'Nav2指令あり・安全監視後の指令は未受信です');
  if (current('driver') === false && current('lock') === false && safe && safe.moving) {
    const output = current('output');
    if (!output || !output.moving) add('final_output','停止解除後の最終指令','attention','mux入力あり・移動出力未確認');
  }
  if (!data || !data.available || !data.navigation) return {tone:'unknown',badge:'未確認',title:'解除前チェックのデータを受信待ち',detail:'現在の準備状態は未確認です。',checks};
  const failed = checks.filter(c => c.tone === 'blocked'), unknown = checks.filter(c => c.tone !== 'ok');
  if (failed.length) return {tone:'blocked',badge:'準備不足',title:'緊急停止以外にも準備不足があります',
    detail:failed.slice(0,3).map(c=>c.label+'：'+c.text).join(' ／ ')+(failed.length>3?' ／ ほか'+(failed.length-3)+'項目（内訳を確認）':''),checks};
  if (unknown.length) return {tone:'attention',badge:'未確認',title:'解除後の自律移動準備は、まだ確認できません',
    detail:unknown.slice(0,3).map(c=>c.label+'：'+c.text).join(' ／ ')+(unknown.length>3?' ／ ほか'+(unknown.length-3)+'項目（内訳を確認）':''),checks};
  return {tone:'ok',badge:'準備OK',title:'緊急停止を除いた自律移動の準備はOK',
    detail:nav && nav.moving ? 'Nav2の移動指令も受信しています。解除操作・試走は行っていません。'
      : 'ゴールを受ける準備はOKです。現在は移動指令待ちです。',checks};
}

if (typeof module !== 'undefined') module.exports = {navigationDiagnosticState,navigationPreflightState,navigationPowerStates};
if (typeof document !== 'undefined') (() => {
  const host = document.querySelector('.top-status');
  if (!host || document.getElementById('navigation-diagnostics-button')) return;
  const style = document.createElement('link'); style.rel = 'stylesheet'; style.href = '/static/navigation_diagnostics.css'; document.head.append(style);
  const button = document.createElement('button'); button.id = 'navigation-diagnostics-button'; button.type = 'button'; button.dataset.tone = 'unknown';
  button.innerHTML = '<span>走行診断</span><b class="nav-diag-badge">未確認</b>'; button.setAttribute('aria-haspopup', 'dialog'); host.append(button);
  const dialog = document.createElement('dialog'); dialog.id = 'navigation-diagnostics-dialog'; dialog.setAttribute('aria-labelledby', 'nav-diag-title');
  dialog.innerHTML = `<div class="nav-diag-heading"><div><small>LIVE / 読み取り専用</small><h2 id="nav-diag-title">走行診断</h2></div><button type="button">閉じる</button></div>
    <section id="nav-diag-summary" data-tone="unknown" role="status"><span id="nav-diag-overall-badge" class="nav-diag-badge">未確認</span><div><h3 id="nav-diag-overall-title">現在の走行状態は未確認</h3><p id="nav-diag-overall-detail">受信待ち</p></div></section>
    <div class="nav-diag-legend"><span data-tone="ok">確認済み</span><span data-tone="blocked">停止要因</span><span data-tone="attention">要確認</span><span data-tone="idle">待機</span><span data-tone="unknown">未確認</span></div>
    <h3 class="nav-diag-section-title">まず、走行を止める要因を確認</h3><div class="nav-diag-gates"></div>
    <h3 class="nav-diag-section-title">自律移動の流れ</h3><p class="nav-diag-note">ゴールを受ける → 経路を作る → 速度を決める → 安全監視 → 指令を選ぶ → 車輪の動きを確認</p><ol class="nav-diag-flow"></ol>
    <details class="nav-diag-help"><summary>数値・表示の読み方</summary><p><b>並進 m/s</b>：1秒間に進む距離。0.5 m/sなら1秒で約50cmです。<br><b>旋回 °/s</b>：1秒間に向きが変わる角度。90 °/sなら1秒で約90度です。<br><b>経路の点数</b>：Nav2が経路を細かく分けた点の数です。ウェイポイント数ではありません。<br><b>受信時刻</b>：その段階の最後の通知です。ゴールや経路は変更時だけ通知される場合があり、古い通知だけではエラーと断定できません。</p></details>
    <details class="nav-diag-technical"><summary>トピック・元の数値</summary><div></div></details>
    <h3 class="nav-diag-section-title">直近のNav2通知</h3><p id="nav-diag-event">未取得</p><p class="nav-diag-note">OKは上記の受信状態を示します。車輪の回転があっても、空転や周囲の安全までは判定しません。表示から走行・停止・解除の指令は送りません。</p>`;
  const preflightPanel = document.createElement('section'); preflightPanel.id = 'nav-diag-preflight'; preflightPanel.dataset.tone = 'unknown';
  preflightPanel.innerHTML = '<small>緊急停止を除いた「解除前チェック」</small><div><b class="nav-diag-badge">未確認</b><h3 id="nav-diag-preflight-title">準備状態を確認中</h3></div><p id="nav-diag-preflight-detail"></p><p class="nav-diag-note">現在の物理・電子停止とmuxロックは、この判定から除外します。解除後の駆動電圧や実際の駆動応答は、停止中には検証できません。</p>';
  dialog.querySelector('#nav-diag-summary').before(preflightPanel);
  const breakdown = document.createElement('details'); breakdown.className = 'nav-diag-preflight-list'; breakdown.open = true;
  breakdown.innerHTML = '<summary>解除前チェックの内訳</summary><ul></ul>';
  preflightPanel.after(breakdown);
  document.body.append(dialog);
  const definitions = [
    ['goal', 'ゴール受付', '目標位置をNav2が受け取る', '/navigate_to_pose/_action/status'],
    ['path', '経路生成', '目的地までの道筋を作る', '/plan'],
    ['nav', '速度計算', '進む速さ・曲がる速さを決める', '/cmd_vel_nav'],
    ['safe', '安全監視', '衝突監視を通った指令を確認', '/cmd_vel_nav_safe'],
    ['output', '指令選択', 'Nav2・手動操作から最終指令を選ぶ', '/cmd_vel'],
    ['motion', '車輪の確認', '車輪Odomで回転を確認', '/odom']
  ];
  dialog.querySelector('.nav-diag-section-title').textContent = '現在の停止状態・電源';
  const gates = [ ['driver', '駆動側の緊急停止', '現在の停止状態（解除前チェックから除外）'], ['lock', 'muxの停止ロック', '現在のロック状態（解除前チェックから除外）'], ['tf', '自己位置のTF', 'mapからロボットまで座標がつながるか'], ['ac','AC電源出力','Anker本体が報告する出力スイッチ'], ['dc','シガー出力','0WでもONの場合があります'] ];
  const cards = {}, technical = {};
  function addCard(key, title, description, parent, step) {
    const card = document.createElement(step ? 'li' : 'section'); card.className = 'nav-diag-stage'; card.dataset.tone = 'unknown';
    const heading = document.createElement('h4'); if (step) { const ordinal = document.createElement('span'); ordinal.className = 'nav-diag-ordinal'; ordinal.textContent = step; heading.append(ordinal); } heading.append(document.createTextNode(title));
    const value = document.createElement('strong'); value.textContent = '未確認';
    const explain = document.createElement('p'); explain.textContent = description;
    const detail = document.createElement('p'); detail.className = 'nav-diag-value-detail';
    const age = document.createElement('small'); age.textContent = '受信待ち';
    card.append(heading, explain, value, detail, age); parent.append(card); cards[key] = {card, value, detail, age};
  }
  gates.forEach(([key, title, explanation]) => addCard(key, title, explanation, dialog.querySelector('.nav-diag-gates')));
  definitions.forEach(([key, title, explanation, topic], index) => {
    addCard(key, title, explanation, dialog.querySelector('.nav-diag-flow'), index + 1);
    const row = document.createElement('p'), code = document.createElement('code'); code.textContent = topic;
    const value = document.createElement('span'); row.append(code, document.createElement('br'), value); technical[key] = value; dialog.querySelector('.nav-diag-technical div').append(row);
  });
  function velocity(value) {
    const x = value.x, yaw = value.yaw;
    const linear = Number.isFinite(x) ? (Math.abs(x) < .001 ? '並進なし' : (x > 0 ? '前進 ' : '後退 ') + Math.abs(x).toFixed(2) + ' m/s') : '並進は未取得';
    const turning = Number.isFinite(yaw) ? (Math.abs(yaw) < .001 ? '旋回なし' : (yaw > 0 ? '左旋回 ' : '右旋回 ') + (Math.abs(yaw) * 180 / Math.PI).toFixed(1) + ' °/s') : '旋回は未取得';
    return linear + ' ／ ' + turning;
  }
  function setCard(key, tone, value, detail, entry) {
    const ui = cards[key]; ui.card.dataset.tone = tone; ui.value.textContent = value; ui.detail.textContent = detail || '';
    ui.age.textContent = entry && Number.isFinite(entry.age) ? '最後の受信：' + entry.age.toFixed(1) + '秒前' + (entry.fresh ? '' : '（現在値は未確認）') : '受信待ち';
  }
  function render(data, failed) {
    const overall = navigationDiagnosticState(data), summary = document.getElementById('nav-diag-summary');
    const readiness = navigationPreflightState(data);
    button.dataset.tone = readiness.tone; button.querySelector('b').textContent = readiness.badge;
    button.setAttribute('aria-label', '解除前チェック：' + readiness.title); button.title = readiness.title+' ／ 現在：'+overall.title;
    preflightPanel.dataset.tone = readiness.tone; preflightPanel.querySelector('b').textContent = readiness.badge;
    document.getElementById('nav-diag-preflight-title').textContent = readiness.title;
    document.getElementById('nav-diag-preflight-detail').textContent = failed || readiness.detail;
    const list = breakdown.querySelector('ul'); list.replaceChildren();
    for (const check of readiness.checks) {
      const row = document.createElement('li'); row.dataset.tone = check.tone;
      const label = document.createElement('span'); label.textContent = check.label;
      const value = document.createElement('strong'); value.textContent = check.text;
      row.append(label,value); list.append(row);
    }
    summary.dataset.tone = overall.tone; document.getElementById('nav-diag-overall-badge').textContent = overall.badge;
    document.getElementById('nav-diag-overall-title').textContent = overall.title;
    document.getElementById('nav-diag-overall-detail').textContent = failed || overall.detail;
    const stages = data && data.available && data.navigation ? (data.navigation.stages || {}) : {};
    const current = key => stages[key] && stages[key].fresh ? stages[key].value : null;
    for (const key of ['driver', 'lock']) {
      const value = current(key); setCard(key, value === true ? 'blocked' : value === false ? 'ok' : 'unknown', value === true ? '停止中' : value === false ? '解除を確認' : '現在値は未確認', key === 'driver' ? '駆動側の停止状態' : '指令選択器のロック状態', stages[key]);
    }
    const tfUnknown = !data || !data.available || !data.readiness;
    const tfError = !tfUnknown && data.readiness.state === 'error';
    const mapPath = !tfUnknown && data.readiness.ready && navigationDiagnosticMapReady(data);
    setCard('tf', tfUnknown ? 'unknown' : tfError ? 'blocked' : mapPath ? 'ok' : 'attention', tfUnknown ? '未確認' : tfError ? '構成エラー' : mapPath ? 'TF受信を確認' : 'TF準備待ち', !tfUnknown ? data.readiness.summary : '', null);
    cards.tf.age.textContent = '詳しくはTFTreeで確認';
    const power = navigationPowerStates(data);
    for (const key of ['ac','dc']) {
      const port = power[key];
      setCard(key,port.value === 'on' ? 'ok' : port.value === 'off' ? 'blocked' : 'unknown',
        port.value ? port.value.toUpperCase()+(port.power === null ? '' : ' · '+port.power.toFixed(0)+' W') : '出力状態は未確認',
        port.power === null ? '電力未取得' : '現在の出力：'+port.power.toFixed(0)+' W',
        {age:power.age,fresh:power.live});
      cards[key].card.title = '取得元：'+(power.mac || '未取得');
    }
    for (const [key] of definitions) {
      const entry = stages[key] || {}, value = entry.value;
      if (technical[key]) technical[key].textContent = value == null ? '未取得' : JSON.stringify(value);
      if (!entry.fresh || value == null) { setCard(key, 'unknown', value == null ? '未取得' : '現在値は未確認', value != null && ['goal', 'path'].includes(key) ? '最後の通知：' + (key === 'path' ? value + '点の経路' : ({1:'受付済み',2:'実行中',3:'キャンセル処理中',4:'到着',5:'キャンセル',6:'失敗'})[value] || '不明') : '', entry); continue; }
      if (key === 'goal') {
        const label = ({0:'状態不明',1:'受付済み',2:'実行中',3:'キャンセル処理中',4:'到着',5:'キャンセル済み',6:'失敗'})[value] || '不明';
        setCard(key, value === 6 ? 'attention' : [1,2,4].includes(value) ? 'ok' : 'idle', label, 'Nav2が通知したゴールの状態', entry);
      } else if (key === 'path') setCard(key, value > 0 ? 'ok' : 'idle', value > 0 ? '経路あり' : '経路待ち', '経路を構成する通過点：' + value + '点', entry);
      else if (key === 'motion') setCard(key, value ? 'ok' : 'idle', value ? '車輪回転あり' : '車輪回転なし', '車輪Odomによる判定', entry);
      else {
        const incoming = key === 'safe' ? current('nav') : key === 'output' ? current('safe') : null;
        const blocked = incoming && incoming.moving && !value.moving;
        const stopped = key === 'output' && (current('lock') === true || current('driver') === true);
        setCard(key, stopped ? 'blocked' : blocked ? 'attention' : value.moving ? 'ok' : 'idle', stopped ? '停止要因あり' : blocked ? 'ここで停止指令に変化' : value.moving ? '移動指令あり' : '停止指令', velocity(value), entry);
      }
    }
    const event = stages.event || {}; document.getElementById('nav-diag-event').textContent = event.value ? '[' + event.value.node + '] ' + event.value.text + '（' + event.age.toFixed(1) + '秒前）' : '通知は未取得です';
  }
  let busy = false, lastPoll = 0;
  async function refresh(force = false) {
    if (document.hidden || busy || (!force && !dialog.open && Date.now() - lastPoll < 2000)) return;
    busy = true; lastPoll = Date.now();
    try {
      const response = await fetch('/api/tf', {cache:'no-store', signal:AbortSignal.timeout(4500)});
      if (!response.ok) throw new Error('HTTP ' + response.status);
      render(await response.json());
    } catch (error) { render(null, '診断データを受信できません：' + error.message); }
    finally { busy = false; }
  }
  button.addEventListener('click', () => { dialog.showModal(); refresh(true); });
  dialog.querySelector('.nav-diag-heading button').addEventListener('click', () => dialog.close());
  document.addEventListener('visibilitychange', () => { if (!document.hidden) refresh(true); });
  render(null); refresh(true); setInterval(refresh, 1000);
})();
