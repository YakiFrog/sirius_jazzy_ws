'use strict';
(() => {
  const header = document.querySelector('.top-status');
  if (!header) return;

  const smallWave = 'M15 10a3 3 0 0 1 0 4';
  const mediumWaves = smallWave + 'M17.5 7.5a6.4 6.4 0 0 1 0 9';
  const levels = {
    unavailable: ['未取得', 'M16 9a2 2 0 1 1 3 1.7c-1 .5-1 1.3-1 2M18 16h.01'],
    muted: ['ミュート中', 'm16 10 5 5m0-5-5 5'],
    silent: ['音量 0%', 'M16 12h5'],
    low: ['音量 小', smallWave],
    medium: ['音量 中', mediumWaves],
    high: ['音量 大', mediumWaves + 'M20 5a10 10 0 0 1 0 14'],
  };
  function volumeLevel(state) {
    if (!state) return 'unavailable';
    if (state.muted) return 'muted';
    if (state.volume === 0) return 'silent';
    if (state.volume <= 33) return 'low';
    if (state.volume <= 66) return 'medium';
    return 'high';
  }
  const speaker = level => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <path d="M3 9h4l5-4v14l-5-4H3z"/>
    <path d="${levels[level][1]}"/>
  </svg>`;

  const desktop = document.createElement('button');
  desktop.id = 'host-volume-toggle';
  desktop.className = 'host-volume-button';
  const mobile = document.createElement('button');
  mobile.id = 'host-volume-menu';
  mobile.className = 'host-volume-button appbar-icon';
  for (const button of [desktop, mobile]) {
    button.type = 'button';
    button.title = 'PCの音量';
    button.setAttribute('aria-label', 'PCの音量');
    button.setAttribute('aria-haspopup', 'dialog');
    button.setAttribute('aria-controls', 'host-volume-panel');
    button.setAttribute('aria-expanded', 'false');
  }
  header.insertBefore(desktop, header.querySelector('.header-more'));
  function attachMobileControl() {
    const more = document.getElementById('header-more-panel');
    if (!more) return false;
    more.append(mobile);
    return true;
  }
  if (!attachMobileControl()) {
    const observer = new MutationObserver(() => {
      if (attachMobileControl()) observer.disconnect();
    });
    observer.observe(header, {childList: true, subtree: true});
  }

  const panel = document.createElement('section');
  panel.id = 'host-volume-panel';
  panel.hidden = true;
  panel.setAttribute('role', 'dialog');
  panel.setAttribute('aria-labelledby', 'host-volume-title');
  panel.innerHTML = `
    <header class="host-volume-heading">
      <h2 id="host-volume-title">PCの音量</h2>
      <button type="button" id="host-volume-close" class="hbtn" aria-label="音量パネルを閉じる">×</button>
    </header>
    <div class="host-volume-output"><span>出力先</span><strong id="host-volume-device">—</strong><span id="host-volume-status" role="status">確認中…</span></div>
    <div class="host-volume-setting">
      <input id="host-volume-slider" type="range" min="0" max="150" step="1" value="0" aria-label="音量" disabled>
      <output id="host-volume-value" for="host-volume-slider">—</output>
    </div>
    <button type="button" class="hbtn" id="host-volume-mute" disabled>ミュート</button>
    <p id="host-volume-error" role="status" hidden></p>`;
  document.body.append(panel);

  const el = id => document.getElementById('host-volume-' + id);
  const slider = el('slider');
  const mute = el('mute');
  let state = null;
  let saving = false;
  let refreshing = false;
  let revision = 0;
  let anchor;

  function paint() {
    const level = volumeLevel(state);
    const muted = level === 'muted';
    const label = levels[level][0];
    const title = 'PCの音量（' + label + (state && !['muted', 'silent'].includes(level) ? '・' + state.volume + '%' : '') + '）';
    for (const button of [desktop, mobile]) {
      button.dataset.volumeLevel = level;
      button.innerHTML = speaker(level) + (button === mobile ? '<span>PCの音量</span>' : '');
      button.title = title;
      button.setAttribute('aria-label', title);
    }
    el('device').textContent = state?.output || (state ? '既定の出力' : '—');
    el('status').textContent = label;
    el('status').dataset.volumeLevel = level;
    if (!slider.matches(':active')) {
      slider.value = state?.volume ?? 0;
      el('value').textContent = state ? state.volume + '%' : '—';
    }
    slider.disabled = mute.disabled = !state || saving;
    mute.innerHTML = speaker(muted ? volumeLevel({...state, muted: false}) : 'muted')
      + (muted ? 'ミュート解除' : 'ミュート');
    mute.classList.toggle('primary-button', muted);
  }

  function showError(message) {
    el('error').textContent = message;
    el('error').hidden = !message;
  }

  async function audioRequest(body) {
    const options = {cache: 'no-store', signal: AbortSignal.timeout(8000)};
    if (body) Object.assign(options, {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
    });
    const response = await fetch('/api/host-audio', options);
    if (!response.ok) throw new Error('Audio request failed');
    return response.json();
  }

  async function refresh() {
    if (saving || refreshing || slider.matches(':active')) return;
    refreshing = true;
    const read = revision;
    try {
      const result = await audioRequest();
      if (read !== revision) return;
      state = result;
      showError('');
    } catch {
      if (read !== revision) return;
      state = null;
      showError('PCの音声出力を取得できません。');
    } finally {
      refreshing = false;
      if (read === revision) paint();
    }
  }

  async function change(body) {
    if (!state || saving) return;
    revision++;
    saving = true;
    slider.disabled = mute.disabled = true;
    showError('');
    try {
      state = await audioRequest(body);
    } catch {
      showError('設定を変更できませんでした。');
    } finally {
      saving = false;
      paint();
    }
  }

  function position() {
    const rect = anchor.getBoundingClientRect();
    const {clientWidth, clientHeight} = document.documentElement;
    const bottomLimit = Math.max(8, clientHeight - panel.offsetHeight - 8);
    const rightLimit = Math.max(10, clientWidth - panel.offsetWidth - 10);
    panel.style.top = Math.min(Math.max(8, rect.bottom + 8), bottomLimit) + 'px';
    panel.style.right = Math.min(Math.max(10, clientWidth - rect.right), rightLimit) + 'px';
  }

  function close(restoreFocus = false) {
    if (panel.hidden) return;
    panel.hidden = true;
    for (const button of [desktop, mobile]) button.setAttribute('aria-expanded', 'false');
    if (restoreFocus) anchor.focus();
  }

  function toggle(event) {
    const button = event.currentTarget;
    if (!panel.hidden) return close(true);
    anchor = button === mobile ? document.getElementById('header-more-button') : desktop;
    button.setAttribute('aria-expanded', 'true');
    panel.hidden = false;
    position();
    requestAnimationFrame(() => { if (!panel.hidden) position(); });
    el('close').focus({preventScroll: true});
    refresh();
  }

  desktop.addEventListener('click', toggle);
  mobile.addEventListener('click', toggle);
  el('close').addEventListener('click', () => close(true));
  slider.addEventListener('input', () => { el('value').textContent = slider.value + '%'; });
  slider.addEventListener('change', () => change({volume: Number(slider.value)}));
  mute.addEventListener('click', () => { if (state) change({muted: !state.muted}); });
  document.addEventListener('click', event => {
    if (!panel.contains(event.target) && !event.target.closest('.host-volume-button')) close();
  });
  panel.addEventListener('keydown', event => {
    if (event.key === 'Escape') {
      event.stopPropagation();
      close(true);
    }
  });
  window.addEventListener('resize', () => close());
  document.addEventListener('visibilitychange', () => { if (!document.hidden) refresh(); });
  setInterval(() => { if (!document.hidden) refresh(); }, 5000);
  paint();
  refresh();
})();
