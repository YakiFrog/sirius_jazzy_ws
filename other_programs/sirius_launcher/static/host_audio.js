'use strict';
(() => {
  const header = document.querySelector('.top-status');
  if (!header) return;

  const speaker = muted => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <path d="M3 9h4l5-4v14l-5-4H3z"/>
    <path d="${muted ? 'm16 10 5 5m0-5-5 5' : 'M16 9a4 4 0 0 1 0 6m2-8a7 7 0 0 1 0 10'}"/>
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
      <input id="host-volume-slider" type="range" min="0" max="100" step="1" value="0" aria-label="音量" disabled>
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
    const muted = state?.muted === true;
    for (const button of [desktop, mobile]) {
      button.dataset.muted = String(muted);
      button.innerHTML = speaker(muted) + (button === mobile ? '<span>PCの音量</span>' : '');
      button.title = muted ? 'PCの音量（ミュート中）' : 'PCの音量';
    }
    el('device').textContent = state?.output || (state ? '既定の出力' : '—');
    el('status').textContent = state ? (muted ? 'ミュート中' : '出力中') : '未取得';
    el('status').dataset.muted = String(muted);
    if (!slider.matches(':active')) {
      slider.value = state?.volume ?? 0;
      el('value').textContent = state ? state.volume + '%' : '—';
    }
    slider.disabled = mute.disabled = !state || saving;
    mute.innerHTML = speaker(!muted) + (muted ? 'ミュート解除' : 'ミュート');
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
    panel.style.top = rect.bottom + 8 + 'px';
    panel.style.right = Math.max(10, document.documentElement.clientWidth - rect.right) + 'px';
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
    position();
    panel.hidden = false;
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
  setInterval(() => { if (!panel.hidden && !document.hidden) refresh(); }, 5000);
  paint();
  refresh();
})();
