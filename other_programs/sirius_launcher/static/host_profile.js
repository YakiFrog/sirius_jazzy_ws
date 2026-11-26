'use strict';
(() => {
  const header = document.querySelector('.appbar .top-status');
  const overflow = document.getElementById('header-more-panel');
  if (!header || !overflow) return;

  const modes = {
    performance: ['高性能', 'm13 2-9 12h7l-1 8 10-12h-7z'],
    balanced: ['バランス', 'M4 17a8 8 0 1 1 16 0M12 12l4-4m-7 10h6'],
    'power-saver': ['省電力', 'M19 4c-9 0-14 3-14 9a5 5 0 0 0 5 5c6 0 9-5 9-14ZM4 20l9-9'],
  };
  const errors = {
    auth: 'Launcherへのログインが必要です。',
    profile_access: 'PC本体のLauncherか、許可されたTailscale接続から切り替えできます。',
    profile_unavailable: 'PCの電源モードを取得できません。',
    invalid_profile: 'このPCでは選択したモードを利用できません。',
    profile_cpu_locked: 'CPUが高性能に固定されているため、OSが切り替えを拒否しました。',
    profile_change_failed: 'OSが電源モードの変更を受け付けませんでした。',
    profile_not_applied: '切り替え後のモードがOSに反映されませんでした。',
  };
  const label = profile => modes[profile]?.[0] || '';
  const icon = profile => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="${(modes[profile] || modes.balanced)[1]}"/></svg>`;

  const desktop = document.createElement('button');
  desktop.id = 'host-profile-toggle';
  desktop.className = 'host-profile-trigger';
  const mobile = document.createElement('button');
  mobile.className = 'host-profile-mobile appbar-icon';
  mobile.id = 'host-profile-mobile';
  for (const button of [desktop, mobile]) {
    button.type = 'button';
    button.setAttribute('aria-haspopup', 'dialog');
    button.setAttribute('aria-controls', 'host-profile-dialog');
    button.setAttribute('aria-expanded', 'false');
  }
  header.insertBefore(desktop, header.querySelector('.header-more'));
  overflow.append(mobile);

  const dialog = document.createElement('dialog');
  dialog.id = 'host-profile-dialog';
  dialog.setAttribute('aria-labelledby', 'host-profile-title');
  dialog.innerHTML = `
    <div class="host-profile-head">
      <h2 id="host-profile-title">PCの電源モード</h2>
      <button type="button" class="hbtn host-profile-close" aria-label="閉じる">×</button>
    </div>
    <div class="host-profile-options" role="group" aria-label="電源モード">
      ${Object.entries(modes).map(([profile, [name]]) =>
        `<button type="button" class="host-profile-choice" data-profile="${profile}" aria-pressed="false" disabled>
          ${icon(profile)}<span>${name}</span><span class="host-profile-selected">選択中</span>
        </button>`).join('')}
    </div>
    <p class="host-profile-notice" role="status" aria-live="polite"></p>`;
  document.body.append(dialog);

  const options = [...dialog.querySelectorAll('[data-profile]')];
  const closeButton = dialog.querySelector('.host-profile-close');
  const notice = dialog.querySelector('.host-profile-notice');
  const narrow = matchMedia('(max-width:900px)');
  let state = null, pending = false, writing = false, revision = 0;

  function message(text, bad = false) {
    notice.textContent = text;
    notice.classList.toggle('error', bad);
  }

  function paint() {
    const active = state?.active;
    const current = label(active);
    for (const button of [desktop, mobile]) {
      button.dataset.profile = active || 'unknown';
      button.title = current ? 'PCの電源モード（' + current + '）' : 'PCの電源モード';
      button.setAttribute('aria-label', button.title);
      button.innerHTML = icon(active) + (button === mobile
        ? '<span>PCの電源モード</span><small>' + current + '</small>' : '');
    }
    for (const button of options) {
      const available = state?.available.includes(button.dataset.profile);
      button.hidden = Boolean(state && !available);
      button.disabled = pending || !available || !state?.can_change;
      button.setAttribute('aria-pressed', String(button.dataset.profile === active));
    }
    closeButton.disabled = writing;
  }

  async function requestProfile(body) {
    const opts = {cache: 'no-store', signal: AbortSignal.timeout(body ? 16000 : 7500)};
    if (body) Object.assign(opts, {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
    });
    const response = await fetch('/api/host-profile', opts);
    const data = await response.json();
    if (!response.ok) {
      if (data.state && state) state = {...state, ...data.state};
      throw Error(errors[data.error] || errors.profile_unavailable);
    }
    return data;
  }

  async function refresh() {
    if (writing) return;
    const id = ++revision;
    pending = true;
    paint();
    if (dialog.open) message('確認中…');
    try {
      const next = await requestProfile();
      if (id !== revision) return;
      state = next;
      message(state.can_change ? '' : errors.profile_access);
    } catch (error) {
      if (id !== revision) return;
      state = null;
      message(error.message, true);
    } finally {
      if (id === revision) {
        pending = false;
        paint();
      }
    }
  }

  async function switchTo(profile) {
    if (pending || !state?.can_change || state.active === profile) return;
    writing = pending = true;
    paint();
    message('切り替え中…');
    try {
      state = await requestProfile({profile});
      message('');
    } catch (error) {
      message(error.message, true);
    } finally {
      writing = pending = false;
      paint();
    }
  }

  for (const button of [desktop, mobile]) {
    button.addEventListener('click', () => {
      if (dialog.open) return;
      button.setAttribute('aria-expanded', 'true');
      dialog.showModal();
      closeButton.focus();
      void refresh();
    });
  }
  for (const button of options) button.addEventListener('click', () => switchTo(button.dataset.profile));
  closeButton.addEventListener('click', () => dialog.close());
  dialog.addEventListener('cancel', event => { if (writing) event.preventDefault(); });
  dialog.addEventListener('close', () => {
    for (const button of [desktop, mobile]) button.setAttribute('aria-expanded', 'false');
    (narrow.matches ? document.getElementById('header-more-button') : desktop)?.focus();
  });
  document.addEventListener('visibilitychange', () => { if (!document.hidden) void refresh(); });
  paint();
  void refresh();
})();
