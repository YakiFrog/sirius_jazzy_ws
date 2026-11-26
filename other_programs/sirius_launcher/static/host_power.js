'use strict';
(() => {
  const headerHost = document.querySelector('.appbar .top-status');
  if (!headerHost) return;

  const labels = {reboot: '再起動', poweroff: 'シャットダウン'};
  const messages = {
    power_access: '別端末では、NUCで許可された接続からLauncherを開いてください。NUC本体のLauncherからも操作できます。',
    power_target: '接続先が変わりました。メニューを開き直してください。',
    power_unavailable: 'OSが電源操作を受け付けませんでした。',
    recording_bag: 'ROSBAG 記録・保存中。終了してから実行できます。',
  };
  const group = document.createElement('section');
  group.className = 'host-power-menu';
  group.innerHTML = `
    <h3>PCの電源</h3>
    <p class="host-power-target">対象 <strong>確認中…</strong></p>
    <button type="button" class="hbtn" data-power="reboot" disabled>
      <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M4 5v5h5M4 10a8 8 0 1 1 0 5"/></svg>再起動
    </button>
    <button type="button" class="hbtn" data-power="poweroff" disabled>
      <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12 2v10M6 5a9 9 0 1 0 12 0"/></svg>シャットダウン
    </button>
    <p class="host-power-menu-status" role="status" hidden></p>`;
  const popover = document.createElement('div');
  popover.id = 'header-power-popover';
  popover.className = 'host-power-popover';
  popover.setAttribute('aria-label', 'PCの電源');
  popover.hidden = true;
  popover.append(group);
  document.body.append(popover);

  const powerTrigger = document.createElement('button');
  powerTrigger.type = 'button';
  powerTrigger.id = 'header-power-button';
  powerTrigger.className = 'host-power-trigger';
  powerTrigger.title = 'PCの電源';
  powerTrigger.setAttribute('aria-label', 'PCの電源');
  powerTrigger.setAttribute('aria-haspopup', 'dialog');
  powerTrigger.setAttribute('aria-controls', 'header-power-popover');
  powerTrigger.setAttribute('aria-expanded', 'false');
  powerTrigger.innerHTML = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M12 2v10M6 5a9 9 0 1 0 12 0"/></svg>';
  const more = document.getElementById('header-more-button');
  const moreWrap = more?.closest('.header-more');
  if (moreWrap?.parentElement) moreWrap.before(powerTrigger);
  else headerHost.append(powerTrigger);

  function closePowerMenu() {
    if (popover.hidden) return;
    popover.hidden = true;
    powerTrigger.setAttribute('aria-expanded', 'false');
  }
  function openPowerMenu() {
    const rect = powerTrigger.getBoundingClientRect();
    const width = Math.min(270, innerWidth - 20);
    const left = Math.min(Math.max(10, rect.right - width), innerWidth - 10 - width);
    popover.style.top = Math.max(8, rect.bottom + 8) + 'px';
    popover.style.left = left + 'px';
    popover.style.right = 'auto';
    popover.hidden = false;
    powerTrigger.setAttribute('aria-expanded', 'true');
    void refresh();
  }
  powerTrigger.addEventListener('click', event => {
    event.stopPropagation();
    if (popover.hidden) openPowerMenu();
    else closePowerMenu();
  });
  document.addEventListener('click', event => {
    if (!popover.hidden && !popover.contains(event.target) && event.target !== powerTrigger) {
      closePowerMenu();
    }
  });
  document.addEventListener('keydown', event => {
    if (event.key === 'Escape' && !popover.hidden) {
      closePowerMenu();
      powerTrigger.focus();
    }
  });
  addEventListener('resize', closePowerMenu);

  const dialog = document.createElement('dialog');
  dialog.className = 'host-power-dialog';
  dialog.setAttribute('aria-labelledby', 'host-power-title');
  dialog.innerHTML = `
    <h2 id="host-power-title"></h2>
    <p>Launcherとの接続が切れます。</p>
    <p class="host-power-status" role="status" hidden></p>
    <div class="host-power-actions">
      <button type="button" class="hbtn" autofocus>キャンセル</button>
      <button type="button" class="hbtn" disabled></button>
    </div>`;
  document.body.append(dialog);

  const target = group.querySelector('strong');
  const menuStatus = group.querySelector('[role=status]');
  const choices = [...group.querySelectorAll('button')];
  const status = dialog.querySelector('[role=status]');
  const [cancel, confirm] = dialog.querySelectorAll('button');
  let info;
  let selection;
  let submitting = false;
  let accepted = false;
  let refreshId = 0;

  function showStatus(element, message, kind = '') {
    element.textContent = message;
    element.hidden = !message;
    element.dataset.kind = kind;
  }

  async function call(body) {
    const options = {cache: 'no-store', signal: AbortSignal.timeout(body ? 42000 : 6500)};
    if (body) Object.assign(options, {
      method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
    });
    const response = await fetch('/api/host-power', options);
    const data = await response.json();
    if (!response.ok) {
      const error = new Error(messages[data.error] || '電源状態を取得できませんでした。');
      error.code = data.error;
      throw error;
    }
    return data;
  }

  async function refresh() {
    const id = ++refreshId;
    try {
      const current = await call();
      if (id !== refreshId || submitting || accepted) return;
      info = current;
      target.textContent = info.host;
      showStatus(menuStatus, '');
      for (const btn of choices) btn.disabled = false;
      if (!dialog.open) return;
      if (selection.host !== info.host) {
        confirm.disabled = true;
        showStatus(status, messages.power_target, 'warning');
      } else {
        const capability = info.available[selection.action];
        confirm.disabled = info.recording || !['yes', 'challenge'].includes(capability);
        if (info.recording) showStatus(status, messages.recording_bag, 'warning');
        else if (confirm.disabled) showStatus(status, messages.power_unavailable, 'warning');
        else if (capability === 'challenge') showStatus(status, 'PC本体の画面で認証が求められる場合があります。');
        else showStatus(status, '');
      }
    } catch (error) {
      if (id !== refreshId || submitting || accepted) return;
      info = undefined;
      target.textContent = error.code === 'power_access' ? '認証が必要' : '取得できません';
      for (const btn of choices) btn.disabled = true;
      confirm.disabled = true;
      const message = error instanceof TypeError ? 'Launcherに接続できません。' : error.message;
      showStatus(dialog.open ? status : menuStatus, message, 'warning');
    }
  }

  group.addEventListener('click', event => {
    const button = event.target.closest('[data-power]');
    if (!button || !info) return;
    closePowerMenu();
    selection = {action: button.dataset.power, host: info.host};
    const label = labels[selection.action];
    dialog.querySelector('h2').textContent = selection.host + ' を' + label + 'しますか？';
    confirm.textContent = label;
    confirm.dataset.action = selection.action;
    confirm.disabled = true;
    accepted = false;
    showStatus(status, '状態を確認中…');
    dialog.showModal();
    cancel.focus();
    void refresh();
  });

  cancel.addEventListener('click', () => dialog.close());
  dialog.addEventListener('close', () => powerTrigger.focus());
  dialog.addEventListener('cancel', event => { if (submitting) event.preventDefault(); });
  confirm.addEventListener('click', async () => {
    submitting = true;
    cancel.disabled = confirm.disabled = true;
    showStatus(status, 'OSに処理を要求しています…');
    try {
      await call(selection);
      accepted = true;
      showStatus(status, '要求を受け付けました。');
    } catch (error) {
      showStatus(status, error instanceof TypeError || error.name === 'TimeoutError'
        ? '接続が切れました。PCの状態を確認してください。' : error.message, 'warning');
    } finally {
      submitting = false;
      cancel.disabled = false;
      cancel.textContent = accepted ? '閉じる' : 'キャンセル';
    }
  });
  dialog.addEventListener('close', () => { cancel.textContent = 'キャンセル'; });
  setInterval(() => {
    if (dialog.open && !submitting && !accepted && status.dataset.kind === 'warning'
        && status.textContent === messages.recording_bag) void refresh();
  }, 2000);
})();
