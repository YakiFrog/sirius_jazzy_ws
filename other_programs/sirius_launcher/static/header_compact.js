'use strict';
(() => {
  const bar = document.querySelector('.appbar'), host = bar && bar.querySelector('.top-status');
  if (!host || document.getElementById('header-more-button')) return;
  const css = document.createElement('link'); css.rel = 'stylesheet'; css.href = '/static/header_compact.css'; document.head.append(css);
  bar.classList.add('compact-header');
  const more = document.createElement('div'); more.className = 'header-more';
  const toggle = document.createElement('button'); toggle.id = 'header-more-button'; toggle.type = 'button'; toggle.textContent = '⋯'; toggle.title = 'その他の表示・操作'; toggle.setAttribute('aria-label', 'その他の表示・操作'); toggle.setAttribute('aria-expanded', 'false'); toggle.setAttribute('aria-controls', 'header-more-panel');
  const panel = document.createElement('div'); panel.id = 'header-more-panel'; panel.hidden = true; panel.setAttribute('aria-label', 'その他の表示・操作');
  const heading = document.createElement('h3'); heading.textContent = 'その他'; panel.append(heading); more.append(toggle, panel); host.append(more);
  const small = matchMedia('(max-width:620px)');
  function metadata(selector, label) {
    const node = bar.querySelector(selector); if (!node || node.parentElement.classList.contains('header-meta-row')) return;
    const row = document.createElement('div'); row.className = 'header-meta-row'; const name = document.createElement('small'); name.textContent = label; row.append(name, node); panel.append(row);
  }
  function secondary(node) {
    if (!node || node.parentElement === panel) return;
    if (node.matches('button,a') && !node.querySelector('.header-extra-label')) {
      const label = document.createElement('span'); label.className = 'header-extra-label'; label.textContent = node.getAttribute('aria-label') || node.title || '操作'; node.append(label);
    }
    panel.append(node);
  }
  function arrange() {
    metadata('#connection', 'サーバー接続'); metadata('#host-name', '接続先'); metadata('#clock', '現在時刻');
    for (const node of [...host.children]) {
      if (node === more) continue;
      if (node.matches('.domain-pill,.appbar-icon')) secondary(node);
    }
    const mode = document.getElementById('header-nav-mode');
    if (mode) {
      if (small.matches && mode.parentElement !== panel) panel.prepend(mode);
      else if (!small.matches && mode.parentElement === panel) host.prepend(mode);
    }
    const ap = document.getElementById('hotspot-toggle');
    if (ap) { ap.setAttribute('aria-label', 'アクセスポイントの状態・接続方法'); ap.title = ap.textContent.trim() + '：接続方法を表示'; }
    const battery = document.getElementById('battery-toggle'); if (battery) { battery.title = 'バッテリーの状態・詳細'; battery.setAttribute('aria-label', 'バッテリーの状態・詳細'); }
  }
  let pending = false;
  new MutationObserver(() => { if (!pending) { pending = true; requestAnimationFrame(() => { pending = false; arrange(); }); } }).observe(host, {childList:true, subtree:true});
  small.addEventListener('change', arrange); arrange();
  function close() { panel.hidden = true; toggle.setAttribute('aria-expanded', 'false'); }
  toggle.addEventListener('click', () => {
    if (!panel.hidden) { close(); return; }
    const rect = toggle.getBoundingClientRect(); panel.style.top = Math.max(8, rect.bottom + 8) + 'px'; panel.style.right = Math.max(10, innerWidth - rect.right) + 'px';
    panel.hidden = false; toggle.setAttribute('aria-expanded', 'true');
  });
  panel.addEventListener('click', event => { if (event.target.closest('button,a')) close(); });
  document.addEventListener('click', event => { if (!more.contains(event.target)) close(); });
  document.addEventListener('keydown', event => { if (event.key === 'Escape' && !panel.hidden) { close(); toggle.focus(); } });
  window.addEventListener('resize', close);
})();
