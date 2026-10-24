from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_launcher_has_header_and_teleop_mode_entry_points():
    html = (ROOT / 'launcher.html').read_text(encoding='utf-8')
    assert 'id="header-nav-mode"' in html
    assert 'id="header-nav-mode-label"' in html
    assert 'id="drive-nav-mode"' in html
    assert 'id="drive-nav-change"' in html
    assert 'id="nav-mode-modal"' in html
    assert 'id="nav-mode-apply"' in html
    assert 'nav_mode.css' in html
    assert 'nav_mode.js' in html
    assert '状態未取得' in html
    assert '一覧を開いただけでは変更されません' in html


def test_mode_list_selection_does_not_apply_immediately():
    script = (ROOT / 'static/nav_mode.js').read_text(encoding='utf-8')
    assert "NAV.selected=item.mode;navPaint();" in script
    assert "async function navApply()" in script
    assert "fetch('/api/navigation_mode',{method:'POST'" in script
    assert "audio.reason==='same_mode'" in script


def test_mode_cache_is_only_updated_from_payload_and_state():
    script = (ROOT / 'static/nav_mode.js').read_text(encoding='utf-8')
    assert "NAV.state=data;" in script
    assert "NAV.state={status:'unavailable'" in script
    assert "current?current.label:navStatusText(status)" in script
