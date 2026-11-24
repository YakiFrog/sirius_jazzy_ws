from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_ios27_pwa_status_strip_is_an_opaque_fixed_color_extension():
    shell = (ROOT / 'shell.html').read_text(encoding='utf-8')
    assert 'class="ios-pwa-status-strip"' in shell
    assert "navigator.standalone===true||matchMedia('(display-mode: standalone)').matches" in shell
    assert "dataset.standalone='true'" in shell
    assert 'height:11px' in shell and 'position:fixed' in shell
    assert '-webkit-background-clip:text' in shell and 'background-clip:text' in shell
    assert 'background-color:var(--shell-bg)' in shell
    assert 'z-index:2147483647' in shell
    assert 'apple-mobile-web-app-status-bar-style' not in shell
    assert shell.index('dataset.standalone') < shell.index('<body>')


def test_status_strip_is_hidden_outside_standalone_webkit():
    shell = (ROOT / 'shell.html').read_text(encoding='utf-8')
    assert '.ios-pwa-status-strip{display:none}' in shell
    assert '@supports (-webkit-touch-callout:none)' in shell
    assert ':root[data-standalone] .ios-pwa-status-strip' in shell
