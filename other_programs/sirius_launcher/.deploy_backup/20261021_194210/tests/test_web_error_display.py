from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_running_errors_get_a_prominent_frame_and_badge():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'console.css').read_text(encoding='utf-8')

    assert "+(err?' err':'')+(op?' busy':'')" in page
    assert "(err?'<span class=\"ritem-alert\">" in page
    assert "s.error?'⚠ '" not in page
    assert 'id="runerrhint"' in page
    assert "reh.hidden=!errTotal" in page
    assert '.run-error-hint{' in css
    assert '#running .ritem.err' in css
    assert 'border:1px solid var(--danger-line)!important' in css
    assert '.ritem-alert{' in css
    assert '.btn.err{border:2px solid var(--danger)!important' in css
