from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_preset_editor_can_snapshot_running_programs():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')

    assert 'onclick="addPresetFromRunning()"' in page
    assert 'async function addPresetFromRunning()' in page
    assert "api('/api/status')" in page
    assert '稼働中のプログラムがありません' in page
    assert "稼働中の '+items.length+' 件からプリセットを作成しました" in page


def test_running_preset_items_are_deduplicated_and_ordered_by_start_time():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')

    assert 's&&s.running&&order.has(name)' in page
    assert '(Number(b[1].uptime)||0)-(Number(a[1].uptime)||0)' in page
    assert '.map(([name])=>name);' in page


def test_preset_rows_can_be_reordered_with_a_drag_handle():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')
    css = (ROOT / 'static' / 'console.css').read_text(encoding='utf-8')

    editor = (ROOT / 'static' / 'preset_editor.js').read_text(encoding='utf-8')
    assert "drag.className='pdrag'" in editor
    assert '左端の6点ハンドルをドラッグして並べ替えできます' in page
    assert "PRESET_ROWS.addEventListener('pointerdown'" in page
    assert "PRESET_ROWS.addEventListener('pointermove'" in page
    assert 'PRESET_ROWS.insertBefore(PRESET_DRAG.row,before)' in page
    assert '.pdrag svg{width:10px;height:18px;pointer-events:none}' in css
    assert editor.count('<circle cx="2.5"') == 3 and editor.count('<circle cx="7.5"') == 3
    assert 'touch-action:none' in css
