from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_common_quick_starts_can_be_hidden_and_are_tagged_inline():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')
    style = (ROOT / 'static' / 'presets.css').read_text(encoding='utf-8')
    assert 'id="preset-hide-common"' in page
    assert 'PRESET_HIDE_COMMON' in page
    assert "!(PRESET_HIDE_COMMON&&pr.category==='共通')" in page
    assert "common.className='preset-common-badge'" in page
    assert "nameRow.append(common)" in page
    assert "b.append(copy);if(pr.category!=='共通')b.append(category)" in page
    assert '.preset-name-row' in style
    assert '.preset-common-badge' in style
    assert 'presets.css?v=20261126-info-hide' in page


def test_preset_editor_starts_with_all_rows_collapsed():
    script = (ROOT / 'static' / 'preset_editor.js').read_text(encoding='utf-8')
    assert 'rows.firstElementChild.open=true' not in script
    assert 'preset_editor.js?v=20261126-collapsed' in (ROOT / 'launcher.html').read_text(encoding='utf-8')


def test_preset_info_can_hide_the_quick_start():
    page = (ROOT / 'launcher.html').read_text(encoding='utf-8')
    style = (ROOT / 'static' / 'presets.css').read_text(encoding='utf-8')
    assert "hide.textContent='非表示にする'" in page
    assert "api('/api/presets/save',{presets})" in page
    assert "item.name===preset.name?false:item.visible!==false" in page
    assert 'preset-info-actions' in page and '.preset-info-actions' in style
    assert 'presets.css?v=20261126-info-hide' in page
