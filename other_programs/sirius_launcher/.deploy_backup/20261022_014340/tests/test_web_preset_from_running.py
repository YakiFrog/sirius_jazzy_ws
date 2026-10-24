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
