from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_battery_display_does_not_duplicate_emergency_stop_details():
    js = (ROOT / 'static' / 'battery.js').read_text(encoding='utf-8')

    assert 'id="battery-estop"' not in js
    assert 'id="battery-physical-estop"' not in js
    assert '物理非常停止（推定）' not in js
    assert '電子非常停止はRoboteqの停止フラグです' not in js
    assert 'record.fault_flags&~0x10' in js
    assert '残量・本体温度はバッテリーから、駆動電圧・電流・温度はRoboteqから取得します。' in js


def test_dedicated_emergency_stop_ui_keeps_its_details():
    js = (ROOT / 'static' / 'battery.js').read_text(encoding='utf-8')

    assert 'id="emergency-stop-physical"' in js
    assert 'id="emergency-stop-electronic"' in js
    assert '物理ボタンは8V以下で押下を推定し' in js


def test_battery_chip_includes_estimated_runtime_when_discharging():
    js = (ROOT / 'static' / 'battery.js').read_text(encoding='utf-8')
    style = (ROOT / 'static' / 'battery.css').read_text(encoding='utf-8')
    assert 'id="battery-chip-runtime"' in js
    assert "details.time_remaining_kind==='discharging'" in js
    assert "timeChip.textContent=discharging?'残り '+duration(runtime,true):''" in js
    assert "推定稼働時間 '+duration(runtime)" in js
    assert '#battery-chip-runtime' in style
