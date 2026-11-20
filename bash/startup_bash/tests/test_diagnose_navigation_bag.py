import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'bag_diagnosis', Path(__file__).resolve().parents[1]/'diagnose_navigation_bag.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_missing_telemetry_does_not_claim_stop_mismatch():
    assert module.classify(True, False, True, None) == 'mux_locked'
    assert module.classify(True, False, None, False) == 'output_missing_unknown'
    assert module.classify(True, False, True, False) == 'mux_lock_mismatch'


def test_command_output_is_not_evidence_of_motion_or_stop_release():
    assert module.classify(True, True, False, True) == 'driver_stopped'
    assert module.classify(True, True, False, False) == 'command_output_present'
    assert module.classify(False, False, True, False) == 'no_motion_request'


def test_report_escapes_bag_name_and_states_read_only_limits():
    text = module.render({'bag': '<script>', 'message_counts': {}, 'segments': []})
    assert '<script>' not in text and '&lt;script&gt;' in text
    assert '走行・停止・解除・プロセス再起動は行いません' in text
