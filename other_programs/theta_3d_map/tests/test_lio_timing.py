"""Recorded rates must not be confused with configured polling intervals."""
import importlib.util
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location('lio_timing', Path(__file__).parents[1]/'check_lio_timing.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_distinguishes_200hz_from_20hz():
    for period, rate in [(5_000_000, 200), (50_000_000, 20)]:
        stamps = np.arange(100, dtype=np.int64)*period+1_700_000_000_000_000_000
        result = module.summarize(stamps, stamps+100_000)
        assert result['rate_hz'] == rate
        assert result['backward_stamps'] == 0
        assert result['long_gap_missing_candidates'] == 0


def test_reports_drops_duplicates_and_backward_stamps():
    result = module.summarize([0, 5_000_000, 10_000_000, 25_000_000, 25_000_000, 24_000_000],
                              [0, 5_000_000, 10_000_000, 20_000_000, 20_000_000, 21_000_000])
    assert result['duplicate_stamps'] == 1
    assert result['backward_stamps'] == 1
    assert result['long_gap_missing_candidates'] == 2


def test_burst_jitter_does_not_invent_a_high_sensor_rate_or_dropped_samples():
    stamps = np.cumsum([0]+[3_000_000, 7_000_000]*100)
    result = module.summarize(stamps, stamps)
    assert result['rate_hz'] == 200
    assert result['long_gap_missing_candidates'] == 0
