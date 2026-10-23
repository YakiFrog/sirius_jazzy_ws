import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from road_map_output import allocate_output, check_reserved_output


def test_source_time_sensor_and_full_provenance(tmp_path):
    bag = tmp_path/'mapping_all_real_20261022_194816_recovered'
    bag.mkdir()
    for sensor in ('theta', 'zed'):
        output = allocate_output(tmp_path/'maps', bag, sensor, 'rtabmap')
        assert output.name == sensor+'_20261022_194816'
        info = json.loads((output/'source_bag.json').read_text())
        assert info['source_bag'] == str(bag)
        assert info['source_bag_name'] == bag.name
        assert info['name_time_source'] == 'bag_name'


def test_parallel_regeneration_never_reuses_a_directory(tmp_path):
    bag = tmp_path/'real_20261022_194816'; bag.mkdir()
    root = tmp_path/'maps with spaces'
    with ThreadPoolExecutor(max_workers=6) as pool:
        paths = list(pool.map(lambda _: allocate_output(root, bag, 'theta', 'sequential'), range(12)))
    assert len(set(paths)) == 12
    assert {p.name for p in paths} == {'theta_20261022_194816'} | {
        f'theta_20261022_194816_{i:02d}' for i in range(2, 13)}
    for path in paths:
        check_reserved_output(path, bag)


def test_reserved_output_refuses_existing_data_or_wrong_bag(tmp_path):
    bag = tmp_path/'bag'; bag.mkdir()
    output = allocate_output(tmp_path/'maps', bag, 'theta', 'sequential')
    info = json.loads((output/'source_bag.json').read_text())
    assert info['name_time_source'] == 'generation_time'
    assert info['created_at'].endswith('+09:00')
    other = tmp_path/'other'; other.mkdir()
    with pytest.raises(ValueError):
        check_reserved_output(output, other)
    (output/'map.colored.pgm').write_bytes(b'existing map')
    with pytest.raises(ValueError):
        check_reserved_output(output, bag)
    assert (output/'map.colored.pgm').read_bytes() == b'existing map'
