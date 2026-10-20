"""The manual-push preset starts localization without EKF or motor commands."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'localization_alias_parser', ROOT/'other_programs/sirius_launcher/alias_parser.py')
parser = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parser)


def test_localization_preset_uses_known_commands_in_dependency_order():
    groups, presets = parser.parse_bash_aliases(ROOT/'bash/bash_alias2.sh')
    name, items = next(p for p in presets if p[0].startswith('3D地図で自己位置推定'))
    assert items == ['roboteq', 'velodyne', 'imu', 'tf_real', 'lio3d_loc', 'loc3d', 'foxglove']
    aliases = {item[0] for values in groups.values() for item in values}
    assert set(items) <= aliases
    assert not {'sf_real', 'amcl', 'nav2_3d', 'record_loc3d'} & set(items)
