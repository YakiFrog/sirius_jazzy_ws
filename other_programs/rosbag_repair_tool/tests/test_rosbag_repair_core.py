"""Tests for read-only Rosbag inspection helpers."""

from pathlib import Path
import tempfile
import unittest
from unittest import mock

from rosbag_repair_core import (
    MCAP_MAGIC,
    format_bytes,
    inspect_bag,
    next_recovered_path,
)


class RosbagRepairCoreTest(unittest.TestCase):
    """Verify damaged-bag detection and safe output naming."""

    def test_detects_missing_metadata_and_footer(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag = Path(temp_dir) / 'bag'
            bag.mkdir()
            (bag / 'bag_0.mcap').write_bytes(MCAP_MAGIC + b'incomplete')

            result = inspect_bag(str(bag))

        self.assertFalse(result.metadata_exists)
        self.assertFalse(result.all_files_finalized)
        self.assertEqual(len(result.mcap_files), 1)

    def test_detects_finalized_mcap(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag = Path(temp_dir) / 'bag'
            bag.mkdir()
            (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}')
            (bag / 'bag_0.mcap').write_bytes(MCAP_MAGIC + b'data' + MCAP_MAGIC)

            result = inspect_bag(str(bag))

        self.assertTrue(result.metadata_exists)
        self.assertTrue(result.all_files_finalized)

    def test_rejects_directory_without_mcap(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            result = inspect_bag(temp_dir)

        self.assertIsNotNone(result.error)
        self.assertFalse(result.can_repair)

    def test_uses_collision_free_recovered_name(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / 'bag'
            source.mkdir()
            (Path(temp_dir) / 'bag_recovered').mkdir()

            output = next_recovered_path(source)

        self.assertEqual(output.name, 'bag_recovered_2')

    def test_requires_source_size_plus_safety_margin(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag = Path(temp_dir) / 'bag'
            bag.mkdir()
            (bag / 'bag_0.mcap').write_bytes(MCAP_MAGIC + b'data')
            disk_usage = shutil_usage(total=100, used=95, free=5)
            with mock.patch(
                'rosbag_repair_core.shutil.disk_usage', return_value=disk_usage
            ):
                result = inspect_bag(str(bag))

        self.assertFalse(result.has_enough_space)
        self.assertFalse(result.can_repair)

    def test_formats_binary_units(self):
        self.assertEqual(format_bytes(1024), '1.00 KiB')


class shutil_usage(tuple):
    """Minimal tuple compatible with shutil.disk_usage_result."""

    __slots__ = ()

    def __new__(cls, total, used, free):
        return tuple.__new__(cls, (total, used, free))

    total = property(lambda self: self[0])
    used = property(lambda self: self[1])
    free = property(lambda self: self[2])


if __name__ == '__main__':
    unittest.main()
