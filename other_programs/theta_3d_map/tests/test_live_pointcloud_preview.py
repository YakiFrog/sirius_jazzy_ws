import importlib.util
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location(
    'preview', Path(__file__).resolve().parents[1] / 'live_pointcloud_preview.py')
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def cloud(big_endian=False, organized=False):
    endian = '>' if big_endian else '<'
    records = [struct.pack(endian + 'fHH', float(i), i % 16, i)
               for i in range(16 * 9)]
    height = 2 if organized else 1
    width = len(records) // height
    rows = [b''.join(records[r * width:(r + 1) * width]) + b'PAD!'
            for r in range(height)]
    return SimpleNamespace(
        header=object(), fields=[SimpleNamespace(name='ring', offset=4, datatype=4, count=1)],
        width=width, height=height, point_step=8, row_step=width * 8 + 4,
        is_bigendian=big_endian, is_dense=False, data=b''.join(rows)), records


@pytest.mark.parametrize('big_endian,organized', [(False, False), (True, True)])
def test_all_rings_selected_bytes_and_metadata_survive(big_endian, organized):
    original, records = cloud(big_endian, organized)
    before = original.data
    result = preview.decimate_cloud(original, 4)
    expected = b''.join(record for i, record in enumerate(records) if (i // 16) % 4 == 0)
    assert result.data == expected
    assert result.width == 48 and result.height == 1
    assert result.row_step == 48 * original.point_step
    assert result.header is original.header and result.fields is original.fields
    assert result.is_bigendian == big_endian and not result.is_dense
    assert original.data == before and original.width == len(records) // original.height
    endian = '>' if big_endian else '<'
    assert {struct.unpack_from(endian + 'H', result.data, i * 8 + 4)[0]
            for i in range(result.width)} == set(range(16))


def test_stride_one_is_original_and_large_stride_keeps_each_ring():
    original, records = cloud()
    assert preview.decimate_cloud(original, 1) is original
    assert preview.decimate_cloud(original, 100).data == b''.join(records[:16])


def test_missing_ring_samples_records_and_invalid_ring_is_rejected():
    original, records = cloud(organized=True)
    original.fields = []
    assert preview.decimate_cloud(original, 4).data == b''.join(records[::4])
    original.fields = [SimpleNamespace(name='ring', offset=100, datatype=4, count=1)]
    with pytest.raises(ValueError):
        preview.decimate_cloud(original, 4)
    with pytest.raises(ValueError):
        preview.decimate_cloud(original, 0)
