#!/usr/bin/env python3
"""Read-only inspection helpers for interrupted ROS 2 MCAP recordings."""

from dataclasses import dataclass
from pathlib import Path
import shutil
from typing import Optional, Tuple


MCAP_MAGIC = b'\x89MCAP0\r\n'
RECOVERY_SAFETY_BYTES = 256 * 1024 * 1024


def format_bytes(value: int) -> str:
    """Format a byte count using binary units."""
    size = float(value)
    for unit in ('B', 'KiB', 'MiB', 'GiB', 'TiB'):
        if size < 1024.0 or unit == 'TiB':
            return f'{size:.2f} {unit}'
        size /= 1024.0
    return f'{size:.2f} TiB'


def has_mcap_footer(path: Path) -> bool:
    """Return whether an MCAP file ends with the required trailing magic."""
    try:
        if path.stat().st_size < len(MCAP_MAGIC):
            return False
        with path.open('rb') as stream:
            stream.seek(-len(MCAP_MAGIC), 2)
            return stream.read(len(MCAP_MAGIC)) == MCAP_MAGIC
    except OSError:
        return False


def next_recovered_path(source: Path) -> Path:
    """Return the same collision-free output path used by the repair script."""
    candidate = source.with_name(f'{source.name}_recovered')
    suffix = 2
    while candidate.exists():
        candidate = source.with_name(f'{source.name}_recovered_{suffix}')
        suffix += 1
    return candidate


@dataclass(frozen=True)
class BagInspection:
    """Inspection result for one ROS 2 bag directory."""

    source: Path
    mcap_files: Tuple[Path, ...]
    total_bytes: int
    available_bytes: int
    metadata_exists: bool
    all_files_finalized: bool
    error: Optional[str] = None

    @property
    def required_bytes(self) -> int:
        """Return conservative free space required for non-destructive repair."""
        return self.total_bytes + RECOVERY_SAFETY_BYTES

    @property
    def has_enough_space(self) -> bool:
        """Return whether the source filesystem has enough recovery space."""
        return self.available_bytes >= self.required_bytes

    @property
    def can_repair(self) -> bool:
        """Return whether repair can safely be attempted."""
        return self.error is None and bool(self.mcap_files) and self.has_enough_space


def inspect_bag(path: str) -> BagInspection:
    """Inspect a bag directory without modifying it."""
    source = Path(path).expanduser().resolve()
    if not source.is_dir():
        return BagInspection(
            source, (), 0, 0, False, False,
            f'Rosbagディレクトリがありません: {source}',
        )

    mcap_files = tuple(sorted(source.glob('*.mcap')))
    if not mcap_files:
        return BagInspection(
            source, (), 0, shutil.disk_usage(source.parent).free,
            (source / 'metadata.yaml').is_file(), False,
            'MCAPファイルが見つかりません。現在はMCAP形式のみ対応しています。',
        )

    try:
        total_bytes = sum(item.stat().st_size for item in mcap_files)
        available_bytes = shutil.disk_usage(source.parent).free
    except OSError as exc:
        return BagInspection(
            source, mcap_files, 0, 0,
            (source / 'metadata.yaml').is_file(), False,
            f'ファイル情報を取得できません: {exc}',
        )

    return BagInspection(
        source=source,
        mcap_files=mcap_files,
        total_bytes=total_bytes,
        available_bytes=available_bytes,
        metadata_exists=(source / 'metadata.yaml').is_file(),
        all_files_finalized=all(has_mcap_footer(item) for item in mcap_files),
    )
