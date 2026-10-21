"""Numbered selection of recorded bags and generated offline maps."""
from datetime import datetime
from pathlib import Path
import fnmatch
import os


def linked_files(root, pattern):
    """Include archived directories reached by links, without following cycles twice."""
    seen=set()
    for folder, directories, filenames in os.walk(root, followlinks=True):
        try:
            stat=os.stat(folder)
        except OSError:
            directories[:]=[];continue
        key=(stat.st_dev,stat.st_ino)
        if key in seen:
            directories[:]=[];continue
        seen.add(key);directories.sort()
        for name in sorted(fnmatch.filter(filenames,pattern)):
            yield Path(folder)/name


def bag_candidates(directory):
    root = Path(directory).expanduser().resolve()
    metadata = [p for p in linked_files(root, 'metadata.yaml') if not (p.parent/'.mapping-diagnostic').exists()]
    return [p.parent for p in sorted(metadata, key=lambda p: p.stat().st_mtime, reverse=True)]


def search_roots(directory):
    values = [directory] if isinstance(directory, (str, Path)) else directory
    return [Path(value).expanduser().resolve() for value in values]


def map_candidates(directory):
    files = {}
    for root in search_roots(directory):
        for path in linked_files(root, '*.ply'):
            if (path.parent/'.mapping-diagnostic').exists():
                continue
            if path.name in {'map_3d_unfiltered.ply', 'map_3d_colored_unfiltered.ply', 'map_3d_colored_recorded.ply', 'dynamic_candidates.ply', 'dynamic_fragments.ply'}:
                continue  # Diagnostics are explicitly available by path, not duplicate final maps.
            if path.name == 'map_3d.ply' and (path.parent / 'map_3d_colored.ply').is_file():
                continue
            files.setdefault(path.resolve(), path)
    return sorted(files.values(), key=lambda p: p.stat().st_mtime, reverse=True)


def choose_numbered(candidates, title, directory):
    roots = search_roots(directory)
    while True:
        entries = candidates() if callable(candidates) else candidates
        print(f'\n{title}（新しい順）\n検索先: '+', '.join(map(str, roots)), flush=True)
        if not entries:
            print('候補がありません。録画・地図生成の完了後に再検索できます。', flush=True)
            if input('Enter=再検索、q=中止: ').strip().lower() == 'q':
                return None
            continue
        for index, path in enumerate(entries, 1):
            label = path
            for root in roots:
                if path.is_relative_to(root):
                    label = path.relative_to(root) if len(roots) == 1 else Path(root.name) / path.relative_to(root)
                    break
            metadata = path / 'metadata.yaml' if path.is_dir() else path
            modified = datetime.fromtimestamp(metadata.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
            status = ' [地図生成済み]' if path.is_dir() and (path / 'offline_3d').exists() else ''
            print(f'  [{index}] {label}  ({modified}){status}', flush=True)
        while True:
            value = input('番号を入力（Enter=1、r=再検索、q=中止）: ').strip() or '1'
            if value.lower() == 'q':
                return None
            if value.lower() == 'r':
                break
            if value.isdecimal():
                index = int(value) - 1
                if 0 <= index < len(entries):
                    return entries[index]
            print(f'1〜{len(entries)}の番号を入力してください。', flush=True)


def next_output_directory(bag):
    bag = Path(bag)
    index = 1
    while True:
        path = bag / ('offline_3d' if index == 1 else f'offline_3d_{index}')
        if not path.exists() or (path.is_dir() and not any(path.iterdir())):
            return path
        index += 1
