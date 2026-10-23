"""Allocate short road-map names and preserve their full source BAG identity."""
import argparse
import json
import re
from datetime import datetime, timedelta, timezone
from pathlib import Path


def allocate_output(root, bag, sensor, method):
    root, bag = Path(root).expanduser().resolve(), Path(bag).expanduser().resolve(strict=True)
    if sensor not in {'theta', 'zed'}:
        raise ValueError('Unknown sensor')
    if not bag.is_dir():
        raise ValueError('BAG must be a directory')
    now = datetime.now(timezone(timedelta(hours=9)))
    match = re.search(r'(?<!\d)(20\d{6}_\d{6})(?!\d)', bag.name)
    stamp = match[1] if match else now.strftime('%Y%m%d_%H%M%S')
    stem = f'{sensor}_{stamp}'
    root.mkdir(parents=True, exist_ok=True)
    number = 1
    while True:
        output = root/(stem if number == 1 else f'{stem}_{number:02d}')
        try:
            output.mkdir()
            break
        except FileExistsError:
            number += 1
    info = dict(source_bag=str(bag), source_bag_name=bag.name, sensor=sensor,
                method=method, created_at=now.isoformat(),
                name_time_source='bag_name' if match else 'generation_time', map_prefix='map')
    (output/'source_bag.json').write_text(json.dumps(info, ensure_ascii=False, indent=2)+'\n')
    return output


def check_reserved_output(output, bag):
    output, bag = Path(output).resolve(), Path(bag).resolve()
    if {p.name for p in output.iterdir()} != {'source_bag.json'}:
        raise ValueError('Reserved output already contains mapping data')
    info = json.loads((output/'source_bag.json').read_text())
    if info['source_bag'] != str(bag) or info['sensor'] != 'theta' or info['method'] != 'sequential':
        raise ValueError('Reserved output does not match this THETA run')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('sensor', choices=['theta', 'zed'])
    parser.add_argument('method', choices=['sequential', 'rtabmap', 'fallback'])
    args = parser.parse_args()
    print(allocate_output(args.root, args.bag, args.sensor, args.method))
