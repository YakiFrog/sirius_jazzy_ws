"""Write the loader's global/local semantic OccupancyGrid values as raw Nav2 maps."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from sirius_navigation.sam3_colored_map_loader import (
    DEFAULT_SOFT_INFLATION_COST,
    DEFAULT_SOFT_INFLATION_RADIUS,
    semantic_cost_grids,
)


def export_costmaps(map_dir):
    map_dir = Path(map_dir)
    meta = json.loads((map_dir/'map.colored.json').read_text())
    encoded = cv2.imread(str(map_dir/'map.colored.pgm'), cv2.IMREAD_UNCHANGED)
    if encoded is None or encoded.shape != (meta['height'], meta['width']):
        raise ValueError('Invalid road map dimensions')
    res = float(meta['resolution'])
    cost_dir = map_dir/'costmaps'
    cost_dir.mkdir(exist_ok=True)
    global_cost, local_cost, _, _ = semantic_cost_grids(encoded[::-1], meta['labels'], res)
    report = {
        'value_range': [0, 100], 'resolution': res, 'origin': meta['origin'][:2],
        'soft_inflation_radius_m': DEFAULT_SOFT_INFLATION_RADIUS,
        'soft_inflation_cost': DEFAULT_SOFT_INFLATION_COST,
    }
    for name, grid in [('global', global_cost), ('local', local_cost)]:
        image = (cost_dir/f'{name}.png')
        temporary = cost_dir/f'{name}.tmp.png'
        if not cv2.imwrite(str(temporary), grid[::-1].astype(np.uint8)):
            raise RuntimeError(f'Cannot write {image}')
        if not np.array_equal(cv2.imread(str(temporary), cv2.IMREAD_UNCHANGED), grid[::-1]):
            raise ValueError('Saved cost grid changed values')
        temporary.replace(image)
        yaml_path = image.with_suffix('.yaml')
        yaml_path.write_text(yaml.safe_dump(dict(image=image.name, mode='raw', resolution=res,
                                                   origin=[*meta['origin'][:2], 0.0], negate=0,
                                                   occupied_thresh=.65, free_thresh=.25)))
        values, counts = np.unique(grid, return_counts=True)
        report[name] = {int(v): int(n) for v, n in zip(values, counts)}
    (cost_dir/'costmaps.json').write_text(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_dir', type=Path)
    print(json.dumps(export_costmaps(parser.parse_args().map_dir), indent=2))
