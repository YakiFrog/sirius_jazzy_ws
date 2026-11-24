"""Serve ROS map rasters and export copies of edited maps and waypoints."""
from collections import OrderedDict
from hashlib import sha256
from io import BytesIO
import heapq
import json
import math
import os
import time
from pathlib import Path
import tempfile
from threading import Lock
from urllib.parse import urlsplit

from flask import Blueprint, jsonify, request, send_file
import numpy as np
from PIL import Image, ImageDraw
import yaml

from file_browser import HostFiles, PGM_LIMIT, TEXT_LIMIT, open_preview_image

YAML_SUFFIXES = {'.yaml', '.yml'}
TEXTURE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.webp'}
MAX_WAYPOINTS = 5000
TILE_SIZE = 512
TILE_CACHE_LIMIT = 2 * PGM_LIMIT
MAX_STROKE_POINTS = 70_000
COSTMAP_DEFAULTS = {
    'inflation_radius': 0.75, 'cost_scaling': 3.0, 'inscribed_radius': 0.35,
    'occupied_thresh': 0.65, 'negate': 0,
}
COSTMAP_CACHE_LIMIT = 3
MAX_INFLATION_CELLS = 400
MAX_PATH_DIMENSION = 900
MAX_PATH_EXPANSIONS = 200_000
PATH_TIME_LIMIT = 6.0
POSE_FIELDS = {'number': 'int', 'x': 'float', 'y': 'float', 'angle_radians': 'float'}
SUPPORTED_TYPES = {'int', 'float', 'bool', 'string', 'str', 'pose'}
ACTION_TYPES = {
    'rotate': {'float'}, 'stop': {'bool'}, 'wait_time': {'float'}, 'threshold': {'float'},
    'change_map': {'str', 'string'}, 'speech': {'str', 'string'}, 'person_area': {'bool'},
    'manual': {'bool'}, 'initial_pose': {'pose'}, 'angle_degrees': {'float'},
}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_waypoints(document):
    if not isinstance(document, dict) or not isinstance(document.get('waypoints'), list):
        raise ValueError('Choose a YAML file with a waypoints array.')
    rows = document['waypoints']
    if len(rows) > MAX_WAYPOINTS:
        raise ValueError(f'A route may contain at most {MAX_WAYPOINTS} waypoints.')
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict) or type(row.get('number')) is not int:
            raise ValueError(f'Waypoint {index} requires an integer number.')
        if not all(finite(row.get(key)) for key in ('x', 'y', 'angle_radians')):
            raise ValueError(f'Waypoint {row["number"]} requires finite coordinates and heading.')
        for key in ('rotate', 'wait_time', 'threshold'):
            if key in row and not finite(row[key]):
                raise ValueError(f'Waypoint {row["number"]}: {key} must be a finite number.')
        for key in ('stop', 'person_area'):
            if key in row and type(row[key]) is not bool:
                raise ValueError(f'Waypoint {row["number"]}: {key} must be true or false.')
        for key in ('change_map', 'speech'):
            if key in row and not isinstance(row[key], str):
                raise ValueError(f'Waypoint {row["number"]}: {key} must be text.')
        initial = row.get('initial_pose')
        if initial is None:
            continue
        if row.get('change_map'):
            raise ValueError(f'Waypoint {row["number"]}: use separate waypoints for change_map and initial_pose.')
        mode = initial.get('yaw_mode', 'fixed') if isinstance(initial, dict) else None
        keys = ('x', 'y') if mode == 'current' else ('x', 'y', 'yaw')
        if mode not in ('current', 'fixed') or not all(finite(initial.get(k)) for k in keys):
            raise ValueError(f'Waypoint {row["number"]}: check the initial pose coordinates and heading.')
    if len({row['number'] for row in rows}) != len(rows):
        raise ValueError('Waypoint numbers must be unique.')


def check_json_values(value, depth=0, seen=None):
    """Reject YAML-only types that the browser cannot round trip."""
    if depth > 32:
        raise ValueError('The YAML nesting is too deep.')
    if type(value) is int and abs(value) > 2**53 - 1:
        raise ValueError('A YAML integer is outside the browser’s exact range.')
    if type(value) is float and not math.isfinite(value):
        raise ValueError('All YAML numbers must be finite.')
    if isinstance(value, (dict, list)):
        seen = set() if seen is None else seen
        if id(value) in seen:
            raise ValueError('Expand YAML aliases before opening the file.')
        seen.add(id(value))
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise ValueError('YAML mapping keys must be text.')
        for child in value.values():
            check_json_values(child, depth + 1, seen)
    elif isinstance(value, list):
        for child in value:
            check_json_values(child, depth + 1, seen)
    elif value is not None and type(value) not in (str, bool, int, float):
        raise ValueError('The YAML contains a value that the browser cannot edit.')


class MapEditor:
    def __init__(self, files):
        self.files = files
        self.root = files.workspace / 'maps_waypoints'
        self.map_directories = (self.root / 'maps', self.root / 'map_archive')
        self.tile_lock = Lock()
        self.tile_sources = OrderedDict()
        self.costmap_lock = Lock()
        self.costmap_cache = OrderedDict()

    @staticmethod
    def revision(path):
        stat = path.stat()
        return sha256(f'{stat.st_dev}:{stat.st_ino}:{stat.st_size}:{stat.st_mtime_ns}'.encode()).hexdigest()[:24]

    def scoped_file(self, value, directories):
        if not value:
            raise ValueError('Select a file.')
        path = self.files.regular_file(value)
        if not any(path.is_relative_to(directory) for directory in directories):
            raise PermissionError('Choose a supported file under Map & Waypoint.')
        return path

    def read_yaml(self, path):
        source = self.files.text(str(path))
        if source['truncated']:
            raise ValueError('YAML files may be up to 512 KiB.')
        try:
            result = yaml.safe_load(source['text'])
            check_json_values(result)
            return result
        except (yaml.YAMLError, RecursionError) as error:
            raise ValueError('Check the YAML syntax.') from error

    def load_map(self, value):
        metadata, _ = self.map_data(value)
        return metadata

    def texture_candidate(self, palette_path, data):
        """Find an optional full-resolution image beside the indexed PGM.

        The formal map remains the PGM + JSON pair. The image is only a visual
        layer for Map Editor; it must use the same cell grid and dimensions.
        """
        explicit = data.get('texture_image') or data.get('preview_image')
        candidates = []
        if isinstance(explicit, str) and explicit.strip():
            candidates.append(palette_path.parent / explicit.strip())
        stem = palette_path.name
        if stem.endswith('.colored.json'):
            base = stem[:-len('.colored.json')]
            if base.endswith('.texture'):
                candidates.append(palette_path.parent / (base + '.png'))
                candidates.append(palette_path.parent / (base + '.jpg'))
            candidates.extend([
                palette_path.parent / (base + '.texture.png'),
                palette_path.parent / (base + '.texture.jpg'),
                palette_path.parent / (base + '.color.png'),
            ])
        else:
            base = palette_path.stem
            candidates.extend([
                palette_path.parent / (base + '.texture.png'),
                palette_path.parent / (base + '.texture.jpg'),
                palette_path.parent / (base + '.color.png'),
            ])
        seen, error = set(), ''
        for candidate in candidates:
            try:
                path = self.scoped_file(str(candidate), self.map_directories)
            except (OSError, ValueError):
                continue
            if path in seen or path.suffix.lower() not in TEXTURE_SUFFIXES:
                continue
            seen.add(path)
            return path, ''
        return None, error

    @staticmethod
    def cost_value(value):
        if type(value) is bool or not finite(value):
            return 0
        return max(0, min(100, int(round(value))))

    def semantic_data(self, value):
        palette_path = self.scoped_file(value, self.map_directories)
        if palette_path.suffix.lower() != '.json':
            raise ValueError('Choose the JSON palette paired with the indexed PGM.')
        image_path = self.scoped_file(str(palette_path.with_suffix('.pgm')), self.map_directories)
        source = self.files.text(str(palette_path))
        if source['truncated']:
            raise ValueError('The road semantic JSON may be up to 512 KiB.')
        data = json.loads(source['text'])
        if not isinstance(data, dict):
            raise ValueError('Expected a road semantic JSON object.')
        palette = data.get('palette')
        if (not isinstance(palette, list) or not 1 <= len(palette) <= 256 or
                any(not isinstance(color, list) or len(color) != 3 or
                    any(type(channel) is not int or not 0 <= channel <= 255 for channel in color)
                    for color in palette)):
            raise ValueError('JSON palette must contain 1–256 RGB colors.')
        resolution = data.get('resolution')
        origin = data.get('origin')
        if not finite(resolution) or resolution <= 0:
            raise ValueError('The road semantic JSON requires a positive resolution.')
        if not isinstance(origin, list) or len(origin) < 2 or not all(map(finite, origin[:2])):
            raise ValueError('The road semantic JSON requires an [x, y] origin.')
        with open_preview_image(image_path) as image:
            if image.mode != 'L':
                raise ValueError('Expected an 8-bit indexed PGM.')
            width, height = image.size
        if data.get('width', width) != width or data.get('height', height) != height:
            raise ValueError('The road semantic JSON dimensions differ from the PGM.')
        revision = self.revision(image_path) + self.revision(palette_path)
        texture_path, texture_error = self.texture_candidate(palette_path, data)
        texture = None
        if texture_path is not None:
            try:
                with open_preview_image(texture_path) as texture_image:
                    if texture_image.size != (width, height):
                        raise ValueError('The texture image dimensions differ from the indexed PGM.')
                texture = dict(path=str(texture_path), revision=self.revision(texture_path),
                               width=width, height=height)
                revision += texture['revision']
            except (OSError, ValueError) as error:
                texture_error = str(error)
        labels = data.get('labels')
        legend, label_info = [], []
        if isinstance(labels, dict):
            for index, color in enumerate(palette):
                label = labels.get(str(index))
                name = label.get('name') if isinstance(label, dict) else None
                global_cost = self.cost_value(label.get('global_cost', 0) if isinstance(label, dict) else 0)
                local_cost = self.cost_value(label.get('local_cost', 0) if isinstance(label, dict) else 0)
                if isinstance(label, dict):
                    label_info.append({
                        'id': index, 'name': str(name or index), 'rgb': color,
                        'global_cost': global_cost, 'local_cost': local_cost,
                    })
                if index and isinstance(name, str) and name.strip() and name.lower() not in ('unknown', 'wall', 'floor'):
                    legend.append({'name': name, 'rgb': color})
        # SAM3MapLoader's indexed view uses the JSON x/y origin and masks out index 0.
        metadata = dict(path=str(image_path), image=str(image_path), width=width, height=height,
                        resolution=resolution, origin=[*origin[:2], 0], editable=False,
                        tile_size=TILE_SIZE, image_revision=revision, revision=revision,
                        semantic={'palette_path': str(palette_path), 'legend': legend,
                                  'labels': label_info})
        if texture:
            metadata['texture'] = texture
        elif texture_error:
            metadata['texture_error'] = texture_error
        return metadata, palette

    def map_data(self, value):
        directories = self.map_directories
        path = self.scoped_file(value, directories)
        if path.suffix.lower() == '.json':
            return self.semantic_data(str(path))[0], {}
        selected_image = path if path.suffix.lower() == '.pgm' else None
        if selected_image:
            sidecar = path.with_suffix('.json')
            config = path.with_suffix('.yaml')
            if path.name.lower().endswith('.colored.pgm') or (not config.is_file() and sidecar.is_file()):
                return self.semantic_data(str(sidecar))[0], {}
            path = self.scoped_file(str(config), directories)
        if path.suffix.lower() not in YAML_SUFFIXES:
            raise ValueError('Choose a map YAML file.')
        meta = self.read_yaml(path)
        if not isinstance(meta, dict) or not isinstance(meta.get('image'), str) or not meta['image']:
            raise ValueError('Choose a ROS map YAML with an image field.')
        image = self.scoped_file(str(path.parent / meta['image']), directories)
        if image.suffix.lower() != '.pgm':
            raise ValueError('Choose a PGM map.')
        if selected_image and image != selected_image:
            raise ValueError('This YAML points to a different image. Choose the map YAML.')
        if image.name.lower().endswith('.colored.pgm') or (meta.get('mode') == 'raw' and image.with_suffix('.json').is_file()):
            return self.semantic_data(str(image.with_suffix('.json')))[0], {}
        resolution, origin = meta.get('resolution'), meta.get('origin')
        if not finite(resolution) or resolution <= 0:
            raise ValueError('Map resolution must be a positive number.')
        if not isinstance(origin, list) or len(origin) != 3 or not all(map(finite, origin)):
            raise ValueError('Map origin must be [x, y, yaw].')
        with open_preview_image(image) as pgm:
            width, height = pgm.size
            editable = pgm.mode == 'L'
        image_revision = self.revision(image)
        return dict(path=str(path), image=str(image), width=width, height=height,
                    resolution=resolution, origin=origin, editable=editable, tile_size=TILE_SIZE,
                    occupied=255 if meta.get('negate') in (1, '1') else 0, image_revision=image_revision,
                    revision=self.revision(path) + image_revision), meta

    @staticmethod
    def apply_palette(source, palette):
        if source.getextrema()[1] >= len(palette):
            raise ValueError('PGM contains an index missing from the JSON palette.')
        source.putpalette([channel for rgb in palette for channel in rgb])
        source.info['transparency'] = 0

    def semantic_preview(self, value, revision):
        metadata, palette = self.semantic_data(value)
        if revision != metadata['image_revision']:
            raise FileExistsError('The road semantic PGM or JSON changed. Reopen it to continue.')
        data = BytesIO()
        with open_preview_image(Path(metadata['image'])) as image:
            self.apply_palette(image, palette)
            image.thumbnail((1800, 1800), Image.Resampling.NEAREST)
            image.save(data, 'PNG')
        data.seek(0)
        return data

    def texture_preview(self, value, revision):
        metadata, _ = self.semantic_data(value)
        texture = metadata.get('texture')
        if not texture:
            raise FileNotFoundError('No optional texture image is available for this map.')
        if revision != metadata['revision']:
            raise FileExistsError('The road semantic map or texture image changed. Reopen it to continue.')
        data = BytesIO()
        with open_preview_image(Path(texture['path'])) as image:
            image.thumbnail((1800, 1800), Image.Resampling.LANCZOS)
            image.save(data, 'PNG')
        data.seek(0)
        return data

    def texture_tile(self, value, revision, col, row):
        path = self.scoped_file(value, self.map_directories)
        if path.suffix.lower() not in TEXTURE_SUFFIXES:
            raise ValueError('Choose a supported texture image.')
        if col is None or row is None or min(col, row) < 0:
            raise ValueError('Invalid texture tile coordinates.')
        key = (path, self.revision(path))
        if revision != key[1]:
            raise FileExistsError('The texture image has changed. Reopen it to continue.')
        left, top = col * TILE_SIZE, row * TILE_SIZE
        with self.tile_lock:
            if key not in self.tile_sources:
                with open_preview_image(path) as source:
                    bytes_needed = source.width * source.height * 4
                    while self.tile_sources and (len(self.tile_sources) >= 2 or
                            bytes_needed + sum(entry[2] for entry in self.tile_sources.values()) > TILE_CACHE_LIMIT):
                        _, (old_raster, _, _) = self.tile_sources.popitem(last=False)
                        old_raster.close()
                    source.load()
                    raster = source.convert('RGBA')
                self.tile_sources[key] = (raster, None, bytes_needed)
            self.tile_sources.move_to_end(key)
            raster, _, _ = self.tile_sources[key]
            if left >= raster.width or top >= raster.height:
                raise ValueError('Texture tile is outside the image.')
            tile = raster.crop((left, top, min(left + TILE_SIZE, raster.width),
                                min(top + TILE_SIZE, raster.height)))
        data = BytesIO()
        tile.save(data, 'PNG')
        data.seek(0)
        return data

    def costmap_prepare(self, body):
        """Build a Nav2 cost grid and inflation layer for the Map Editor.

        The browser keeps only the compact class-cost table and vector cost
        strokes. The full-resolution grid is prepared here and addressed by a
        content token so previews and detail tiles stay fast on large maps.
        """
        if not isinstance(body, dict):
            raise ValueError('Open a road semantic map first.')
        metadata, _ = self.semantic_data(body.get('path'))
        if metadata['revision'] != body.get('revision'):
            raise FileExistsError('The road semantic map changed. Reopen it to continue.')
        labels = {int(item['id']): item for item in metadata['semantic']['labels']}
        overrides = body.get('labels') or {}
        if not isinstance(overrides, dict) or len(overrides) > 256:
            raise ValueError('Invalid semantic cost table.')
        for key, spec in overrides.items():
            try:
                index = int(key)
            except (TypeError, ValueError):
                raise ValueError('Invalid semantic class id.') from None
            if index not in labels or not isinstance(spec, dict):
                raise ValueError('Invalid semantic cost table.')
            labels[index]['global_cost'] = self.cost_value(spec.get('global_cost'))
            labels[index]['local_cost'] = self.cost_value(spec.get('local_cost'))
        inflation = body.get('inflation') if isinstance(body.get('inflation'), dict) else {}
        params = {}
        for key, default in COSTMAP_DEFAULTS.items():
            value = inflation.get(key, default)
            params[key] = self.cost_value(value) if key == 'negate' else value
        for key in ('inflation_radius', 'cost_scaling', 'inscribed_radius', 'occupied_thresh'):
            if type(params[key]) is bool or not finite(params[key]):
                raise ValueError('Inflation settings must be finite numbers.')
        params['inflation_radius'] = max(0.0, min(10.0, float(params['inflation_radius'])))
        params['cost_scaling'] = max(0.0, min(50.0, float(params['cost_scaling'])))
        params['inscribed_radius'] = max(0.0, min(10.0, float(params['inscribed_radius'])))
        params['occupied_thresh'] = max(0.0, min(1.0, float(params['occupied_thresh'])))
        params['negate'] = 1 if params['negate'] else 0

        with open_preview_image(Path(metadata['image'])) as image:
            if image.mode != 'L':
                raise ValueError('Cost maps require an 8-bit indexed PGM.')
            indices = np.asarray(image, dtype=np.uint8).copy()
        height, width = indices.shape
        base_occupied = None
        base = body.get('base')
        if isinstance(base, dict) and base.get('path'):
            base_metadata, _ = self.map_data(base.get('path'))
            if base_metadata['revision'] != base.get('revision'):
                raise FileExistsError('The base map changed. Reopen it to continue.')
            same_grid = (base_metadata['width'] == width and base_metadata['height'] == height and
                         abs(float(base_metadata['resolution']) - float(metadata['resolution'])) < 1e-9 and
                         all(abs(float(a) - float(b)) < 1e-9 for a, b in
                             zip(base_metadata['origin'][:2], metadata['origin'][:2])))
            if same_grid and 'occupied' in base_metadata:
                with open_preview_image(Path(base_metadata['image'])) as image:
                    pixels = np.asarray(image, dtype=np.float32) / 255.0
                    probability = pixels if base_metadata['occupied'] else (1.0 - pixels)
                    base_occupied = probability >= params['occupied_thresh']
        values = np.zeros((height, width), dtype=np.uint8)
        values[indices == 0] = 255
        for index, info in labels.items():
            if index <= 0:
                continue
            cost = self.cost_value(max(info['global_cost'], info['local_cost']))
            if cost:
                values[indices == index] = cost
        strokes = self.costmap_strokes(body.get('strokes'), width, height)
        if strokes:
            raster = Image.fromarray(values, 'L')
            draw = ImageDraw.Draw(raster)
            for stroke in strokes:
                self.draw_stroke(draw, stroke, stroke.get('value', 0) if stroke['tool'] == 'cost' else 0)
            values = np.asarray(raster, dtype=np.uint8).copy()

        key_data = json.dumps(dict(
            revision=metadata['revision'],
            base=None if not isinstance(base, dict) else [base.get('path'), base.get('revision')],
            labels={str(index): [info['global_cost'], info['local_cost']] for index, info in labels.items()},
            inflation=params, strokes=strokes,
        ), sort_keys=True, separators=(',', ':'))
        token = sha256(key_data.encode()).hexdigest()[:32]
        with self.costmap_lock:
            entry = self.costmap_cache.get(token)
            if entry is None:
                occupied = (values >= 100) & (values < 255)
                if base_occupied is not None:
                    occupied = occupied | base_occupied
                inflation_cost = self.compute_inflation(
                    occupied, float(metadata['resolution']), params['inflation_radius'],
                    params['cost_scaling'], min(params['inscribed_radius'], params['inflation_radius']))
                entry = dict(values=values, indices=indices, labels=labels, strokes=strokes,
                             inflation=inflation_cost, width=width, height=height,
                             resolution=float(metadata['resolution']), origin=metadata['origin'])
                self.costmap_cache[token] = entry
                while len(self.costmap_cache) > COSTMAP_CACHE_LIMIT:
                    self.costmap_cache.popitem(last=False)
            self.costmap_cache.move_to_end(token)
        return dict(token=token, width=width, height=height,
                    stats=dict(cost_cells=int(np.count_nonzero((values > 0) & (values < 255))),
                               lethal_cells=int(np.count_nonzero(values == 100)),
                               inflated_cells=int(np.count_nonzero((entry['inflation'] > 0) &
                                                                   (entry['inflation'] < 254))),
                               inflation_max=int(entry['inflation'].max()) if entry['inflation'].size else 0,
                               classes=len([info for index, info in labels.items() if index > 0 and
                                            max(info['global_cost'], info['local_cost']) > 0])))

    def costmap_strokes(self, strokes, width, height):
        if strokes is None:
            return []
        if not isinstance(strokes, list) or len(strokes) > 5000:
            raise ValueError('Invalid cost map strokes.')
        checked, count = [], 0
        for stroke in strokes:
            if not isinstance(stroke, dict) or stroke.get('tool') not in ('cost', 'cost-eraser'):
                raise ValueError('Invalid cost map tool.')
            size = stroke.get('size')
            points = stroke.get('points')
            value = self.cost_value(stroke.get('value', 0))
            color = stroke.get('color', [255, 0, 0])
            if (type(size) is not int or not 1 <= size <= 128 or not isinstance(points, list) or not points or
                    not isinstance(color, list) or len(color) != 3 or
                    any(type(channel) is not int or not 0 <= channel <= 255 for channel in color)):
                raise ValueError('Invalid cost map brush or color.')
            count += len(points)
            if count > MAX_STROKE_POINTS:
                raise ValueError('Cost map edits are too large.')
            for point in points:
                if (not isinstance(point, list) or len(point) != 2 or
                        any(type(v) is not int for v in point) or
                        not (0 <= point[0] < width and 0 <= point[1] < height)):
                    raise ValueError('A cost map brush point is outside the map.')
            checked.append({**stroke, 'size': size, 'points': points, 'value': value, 'color': color})
        return checked

    @staticmethod
    def draw_stroke(draw, stroke, value):
        size = stroke['size']
        points = [tuple(point) for point in stroke['points']]
        if len(points) > 1:
            draw.line(points, fill=value, width=size)
        if size == 1:
            draw.point(points, fill=value)
            return
        for x, y in points:
            draw.ellipse((x - (size - 1) // 2, y - (size - 1) // 2,
                          x + size // 2, y + size // 2), fill=value)

    @staticmethod
    def compute_inflation(occupied, resolution, radius_m, cost_scaling, inscribed_m):
        """Compute Nav2 inflation_layer-compatible costs (0-254)."""
        height, width = occupied.shape
        max_cells = int(np.ceil(radius_m / resolution)) if radius_m > 0 and resolution > 0 else 0
        if max_cells > MAX_INFLATION_CELLS:
            raise ValueError('Inflation radius is too large for this map resolution.')
        if not occupied.any() or max_cells == 0:
            return np.zeros((height, width), dtype=np.uint8)
        inf = np.float32(np.inf)
        index = np.arange(width, dtype=np.float32)
        left_source = np.where(occupied, index, np.float32(-1.0))
        nearest_left = np.maximum.accumulate(left_source, axis=1)
        right_source = np.where(occupied, index, np.float32(width))
        nearest_right = np.minimum.accumulate(right_source[:, ::-1], axis=1)[:, ::-1]
        left_distance = np.where(nearest_left >= 0.0, index - nearest_left, inf)
        right_distance = np.where(nearest_right < width, nearest_right - index, inf)
        horizontal = np.minimum(left_distance, right_distance).astype(np.float32)
        padded = np.full((height + 2 * max_cells, width), inf, dtype=np.float32)
        padded[max_cells:max_cells + height, :] = horizontal
        distance_squared = np.full((height, width), inf, dtype=np.float32)
        for dy in range(-max_cells, max_cells + 1):
            block = padded[max_cells + dy:max_cells + dy + height, :]
            np.minimum(distance_squared, block * block + np.float32(dy * dy), out=distance_squared)
        distance_m = np.sqrt(distance_squared) * resolution
        cost = np.zeros((height, width), dtype=np.uint8)
        inflated = (~occupied) & (distance_m <= radius_m)
        if inflated.any():
            if cost_scaling > 0:
                decay = np.exp(-cost_scaling * (distance_m[inflated] - inscribed_m))
            else:
                decay = np.ones(int(inflated.sum()), dtype=np.float32)
            cost[inflated] = np.clip((decay * 252.0).astype(np.int32), 1, 252).astype(np.uint8)
        cost[(~occupied) & (distance_m <= inscribed_m)] = 253
        cost[occupied] = 254
        return cost

    def costmap_entry(self, token):
        if not isinstance(token, str) or len(token) != 32:
            raise FileNotFoundError('The cost map preview expired. Refresh it.')
        with self.costmap_lock:
            entry = self.costmap_cache.get(token)
            if entry is None:
                raise FileNotFoundError('The cost map preview expired. Refresh it.')
            self.costmap_cache.move_to_end(token)
        return entry

    @staticmethod
    def costmap_rgba(entry, box=None):
        values, indices = entry['values'], entry['indices']
        left, top, right, bottom = box or (0, 0, entry['width'], entry['height'])
        values = values[top:bottom, left:right]
        indices = indices[top:bottom, left:right]
        rgba = np.zeros((*values.shape, 4), dtype=np.uint8)
        for index, info in entry['labels'].items():
            if index <= 0:
                continue
            cost = max(info['global_cost'], info['local_cost'])
            if not cost:
                continue
            mask = indices == index
            if not mask.any():
                continue
            alpha = max(1, min(255, int(round(cost / 100.0 * 255))))
            rgba[mask] = (*info['rgb'][:3], alpha)
        if entry['strokes']:
            image = Image.fromarray(rgba, 'RGBA')
            draw = ImageDraw.Draw(image)
            for stroke in entry['strokes']:
                color = tuple(stroke['color'])
                opacity = 0 if stroke['tool'] == 'cost-eraser' or stroke['value'] <= 0 else max(
                    1, min(255, int(round(stroke['value'] / 100.0 * 255))))
                shifted = {**stroke, 'points': [[x - left, y - top] for x, y in stroke['points']]}
                size = shifted['size']
                points = [tuple(point) for point in shifted['points']]
                fill = (*color, opacity)
                if len(points) > 1:
                    draw.line(points, fill=fill, width=size)
                if size == 1:
                    draw.point(points, fill=fill)
                else:
                    for x, y in points:
                        draw.ellipse((x - (size - 1) // 2, y - (size - 1) // 2,
                                      x + size // 2, y + size // 2), fill=fill)
            rgba = np.asarray(image, dtype=np.uint8).copy()
        return rgba

    @staticmethod
    def inflation_rgba(entry, box=None):
        cost = entry['inflation']
        left, top, right, bottom = box or (0, 0, entry['width'], entry['height'])
        cost = cost[top:bottom, left:right]
        rgba = np.zeros((*cost.shape, 4), dtype=np.uint8)
        inflated = (cost >= 1) & (cost <= 252)
        if inflated.any():
            value = (255.0 * cost[inflated].astype(np.float32) / 252.0).astype(np.uint8)
            rgba[inflated, 0] = value
            rgba[inflated, 1] = 0
            rgba[inflated, 2] = (255 - value).astype(np.uint8)
            rgba[inflated, 3] = 255
        rgba[cost == 253] = (0, 255, 255, 255)
        rgba[cost == 254] = (255, 0, 255, 255)
        return rgba

    def costmap_preview(self, token, kind):
        entry = self.costmap_entry(token)
        rgba = self.costmap_rgba(entry) if kind != 'inflation' else self.inflation_rgba(entry)
        image = Image.fromarray(rgba, 'RGBA')
        image.thumbnail((1800, 1800), Image.Resampling.NEAREST)
        data = BytesIO()
        image.save(data, 'PNG')
        data.seek(0)
        return data

    def costmap_tile(self, token, kind, col, row):
        entry = self.costmap_entry(token)
        if col is None or row is None or min(col, row) < 0:
            raise ValueError('Invalid cost map tile coordinates.')
        left, top = col * TILE_SIZE, row * TILE_SIZE
        if left >= entry['width'] or top >= entry['height']:
            raise ValueError('Cost map tile is outside the map.')
        box = (left, top, min(left + TILE_SIZE, entry['width']), min(top + TILE_SIZE, entry['height']))
        rgba = self.costmap_rgba(entry, box) if kind != 'inflation' else self.inflation_rgba(entry, box)
        data = BytesIO()
        Image.fromarray(rgba, 'RGBA').save(data, 'PNG')
        data.seek(0)
        return data

    def export_costmap(self, body):
        if not isinstance(body, dict):
            raise ValueError('Select a cost map to export.')
        entry = self.costmap_entry(body.get('token'))
        name = self.files.filename(body.get('name'))
        if name.startswith('.') or Path(name).suffix:
            raise ValueError('Use a visible map name without an extension.')
        pgm_name = self.files.filename(name + '.pgm')
        yaml_name = self.files.filename(name + '.yaml')
        parent = self.files.resolve(str(self.root / 'maps'))
        if parent != self.root / 'maps' or not parent.is_dir():
            raise PermissionError('The maps folder is unavailable.')
        target = parent / name
        if os.path.lexists(target):
            raise FileExistsError('A map with this name already exists.')
        with tempfile.TemporaryDirectory(dir=parent, prefix='.map-editor-') as directory:
            staging = Path(directory)
            Image.fromarray(entry['values'], 'L').save(staging / pgm_name, 'PPM')
            metadata = dict(image=pgm_name, mode='raw', resolution=entry['resolution'],
                            origin=[entry['origin'][0], entry['origin'][1], 0.0],
                            negate=0, occupied_thresh=0.65, free_thresh=0.196)
            (staging / yaml_name).write_text(
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True), encoding='utf-8')
            self.files.rename_exclusive(staging, target)
        return dict(path=str(target / yaml_name), name=name)

    def costmap_path(self, body):
        """Plan a cost-aware path through prepared costmap waypoints."""
        if not isinstance(body, dict):
            raise ValueError('Generate a cost map first.')
        entry = self.costmap_entry(body.get('token'))
        points = body.get('points')
        if (not isinstance(points, list) or not 2 <= len(points) <= MAX_WAYPOINTS or
                any(not isinstance(point, list) or len(point) != 2 or not all(map(finite, point))
                    for point in points)):
            raise ValueError('Place at least two waypoints on the cost map.')
        combine = body.get('combine', 'max')
        combine = combine if combine in ('max', 'add') else 'max'
        height, width = entry['height'], entry['width']
        values = entry['values']
        semantic = np.zeros((height, width), dtype=np.float32)
        soft = (values > 0) & (values < 100)
        semantic[soft] = np.clip(np.round(values[soft].astype(np.float32) * 2.52), 1, 252)
        semantic[values >= 100] = 254
        inflation = entry['inflation'].astype(np.float32)
        if combine == 'add':
            combined = np.clip(semantic + inflation, 0, 254).astype(np.float32)
        else:
            combined = np.maximum(semantic, inflation)
        factor = max(1, int(math.ceil(max(width, height) / MAX_PATH_DIMENSION)))
        grid_width = int(math.ceil(width / factor))
        grid_height = int(math.ceil(height / factor))
        padded = np.zeros((grid_height * factor, grid_width * factor), dtype=np.float32)
        padded[:height, :width] = combined
        grid = padded.reshape(grid_height, factor, grid_width, factor).max(axis=(1, 3))

        def to_pixel(point):
            return ((float(point[1]) - entry['origin'][1]) / entry['resolution'],
                    height - (float(point[0]) - entry['origin'][0]) / entry['resolution'])

        def nearest_free(sx, sy):
            sx = min(max(int(round(sx)), 0), width - 1)
            sy = min(max(int(round(sy)), 0), height - 1)
            gx, gy = min(sx // factor, grid_width - 1), min(sy // factor, grid_height - 1)
            if grid[gy, gx] < 254:
                return gx, gy
            for radius in range(1, 13):
                best = None
                for y in range(max(0, gy - radius), min(grid_height, gy + radius + 1)):
                    for x in range(max(0, gx - radius), min(grid_width, gx + radius + 1)):
                        if grid[y, x] >= 254:
                            continue
                        distance = (x - gx) ** 2 + (y - gy) ** 2
                        if best is None or distance < best[0]:
                            best = (distance, x, y)
                if best:
                    return best[1], best[2]
            return gx, gy

        def astar(start, goal):
            sx, sy = start
            gx, gy = goal
            if grid[sy, sx] >= 254 or grid[gy, gx] >= 254:
                return None
            size = grid_width * grid_height
            start_index, goal_index = sy * grid_width + sx, gy * grid_width + gx
            scores = np.full(size, np.inf, dtype=np.float32)
            parents = np.full(size, -1, dtype=np.int32)
            closed = np.zeros(size, dtype=bool)
            scores[start_index] = 0.0
            queue = [(math.hypot(gx - sx, gy - sy), 0.0, start_index)]
            expanded = 0
            deadline = time.monotonic() + PATH_TIME_LIMIT
            while queue:
                _, score, index = heapq.heappop(queue)
                if closed[index]:
                    continue
                if index == goal_index:
                    break
                expanded += 1
                if expanded > MAX_PATH_EXPANSIONS or (expanded % 2048 == 0 and time.monotonic() > deadline):
                    return None
                closed[index] = True
                y, x = divmod(index, grid_width)
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if not (dx or dy):
                            continue
                        nx, ny = x + dx, y + dy
                        if not (0 <= nx < grid_width and 0 <= ny < grid_height):
                            continue
                        nindex = ny * grid_width + nx
                        if closed[nindex] or grid[ny, nx] >= 254:
                            continue
                        step = math.hypot(dx, dy) * (1.0 + float(grid[ny, nx]) / 25.0)
                        tentative = score + step
                        if tentative >= scores[nindex]:
                            continue
                        scores[nindex] = tentative
                        parents[nindex] = index
                        heuristic = math.hypot(gx - nx, gy - ny)
                        heapq.heappush(queue, (tentative + heuristic, tentative, nindex))
            if scores[goal_index] == np.inf:
                return None
            chain = []
            index = goal_index
            while index >= 0:
                y, x = divmod(index, grid_width)
                chain.append((x, y))
                if index == start_index:
                    break
                index = parents[index]
            return chain[::-1]

        path = []
        planned = 0
        for first, second in zip(points, points[1:]):
            start_world = [float(first[0]), float(first[1])]
            goal_world = [float(second[0]), float(second[1])]
            start = nearest_free(*to_pixel(start_world))
            goal = nearest_free(*to_pixel(goal_world))
            chain = astar(start, goal)
            if chain:
                segment = [[entry['origin'][0] + (x * factor + factor / 2.0) * entry['resolution'],
                            entry['origin'][1] + (height - y * factor - factor / 2.0) * entry['resolution']]
                           for x, y in chain]
                segment[0], segment[-1] = start_world, goal_world
                planned += 1
            else:
                segment = [start_world, goal_world]
            if path:
                segment = segment[1:]
            path.extend(segment)
        return dict(path=path, planned_segments=planned, segments=len(points) - 1)

    def tile(self, value, revision, col, row):
        path = self.scoped_file(value, self.map_directories)
        if col is None or row is None or min(col, row) < 0:
            raise ValueError('Invalid PGM tile coordinates.')
        palette = None
        if path.suffix.lower() == '.json':
            metadata, palette = self.semantic_data(str(path))
            key = (path, metadata['image_revision'])
            path = Path(metadata['image'])
        else:
            if path.suffix.lower() != '.pgm':
                raise ValueError('Choose a PGM or its semantic JSON.')
            key = (path, self.revision(path))
        if revision != key[1]:
            raise FileExistsError('The PGM has changed. Reopen it to continue.')
        left, top = col * TILE_SIZE, row * TILE_SIZE
        with self.tile_lock:
            if key not in self.tile_sources:
                with open_preview_image(path) as source:
                    bytes_needed = source.width * source.height * (1 if source.mode == 'L' else 4)
                    while self.tile_sources and (len(self.tile_sources) >= 2 or
                            bytes_needed + sum(entry[2] for entry in self.tile_sources.values()) > TILE_CACHE_LIMIT):
                        _, (old_raster, _, _) = self.tile_sources.popitem(last=False)
                        old_raster.close()
                    source.load()
                    if palette is not None:
                        self.apply_palette(source, palette)
                    raster = source.copy()
                extent = None if raster.mode in ('L', 'P') else raster.getextrema()
                self.tile_sources[key] = (raster, extent, bytes_needed)
            self.tile_sources.move_to_end(key)
            raster, extent, _ = self.tile_sources[key]
            if left >= raster.width or top >= raster.height:
                raise ValueError('Tile is outside the PGM.')
            tile = raster.crop((left, top, min(left + TILE_SIZE, raster.width), min(top + TILE_SIZE, raster.height)))
        if extent is not None:
            low, high = extent
            gain = 255 / max(1, high - low)
            tile = tile.point(lambda value: value * gain - low * gain).convert('L')
        data = BytesIO()
        tile.save(data, 'PNG')
        data.seek(0)
        return data

    def export_map(self, body):
        if not isinstance(body, dict):
            raise ValueError('Select a map to export.')
        name = self.files.filename(body.get('name'))
        if name.startswith('.'):
            raise ValueError('Map name must be visible in the file picker.')
        pgm_name = self.files.filename(name + '.pgm')
        yaml_name = self.files.filename(name + '.yaml')
        metadata, map_yaml = self.map_data(body.get('path'))
        if metadata['revision'] != body.get('revision'):
            raise FileExistsError('The source map has changed. Reopen it before exporting.')
        if not metadata['editable']:
            raise ValueError('Drawing is available for 8-bit occupancy PGM maps.')
        strokes = body.get('strokes')
        if not isinstance(strokes, list):
            raise ValueError('Invalid drawing.')
        count = 0
        width, height = metadata['width'], metadata['height']
        for stroke in strokes:
            if not isinstance(stroke, dict) or stroke.get('tool') not in ('pen', 'eraser'):
                raise ValueError('Invalid drawing tool.')
            size, points = stroke.get('size'), stroke.get('points')
            if type(size) is not int or not 1 <= size <= 128 or not isinstance(points, list) or not points:
                raise ValueError('Invalid brush or stroke.')
            count += len(points)
            if count > MAX_STROKE_POINTS:
                raise ValueError('Drawing is too large. Export the current map and continue from the copy.')
            for point in points:
                if (not isinstance(point, list) or len(point) != 2 or any(type(v) is not int for v in point)
                        or not (0 <= point[0] < width and 0 <= point[1] < height)):
                    raise ValueError('A brush point is outside the PGM.')

        parent = self.files.resolve(str(self.root / 'maps'))
        if parent != self.root / 'maps' or not parent.is_dir():
            raise PermissionError('The maps folder is unavailable.')
        target = parent / name
        if os.path.lexists(target):
            raise FileExistsError('A map with this name already exists.')
        with tempfile.TemporaryDirectory(dir=parent, prefix='.map-editor-') as directory:
            staging = Path(directory)
            with open_preview_image(Path(metadata['image'])) as image:
                draw = ImageDraw.Draw(image)
                for stroke in strokes:
                    size = stroke['size']
                    color = metadata['occupied'] if stroke['tool'] == 'pen' else 255 - metadata['occupied']
                    points = [tuple(point) for point in stroke['points']]
                    if len(points) > 1:
                        draw.line(points, fill=color, width=size)
                    if size == 1:
                        draw.point(points, fill=color)
                        continue
                    for x, y in points:
                        draw.ellipse((x - (size - 1) // 2, y - (size - 1) // 2, x + size // 2, y + size // 2), fill=color)
                image.save(staging / pgm_name, 'PPM')
            map_yaml['image'] = pgm_name
            (staging / yaml_name).write_text(yaml.safe_dump(map_yaml, sort_keys=False, allow_unicode=True), encoding='utf-8')
            if self.map_data(metadata['path'])[0]['revision'] != metadata['revision']:
                raise FileExistsError('The source map changed during export.')
            self.files.rename_exclusive(staging, target)
        return dict(path=str(target / yaml_name), name=name)

    @staticmethod
    def parse_format(body):
        text = body.get('yaml') if isinstance(body, dict) else None
        if not isinstance(text, str) or not text.strip() or len(text.encode()) > 32_768:
            raise ValueError('Enter a format YAML of up to 32 KiB.')
        try:
            document = yaml.safe_load(text)
            check_json_values(document)
        except (yaml.YAMLError, RecursionError) as error:
            raise ValueError('Invalid format YAML. Check the indentation and field types.') from error
        if not isinstance(document, dict) or type(document.get('version')) not in (int, float, str):
            raise ValueError('Format YAML requires a version and a format mapping.')
        fields = document.get('format')
        if not isinstance(fields, dict) or len(fields) > 100:
            raise ValueError('Define the waypoint fields under format (up to 100).')
        for key, value in fields.items():
            if not key.strip() or len(key) > 120 or key in ('__proto__', 'constructor', 'prototype'):
                raise ValueError('Invalid waypoint field name.')
            if not isinstance(value, str) or value not in SUPPORTED_TYPES:
                raise ValueError(f'Unsupported field type for {key}: use int, float, bool, string or pose.')
            expected = ACTION_TYPES.get(key, {POSE_FIELDS[key]} if key in POSE_FIELDS else SUPPORTED_TYPES)
            if value not in expected:
                raise ValueError(f'{key} requires type {" or ".join(sorted(expected))}.')
        fields = POSE_FIELDS | fields
        version = str(document['version']).strip()
        if not version:
            raise ValueError('Enter a format version.')
        parsed = dict(version=version, format=fields)
        return dict(document=parsed, yaml=yaml.safe_dump(parsed, sort_keys=False, allow_unicode=True))

    def load_waypoints(self, value):
        path = self.scoped_file(value, (self.root / 'waypoints',))
        if path.suffix.lower() not in YAML_SUFFIXES:
            raise ValueError('Choose a waypoint YAML file.')
        document = self.read_yaml(path)
        # Older editors can write an empty list as "waypoints:".
        if isinstance(document, dict) and 'waypoints' in document and document['waypoints'] is None:
            document['waypoints'] = []
        validate_waypoints(document)
        return dict(path=str(path), document=document)

    def save_waypoints(self, body):
        if not isinstance(body, dict):
            raise ValueError('Select waypoints to export.')
        name = self.files.filename(body.get('name'))
        if name.startswith('.') or Path(name).suffix.lower() not in YAML_SUFFIXES:
            raise ValueError('Use a visible filename ending in .yaml or .yml.')
        document = body.get('document')
        check_json_values(document)
        validate_waypoints(document)
        data = yaml.safe_dump(document, allow_unicode=True, sort_keys=False).encode()
        if len(data) > TEXT_LIMIT:
            raise ValueError('YAML files may be up to 512 KiB.')
        parent = self.files.resolve(str(self.root / 'waypoints'))
        if parent != self.root / 'waypoints' or not parent.is_dir():
            raise PermissionError('The waypoints folder is unavailable.')
        target = parent / name
        with tempfile.NamedTemporaryFile(dir=parent, prefix='.waypoint-', delete=False) as stream:
            tmp = Path(stream.name)
            try:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
                # link publishes the complete file and fails when the name is occupied.
                os.link(tmp, target)
            except FileExistsError:
                raise FileExistsError('A waypoint YAML with this name already exists.') from None
            finally:
                tmp.unlink(missing_ok=True)
        return dict(path=str(target), name=name, count=len(document['waypoints']))


def register_map_editor(app, auth_ok, files=None):
    editor = MapEditor(files or HostFiles())
    bp = Blueprint('map_editor', __name__, url_prefix='/api/map-editor')

    @bp.before_request
    def authenticate():
        if not auth_ok():
            return jsonify(error='auth'), 401
        if request.method == 'POST':
            origin = request.headers.get('Origin')
            if (origin and urlsplit(origin).netloc != request.host) or request.headers.get('Sec-Fetch-Site') == 'cross-site':
                return jsonify(error='Export from the same Launcher session.'), 403
            if request.content_length is None or request.content_length > 6 * TEXT_LIMIT:
                return jsonify(error='The request is too large.'), 413

    @bp.errorhandler(OSError)
    @bp.errorhandler(ValueError)
    def invalid(error):
        status = 403 if isinstance(error, PermissionError) else 409 if isinstance(error, FileExistsError) else 404 if isinstance(error, FileNotFoundError) else 400
        return jsonify(error=str(error)), status

    @bp.get('/roots')
    def roots():
        semantic = editor.root / 'maps' / 'road_semantic'
        return jsonify(maps=str(editor.root / 'maps'), archive=str(editor.root / 'map_archive'),
                       semantic=str(semantic) if semantic.is_dir() else None,
                       waypoints=str(editor.root / 'waypoints'))

    @bp.get('/map')
    def load_map():
        return jsonify(editor.load_map(request.args.get('path')))

    @bp.get('/semantic-preview')
    def semantic_preview():
        response = send_file(editor.semantic_preview(request.args.get('path'), request.args.get('revision')),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.get('/texture-preview')
    def texture_preview():
        response = send_file(editor.texture_preview(request.args.get('path'), request.args.get('revision')),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.get('/texture-tile')
    def texture_tile():
        response = send_file(editor.texture_tile(request.args.get('path'), request.args.get('revision'),
                                                  request.args.get('col', type=int), request.args.get('row', type=int)),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.get('/tile')
    def tile():
        response = send_file(editor.tile(request.args.get('path'), request.args.get('revision'),
                                         request.args.get('col', type=int), request.args.get('row', type=int)),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.post('/costmap')
    def costmap_prepare():
        return jsonify(editor.costmap_prepare(request.get_json(silent=True)))

    @bp.get('/costmap-preview')
    def costmap_preview():
        response = send_file(editor.costmap_preview(request.args.get('token'), request.args.get('kind')),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.get('/costmap-tile')
    def costmap_tile():
        response = send_file(editor.costmap_tile(request.args.get('token'), request.args.get('kind'),
                                                  request.args.get('col', type=int),
                                                  request.args.get('row', type=int)),
                             mimetype='image/png', max_age=86400)
        response.cache_control.public = False
        response.cache_control.private = True
        return response

    @bp.post('/map')
    def export_map():
        return jsonify(editor.export_map(request.get_json(silent=True))), 201

    @bp.post('/costmap-export')
    def export_costmap():
        return jsonify(editor.export_costmap(request.get_json(silent=True))), 201

    @bp.post('/costmap-path')
    def costmap_path():
        return jsonify(editor.costmap_path(request.get_json(silent=True)))

    @bp.post('/format')
    def parse_format():
        return jsonify(editor.parse_format(request.get_json(silent=True)))

    @bp.get('/waypoints')
    def load_waypoints():
        return jsonify(editor.load_waypoints(request.args.get('path')))

    @bp.post('/waypoints')
    def save_waypoints():
        return jsonify(editor.save_waypoints(request.get_json(silent=True))), 201

    app.register_blueprint(bp)
