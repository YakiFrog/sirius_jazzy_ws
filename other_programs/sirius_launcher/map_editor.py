"""Serve ROS map rasters and export copies of edited maps and waypoints."""
from collections import OrderedDict
from contextlib import contextmanager
from hashlib import sha256
from io import BytesIO
import heapq
import json
import math
import os
from pathlib import Path
import stat
import tempfile
from threading import Lock
from urllib.parse import urlsplit

from flask import Blueprint, jsonify, request, send_file
import numpy as np
from PIL import Image, ImageDraw
import yaml

try:
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra
except ImportError:
    dijkstra = None

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
PATH_CACHE_SEGMENTS = 2 * MAX_WAYPOINTS
PATH_CACHE_POINTS = 250_000
MAX_INFLATION_CELLS = 400
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
        if row.get('change_map') or (isinstance(initial, dict) and 'position_mode' in initial):
            if row.get('change_map') and initial == {'position_mode': 'first_waypoint', 'yaw_mode': 'current'}:
                continue
            raise ValueError(f'Waypoint {row["number"]}: initial_pose with change_map requires the first waypoint and current heading.')
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


class ExportConflict(FileExistsError):
    def __init__(self, existing):
        super().__init__('Files with these names already exist.')
        self.names = [path.name for path in existing]
        versions = [(str(path), info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
                    for path, info in existing.items()]
        self.approval = sha256(repr(versions).encode()).hexdigest()


class MapEditor:
    def __init__(self, files):
        self.files = files
        self.root = files.workspace / 'maps_waypoints'
        self.map_directories = (self.root / 'maps', self.root / 'map_archive')
        self.tile_lock = Lock()
        self.tile_sources = OrderedDict()
        self.costmap_lock = Lock()
        self.costmap_cache = OrderedDict()
        self.path_cache_lock = Lock()
        self.combined_grid = None

    def export_directory(self, kind, value=None):
        if kind not in ('maps', 'waypoints'):
            raise ValueError('Choose Maps or Waypoints.')
        root = self.root / kind
        parent = self.files.resolve(str(root) if value is None else value)
        if not parent.is_relative_to(root):
            raise PermissionError('Choose a folder within the export location.')
        if not parent.is_dir():
            raise ValueError('Choose a folder.')
        return parent

    def export_folders(self, kind, value):
        parent = self.export_directory(kind, value)
        listing = self.files.listing(str(parent))
        root = self.root / kind
        folders = [item for item in listing['entries'] if item['directory'] and not item['symlink']]
        files = [{'name': item['name']} for item in listing['entries'] if not item['directory']]
        return dict(path=str(parent), root=str(root), parent=str(parent.parent) if parent != root else None,
                    folders=folders, files=files, truncated=listing['truncated'])

    def create_export_folder(self, body):
        if not isinstance(body, dict):
            raise ValueError('Choose a folder in which to create the new folder.')
        parent = self.export_directory(body.get('kind'), body.get('directory'))
        name = self.files.filename(body.get('name'))
        if name.startswith('.'):
            raise ValueError('Use a visible folder name.')
        return self.files.mutate(dict(action='mkdir', parent=str(parent), name=name))

    @staticmethod
    def check_export_targets(targets, approval):
        existing = {}
        for target in targets:
            try:
                info = target.lstat()
            except FileNotFoundError:
                continue
            if not stat.S_ISREG(info.st_mode):
                raise FileExistsError(f'{target.name} is not a regular file. Choose another name.')
            existing[target] = info
        if existing:
            conflict = ExportConflict(existing)
            if approval != conflict.approval:
                raise conflict
        elif approval is not None:
            raise FileExistsError('The destination changed. Check it before saving again.')
        return existing

    @staticmethod
    def publish_export_file(source, target, existing):
        if target in existing:
            os.chmod(source, stat.S_IMODE(existing[target].st_mode) & 0o777)
            os.replace(source, target)
        else:
            os.link(source, target)

    @contextmanager
    def stage_raster_export(self, body, name, pgm_name, yaml_name):
        parent = self.export_directory('maps', body.get('directory'))
        bundled = body.get('directory') is None
        target = parent / name if bundled else parent
        raster = target / pgm_name
        config = target / yaml_name
        approval = body.get('overwrite')
        if bundled:
            if os.path.lexists(target):
                raise FileExistsError('A map with this name already exists.')
        else:
            with self.files.lock:
                self.check_export_targets((raster, config), approval)
        with tempfile.TemporaryDirectory(dir=parent, prefix='.map-editor-') as directory:
            staging = Path(directory)
            yield staging, config
            with self.files.lock:
                if bundled:
                    self.files.rename_exclusive(staging, target)
                    return
                existing = self.check_export_targets((raster, config), approval)
                # Publish the image first and keep the original for rollback until the YAML is visible.
                staged_raster = staging / pgm_name
                staged_info = staged_raster.stat()
                previous = staging / 'previous'
                if raster in existing:
                    os.link(raster, previous)
                self.publish_export_file(staged_raster, raster, existing)
                try:
                    self.publish_export_file(staging / yaml_name, config, existing)
                except OSError:
                    if raster.exists() and os.path.samestat(raster.lstat(), staged_info):
                        if raster in existing:
                            os.replace(previous, raster)
                        else:
                            raster.unlink(missing_ok=True)
                    raise

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
                    occupied=255 if meta.get('negate') in (1, '1') else 0,
                    occupied_thresh=meta.get('occupied_thresh') if finite(meta.get('occupied_thresh')) else None,
                    image_revision=image_revision,
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
        """Prepare semantic costs in their own grid and plan on the base grid when present."""
        if not isinstance(body, dict) or not (body.get('path') or body.get('base')):
            raise ValueError('Open a map first.')
        semantic = None
        if body.get('path'):
            semantic, _ = self.semantic_data(body['path'])
            if semantic['revision'] != body.get('revision'):
                raise FileExistsError('The road semantic map changed. Reopen it to continue.')

        base, base_yaml, drawing = None, {}, []
        source = body.get('base')
        if source is not None:
            if not isinstance(source, dict) or not source.get('path'):
                raise ValueError('Choose a base PGM and YAML.')
            base, base_yaml = self.map_data(source['path'])
            if 'semantic' in base:
                raise ValueError('Choose an occupancy PGM and YAML as the base map.')
            if base['revision'] != source.get('revision'):
                raise FileExistsError('The base map changed. Reopen it to continue.')
            drawing = self.validate_map_drawing(source.get('strokes', []), base['width'], base['height'])

        display = semantic or base
        planning = base or semantic
        width, height = display['width'], display['height']
        labels = {int(row['id']): row.copy() for row in semantic['semantic']['labels']} if semantic else {}
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
            if key == 'occupied_thresh' and finite(base_yaml.get(key)):
                default = base_yaml[key]
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

        strokes = self.costmap_strokes(body.get('strokes'), width, height)
        key_data = json.dumps(dict(
            semantic=semantic['revision'] if semantic else None,
            base=base['revision'] if base else None, drawing=drawing,
            labels={str(index): [info['global_cost'], info['local_cost']] for index, info in labels.items()},
            inflation=params, strokes=strokes,
        ), sort_keys=True, separators=(',', ':'))
        token = sha256(key_data.encode()).hexdigest()[:32]
        with self.costmap_lock:
            entry = self.costmap_cache.get(token)
            if entry is None:
                indices = None
                base_values = self.base_map_costs(base, base_yaml, params['occupied_thresh'], drawing) if base else None
                if semantic:
                    with open_preview_image(Path(semantic['image'])) as image:
                        if image.mode != 'L':
                            raise ValueError('Cost maps require an 8-bit indexed PGM.')
                        indices = np.asarray(image, dtype=np.uint8).copy()
                    table = np.zeros(256, dtype=np.uint8)
                    for index, row in labels.items():
                        table[index] = self.cost_value(max(row['global_cost'], row['local_cost']))
                    table[0] = 255
                    values = table[indices]
                else:
                    values = base_values
                if strokes:
                    raster = Image.fromarray(values, 'L')
                    draw = ImageDraw.Draw(raster)
                    for stroke in strokes:
                        self.draw_stroke(draw, stroke, stroke.get('value', 0) if stroke['tool'] == 'cost' else 0)
                    values = np.asarray(raster, dtype=np.uint8).copy()

                if base:
                    overlay = self.resample_cost_grid(values, display, base) if semantic else values
                    plan_values = np.maximum(base_values, np.where(overlay == 255, 0, overlay))
                else:
                    plan_values = np.where(values == 255, 0, values)
                if not semantic:
                    values = plan_values
                inflation_cost = self.compute_inflation(
                    plan_values == 100, float(planning['resolution']), params['inflation_radius'],
                    params['cost_scaling'], min(params['inscribed_radius'], params['inflation_radius']))
                grid = {key: planning[key] for key in ('width', 'height', 'resolution', 'origin')}
                entry = dict(values=values, indices=indices, labels=labels, strokes=strokes,
                             inflation=inflation_cost, plan_values=plan_values, grid=grid,
                             path_segments=OrderedDict(), path_points=0,
                             width=width, height=height,
                             resolution=float(display['resolution']), origin=display['origin'])
                self.costmap_cache[token] = entry
                while len(self.costmap_cache) > COSTMAP_CACHE_LIMIT:
                    self.costmap_cache.popitem(last=False)
            self.costmap_cache.move_to_end(token)
        values = entry['plan_values']
        return dict(token=token, width=width, height=height, planning_grid=entry['grid'],
                    stats=dict(cost_cells=int(np.count_nonzero(values)),
                               lethal_cells=int(np.count_nonzero(values == 100)),
                               inflated_cells=int(np.count_nonzero((entry['inflation'] > 0) &
                                                                   (entry['inflation'] < 254))),
                               inflation_max=int(entry['inflation'].max()) if entry['inflation'].size else 0,
                               classes=sum(index > 0 and max(row['global_cost'], row['local_cost']) > 0
                                           for index, row in labels.items())))

    def base_map_costs(self, base, metadata, occupied_thresh, drawing):
        with open_preview_image(Path(base['image'])) as image:
            pixels = np.asarray(image)
            if metadata.get('mode') == 'raw':
                values = np.where(pixels <= 100, pixels, 0).astype(np.uint8)
            else:
                probability = pixels.astype(np.float32) / (255 if image.mode == 'L' else 65535)
                if not base['occupied']:
                    probability = 1.0 - probability
                values = (probability >= occupied_thresh).astype(np.uint8) * 100
        if drawing:
            raster = Image.fromarray(values, 'L')
            draw = ImageDraw.Draw(raster)
            for stroke in drawing:
                self.draw_stroke(draw, stroke, 100 if stroke['tool'] == 'pen' else 0)
            values = np.asarray(raster, dtype=np.uint8).copy()
        return values

    @staticmethod
    def resample_cost_grid(values, source, target):
        """Sample semantic costs at the centers of the occupancy map cells."""
        sx, sy, angle_source = source['origin']
        tx, ty, angle_target = target['origin']
        height, width = target['height'], target['width']
        if values.shape == (height, width) and source['origin'] == target['origin'] and (
                source['resolution'] == target['resolution']):
            return values
        result = np.zeros((height, width), dtype=np.uint8)
        resolution = source['resolution']
        dx, dy = tx - sx, ty - sy
        cos_source, sin_source = math.cos(angle_source), math.sin(angle_source)
        ox = cos_source * dx + sin_source * dy
        oy = -sin_source * dx + cos_source * dy
        delta = angle_target - angle_source
        cos_delta, sin_delta = math.cos(delta), math.sin(delta)
        xs = (np.arange(width) + 0.5) * target['resolution']
        ys = (height - np.arange(height) - 0.5) * target['resolution']
        if sin_delta == 0.0:
            cols = np.floor((ox + xs * cos_delta) / resolution).astype(np.int64)
            rows = values.shape[0] - 1 - np.floor((oy + ys * cos_delta) / resolution).astype(np.int64)
            valid_x = (cols >= 0) & (cols < values.shape[1])
            valid_y = (rows >= 0) & (rows < values.shape[0])
            result[np.ix_(valid_y, valid_x)] = values[np.ix_(rows[valid_y], cols[valid_x])]
            return result
        rows_per_chunk = max(1, 500_000 // width)
        for top in range(0, height, rows_per_chunk):
            local_y = ys[top:top + rows_per_chunk, None]
            cols = np.floor((ox + xs * cos_delta - local_y * sin_delta) / resolution).astype(np.int64)
            rows = values.shape[0] - 1 - np.floor(
                (oy + xs * sin_delta + local_y * cos_delta) / resolution).astype(np.int64)
            valid = (cols >= 0) & (cols < values.shape[1]) & (rows >= 0) & (rows < values.shape[0])
            chunk = result[top:top + rows_per_chunk]
            chunk[valid] = values[rows[valid], cols[valid]]
        return result

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
        if max_cells == 0:
            return occupied.astype(np.uint8) * 254
        costs = np.zeros((height, width), dtype=np.uint8)
        rows_per_chunk = max(1, 2_000_000 // width)
        for start in range(0, height, rows_per_chunk):
            stop = min(height, start + rows_per_chunk)
            top, bottom = max(0, start - max_cells), min(height, stop + max_cells)
            band = occupied[top:bottom]
            if band.any():
                inflated = MapEditor.inflation_band(band, resolution, radius_m, cost_scaling, inscribed_m, max_cells)
                costs[start:stop] = inflated[start - top:stop - top]
        return costs

    @staticmethod
    def inflation_band(occupied, resolution, radius_m, cost_scaling, inscribed_m, max_cells):
        height, width = occupied.shape
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
        rgba = np.zeros((*values.shape, 4), dtype=np.uint8)
        if indices is None:
            rgba[..., 0] = 255
            rgba[..., 3] = np.round(values.astype(np.float32) * 2.55).astype(np.uint8)
            return rgba
        indices = indices[top:bottom, left:right]
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
        left, top, right, bottom = box or (0, 0, cost.shape[1], cost.shape[0])
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
        grid = entry['grid'] if kind == 'inflation' else entry
        if left >= grid['width'] or top >= grid['height']:
            raise ValueError('Cost map tile is outside the map.')
        box = (left, top, min(left + TILE_SIZE, grid['width']), min(top + TILE_SIZE, grid['height']))
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
        if name.startswith('.') or (body.get('directory') is None and Path(name).suffix):
            raise ValueError('Use a visible map name without an extension.')
        pgm_name = self.files.filename(name + '.pgm')
        yaml_name = self.files.filename(name + '.yaml')
        with self.stage_raster_export(body, name, pgm_name, yaml_name) as (staging, config):
            Image.fromarray(entry['values'], 'L').save(staging / pgm_name, 'PPM')
            metadata = dict(image=pgm_name, mode='raw', resolution=entry['resolution'],
                            origin=entry['origin'],
                            negate=0, occupied_thresh=0.65, free_thresh=0.196)
            (staging / yaml_name).write_text(
                yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True), encoding='utf-8')
        return dict(path=str(config), image=str(config.with_name(pgm_name)), name=name)

    def costmap_path(self, body):
        """Plan Nav2-style cost-aware paths through prepared costmap waypoints."""
        if not isinstance(body, dict):
            raise ValueError('Generate a cost map first.')
        token = body.get('token')
        entry = self.costmap_entry(token)
        points = body.get('points')
        if (not isinstance(points, list) or not 2 <= len(points) <= MAX_WAYPOINTS or
                any(not isinstance(point, list) or len(point) != 2 or not all(map(finite, point))
                    for point in points)):
            raise ValueError('Place at least two waypoints on the cost map.')
        combine = body.get('combine', 'max')
        combine = combine if combine in ('max', 'add') else 'max'
        grid = entry['grid']
        height, width = grid['height'], grid['width']
        ox, oy, yaw = grid['origin']
        resolution = grid['resolution']
        cosine, sine = math.cos(yaw), math.sin(yaw)
        def to_pixel(point):
            dx, dy = point[0] - ox, point[1] - oy
            x = (cosine * dx + sine * dy) / resolution
            y = (-sine * dx + cosine * dy) / resolution
            if not (0 <= x < width and 0 <= y < height):
                return None
            return math.floor(x), height - 1 - math.floor(y)

        def to_world(px, py):
            x, y = (px + 0.5) * resolution, (height - py - 0.5) * resolution
            return [ox + cosine * x - sine * y, oy + sine * x + cosine * y]

        cells = [to_pixel(point) for point in points]
        keys = [(combine, start, goal) for start, goal in zip(cells, cells[1:])]
        cached = entry['path_segments']
        with self.path_cache_lock:
            existing = {}
            for key in keys:
                if key in cached:
                    existing[key] = cached[key]
                    cached.move_to_end(key)
        combined = None
        path, failed_edges = [points[0]], []
        for key, second in zip(keys, points[1:]):
            _, start, goal = key
            middle = None
            if start is not None and goal is not None:
                if key not in existing:
                    if combined is None:
                        combined = self.combined_planning_cost(token, entry, combine)
                    segment = self.plan_cost_segment(combined, start, goal)
                    existing[key] = [to_world(px, py) for px, py in segment[1:-1]] if segment else None
                    self.cache_path_segment(entry, key, existing[key])
                middle = existing[key]
            if middle is None:
                failed_edges.append(len(path))
            else:
                path.extend(middle)
            path.append(second)
        return dict(path=path, planned_segments=len(points) - 1 - len(failed_edges),
                    segments=len(points) - 1, failed_edges=failed_edges)

    def combined_planning_cost(self, token, entry, mode):
        key = (token, mode)
        with self.path_cache_lock:
            if self.combined_grid is not None and self.combined_grid[0] == key:
                return self.combined_grid[1]
        lookup = np.rint(np.arange(101, dtype=np.float32) * 2.52).astype(np.uint8)
        lookup[100] = 254
        cost = lookup[entry['plan_values']]
        inflation = entry['inflation']
        if mode == 'add':
            cost = np.minimum(cost.astype(np.uint16) + inflation, 254).astype(np.uint8)
        else:
            np.maximum(cost, inflation, out=cost)
        with self.path_cache_lock:
            self.combined_grid = (key, cost)
        return cost

    def cache_path_segment(self, entry, key, middle):
        count = len(middle) if middle is not None else 0
        if count > PATH_CACHE_POINTS:
            return
        with self.path_cache_lock:
            cache = entry['path_segments']
            if key in cache:
                cache.move_to_end(key)
                return
            cache[key] = middle
            entry['path_points'] += count
            while len(cache) > PATH_CACHE_SEGMENTS or entry['path_points'] > PATH_CACHE_POINTS:
                _, expired = cache.popitem(last=False)
                if expired is not None:
                    entry['path_points'] -= len(expired)

    def plan_cost_segment(self, cost, start, goal):
        """Plan one segment like Nav2 NavFn: potential field, gradient, smoother."""
        height, width = cost.shape
        sx, sy = start
        gx, gy = goal
        if cost[sy, sx] >= 253 or cost[gy, gx] >= 253:
            return None
        if (sx, sy) == (gx, gy):
            return [(sx, sy)]
        margin = max(64, int(math.hypot(gx - sx, gy - sy) * 0.5))
        for distance in (margin, margin * 3, max(width, height)):
            x0 = max(0, min(sx, gx) - distance)
            x1 = min(width, max(sx, gx) + distance + 1)
            y0 = max(0, min(sy, gy) - distance)
            y1 = min(height, max(sy, gy) + distance + 1)
            window = cost[y0:y1, x0:x1]
            area = window.shape[0] * window.shape[1]
            step = int(math.ceil(math.sqrt(area / 250_000.0))) if area > 250_000 else 1
            if step > 1:
                grid, cstart, cgoal = self.downsample_cost_window(
                    window, (sx - x0, sy - y0), (gx - x0, gy - y0), step)
            else:
                grid, cstart, cgoal = window, (sx - x0, sy - y0), (gx - x0, gy - y0)
            found = self.navfn_path(grid, cstart, cgoal)
            if found is None:
                raw = self.astar_cost_window(grid, cstart, cgoal)
                if raw is None:
                    continue
                found = [(float(px), float(py)) for px, py in raw]
            if step > 1:
                full = [(min(x1 - 1, x0 + px * step + step // 2),
                         min(y1 - 1, y0 + py * step + step // 2)) for px, py in found]
            else:
                full = [(x0 + px, y0 + py) for px, py in found]
            full[0], full[-1] = (sx, sy), (gx, gy)
            smoothed = self.smooth_cost_path(full, cost)
            if self.cost_path_is_free(smoothed, cost):
                return smoothed
            if self.cost_path_is_free(full, cost):
                return full
        return None

    def navfn_path(self, grid, start, goal):
        potential = (self.sparse_cost_potential(grid, goal) if dijkstra is not None
                     else self.astar_cost_potential(grid, goal))
        return self.gradient_cost_path(potential, start, goal)

    @staticmethod
    def sparse_cost_potential(grid, goal):
        """Compute the weighted NavFn-style field through traversable cells."""
        height, width = grid.shape
        gx, gy = goal
        size = grid.size
        cells = np.arange(size, dtype=np.int32).reshape(grid.shape)
        free = grid < 253
        free[gy, gx] = True
        step_cost = 50.0 + grid.astype(np.float32) * 0.8
        source_indices, target_indices, weights = [], [], []
        for dy, dx in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
            source = (slice(max(-dy, 0), min(height, height - dy)),
                      slice(max(-dx, 0), min(width, width - dx)))
            target = (slice(max(dy, 0), min(height, height + dy)),
                      slice(max(dx, 0), min(width, width + dx)))
            connected = free[source] & free[target]
            source_indices.append(cells[source][connected])
            target_indices.append(cells[target][connected])
            weights.append(step_cost[target][connected] * math.hypot(dx, dy))
        edges = csr_matrix((np.concatenate(weights),
                            (np.concatenate(source_indices), np.concatenate(target_indices))),
                           shape=(size, size))
        distances = dijkstra(edges, directed=True, indices=gy * width + gx)
        return np.minimum(distances, 1.0e10).astype(np.float32).reshape(grid.shape)

    @staticmethod
    def astar_cost_potential(grid, goal):
        """Goal potential field using NavFn-like COST_NEUTRAL/cost weighting."""
        height, width = grid.shape
        gx = min(max(int(goal[0]), 0), width - 1)
        gy = min(max(int(goal[1]), 0), height - 1)
        blocked = grid >= 253
        blocked[gy, gx] = False
        neutral, factor = 50.0, 0.8
        step_cost = (neutral + grid.astype(np.float32) * factor).astype(np.float32)
        ys, xs = np.mgrid[0:height, 0:width]
        heuristic = (np.hypot(xs - gx, ys - gy) * neutral).astype(np.float32)
        size = height * width
        potential = np.full(size, np.float32(1.0e10), dtype=np.float32)
        closed = np.zeros(size, dtype=bool)
        goal_index = gy * width + gx
        potential[goal_index] = 0.0
        queue = [(0.0, goal_index)]
        neighbors = ((-1, -1, 1.41421356), (-1, 0, 1.0), (-1, 1, 1.41421356),
                     (0, -1, 1.0), (0, 1, 1.0),
                     (1, -1, 1.41421356), (1, 0, 1.0), (1, 1, 1.41421356))
        while queue:
            _, index = heapq.heappop(queue)
            if closed[index]:
                continue
            closed[index] = True
            cy, cx = divmod(index, width)
            base = potential[index]
            for dy, dx, distance in neighbors:
                ny, nx = cy + dy, cx + dx
                if ny < 0 or nx < 0 or ny >= height or nx >= width:
                    continue
                nindex = ny * width + nx
                if closed[nindex] or blocked[ny, nx]:
                    continue
                tentative = base + distance * step_cost[ny, nx]
                if tentative < potential[nindex]:
                    potential[nindex] = tentative
                    heapq.heappush(queue, (float(tentative) + float(heuristic[ny, nx]), nindex))
        return potential.reshape(height, width)

    @staticmethod
    def gradient_cost_path(potential, start, goal):
        """Descend the potential field to produce a sub-pixel path."""
        height, width = potential.shape
        if height < 3 or width < 3:
            return None
        flat = potential.reshape(-1)
        high, neutral, path_step = 1.0e10, 50.0, 0.5
        sx = min(max(int(start[0]), 1), width - 2)
        sy = min(max(int(start[1]), 1), height - 2)
        cell, dx, dy = sy * width + sx, 0.0, 0.0
        path = [(float(sx), float(sy))]
        for _ in range(200_000):
            nearest = cell + int(round(dx)) + width * int(round(dy))
            nearest = min(max(nearest, 0), height * width - 1)
            if flat[nearest] < neutral:
                path.append((float(goal[0]), float(goal[1])))
                return path
            if cell < width or cell >= (height - 1) * width:
                return path if len(path) > 1 else None
            path.append((float(cell % width) + dx, float(cell // width) + dy))
            below, above = cell + width, cell - width
            neighbors = (cell, cell + 1, cell - 1, below, below + 1, below - 1,
                         above, above + 1, above - 1)
            if any(flat[index] >= high for index in neighbors) or (
                    len(path) > 2 and path[-1] == path[-3]):
                minimum = min(neighbors, key=lambda index: flat[index])
                if flat[minimum] >= high:
                    return None
                cell, dx, dy = minimum, 0.0, 0.0
                continue
            gx = MapEditor.interpolate_cost_gradient(
                flat, cell, cell + 1, below, below + 1, dx, dy, width, height, True)
            gy = MapEditor.interpolate_cost_gradient(
                flat, cell, cell + 1, below, below + 1, dx, dy, width, height, False)
            if gx == 0.0 and gy == 0.0:
                return None
            scale = path_step / math.hypot(gx, gy)
            dx += gx * scale
            dy += gy * scale
            if dx > 1.0:
                cell += 1
                dx -= 1.0
            elif dx < -1.0:
                cell -= 1
                dx += 1.0
            if dy > 1.0:
                cell += width
                dy -= 1.0
            elif dy < -1.0:
                cell -= width
                dy += 1.0
        return path if len(path) > 1 else None

    @staticmethod
    def interpolate_cost_gradient(potential, a, b, c, d, dx, dy, width, height, x_axis):
        ga = MapEditor.cost_gradient_at(potential, a, width, height, x_axis)
        gb = MapEditor.cost_gradient_at(potential, b, width, height, x_axis)
        gc = MapEditor.cost_gradient_at(potential, c, width, height, x_axis)
        gd = MapEditor.cost_gradient_at(potential, d, width, height, x_axis)
        top = (1.0 - dx) * ga + dx * gb
        bottom = (1.0 - dx) * gc + dx * gd
        return (1.0 - dy) * top + dy * bottom

    @staticmethod
    def cost_gradient_at(potential, index, width, height, x_axis):
        high = 1.0e10
        x, y = index % width, index // width
        if x <= 0 or x >= width - 1 or y <= 0 or y >= height - 1:
            return 0.0
        current = potential[index]
        if current >= high:
            if x_axis:
                if potential[index - 1] < high:
                    return -1.0
                if potential[index + 1] < high:
                    return 1.0
            else:
                if potential[index - width] < high:
                    return -1.0
                if potential[index + width] < high:
                    return 1.0
            return 0.0
        dx = dy = 0.0
        if potential[index - 1] < high:
            dx += potential[index - 1] - current
        if potential[index + 1] < high:
            dx += current - potential[index + 1]
        if potential[index - width] < high:
            dy += potential[index - width] - current
        if potential[index + width] < high:
            dy += current - potential[index + width]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return 0.0
        return (dx if x_axis else dy) / length

    @staticmethod
    def smooth_cost_path(points, cost, data_weight=0.2, smooth_weight=0.3,
                         tolerance=1e-10, max_iterations=1000, refinement=2):
        """Approximate Nav2 SimpleSmoother without cutting into lethal cells."""
        if len(points) <= 2:
            return points
        height, width = cost.shape
        values = np.array(points, dtype=np.float64)
        for _ in range(refinement + 1):
            original = values.copy()
            for _ in range(max_iterations):
                previous = values.copy()
                values[1:-1] = (values[1:-1] + data_weight * (original[1:-1] - values[1:-1]) +
                                smooth_weight * (values[2:] + values[:-2] - 2.0 * values[1:-1]))
                change = float(np.abs(values[1:-1] - previous[1:-1]).sum())
                xs = np.clip(np.round(values[:, 0]).astype(np.int64), 0, width - 1)
                ys = np.clip(np.round(values[:, 1]).astype(np.int64), 0, height - 1)
                if np.any(cost[ys, xs] >= 253):
                    values = previous
                    break
                if change < tolerance:
                    break
        return [(float(point[0]), float(point[1])) for point in values]

    @staticmethod
    def cost_path_is_free(points, cost):
        height, width = cost.shape
        for first, second in zip(points, points[1:]):
            x0, y0 = first
            x1, y1 = second
            steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
            for step in range(steps + 1):
                x = int(round(x0 + (x1 - x0) * step / steps))
                y = int(round(y0 + (y1 - y0) * step / steps))
                if 0 <= x < width and 0 <= y < height and cost[y, x] >= 253:
                    return False
        return True

    @staticmethod
    def downsample_cost_window(window, start, goal, step):
        height, width = window.shape
        rows = (height + step - 1) // step
        cols = (width + step - 1) // step
        padded = np.pad(window, ((0, rows * step - height), (0, cols * step - width)),
                        constant_values=0)
        coarse = padded.reshape(rows, step, cols, step).max(axis=(1, 3))
        return coarse, (start[0] // step, start[1] // step), (goal[0] // step, goal[1] // step)

    @staticmethod
    def astar_cost_window(grid, start, goal):
        height, width = grid.shape
        sx, sy = start
        gx, gy = goal
        blocked = grid >= 254
        blocked[sy, sx] = False
        blocked[gy, gx] = False
        neutral, factor = 50.0, 0.8
        step_cost = (neutral + grid.astype(np.float32) * factor) / neutral
        ys, xs = np.mgrid[0:height, 0:width]
        heuristic = np.hypot(xs - gx, ys - gy).astype(np.float32)
        size = height * width
        scores = np.full(size, np.inf, dtype=np.float32)
        parents = np.full(size, -1, dtype=np.int32)
        closed = np.zeros(size, dtype=bool)
        start_index, goal_index = sy * width + sx, gy * width + gx
        scores[start_index] = 0.0
        neighbors = ((-1, -1, 1.41421356), (-1, 0, 1.0), (-1, 1, 1.41421356),
                     (0, -1, 1.0), (0, 1, 1.0),
                     (1, -1, 1.41421356), (1, 0, 1.0), (1, 1, 1.41421356))
        queue = [(float(heuristic[sy, sx]), start_index)]
        found = False
        while queue:
            _, index = heapq.heappop(queue)
            if index == goal_index:
                found = True
                break
            if closed[index]:
                continue
            closed[index] = True
            cy, cx = divmod(index, width)
            base = scores[index]
            for dy, dx, distance in neighbors:
                ny, nx = cy + dy, cx + dx
                if ny < 0 or nx < 0 or ny >= height or nx >= width:
                    continue
                nindex = ny * width + nx
                if closed[nindex] or blocked[ny, nx]:
                    continue
                tentative = base + distance * step_cost[ny, nx]
                if tentative < scores[nindex]:
                    scores[nindex] = tentative
                    parents[nindex] = index
                    heapq.heappush(queue, (tentative + float(heuristic[ny, nx]), nindex))
        if not found:
            return None
        path, current = [], goal_index
        while current != -1:
            cy, cx = divmod(current, width)
            path.append((cx, cy))
            if current == start_index:
                break
            current = parents[current]
        path.reverse()
        return path

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

    @staticmethod
    def validate_map_drawing(strokes, width, height):
        if not isinstance(strokes, list):
            raise ValueError('Invalid drawing.')
        count = 0
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
        return strokes

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
        strokes = self.validate_map_drawing(body.get('strokes'), metadata['width'], metadata['height'])

        with self.stage_raster_export(body, name, pgm_name, yaml_name) as (staging, config):
            with open_preview_image(Path(metadata['image'])) as image:
                draw = ImageDraw.Draw(image)
                for stroke in strokes:
                    color = metadata['occupied'] if stroke['tool'] == 'pen' else 255 - metadata['occupied']
                    self.draw_stroke(draw, stroke, color)
                image.save(staging / pgm_name, 'PPM')
            map_yaml['image'] = pgm_name
            (staging / yaml_name).write_text(yaml.safe_dump(map_yaml, sort_keys=False, allow_unicode=True), encoding='utf-8')
            if self.map_data(metadata['path'])[0]['revision'] != metadata['revision']:
                raise FileExistsError('The source map changed during export.')
        return dict(path=str(config), image=str(config.with_name(pgm_name)), name=name)

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
        parent = self.export_directory('waypoints', body.get('directory'))
        target = self.save_export_file(parent, name, data, body.get('overwrite'))
        return dict(path=str(target), name=name, count=len(document['waypoints']))

    def save_landmarks(self, body):
        if not isinstance(body, dict):
            raise ValueError('Select landmarks to export.')
        name = self.files.filename(body.get('name'))
        if name.startswith('.') or Path(name).suffix.lower() != '.json':
            raise ValueError('Use a visible filename ending in .json.')
        document = body.get('document')
        rows = document.get('landmarks') if isinstance(document, dict) else None
        if not isinstance(rows, list) or len(rows) > 1000:
            raise ValueError('Expected a landmarks array (up to 1,000).')
        check_json_values(document)
        data = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()
        if len(data) > TEXT_LIMIT:
            raise ValueError('JSON files may be up to 512 KiB.')
        target = self.save_export_file(self.export_directory('maps', body.get('directory')), name, data, body.get('overwrite'))
        return dict(path=str(target), name=name)

    def save_export_file(self, parent, name, data, approval):
        target = parent / name
        with tempfile.NamedTemporaryFile(dir=parent, prefix='.map-editor-', delete=False) as stream:
            tmp = Path(stream.name)
            try:
                stream.write(data)
                stream.flush()
                os.fsync(stream.fileno())
                with self.files.lock:
                    existing = self.check_export_targets((target,), approval)
                    self.publish_export_file(tmp, target, existing)
            finally:
                tmp.unlink(missing_ok=True)
        return target


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

    @bp.errorhandler(ExportConflict)
    def export_conflict(error):
        return jsonify(error=str(error), existing=error.names, overwrite=error.approval), 409

    @bp.get('/roots')
    def roots():
        semantic = editor.root / 'maps' / 'road_semantic'
        return jsonify(maps=str(editor.root / 'maps'), archive=str(editor.root / 'map_archive'),
                       semantic=str(semantic) if semantic.is_dir() else None,
                       waypoints=str(editor.root / 'waypoints'))

    @bp.get('/export-folders')
    def list_export_folders():
        return jsonify(editor.export_folders(request.args.get('kind'), request.args.get('path')))

    @bp.post('/export-folders')
    def create_export_folder():
        return jsonify(editor.create_export_folder(request.get_json(silent=True))), 201

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

    @bp.post('/landmarks')
    def save_landmarks():
        return jsonify(editor.save_landmarks(request.get_json(silent=True))), 201

    app.register_blueprint(bp)
