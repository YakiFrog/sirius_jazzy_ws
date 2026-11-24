from io import BytesIO
import json
from pathlib import Path
import sys

from flask import Flask
from PIL import Image
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from file_browser import HostFiles, register_file_browser
from map_editor import register_map_editor


@pytest.fixture
def editor(tmp_path):
    workspace = tmp_path / 'workspace'
    root = workspace / 'maps_waypoints'
    for name in ('maps', 'waypoints'):
        (root / name).mkdir(parents=True)
    app = Flask(__name__)
    files = HostFiles(home=tmp_path, workspace=workspace)
    register_map_editor(app, lambda: True, files)
    register_file_browser(app, lambda: True, files)
    return app.test_client(), root


def test_opens_a_map_using_yaml_metadata_and_checks_image_reference(editor):
    client, root = editor
    image = root / 'maps' / 'floor.pgm'
    image.write_bytes(b'P5\n3 2\n255\n' + bytes(6))
    config = image.with_suffix('.yaml')
    config.write_text(yaml.safe_dump(dict(image=image.name, resolution=0.05, origin=[-1, 2, 0.3])))
    response = client.get('/api/map-editor/map', query_string={'path': str(image)})
    assert response.status_code == 200
    expected = dict(path=str(config), image=str(image), width=3, height=2,
                    resolution=0.05, origin=[-1, 2, 0.3], editable=True, tile_size=512)
    assert {key: response.json[key] for key in expected} == expected
    outside = root / 'waypoints' / image.name
    outside.write_bytes(image.read_bytes())
    config.write_text(yaml.safe_dump(dict(image=str(outside), resolution=0.05, origin=[0, 0, 0])))
    assert client.get('/api/map-editor/map', query_string={'path': str(config)}).status_code == 403


def test_opens_and_previews_pgm_at_the_128_mib_limit(editor):
    client, root = editor
    width, height = 16384, 8191
    limit = 128 * 1024 * 1024
    header = f'{width} {height}\n255\n'.encode()
    padding = limit - width * height - len(header) - len(b'P5\n#\n')
    image = root / 'maps' / 'large.pgm'
    with image.open('wb') as stream:
        stream.write(b'P5\n#' + b' ' * padding + b'\n' + header)
        stream.truncate(limit)
    image.with_suffix('.yaml').write_text(
        yaml.safe_dump(dict(image=image.name, resolution=0.05, origin=[0, 0, 0])))

    params = {'path': str(image)}
    metadata = client.get('/api/map-editor/map', query_string=params)
    assert metadata.status_code == 200
    assert (metadata.json['width'], metadata.json['height']) == (width, height)
    preview = client.get('/api/files/preview', query_string=params)
    assert preview.status_code == 200
    assert preview.headers['X-Image-Width'] == str(width)
    assert preview.headers['X-Image-Height'] == str(height)
    with Image.open(BytesIO(preview.data)) as rendered:
        assert rendered.format == 'PNG'
        assert rendered.width == 1800

    with image.open('ab') as stream:
        stream.write(b'\0')
    assert client.get('/api/map-editor/map', query_string=params).status_code == 400
    over_limit = client.get('/api/files/preview', query_string=params)
    assert over_limit.status_code == 400
    assert '128 MiB' in over_limit.json['error']


def test_edit_export_preserves_other_fields_and_refuses_overwrite(editor):
    client, root = editor
    source = root / 'waypoints' / 'course.yaml'
    original = dict(format_version='1.0', custom={'course': '実験'}, waypoints=[
        dict(number=5, x=1.0, y=2.0, angle_radians=0.2, manual=True,
             initial_pose={'x': 1.0, 'y': 2.0, 'yaw_mode': 'current'}),
        dict(number=10, x=3.0, y=4.0, angle_radians=0.3, speech='到着'),
    ])
    before = yaml.safe_dump(original, allow_unicode=True)
    source.write_text(before)
    loaded = client.get('/api/map-editor/waypoints', query_string={'path': str(source)})
    assert loaded.status_code == 200
    document = loaded.json['document']
    document['waypoints'].reverse()
    document['waypoints'][0]['x'] = 1.5
    payload = dict(name='course-edit.yaml', document=document)
    result = client.post('/api/map-editor/waypoints', json=payload)
    assert result.status_code == 201
    exported = Path(result.json['path'])
    assert yaml.safe_load(exported.read_text()) == document
    assert document['custom'] == original['custom']
    assert document['waypoints'][1]['initial_pose'] == original['waypoints'][0]['initial_pose']
    assert source.read_text() == before
    assert client.post('/api/map-editor/waypoints', json=payload).status_code == 409
    assert client.post('/api/map-editor/waypoints', json={**payload, 'name': '../unsafe.yaml'}).status_code == 400


def test_rejects_actions_that_mv_goal_cannot_use_together(editor):
    client, root = editor
    invalid = dict(waypoints=[dict(number=1, x=0, y=0, angle_radians=0, change_map='next.yaml',
                                  initial_pose={'x': 0, 'y': 0, 'yaw_mode': 'current'})])
    response = client.post('/api/map-editor/waypoints', json=dict(name='invalid.yaml', document=invalid))
    assert response.status_code == 400 and 'separate waypoints' in response.json['error']
    assert not (root / 'waypoints' / 'invalid.yaml').exists()


def test_native_tile_and_map_export_preserve_pixels_and_ros_metadata(editor):
    client, root = editor
    source = root / 'maps' / 'original.pgm'
    source_yaml = source.with_suffix('.yaml')
    metadata = dict(image=source.name, resolution=0.05, origin=[-5.0, 6.0, 0.3],
                    mode='trinary', negate=0, occupied_thresh=0.65, free_thresh=0.2)
    raster = Image.new('L', (513, 10), 205)
    raster.putpixel((512, 1), 0)
    raster.save(source)
    source_yaml.write_text(yaml.safe_dump(metadata))
    original = source.read_bytes()

    result = client.get('/api/map-editor/map', query_string={'path': str(source_yaml)}).json
    params = dict(path=str(source), revision=result['image_revision'], col=1, row=0)
    tile = client.get('/api/map-editor/tile', query_string=params)
    assert tile.status_code == 200
    with Image.open(BytesIO(tile.data)) as detail:
        assert detail.size == (1, 10)
        assert list(detail.getdata())[:3] == [205, 0, 205]
    assert client.get('/api/map-editor/tile', query_string={**params, 'revision': 'stale'}).status_code == 409
    payload = dict(name='edited', path=str(source_yaml), revision=result['revision'], strokes=[
        dict(tool='pen', size=1, points=[[2, 2], [4, 2]]),
        dict(tool='eraser', size=1, points=[[512, 1]]),
    ])
    export = client.post('/api/map-editor/map', json=payload)
    assert export.status_code == 201
    output_yaml = Path(export.json['path'])
    assert yaml.safe_load(output_yaml.read_text()) == {**metadata, 'image': 'edited.pgm'}
    with Image.open(output_yaml.with_suffix('.pgm')) as edited:
        assert edited.size == raster.size
        assert [edited.getpixel((x, 2)) for x in range(1, 6)] == [205, 0, 0, 0, 205]
        assert edited.getpixel((512, 1)) == 255
    assert source.read_bytes() == original
    assert client.post('/api/map-editor/map', json=payload).status_code == 409


def test_semantic_map_uses_the_pgm_indices_and_json_palette_and_origin(editor):
    client, root = editor
    image = root / 'maps' / 'road.colored.pgm'
    image.write_bytes(b'P5\n3 2\n255\n' + bytes([1, 2, 0, 0, 1, 2]))
    palette = image.with_suffix('.json')
    document = dict(resolution=0.1, origin=[-2, -3], width=3, height=2,
                    palette=[[127, 127, 127], [255, 255, 0], [12, 55, 81]],
                    labels={'1': {'name': 'tactile_paving', 'color': [9, 9, 9]}})
    palette.write_text(json.dumps(document))
    config = root / 'maps' / 'road.yaml'
    config.write_text(yaml.safe_dump(dict(image=image.name, resolution=0.2, origin=[0, 0, 0])))
    for source in (image, palette, config):
        response = client.get('/api/map-editor/map', query_string={'path': str(source)})
        assert response.status_code == 200
        metadata = response.json
        assert metadata['origin'] == [-2, -3, 0] and metadata['resolution'] == 0.1
        assert metadata['editable'] is False
        assert metadata['semantic']['legend'] == [{'name': 'tactile_paving', 'rgb': [255, 255, 0]}]

    params = dict(path=str(palette), revision=metadata['image_revision'])
    for endpoint, query in [
        ('/api/map-editor/semantic-preview', params),
        ('/api/map-editor/tile', {**params, 'col': 0, 'row': 0}),
    ]:
        response = client.get(endpoint, query_string=query)
        assert response.status_code == 200
        with Image.open(BytesIO(response.data)) as rendered:
            assert list(rendered.convert('RGBA').getdata()) == [
                (255, 255, 0, 255), (12, 55, 81, 255), (127, 127, 127, 0),
                (127, 127, 127, 0), (255, 255, 0, 255), (12, 55, 81, 255),
            ]

    document['palette'][1] = [80, 90, 100]
    palette.write_text(json.dumps(document))
    assert client.get('/api/map-editor/tile', query_string={**params, 'col': 0, 'row': 0}).status_code == 409
    document['width'] = 10
    palette.write_text(json.dumps(document))
    assert client.get('/api/map-editor/map', query_string={'path': str(image)}).status_code == 400


def test_imports_amr_toolkit_format_and_retains_sirius_geometry(editor):
    client, _ = editor
    response = client.post('/api/map-editor/format', json=dict(yaml="""
version: '2.0'
format:
  angle_degrees: float
  speech: string
  note: str
  enabled: bool
  lap: int
"""))
    assert response.status_code == 200
    document = response.json['document']
    assert document['version'] == '2.0'
    assert document['format']['angle_radians'] == 'float'
    assert document['format']['note'] == 'str'
    assert document['format']['enabled'] == 'bool'
    assert yaml.safe_load(response.json['yaml']) == document
    invalid = client.post('/api/map-editor/format', json={'yaml': 'version: 1\nformat: {wait_time: str}'})
    assert invalid.status_code == 400


def test_optional_texture_image_is_exposed_as_a_preview_layer(editor):
    client, root = editor
    image = root / 'maps' / 'road.colored.pgm'
    image.write_bytes(b'P5\n3 2\n255\n' + bytes([1, 2, 0, 0, 1, 2]))
    palette = image.with_suffix('.json')
    palette.write_text(json.dumps(dict(
        resolution=0.1, origin=[-2, -3], width=3, height=2,
        palette=[[127, 127, 127], [255, 255, 0], [12, 55, 81]],
    )))
    texture = root / 'maps' / 'road.texture.png'
    texture_image = Image.new('RGBA', (3, 2))
    texture_image.putdata([(1, 2, 3, 255), (4, 5, 6, 255), (7, 8, 9, 0),
                           (10, 11, 12, 255), (13, 14, 15, 255), (16, 17, 18, 255)])
    texture_image.save(texture)

    metadata = client.get('/api/map-editor/map', query_string={'path': str(palette)}).json
    assert metadata['texture']['path'] == str(texture)
    assert metadata['texture']['width'] == 3 and metadata['texture']['height'] == 2

    preview = client.get('/api/map-editor/texture-preview', query_string={
        'path': str(palette), 'revision': metadata['revision']})
    assert preview.status_code == 200
    with Image.open(BytesIO(preview.data)) as rendered:
        assert rendered.size == (3, 2)

    tile = client.get('/api/map-editor/texture-tile', query_string={
        'path': str(texture), 'revision': metadata['texture']['revision'], 'col': 0, 'row': 0})
    assert tile.status_code == 200
    with Image.open(BytesIO(tile.data)) as rendered:
        assert list(rendered.convert('RGBA').getdata()) == list(texture_image.getdata())

    texture_image.resize((2, 2)).save(texture)
    mismatch = client.get('/api/map-editor/map', query_string={'path': str(palette)}).json
    assert 'texture' not in mismatch and 'dimensions differ' in mismatch['texture_error']


def test_map_editor_has_an_optional_texture_image_layer():
    script = (Path(__file__).resolve().parents[1] / 'static' / 'map_editor.js').read_text(encoding='utf-8')
    assert "texture: 'Texture Image'" in script
    assert "state.rasters[layer] = {map, image, tiles: new Map()}" in script
    assert "map.texture" in script
    assert "/api/map-editor/texture-preview" in script
    assert "/api/map-editor/texture-tile" in script
    assert "state.rasters.texture" in script


def test_map_editor_tools_have_keyboard_shortcuts():
    root = Path(__file__).resolve().parents[1]
    script = (root / 'static' / 'map_editor.js').read_text(encoding='utf-8')
    style = (root / 'static' / 'map_editor.css').read_text(encoding='utf-8')
    assert "const toolShortcuts = {select: 'V', pan: 'H', pen: 'P', eraser: 'E', cost: 'C', 'cost-eraser': 'X',\n    add: 'W', landmark: 'L'}" in script
    assert "aria-keyshortcuts=\"${shortcut}\"" in script
    assert "historyStep(!event.shiftKey)" in script
    assert "el('me-grid').click()" in script
    assert 'class="me-shortcuts"' in script
    assert '<kbd class="me-key"' in script
    assert '<kbd>V</kbd> Select' in script
    assert '.me-shortcuts {' in style
    assert '.me-shortcuts kbd,' in style


def test_nav2_costmap_previews_inflates_edits_and_exports(editor):
    client, root = editor
    image = root / 'maps' / 'cost.colored.pgm'
    image.write_bytes(b'P5\n5 4\n255\n' + bytes([
        0, 1, 2, 3, 1,
        1, 2, 2, 3, 1,
        1, 1, 2, 2, 1,
        1, 1, 1, 1, 1,
    ]))
    palette = image.with_suffix('.json')
    palette.write_text(json.dumps(dict(
        resolution=0.1, origin=[-1.0, -2.0], width=5, height=4,
        palette=[[127, 127, 127], [255, 255, 255], [0, 255, 0], [255, 255, 0]],
        labels={
            '0': {'name': 'unknown', 'color': [127, 127, 127], 'global_cost': 0, 'local_cost': 0},
            '1': {'name': 'floor', 'color': [255, 255, 255], 'global_cost': 0, 'local_cost': 0},
            '2': {'name': 'grass', 'color': [0, 255, 0], 'global_cost': 100, 'local_cost': 70},
            '3': {'name': 'tactile paving', 'color': [255, 255, 0], 'global_cost': 50, 'local_cost': 30},
        },
    )))
    metadata = client.get('/api/map-editor/map', query_string={'path': str(palette)}).json
    labels = {str(item['id']): item for item in metadata['semantic']['labels']}
    assert labels['2']['global_cost'] == 100 and labels['3']['local_cost'] == 30

    payload = dict(path=str(palette), revision=metadata['revision'],
                   labels={'3': {'global_cost': 80, 'local_cost': 20}},
                   inflation={'inflation_radius': 0.25, 'cost_scaling': 3.0,
                              'inscribed_radius': 0.1, 'occupied_thresh': 0.65, 'negate': 0},
                   strokes=[dict(tool='cost', size=1, value=100, color=[255, 0, 0],
                                 points=[[0, 0]])])
    prepared = client.post('/api/map-editor/costmap', json=payload)
    assert prepared.status_code == 200
    result = prepared.json
    assert result['token'] and result['stats']['lethal_cells'] == 6
    assert result['stats']['inflated_cells'] > 0
    assert result['stats']['inflation_max'] == 254

    preview = client.get('/api/map-editor/costmap-preview', query_string={
        'token': result['token'], 'kind': 'costmap'})
    assert preview.status_code == 200
    with Image.open(BytesIO(preview.data)) as rendered:
        assert rendered.size == (5, 4)
        assert rendered.getpixel((2, 0)) == (0, 255, 0, 255)
        assert rendered.getpixel((0, 0)) == (255, 0, 0, 255)
        assert rendered.getpixel((1, 0)) == (0, 0, 0, 0)

    tile = client.get('/api/map-editor/costmap-tile', query_string={
        'token': result['token'], 'kind': 'inflation', 'col': 0, 'row': 0})
    assert tile.status_code == 200
    with Image.open(BytesIO(tile.data)) as rendered:
        assert rendered.size == (5, 4)
        assert rendered.getpixel((2, 0))[3] == 255

    exported = client.post('/api/map-editor/costmap-export',
                           json=dict(name='cost-edit', token=result['token']))
    assert exported.status_code == 201
    output = Path(exported.json['path'])
    exported_yaml = yaml.safe_load(output.read_text())
    assert exported_yaml['mode'] == 'raw' and exported_yaml['image'] == 'cost-edit.pgm'
    with Image.open(output.with_suffix('.pgm')) as rendered:
        assert rendered.getpixel((2, 0)) == 100
        assert rendered.getpixel((0, 0)) == 100
        assert rendered.getpixel((1, 0)) == 0
    assert client.get('/api/map-editor/costmap-preview',
                      query_string={'token': '0' * 32, 'kind': 'costmap'}).status_code == 404


def test_map_editor_exposes_nav2_costmap_layers_and_tools():
    root = Path(__file__).resolve().parents[1]
    script = (root / 'static' / 'map_editor.js').read_text(encoding='utf-8')
    style = (root / 'static' / 'map_editor.css').read_text(encoding='utf-8')
    assert "costmap: 'Cost Map'" in script and "inflation: 'Inflation (Nav2)'" in script
    assert "cost: ['Cost Pen'" in script and "'cost-eraser': ['Cost Eraser'" in script
    assert "id=\"me-costmap-panel\"" in script
    assert "post('costmap'" in script and "post('costmap-export'" in script
    assert "source = '/api/map-editor/costmap-tile?'" in script
    assert "scheduleCostmap()" in script
    assert '.me-cost-table' in style and '.me-cost-row' in style


def test_costmap_generates_a_planned_path_around_lethal_cells(editor):
    client, root = editor
    width, height = 20, 20
    pixels = bytearray([1] * (width * height))
    for row in range(0, height - 2):
        pixels[row * width + 10] = 2
    image = root / 'maps' / 'path.colored.pgm'
    image.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes(pixels))
    palette = image.with_suffix('.json')
    palette.write_text(json.dumps(dict(
        resolution=0.1, origin=[0.0, 0.0], width=width, height=height,
        palette=[[127, 127, 127], [255, 255, 255], [0, 255, 0]],
        labels={
            '0': {'name': 'unknown', 'global_cost': 0, 'local_cost': 0},
            '1': {'name': 'floor', 'global_cost': 0, 'local_cost': 0},
            '2': {'name': 'grass', 'global_cost': 100, 'local_cost': 100},
        },
    )))
    metadata = client.get('/api/map-editor/map', query_string={'path': str(palette)}).json
    prepared = client.post('/api/map-editor/costmap', json=dict(
        path=str(palette), revision=metadata['revision'],
        inflation={'inflation_radius': 0.0, 'cost_scaling': 3.0,
                   'inscribed_radius': 0.0, 'occupied_thresh': 0.65, 'negate': 0},
        strokes=[],
    )).json
    # Start left of the barrier, finish right of it, both on the top row.
    points = [[0.25, 1.75], [1.85, 1.75]]
    result = client.post('/api/map-editor/costmap-path',
                         json=dict(token=prepared['token'], points=points, combine='max'))
    assert result.status_code == 200
    assert result.json['planned_segments'] == 1
    path = result.json['path']
    assert len(path) > 2
    assert path[0] == points[0] and path[-1] == points[1]
    # The barrier only has a gap near the bottom, so the route must leave row 1.
    assert min(point[1] for point in path) < 1.0
    assert client.post('/api/map-editor/costmap-path',
                       json=dict(token=prepared['token'], points=points[:1])).status_code == 400


def test_map_editor_imports_waypoint_drag_reorder_and_generated_path_ui():
    root = Path(__file__).resolve().parents[1]
    script = (root / 'static' / 'map_editor.js').read_text(encoding='utf-8')
    style = (root / 'static' / 'map_editor.css').read_text(encoding='utf-8')
    assert "row.draggable = !state.draft" in script
    assert "function reorderWaypoint(from, to)" in script
    assert "event.dataTransfer.setData('text/plain', String(index))" in script
    assert "id=\"me-generate-path\"" in script
    assert "post('costmap-path'" in script
    assert "state.showGeneratedPath ? '#169b62'" in script
    assert '.me-waypoint-handle' in style and '.me-waypoint-row.drop-target' in style


def test_map_editor_headers_are_themed_in_light_mode():
    root = Path(__file__).resolve().parents[1]
    style = (root / 'static' / 'map_editor.css').read_text(encoding='utf-8')
    assert '#page-map-editor header {' in style
    assert 'background: transparent;' in style
    assert '#page-map-editor .me-header {' in style
    page = (root / 'launcher.html').read_text(encoding='utf-8')
    assert 'map_editor.css?v=20261124-light' in page
