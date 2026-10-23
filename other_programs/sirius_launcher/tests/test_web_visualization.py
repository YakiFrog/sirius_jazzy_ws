import importlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def test_visualization_config_derives_browser_reachable_defaults():
    web = importlib.import_module('web_server')
    client = web.app.test_client()
    response = client.get('/api/visualization/config', base_url='http://100.107.5.14:8090')
    assert response.status_code == 200
    data = response.get_json()
    assert data['viewer_url'] == 'http://100.107.5.14:8091/'
    assert data['bridge_url'] == 'ws://100.107.5.14:8766'
    assert data['layout_url'] == 'http://100.107.5.14:8090/api/visualization/layout'


def test_visualization_layout_is_embeddable_json_with_safe_publish_topics():
    web = importlib.import_module('web_server')
    client = web.app.test_client()
    response = client.get('/api/visualization/layout')
    assert response.status_code == 200
    assert response.mimetype == 'application/json'
    assert response.headers['Access-Control-Allow-Origin'] == '*'
    assert response.headers['Cross-Origin-Resource-Policy'] == 'cross-origin'
    assert response.headers['Cache-Control'] == 'no-store'
    layout = json.loads(response.get_data(as_text=True))
    panel = layout['configById']['3D!sirius']
    assert layout['layout'] == '3D!sirius'
    assert panel['followTf'] == 'map'
    for topic in ('/velodyne_points', '/scan', '/map', '/plan', '/local_plan', '/sirius_markers'):
        assert topic in panel['topics']
    assert panel['topics']['/localization/map']['visible'] is False
    publish = panel['publish']
    assert publish['poseTopic'].startswith('/sirius_visualization/')
    assert publish['pointTopic'].startswith('/sirius_visualization/')
    assert publish['poseEstimateTopic'] == '/initialpose'


def test_visualization_page_and_scripts_are_wired():
    web = importlib.import_module('web_server')
    assert 'id="page-visualization"' in web.PAGE
    assert 'data-page="visualization"' in web.PAGE
    assert 'src="/static/visualization.js"' in web.PAGE
    assert 'id="viz-layout-select"' in web.PAGE
    assert 'id="restart-all-btn"' in web.PAGE
    assert 'id="restart-restore-inputs"' in web.PAGE
    assert 'function restartAll()' in web.PAGE
    assert 'sirius-restart-restore' in web.PAGE
    assert "PAGE_LABELS" in (ROOT/'static/console.js').read_text(encoding='utf-8')
    assert 'visualizationPageChanged' in (ROOT/'static/console.js').read_text(encoding='utf-8')


def test_named_layouts_can_be_listed_imported_served_and_deleted(tmp_path, monkeypatch):
    web = importlib.import_module('web_server')
    monkeypatch.setattr(web, 'VISUALIZATION_LAYOUTS_DIR', tmp_path)
    client = web.app.test_client()
    base = 'http://100.107.5.14:8090'
    layout = {'layout': '3D!custom', 'configById': {'3D!custom': {'layers': {}}}}

    response = client.post('/api/visualization/layouts',
                           json={'name': '現場A', 'layout': layout}, base_url=base)
    assert response.status_code == 200
    entry = response.get_json()['layout']
    assert entry['name'] == '現場A'
    assert entry['builtin'] is False
    assert entry['url'].endswith('/api/visualization/layouts/' + entry['id'])

    response = client.get('/api/visualization/layouts', base_url=base)
    assert response.status_code == 200
    data = response.get_json()
    assert data['default'] == 'default'
    assert data['layouts'][0]['id'] == 'default'
    assert any(item['name'] == '現場A' for item in data['layouts'])

    response = client.get('/api/visualization/layouts/' + entry['id'])
    assert response.status_code == 200
    assert response.headers['Access-Control-Allow-Origin'] == '*'
    assert json.loads(response.get_data(as_text=True)) == layout

    response = client.delete('/api/visualization/layouts/' + entry['id'], base_url=base)
    assert response.status_code == 200
    assert client.get('/api/visualization/layouts/' + entry['id']).status_code == 404
    assert client.delete('/api/visualization/layouts/default', base_url=base).status_code == 400


def test_visualization_auto_refresh_keeps_saved_layout():
    web = importlib.import_module('web_server')
    script = (ROOT/'static/visualization.js').read_text(encoding='utf-8')
    assert 'id="viz-refresh"' in web.PAGE
    assert '<option value="0">オフ</option>' in web.PAGE
    assert '<option value="60" selected>1時間</option>' in web.PAGE
    assert "refresh:'sirius-viz-refresh-minutes'" in script
    assert 'const VIZ_REFRESH_DEFAULT_MINUTES=60;' in script
    assert 'function scheduleVizRefresh()' in script
    assert "setVizFrameState('自動更新中','loading')" in script
    assert 'loadVisualizationFrame(true)' in script
    assert 'layoutUrl.searchParams.set' in script


def test_viewer_bootstrap_and_compose_are_pinned():
    viewer = ROOT/'viewer'
    bootstrap = (viewer/'sirius-start.html').read_text(encoding='utf-8')
    compose = (viewer/'docker-compose.yml').read_text(encoding='utf-8')
    assert "sirius:viz-workspace-ready" in bootstrap
    assert 'open:false' in bootstrap.replace(' ', '').replace('\n', '')
    assert 'ghcr.io/lichtblick-suite/lichtblick@sha256:' in compose
    assert './sirius-start.html:/src/sirius-start.html:ro' in compose
    assert '"8091:8080"' in compose
