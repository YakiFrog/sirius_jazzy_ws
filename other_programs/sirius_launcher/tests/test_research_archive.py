import json
from pathlib import Path
import sys

from flask import Flask
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from research_archive import ResearchArchive, register_research_archive


@pytest.fixture
def archive(tmp_path):
    first, second = tmp_path/'first', tmp_path/'second'
    first.mkdir()
    second.mkdir()
    (first/'index.html').write_text('<body><a href="../second/result.html">比較</a><img src="image.png"></body>')
    (first/'image.png').write_bytes(b'png')
    (second/'result.html').write_text('<body>結果</body>')
    catalogue = tmp_path/'catalogue.json'
    catalogue.write_text(json.dumps(dict(
        categories=[dict(id='road', label='路面')],
        bundles=dict(first=str(first), second=str(second)),
        aliases={'/old/': 'second'},
        entries=[dict(id='example', bundle='first', page='index.html',
                      image='image.png', date='2026-10-23', category='road', title='地図')]
    )))
    return ResearchArchive(catalogue, workspace=tmp_path, home=tmp_path)


def client(archive, authenticated=True):
    app = Flask(__name__)
    app.config['TESTING'] = True
    register_research_archive(app, lambda: authenticated, archive)
    return app.test_client()


def test_catalogue_has_only_public_links_and_handles_missing_output(archive):
    response = client(archive).get('/api/research')
    assert response.status_code == 200
    assert str(archive.home) not in response.text
    entry = response.json['entries'][0]
    assert entry['available']
    assert entry['image'] == '/research/assets/first/image.png'
    (archive.home/'first/index.html').unlink()
    assert archive.public()['entries'][0]['available'] is False


def test_auth_applies_to_catalogue_home_and_assets(archive):
    c = client(archive, False)
    assert c.get('/research/').status_code == 302
    assert c.get('/api/research').status_code == 401
    assert c.get('/research/assets/first/index.html').status_code == 401


def test_bundle_boundary_blocks_symlink_escape_traversal_and_recordings(archive):
    secret = archive.home/'private.json'
    secret.write_text('secret')
    (archive.home/'first/escape.json').symlink_to(secret)
    (archive.home/'first/recording.mcap').write_bytes(b'recording')
    c = client(archive)
    for path in ['escape.json', '../private.json', '%2e%2e/private.json', 'recording.mcap']:
        assert c.get('/research/assets/first/'+path).status_code == 404
    assert c.get('/research/assets/unknown/index.html').status_code == 404
    assert c.post('/research/assets/first/index.html').status_code == 405


def test_old_relative_links_remapped_without_changing_original(archive):
    c = client(archive)
    source = archive.home/'first/index.html'
    original = source.read_text()
    html = c.get('/research/assets/first/index.html').text
    assert 'href="/research/assets/second/result.html"' in html
    assert 'src="/research/assets/first/image.png"' in html
    assert 'href="/research/"' in html
    config, roots, _ = archive.load()
    alias = archive.rewrite_html(source, '<a href="/old/result.html#floor">比較</a>', config, roots)
    assert '/research/assets/second/result.html#floor' in alias
    assert source.read_text() == original


def test_reproduction_script_is_served_as_text_without_executing(archive):
    (archive.home/'first/replay.py').write_text('raise RuntimeError("never execute")')
    response = client(archive).get('/research/assets/first/replay.py')
    assert response.status_code == 200
    assert response.mimetype == 'text/plain'
    assert response.headers['X-Content-Type-Options'] == 'nosniff'


def test_only_complete_maps_are_discovered_and_deduplicated(archive):
    maps = archive.workspace/'maps_waypoints/maps/road_semantic'
    maps.mkdir(parents=True)
    for index, status in enumerate(['complete', 'running']):
        folder = maps/f'theta_20261023_12000{index}'
        folder.mkdir()
        (folder/'manifest.json').write_text(json.dumps(dict(status=status, frame_count=123)))
        (folder/'source_bag.json').write_text(json.dumps(dict(created_at='2026-10-23T12:00:00', source_bag_name='test_bag')))
        (folder/'completed.json').write_text('{}')
        (folder/'images.html').write_text('<body>地図</body>')
    (maps/'theta_20261023_120002').symlink_to(archive.home/'first')
    rows = archive.public()['entries']
    assert len(rows) == 2
    assert any(row.get('recording') == 'test_bag' for row in rows)
    config = json.loads(archive.catalogue.read_text())
    config['bundles']['curated'] = str(maps/'theta_20261023_120000')
    config['entries'].append(dict(id='curated', bundle='curated', page='images.html',
                                  date='2026-10-23', category='road'))
    archive.catalogue.write_text(json.dumps(config))
    assert len(archive.public()['entries']) == 2
