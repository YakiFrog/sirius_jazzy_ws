from pathlib import Path
import io
import sys

from flask import Flask
from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from file_browser import HostFiles, register_file_browser, TEXT_LIMIT


@pytest.fixture
def browser(tmp_path):
    home=tmp_path/'home';home.mkdir();workspace=home/'sirius_jazzy_ws';maps=workspace/'maps_waypoints';maps.mkdir(parents=True)
    (home/'rosbag2_data').mkdir();(maps/'map.pgm').write_bytes(b'P5\n3 2\n255\n'+bytes([0,128,255,255,128,0]))
    (maps/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\n')
    (maps/'binary.mcap').write_bytes(b'\0\1bag')
    app=Flask(__name__);files=HostFiles(home,workspace)
    register_file_browser(app,lambda:True,files)
    return app.test_client(),files,maps


def test_browse_pgm_and_text_without_modifying_source(browser):
    client,files,maps=browser
    listed=client.get('/api/files/list').json
    assert listed['path']==str(maps)
    assert {x['label'] for x in listed['bookmarks']} >= {'Map & Waypoint','ROSBAG'}
    original=(maps/'map.pgm').read_bytes()
    response=client.get('/api/files/preview',query_string={'path':str(maps/'map.pgm')})
    assert response.status_code==200 and response.mimetype=='image/png'
    image=Image.open(io.BytesIO(response.data))
    assert image.size==(3,2) and image.getpixel((0,0))==(0,0,0) and image.getpixel((2,0))==(255,255,255)
    assert (maps/'map.pgm').read_bytes()==original
    assert client.get('/api/files/text',query_string={'path':str(maps/'map.yaml')}).json['text'].startswith('image:')
    assert client.get('/api/files/text',query_string={'path':str(maps/'binary.mcap')}).status_code==400
    response=client.get('/api/files/download',query_string={'path':str(maps/'binary.mcap')})
    assert 'attachment' in response.headers['Content-Disposition']


def test_traversal_symlinks_and_special_files_are_rejected(browser,tmp_path):
    client,files,maps=browser
    outside=tmp_path/'secret.txt';outside.write_text('outside')
    (maps/'outside.txt').symlink_to(outside)
    for endpoint in ['text','preview','download']:
        assert client.get('/api/files/'+endpoint,query_string={'path':str(maps/'outside.txt')}).status_code in (400,403)
    assert client.get('/api/files/list',query_string={'path':str(files.home/'..')}).status_code==403
    assert 'outside.txt' not in {r['name'] for r in client.get('/api/files/list').json['entries']}
    assert client.get('/api/files/download',query_string={'path':str(maps)}).status_code==400


def test_favorites_persist_deduplicate_and_remove_deleted_folder(browser):
    client,files,maps=browser
    body={'action':'add','path':str(maps)}
    assert len(client.post('/api/files/favorites',json=body).json['favorites'])==1
    assert len(client.post('/api/files/favorites',json=body).json['favorites'])==1
    assert HostFiles(files.home,files.workspace).favorites()[0]['path']==str(maps)
    missing=maps/'temporary';missing.mkdir()
    client.post('/api/files/favorites',json={**body,'path':str(missing)})
    missing.rmdir()
    assert len(client.post('/api/files/favorites',json={'action':'remove','path':str(missing)}).json['favorites'])==1
    assert client.post('/api/files/favorites',json={**body,'path':str(maps/'map.pgm')}).status_code==400


def test_text_preview_bounded_and_auth_applies_to_all_routes(browser):
    client,files,maps=browser
    (maps/'large.log').write_bytes(b'a'*(TEXT_LIMIT+10))
    data=client.get('/api/files/text',query_string={'path':str(maps/'large.log')}).json
    assert data['truncated'] and len(data['text'])==TEXT_LIMIT
    app=Flask('locked');register_file_browser(app,lambda:False,files);locked=app.test_client()
    for endpoint in ['list','text','preview','download','favorites']:
        assert locked.get('/api/files/'+endpoint).status_code==401
    assert locked.post('/api/files/favorites',json={}).status_code==401
    assert locked.post('/api/files/text',json={}).status_code==401


def test_json_save_with_revision_and_original_backup(browser):
    client,files,maps=browser
    path=maps/'settings.json';original=b'{"speed": 0.1}\n';path.write_bytes(original)
    opened=client.get('/api/files/text',query_string={'path':str(path)}).json
    assert opened['editable'] and opened['revision']
    changed=client.post('/api/files/text',json=dict(path=str(path),revision=opened['revision'],text='{"speed": 0.2}\n'))
    assert changed.status_code==200 and path.read_text()=='{"speed": 0.2}\n'
    assert Path(changed.json['backup_path']).read_bytes()==original
    assert Path(changed.json['backup_path']).stat().st_mode&0o777==0o600
    assert changed.json['revision']!=opened['revision']
    stale=client.post('/api/files/text',json=dict(path=str(path),revision=opened['revision'],text='{}'))
    assert stale.status_code==409 and path.read_text()=='{"speed": 0.2}\n'


@pytest.mark.parametrize('text',['{broken}', '{"a":NaN}', '{"a":Infinity}', '"unterminated'])
def test_invalid_json_is_never_written(browser,text):
    client,files,maps=browser
    path=maps/'settings.json';path.write_text('{}')
    opened=files.text(str(path))
    result=client.post('/api/files/text',json=dict(path=str(path),revision=opened['revision'],text=text))
    assert result.status_code==400 and path.read_text()=='{}'


def test_external_change_is_reported_without_losing_it(browser):
    client,files,maps=browser;path=maps/'map.yaml'
    opened=files.text(str(path));path.write_text('new external configuration')
    result=client.post('/api/files/text',json=dict(path=str(path),revision=opened['revision'],text='overwrite'))
    assert result.status_code==409 and path.read_text()=='new external configuration'


def test_non_utf8_large_and_readonly_text_are_not_editable(browser,monkeypatch):
    client,files,maps=browser
    bad=maps/'bad.txt';bad.write_bytes(b'\xff')
    large=maps/'large.txt';large.write_bytes(b'a'*(TEXT_LIMIT+1))
    assert not files.text(str(bad))['editable']
    assert not files.text(str(large))['editable']
    monkeypatch.setattr('file_browser.os.access',lambda *_:False)
    assert not files.text(str(maps/'map.yaml'))['editable']


def test_save_keeps_script_mode_bom_and_crlf(browser):
    client,files,maps=browser
    path=maps/'script.sh';path.write_bytes(b'\xef\xbb\xbf#!/bin/sh\r\necho before\r\n');path.chmod(0o755)
    opened=files.text(str(path))
    saved=files.save_text(dict(path=str(path),revision=opened['revision'],text='#!/bin/sh\necho after\n'))
    assert saved['ok'] and path.read_bytes()==b'\xef\xbb\xbf#!/bin/sh\r\necho after\r\n'
    assert path.stat().st_mode&0o777==0o755


def test_failed_atomic_replace_leaves_original_and_no_temp_file(browser,monkeypatch):
    client,files,maps=browser;path=maps/'map.yaml';original=path.read_bytes()
    opened=files.text(str(path));names={p.name for p in maps.iterdir()}
    def fail(*args):raise OSError('write failed')
    monkeypatch.setattr('file_browser.os.replace',fail)
    with pytest.raises(OSError):files.save_text(dict(path=str(path),revision=opened['revision'],text='new'))
    assert path.read_bytes()==original and {p.name for p in maps.iterdir()}==names


def test_create_rename_move_and_favorites_follow(browser):
    client,files,maps=browser
    def operate(**body):
        return client.post('/api/files/operate',json=body)
    folder=maps/'new folder';child=folder/'child'
    assert operate(action='mkdir',parent=str(maps),name=folder.name).status_code==200
    child.mkdir();(child/'data.txt').write_text('keep')
    client.post('/api/files/favorites',json={'action':'add','path':str(child)})
    renamed=maps/'名前変更後'
    assert operate(action='rename',path=str(folder),name=renamed.name).status_code==200
    assert (renamed/'child/data.txt').read_text()=='keep'
    assert files.favorites()[0]['path']==str(renamed/'child')
    destination=files.home/'rosbag2_data'
    assert operate(action='move',path=str(renamed),destination=str(destination)).status_code==200
    assert (destination/renamed.name/'child/data.txt').read_text()=='keep'
    assert files.favorites()[0]['path']==str(destination/renamed.name/'child')


def test_operations_never_overwrite_or_move_into_descendants(browser):
    client,files,maps=browser
    source=maps/'map.yaml';original=source.read_bytes();target=maps/'existing.yaml';target.write_text('existing')
    response=client.post('/api/files/operate',json={'action':'rename','path':str(source),'name':target.name})
    assert response.status_code==409 and source.read_bytes()==original and target.read_text()=='existing'
    folder=maps/'parent';child=folder/'child';child.mkdir(parents=True)
    assert client.post('/api/files/operate',json={'action':'move','path':str(folder),'destination':str(child)}).status_code==400
    assert folder.is_dir()
    assert client.post('/api/files/operate',json={'action':'mkdir','parent':str(maps),'name':'../escape'}).status_code==400
    assert client.post('/api/files/operate',json={'action':'trash','path':str(files.home)}).status_code==403


def test_symlink_operations_affect_link_not_target(browser):
    client,files,maps=browser
    target=maps/'map.yaml';original=target.read_bytes();link=maps/'alias.yaml';link.symlink_to(target)
    listed=client.get('/api/files/list').json['entries']
    assert next(row for row in listed if row['name']==link.name)['path']==str(link)
    assert client.post('/api/files/operate',json={'action':'rename','path':str(link),'name':'renamed-link.yaml'}).status_code==200
    renamed=maps/'renamed-link.yaml'
    response=client.post('/api/files/operate',json={'action':'trash','path':str(renamed)})
    assert response.status_code==200 and not renamed.is_symlink() and target.read_bytes()==original
    assert client.post('/api/files/restore',json={'id':response.json['trash_id']}).status_code==200
    assert renamed.is_symlink() and target.read_bytes()==original


def test_trash_persistence_restore_and_collision(browser):
    client,files,maps=browser
    folder=maps/'delete me';folder.mkdir();(folder/'data.txt').write_text('valuable')
    result=client.post('/api/files/operate',json={'action':'trash','path':str(folder)})
    assert result.status_code==200 and not folder.exists()
    token=result.json['trash_id']
    assert HostFiles(files.home,files.workspace).trash_items()[0]['original']==str(folder)
    folder.mkdir()
    assert client.post('/api/files/restore',json={'id':token}).status_code==409
    assert (files.trash/'files'/token/'data.txt').read_text()=='valuable'
    folder.rmdir()
    assert client.post('/api/files/restore',json={'id':token}).status_code==200
    assert (folder/'data.txt').read_text()=='valuable'
    assert client.get('/api/files/trash').json['entries']==[]
    assert client.post('/api/files/restore',json={'id':'../outside'}).status_code==400


def test_mutation_auth_and_external_destinations(browser,tmp_path):
    client,files,maps=browser
    assert client.post('/api/files/operate',json={'action':'move','path':str(maps/'map.yaml'),'destination':str(tmp_path)}).status_code==403
    assert (maps/'map.yaml').is_file()
    app=Flask('locked operations');register_file_browser(app,lambda:False,files);locked=app.test_client()
    assert locked.post('/api/files/operate',json={}).status_code==401
    assert locked.get('/api/files/trash').status_code==401
    assert locked.post('/api/files/restore',json={}).status_code==401


def test_batch_move_folders_preserves_contents_and_favorites(browser):
    client,files,maps=browser
    paths=[maps/'folder_a',maps/'folder_b']
    for path in paths:
        path.mkdir();(path/'data.txt').write_text(path.name)
    client.post('/api/files/favorites',json={'action':'add','path':str(paths[1])})
    dest=files.home/'rosbag2_data'
    result=client.post('/api/files/operate',json={'action':'move_many','paths':list(map(str,paths)),'destination':str(dest)})
    assert result.status_code==200 and not result.json['partial'] and len(result.json['moved'])==2
    assert all(not path.exists() and (dest/path.name/'data.txt').read_text()==path.name for path in paths)
    assert files.favorites()[0]['path']==str(dest/'folder_b')


def test_batch_preflight_rejects_collision_without_moving_anything(browser):
    client,files,maps=browser
    a,b=maps/'a',maps/'b';a.mkdir();b.mkdir()
    destination=files.home/'rosbag2_data';(destination/'b').mkdir()
    result=client.post('/api/files/operate',json={'action':'move_many','paths':[str(a),str(b)],'destination':str(destination)})
    assert result.status_code==409 and a.exists() and b.exists() and not (destination/'a').exists()
    child=a/'child';child.mkdir()
    for paths in ([str(a),str(child)],[str(a),str(a)]):
        assert client.post('/api/files/operate',json={'action':'move_many','paths':paths,'destination':str(destination)}).status_code==400
    assert client.post('/api/files/operate',json={'action':'move_many','paths':[str(a)],'destination':str(child)}).status_code==400


def test_batch_mid_move_failure_reports_moved_and_remaining(browser,monkeypatch):
    client,files,maps=browser
    paths=[maps/name for name in ['one','two','three']]
    for path in paths:path.mkdir()
    actual=files.rename_exclusive
    def fail_second(source,target):
        if source==paths[1]:raise PermissionError('test permission change')
        return actual(source,target)
    monkeypatch.setattr(files,'rename_exclusive',fail_second)
    destination=files.home/'rosbag2_data'
    result=client.post('/api/files/operate',json={'action':'move_many','paths':list(map(str,paths)),'destination':str(destination)}).json
    assert result['partial'] and [row['source'] for row in result['moved']]==[str(paths[0])]
    assert result['remaining']==list(map(str,paths[1:]))
    assert not paths[0].exists() and (destination/'one').exists() and paths[1].exists() and paths[2].exists()
