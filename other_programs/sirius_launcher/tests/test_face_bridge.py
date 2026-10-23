import sys
from pathlib import Path
from types import SimpleNamespace

from flask import Flask
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from face_bridge import FaceBridge, FaceError, register_face


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    desktop=tmp_path/'Desktop';desktop.mkdir()
    (desktop/'sirius_face.desktop').write_text(
        '[Desktop Entry]\nType=Application\nName=Sirius Face UI\n'
        'Exec=/robot/face/dist/robot.AppImage --no-sandbox\nStartupWMClass=sirius-face-2\n')
    instance=FaceBridge(tmp_path)
    monkeypatch.setattr(instance,'windows',lambda _:[])
    monkeypatch.setattr(instance,'launcher_windows',lambda:[])
    monkeypatch.setattr(instance,'processes',lambda _:[])
    monkeypatch.setattr(instance,'listening',lambda _:False)
    return instance


def client(bridge, authenticated=True):
    app=Flask(__name__);app.config['TESTING']=True
    register_face(app,lambda:authenticated,bridge)
    return app.test_client()


def test_auth_guards_read_and_control(bridge,monkeypatch):
    monkeypatch.setattr(bridge,'status',lambda:pytest.fail('Unauthenticated status read'))
    c=client(bridge,False)
    assert c.get('/api/face/status').status_code==401
    assert c.post('/api/face/action',json={'action':'launch'}).status_code==401


def test_status_is_read_only_cached_and_recovers_from_failure(bridge,monkeypatch):
    calls=[]
    monkeypatch.setattr(bridge,'listening',lambda _:True)
    monkeypatch.setattr(bridge,'rpc',lambda p:(calls.append(p) or dict(connected=True,expression='normal')))
    monkeypatch.setattr('face_bridge.subprocess.Popen',lambda *_a,**_k:pytest.fail('GET must never launch Face'))
    c=client(bridge)
    assert c.get('/api/face/status').json['connected']
    assert c.get('/api/face/status').json['expression']=='normal'
    assert calls==[{'action':'status'}]
    monkeypatch.setattr(bridge,'rpc',lambda _:(_ for _ in ()).throw(FaceError('offline')))
    assert not bridge.status(force=True)['connected']
    monkeypatch.setattr(bridge,'rpc',lambda _:dict(connected=True,expression='happy'))
    assert bridge.status(force=True)['connected']


def test_desktop_shortcut_is_used_and_repeated_launch_is_suppressed(bridge,monkeypatch):
    calls=[]
    monkeypatch.setattr(bridge,'gui_env',lambda:{'DISPLAY':':0'})
    monkeypatch.setattr('face_bridge.shutil.which',lambda name:'/usr/bin/'+name)
    monkeypatch.setattr('face_bridge.subprocess.Popen',lambda args,**kw:calls.append((args,kw)))
    c=client(bridge)
    assert c.post('/api/face/action',json={'action':'launch'}).json['starting']
    assert c.post('/api/face/action',json={'action':'launch'}).json['already_running']
    assert len(calls)==1
    assert calls[0][0]==['gio','launch',str(bridge.home/'Desktop/sirius_face.desktop')]
    assert calls[0][1]['start_new_session']


@pytest.mark.parametrize('connected,running',[(True,False),(False,True)])
def test_existing_face_is_reused_without_launch(bridge,monkeypatch,connected,running):
    monkeypatch.setattr(bridge,'status',lambda:dict(connected=connected,running=running,starting=False))
    monkeypatch.setattr('face_bridge.subprocess.Popen',lambda *_a,**_k:pytest.fail('Duplicate Face launch'))
    assert bridge.action({'action':'launch'})['already_running']


@pytest.mark.parametrize('payload',[
    {'action':'arbitrary_rpc'}, {'action':'expression','expression':'not-an-expression'},
    {'action':'gaze','direction':'unbounded'}, {'action':'speak','text':'[nav]進め'},
    {'action':'speak','text':'[llm]質問'}, {'action':'subtitle','text':'x'*141},
    {'action':'speak','text':''}, ['launch'],
])
def test_invalid_values_and_navigation_tags_never_reach_face(bridge,monkeypatch,payload):
    monkeypatch.setattr(bridge,'rpc',lambda _:pytest.fail('Invalid payload sent to Face'))
    assert client(bridge).post('/api/face/action',json=payload).status_code==400


def test_allowed_commands_map_to_only_the_expected_face_rpc(bridge,monkeypatch):
    calls=[]
    monkeypatch.setattr(bridge,'status',lambda:dict(connected=True,running=True,starting=False))
    monkeypatch.setattr(bridge,'windows',lambda _:[dict(id='0x123',width=1920,height=1080)])
    monkeypatch.setattr(bridge,'rpc',lambda p:(calls.append(p) or {'ok':True}))
    c=client(bridge)
    for payload in [dict(action='expression',expression='happy'),dict(action='gaze',direction='up'),
                    dict(action='subtitle',text='テスト'),dict(action='speak',text='こんにちは'),dict(action='clear_text')]:
        assert c.post('/api/face/action',json=payload).json['ok']
    assert calls==[dict(action='expression',expression='happy'),dict(action='gaze',x=960,y=162),
                   dict(action='subtitle',text='テスト'),dict(action='speak',text='こんにちは'),dict(action='clear_text')]


def test_disconnected_face_does_not_accept_controls(bridge):
    c=client(bridge)
    assert c.get('/api/face/status').status_code==200
    assert c.post('/api/face/action',json=dict(action='expression',expression='normal')).status_code==503


@pytest.mark.parametrize('expression', ['battery', 'failure', 'hurt'])
def test_additional_display_modes_are_forwarded_to_face(bridge, monkeypatch, expression):
    calls=[]
    monkeypatch.setattr(bridge,'status',lambda:dict(connected=True,running=True,starting=False))
    monkeypatch.setattr(bridge,'rpc',lambda payload:(calls.append(payload) or {'ok':True}))
    response=client(bridge).post('/api/face/action',json=dict(action='expression',expression=expression))
    assert response.status_code==200 and response.json['ok']
    assert calls==[dict(action='expression',expression=expression)]


def test_worker_sends_typed_grpc_and_blocks_control_tags(monkeypatch):
    import face_worker
    calls=[]
    class Channel:
        def __enter__(self):return self
        def __exit__(self,*args):pass
    class Stub:
        def GetStatus(self,request,**kwargs):return SimpleNamespace(face_state='normal',eye_style='standard',mouth_style='standard')
        def SetExpression(self,request,**kwargs):calls.append(('expression',request.state,kwargs['timeout']))
        def Speak(self,request,**kwargs):calls.append(('speak',request.text,kwargs['timeout']))
    monkeypatch.setitem(sys.modules,'grpc',SimpleNamespace(insecure_channel=lambda _:Channel()))
    monkeypatch.setitem(sys.modules,'face_control_pb2',SimpleNamespace(ExpressionRequest=SimpleNamespace,SpeakRequest=SimpleNamespace))
    monkeypatch.setitem(sys.modules,'face_control_pb2_grpc',SimpleNamespace(FaceServiceStub=lambda _:Stub(),PythonControlServiceStub=lambda _:Stub()))
    assert face_worker.call('/face',dict(action='expression',expression='happy'))['ok']
    assert face_worker.call('/face',dict(action='speak',text='こんにちは'))['ok']
    with pytest.raises(ValueError):face_worker.call('/face',dict(action='speak',text='[nav]進め'))
    assert calls==[('expression','happy',1.5),('speak','こんにちは',2.5)]
    result=face_worker.call('/face',dict(action='status'))
    assert result['connected'] and result['speech_ready']
    assert calls[-1]==('speak','[ping]',.4)  # health probe produces no speech
