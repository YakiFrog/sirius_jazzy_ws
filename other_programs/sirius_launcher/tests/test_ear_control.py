import ast
import importlib.util
from pathlib import Path
import sys

from flask import Flask
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from ear_control import PATTERNS,register_ear
spec=importlib.util.spec_from_file_location('ear_patterns_test',ROOT.parents[1]/'src/sirius/sirius_navigation/sirius_navigation/ear_led_patterns.py')
patterns=importlib.util.module_from_spec(spec);spec.loader.exec_module(patterns)


def client(authorized=True):
    class Bridge:
        def __init__(self):self.calls=[]
        def ensure(self,domain):self.calls.append(('ensure',domain));return True
        def status(self):return dict(gateway_available=True,ready=True)
        def command(self,command,domain):self.calls.append((command,domain));return dict(ok=True)
    bridge=Bridge();app=Flask(__name__);register_ear(app,lambda:authorized,lambda:'53',bridge)
    return app.test_client(),bridge


def test_auth_guards_state_and_commands():
    c,b=client(False)
    assert c.get('/api/ear/status').status_code==401
    assert c.post('/api/ear/test',json=dict(pattern='left')).status_code==401
    assert b.calls==[]


def test_presets_are_consistent_and_only_led_fields_are_forwarded():
    assert set(PATTERNS)==patterns.PATTERNS
    c,b=client()
    state=c.get('/api/ear/status');assert state.json['ready']
    assert state.headers['Cache-Control']=='no-store'
    result=c.post('/api/ear/test',json=dict(pattern='emergency',duration=5,domain='53',active=True,action='stop'))
    assert result.status_code==200
    assert b.calls[-1]==(dict(pattern='emergency',duration=5,color=[0,160,255]),'53')


@pytest.mark.parametrize('body',[{'pattern':'arbitrary'},{'pattern':[]},{'pattern':'left','duration':True},
    {'pattern':'blink','duration':31},{'pattern':'solid','color':[999,0,0]},
    {'pattern':'left','domain':'53;bad'}, {'pattern':'left','duration':float('nan')}])
def test_invalid_commands_do_not_reach_gateway(body):
    c,b=client();assert c.post('/api/ear/test',json=body).status_code==400
    assert b.calls==[]


def test_timed_patterns_match_pos_and_expire():
    for pattern,color in [('left','C:0,255,0'),('turning','C:255,255,0'),('emergency','C:255,0,0')]:
        t=patterns.make_test(dict(pattern=pattern,duration=5),100)
        first=patterns.output(t,100)
        assert first[0]==color
        assert patterns.output(t,105) is None
    t=patterns.make_test(dict(pattern='right'),100)
    assert patterns.output(t,100)[:2]==('C:0,0,0','C:0,255,0')
    assert patterns.output(t,100.6)[:2]==('C:0,0,0','C:0,0,0')
    assert patterns.make_test(dict(pattern='auto'),100) is None


def test_animation_commands_always_have_valid_rgb_values():
    for name in ['solid','blink','alternate','breathe','rainbow']:
        t=patterns.make_test(dict(pattern=name,duration=5,color=[255,160,1]),100)
        for now in [100,100.6,101,102.5,104.9]:
            for command in patterns.output(t,now)[:2]:
                assert command.startswith('C:')
                assert all(0<=int(v)<=255 for v in command[2:].split(','))


def test_ros_worker_can_only_publish_to_the_ear_test_topic():
    tree=ast.parse((ROOT/'ear_worker.py').read_text())
    topics={node.args[1].value for node in ast.walk(tree) if isinstance(node,ast.Call)
            and isinstance(node.func,ast.Attribute) and node.func.attr=='create_publisher'}
    assert topics=={'/sirius/ear_led_test'}
