from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from flask import Flask
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hotspot_status import HotspotMonitor, register_hotspot


@pytest.fixture
def monitor(monkeypatch):
    monitor=HotspotMonitor()
    def run(*args):
        assert '--show-secrets' not in args
        if 'connection' in args:
            return SimpleNamespace(returncode=0, stdout='wlx123\nyes\nap\nSiriusNUC\n')
        return SimpleNamespace(returncode=0, stdout='GENERAL.STATE:100 (connected)\n'
            'GENERAL.CONNECTION:Sirius-Hotspot\nIP4.ADDRESS[1]:10.42.0.1/24\n')
    monkeypatch.setattr(monitor,'run',run)
    return monitor


def test_reports_only_a_connected_ap_with_actual_address(monitor):
    assert monitor.status()==dict(active=True,state='active',ssid='SiriusNUC',
        interface='wlx123',address='10.42.0.1',autostart=True)


@pytest.mark.parametrize('device',[
    'GENERAL.STATE:30 (disconnected)\nGENERAL.CONNECTION:--\nIP4.ADDRESS[1]:10.42.0.1/24\n',
    'GENERAL.STATE:100 (connected)\nGENERAL.CONNECTION:CampusWiFi\nIP4.ADDRESS[1]:172.22.0.1/18\n',
    'GENERAL.STATE:100 (connected)\nGENERAL.CONNECTION:Sirius-Hotspot\n',
])
def test_disconnection_or_other_network_does_not_keep_old_active_state(monitor,monkeypatch,device):
    assert monitor.status()['active']
    original=monitor.run
    monkeypatch.setattr(monitor,'run',lambda *args:original(*args) if 'connection' in args
                        else SimpleNamespace(returncode=0,stdout=device))
    monitor.cached_at=-100
    state=monitor.status()
    assert not state['active'] and state['address'] is None


def test_missing_device_retains_profile_but_is_inactive(monitor,monkeypatch):
    original=monitor.run
    monkeypatch.setattr(monitor,'run',lambda *args:original(*args) if 'connection' in args
                        else SimpleNamespace(returncode=10,stdout=''))
    assert monitor.status()['state']=='unavailable'
    assert not monitor.status()['active']


def test_timeout_clears_old_active_state_and_status_is_cached(monitor,monkeypatch):
    assert monitor.status()['active']
    calls=[]
    def fail(*args):
        calls.append(args)
        raise subprocess.TimeoutExpired('nmcli',1.5)
    monkeypatch.setattr(monitor,'run',fail)
    assert monitor.status()['active'] and calls==[]
    monitor.cached_at=-100
    assert monitor.status()['state']=='error'
    assert not monitor.status()['active'] and len(calls)==1


def test_unknown_profile_is_not_an_active_ap(monitor,monkeypatch):
    monkeypatch.setattr(monitor,'run',lambda *args:SimpleNamespace(returncode=10,stdout=''))
    assert monitor.status()['state']=='unconfigured'


def test_auth_and_local_url_do_not_follow_the_remote_browser_hostname(monitor):
    app=Flask(__name__)
    register_hotspot(app,lambda:False,monitor)
    assert app.test_client().get('/api/hotspot/status').status_code==401
    app=Flask(__name__);app.config['SIRIUS_LISTEN_PORT']=8095
    register_hotspot(app,lambda:True,monitor)
    response=app.test_client().get('/api/hotspot/status',base_url='https://example.ts.net')
    assert response.json['url']=='http://10.42.0.1:8095/console'
    assert response.headers['Cache-Control']=='no-store'
    assert not any('password' in key or 'psk' in key for key in response.json)
