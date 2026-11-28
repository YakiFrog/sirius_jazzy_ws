"""Nav2 lifecycle jobs and opt-in automatic activation."""
import json
import logging
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
import uuid
from urllib.parse import urlsplit

from flask import Blueprint,jsonify,request
from lifecycle_protocol import ACTIONS, NODES, REQUIRED_NODES, validate_command


AUTO_WAIT_SECONDS = 5
AUTO_ACTIVATION_ORDER = (
    'map_server', 'amcl', 'planner_server', 'controller_server',
    'global_costmap/global_costmap', 'local_costmap/local_costmap',
    'smoother_server', 'behavior_server', 'velocity_smoother',
    'collision_monitor', 'bt_navigator',
)
COSTMAP_PARENTS = {'global_costmap/global_costmap': 'planner_server',
                   'local_costmap/local_costmap': 'controller_server'}


class LifecycleService:
    def __init__(self):
        self.lock = threading.Lock()
        self.operations = []
        self.auto_enabled = threading.Event()
        self.auto_thread = None
        self.auto_attempted = set()

    def snapshot(self):
        with self.lock:return [dict(row) for row in self.operations]

    def start(self,data,domain):
        command=validate_command(data)
        if not str(domain).isdigit() or not 0<=int(domain)<=232:raise ValueError('ROS domainが不正です')
        with self.lock:
            if any(o['status']=='running' for o in self.operations):raise RuntimeError('別のライフサイクル操作を実行中です')
            if command['action'] in ('deactivate', 'cleanup'):
                self.auto_enabled.clear()
            return self._start_locked(command, domain, 'manual')

    def _start_locked(self, command, domain, source):
        job = {**command, 'id': uuid.uuid4().hex, 'domain': str(domain), 'source': source,
               'status': 'running', 'started': time.time(), 'result': None}
        self.operations.insert(0, job)
        del self.operations[20:]
        threading.Thread(target=self._run, args=(job,), daemon=True).start()
        return dict(job)

    def auto_status(self):
        return {'enabled': self.auto_enabled.is_set(), 'wait_seconds': AUTO_WAIT_SECONDS}

    def set_auto_mode(self, enabled, snapshot, domain):
        with self.lock:
            if enabled and not self.auto_enabled.is_set():
                self.auto_attempted.clear()
                self.auto_enabled.set()
                if self.auto_thread is None:
                    self.auto_thread = threading.Thread(target=self._watch, args=(snapshot, domain),
                                                        daemon=True, name='sirius-lifecycle-auto')
                    self.auto_thread.start()
            elif not enabled:
                self.auto_enabled.clear()
            return self.auto_status()

    def _watch(self, snapshot, domain):
        while True:
            self.auto_enabled.wait()
            try:
                self.poll_auto(snapshot(), str(domain()))
            except Exception:
                logging.getLogger(__name__).exception('Automatic lifecycle monitoring failed')
            time.sleep(1)

    def poll_auto(self, data, domain):
        if not data.get('available') or str(data.get('domain')) != domain:
            return
        rows = data.get('navigation', {}).get('lifecycle', {}).get('nodes', {})
        states = {name: (row.get('value') or {}).get('id') for name, row in rows.items() if row.get('fresh')}
        with self.lock:
            if not self.auto_enabled.is_set() or any(job['status'] == 'running' for job in self.operations):
                return
            self.auto_attempted.difference_update(name for name, state in states.items() if state == 3)
            if any(states.get(name) not in (2, 3) for name in REQUIRED_NODES):
                return
            for name in AUTO_ACTIVATION_ORDER:
                parent = COSTMAP_PARENTS.get(name)
                if states[name] != 2 or name in self.auto_attempted or (parent and states[parent] != 3):
                    continue
                if rows[name].get('observed_seconds', 0) < AUTO_WAIT_SECONDS:
                    return
                self.auto_attempted.add(name)
                return self._start_locked({'node': name, 'action': 'activate', 'expected_state': 2}, domain, 'automatic')

    def _run(self,job):
        root=Path(__file__).resolve().parent;workspace=root.parents[1]
        shell='source /opt/ros/jazzy/setup.bash && source '+shlex.quote(str(workspace/'install/setup.bash'))+' && exec python3 '+shlex.quote(str(root/'lifecycle_worker.py'))
        try:
            result=subprocess.run(['bash','-c',shell],input=json.dumps({k:job[k] for k in ('node','action','expected_state')}),
                capture_output=True,text=True,timeout=115,env={**os.environ,'ROS_DOMAIN_ID':job['domain']})
            payload=json.loads(result.stdout.strip().splitlines()[-1])
        except Exception as error:
            payload={'ok':False,'uncertain':True,'error':str(error)+'。自動再送はしません。実機状態を再確認してください'}
        with self.lock:
            job.update(status='done' if payload.get('ok') else 'failed',result=payload,finished=time.time())
            if job.get('source') == 'automatic' and job['status'] == 'failed':
                self.auto_enabled.clear()


def register_lifecycle(app,auth_ok,domain,snapshot,service=None):
    service=service or LifecycleService();bp=Blueprint('nav2_lifecycle',__name__)

    @bp.before_request
    def authorize():
        if not auth_ok():
            return jsonify(error='auth'),401
        if request.method == 'POST':
            origin = request.headers.get('Origin')
            if (origin and urlsplit(origin).netloc != request.host) or request.headers.get('Sec-Fetch-Site') == 'cross-site':
                return jsonify(error='同じLauncher画面から操作してください'),403

    @bp.get('/api/lifecycle')
    def state():
        data=snapshot();telemetry=data.get('navigation',{}).get('lifecycle',{})
        return jsonify(available=data.get('available',False),domain=str(domain()),
                       nodes=NODES,actions={k:list(v) for k,v in ACTIONS.items()},
                       required_nodes=REQUIRED_NODES, telemetry=telemetry,
                       operations=service.snapshot(), auto_activation=service.auto_status())

    @bp.post('/api/lifecycle/auto-activation')
    def auto_activation():
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or type(body.get('enabled')) is not bool:
            return jsonify(error='自動有効化のON・OFFを指定してください'),400
        return jsonify(auto_activation=service.set_auto_mode(body['enabled'], snapshot, domain))

    @bp.post('/api/lifecycle/transition')
    def transition():
        try:return jsonify(operation=service.start(request.get_json(silent=True),domain())),202
        except ValueError as error:return jsonify(error=str(error)),400
        except RuntimeError as error:return jsonify(error=str(error)),409

    app.register_blueprint(bp)
