"""Manual Nav2 lifecycle jobs, separate from the read-only state receiver."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
import uuid
from urllib.parse import urlsplit

from flask import Blueprint,jsonify,request
from lifecycle_protocol import ACTIONS,NODES,validate_command


class LifecycleService:
    def __init__(self):
        self.lock=threading.Lock();self.operations=[]

    def snapshot(self):
        with self.lock:return [dict(row) for row in self.operations]

    def start(self,data,domain):
        command=validate_command(data)
        if not str(domain).isdigit() or not 0<=int(domain)<=232:raise ValueError('ROS domainが不正です')
        with self.lock:
            if any(o['status']=='running' for o in self.operations):raise RuntimeError('別のライフサイクル操作を実行中です')
            job={**command,'id':uuid.uuid4().hex,'domain':str(domain),'status':'running','started':time.time(),'result':None}
            self.operations.insert(0,job);self.operations=self.operations[:20]
            response=dict(job)
        threading.Thread(target=self._run,args=(job,),daemon=True).start()
        return response

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


def register_lifecycle(app,auth_ok,domain,snapshot,service=None):
    service=service or LifecycleService();bp=Blueprint('nav2_lifecycle',__name__)

    @bp.get('/api/lifecycle')
    def state():
        if not auth_ok():return jsonify(error='auth'),401
        data=snapshot();telemetry=data.get('navigation',{}).get('lifecycle',{})
        return jsonify(available=data.get('available',False),domain=str(domain()),
                       nodes=NODES,actions={k:list(v) for k,v in ACTIONS.items()},
                       telemetry=telemetry,operations=service.snapshot())

    @bp.post('/api/lifecycle/transition')
    def transition():
        if not auth_ok():return jsonify(error='auth'),401
        origin=request.headers.get('Origin')
        if (origin and urlsplit(origin).netloc!=request.host) or request.headers.get('Sec-Fetch-Site')=='cross-site':
            return jsonify(error='同じLauncher画面から操作してください'),403
        try:return jsonify(operation=service.start(request.get_json(silent=True),domain())),202
        except ValueError as error:return jsonify(error=str(error)),400
        except RuntimeError as error:return jsonify(error=str(error)),409

    app.register_blueprint(bp)
