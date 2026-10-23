"""Persistent ROS helper and acknowledged commands; no ROS dependency in Flask."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
import time
import uuid


class TeleopBridge:
    def __init__(self, script, log_path):
        self.script, self.log_path = script, log_path
        self.lock = threading.RLock()
        self.condition = threading.Condition()
        self.proc = None
        self.domain = None
        self.state = {}
        self.updated = 0.
        self.replies = {}
        self.error = ''

    def _read(self, process):
        for line in process.stdout:
            try:
                data = json.loads(line)
            except ValueError:
                continue
            with self.condition:
                if process is not self.proc:
                    return
                self.state, self.updated = data, time.monotonic()
                if data.get('request_id'):
                    self.replies[data['request_id']] = data
                    if len(self.replies) > 128:
                        self.replies.pop(next(iter(self.replies)))
                self.condition.notify_all()

    def _spawn(self, domain):
        if not str(domain).isdigit() or not 0 <= int(domain) <= 232:
            raise ValueError('invalid ROS domain')
        if self.proc and self.proc.poll() is None:
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                self.proc.wait(timeout=2)
        workspace = Path(self.script).resolve().parents[2]
        command = ('source /opt/ros/jazzy/setup.bash && source '+
                   shlex.quote(str(workspace/'install/setup.bash'))+
                   ' && exec python3 '+shlex.quote(self.script))
        with open(self.log_path, 'ab', buffering=0) as log:
            self.proc = subprocess.Popen(['bash', '-c', command], stdin=subprocess.PIPE,
                                         stdout=subprocess.PIPE, stderr=log, text=True, bufsize=1,
                                         env={**os.environ, 'ROS_DOMAIN_ID': str(domain)})
        self.domain, self.error = str(domain), ''
        self.state, self.updated = {}, 0.
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def ensure(self, domain):
        with self.lock:
            try:
                if not self.proc or self.proc.poll() is not None or self.domain != str(domain):
                    self._spawn(domain)
                return True
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                self.error = str(error)
                return False

    def command(self, data, domain):
        if not self.ensure(domain):
            return {'ok': False, 'error': self.error}
        request_id = uuid.uuid4().hex
        with self.lock:
            try:
                self.proc.stdin.write(json.dumps({**data, 'request_id': request_id})+'\n')
                self.proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as error:
                return {'ok': False, 'error': str(error)}
        deadline = time.monotonic()+3
        with self.condition:
            while request_id not in self.replies and time.monotonic() < deadline:
                self.condition.wait(timeout=max(.01, deadline-time.monotonic()))
            result = self.replies.pop(request_id, None)
        if result is None:
            return {'ok': False, 'error': '速度指令の応答を確認できません'}
        return {k: v for k, v in result.items() if k not in ('owner', 'request_id')}

    def status(self, session=None):
        with self.lock, self.condition:
            alive = self.proc is not None and self.proc.poll() is None
            fresh = alive and time.monotonic()-self.updated < 1.5
            state = {k: v for k, v in self.state.items() if k not in ('owner', 'request_id', 'ok', 'error')}
            state.update(alive=alive, domain=self.domain or '', error=self.error,
                         owned=bool(fresh and session and self.state.get('owner') == session))
            if not fresh:
                state.update(enabled=False, linear=0., angular=0.,
                             target_linear=0., target_angular=0., blocked='速度配信の状態を確認できません')
            return state
