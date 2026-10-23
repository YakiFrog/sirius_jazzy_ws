"""Authenticated ear-only requests through the existing ROS/BLE gateway."""
import math
from pathlib import Path
import time

from flask import Blueprint, jsonify, request
from teleop_process import TeleopBridge

PATTERNS = {'normal':'通常','left':'左折','right':'右折','turning':'旋回・ハザード',
            'emergency':'非常停止表示','solid':'点灯','blink':'点滅','alternate':'左右交互',
            'breathe':'呼吸','rainbow':'虹色','off':'消灯'}


class EarBridge(TeleopBridge):
    def status(self, session=None):
        with self.lock, self.condition:
            fresh=self.proc is not None and self.proc.poll() is None and time.monotonic()-self.updated<2
            state={k:v for k,v in self.state.items() if k not in {'request_id','ok','error','owner'}}
            if not fresh:
                state.update(ready=False,gateway_available=False,status='waiting',left_connected=False,right_connected=False)
            return state

    def command(self, data, domain):
        result=super().command(data,domain)
        if result.get('error')=='速度指令の応答を確認できません':
            result['error']='耳LED操作の応答を確認できません'
        return result


def validate(data):
    if not isinstance(data,dict) or not isinstance(data.get('pattern'),str) or data['pattern'] not in set(PATTERNS)|{'auto'}:
        raise ValueError('耳LEDの表示を一覧から選んでください')
    duration=data.get('duration',10);color=data.get('color',[0,160,255])
    if type(duration) not in (int,float) or not math.isfinite(duration) or not 1<=duration<=30:
        raise ValueError('テスト時間は1〜30秒です')
    if not isinstance(color,list) or len(color)!=3 or any(type(v) is not int or not 0<=v<=255 for v in color):
        raise ValueError('RGBは0〜255で指定してください')
    return dict(pattern=data['pattern'],duration=duration,color=color)


def register_ear(app, auth_ok, domain=lambda:'53', bridge=None):
    log=Path.home()/'.ros/sirius_launcher_ear.log'
    bridge=bridge or EarBridge(str(Path(__file__).with_name('ear_worker.py')),str(log))
    bp=Blueprint('ear_control',__name__)
    def selected_domain(data=None):
        value=str((data or {}).get('domain') or request.args.get('domain') or domain())
        if not value.isdecimal() or not 0<=int(value)<=232:raise ValueError('ROS Domainが不正です')
        return value

    @bp.before_request
    def authenticate():
        if not auth_ok():return jsonify(error='auth'),401

    @bp.get('/api/ear/status')
    def status():
        try:
            log.parent.mkdir(parents=True,exist_ok=True)
            bridge.ensure(selected_domain())
            response=jsonify({**bridge.status(),'patterns':PATTERNS})
            response.headers['Cache-Control']='no-store'
            return response
        except ValueError as error:return jsonify(error=str(error)),400

    @bp.post('/api/ear/test')
    def test():
        try:
            log.parent.mkdir(parents=True,exist_ok=True)
            data=request.get_json(silent=True)
            command=validate(data)
            result=bridge.command(command,selected_domain(data))
            return jsonify(result),200 if result.get('ok') else 409
        except ValueError as error:return jsonify(error=str(error)),400

    app.register_blueprint(bp)
