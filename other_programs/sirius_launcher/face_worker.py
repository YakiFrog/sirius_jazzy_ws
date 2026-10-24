"""One bounded Face RPC, isolated in SiriusFace's own Python environment."""
import json
from pathlib import Path
import sys


def call(root, data):
    sys.path.insert(0, str(Path(root)/'scripts/stubs'))
    import grpc
    import face_control_pb2 as messages
    import face_control_pb2_grpc as services
    from google.protobuf.empty_pb2 import Empty

    action = data['action']
    target = '127.0.0.1:50052' if action in {'speak', 'instruction'} else '127.0.0.1:50051'
    with grpc.insecure_channel(target) as channel:
        if action == 'speak':
            text = data['text']
            # Plain speech must never route commands; instruction actions add the
            # [llm]/[nav] prefix explicitly below.
            if '[' in text or ']' in text:
                raise ValueError('読み上げには制御タグを使えません')
            services.PythonControlServiceStub(channel).Speak(messages.SpeakRequest(text=text), timeout=2.5)
            return {'ok': True}
        if action == 'instruction':
            mode = data.get('mode')
            text = str(data.get('text') or '').strip()
            if mode not in {'llm', 'nav'}:
                raise ValueError('行動指示の方式を選んでください')
            if not text or len(text) > 200:
                raise ValueError('行動指示を1〜200文字で入力してください')
            if '[' in text or ']' in text:
                raise ValueError('行動指示には制御タグを使えません')
            prefix = '[llm]' if mode == 'llm' else '[nav]'
            services.PythonControlServiceStub(channel).Speak(
                messages.SpeakRequest(text=prefix + text), timeout=2.5)
            return {'ok': True, 'mode': mode}
        stub = services.FaceServiceStub(channel)
        if action == 'status':
            status = stub.GetStatus(Empty(), timeout=1.0)
            speech_ready = False
            # The existing speech server treats this exact marker as a no-op
            # health check, returning before synthesis or command-tag parsing.
            try:
                with grpc.insecure_channel('127.0.0.1:50052') as speech_channel:
                    services.PythonControlServiceStub(speech_channel).Speak(
                        messages.SpeakRequest(text='[ping]'), timeout=.4)
                    speech_ready = True
            except grpc.RpcError:
                pass
            return dict(connected=True, expression=status.face_state,
                        eye_style=status.eye_style, mouth_style=status.mouth_style,
                        speech_ready=speech_ready)
        if action == 'expression':
            stub.SetExpression(messages.ExpressionRequest(state=data['expression']), timeout=1.5)
        elif action == 'gaze':
            stub.LookAt(messages.LookAtRequest(x=data['x'], y=data['y']), timeout=1.5)
        elif action in ('subtitle', 'clear_text'):
            stub.ShowText(messages.TextRequest(text=data.get('text', ''), duration=3), timeout=1.5)
        else:
            raise ValueError('対応していないFace操作です')
        return {'ok': True}


if __name__ == '__main__':
    try:
        result = call(sys.argv[1], json.load(sys.stdin))
    except Exception as error:
        code = getattr(error, 'code', lambda: None)()
        message = ('Face APIの応答を確認できません（'+str(getattr(code, 'name', '接続エラー'))+'）')
        if isinstance(error, ValueError):
            message = str(error)
        result = {'error': message}
    print(json.dumps(result, ensure_ascii=False))
