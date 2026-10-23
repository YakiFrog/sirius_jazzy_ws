"""ROS JSON-lines adapter. Publishes only /sirius/ear_led_test; no BLE connection."""
import json
import queue
import sys
import threading
import time


def main():
    import rclpy
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('sirius_launcher_ear')
    commands=queue.Queue();pending={};latest={};received=0.;running=True
    def snapshot():
        fresh=time.monotonic()-received<3
        connected=bool(fresh and latest.get('left_connected') and latest.get('right_connected'))
        return {**latest,'gateway_available':fresh,'ready':bool(connected and latest.get('enabled') and latest.get('web_test_supported')),
                'gateway_error':latest.get('error') if fresh else None,
                'left_connected':bool(fresh and latest.get('left_connected')),
                'right_connected':bool(fresh and latest.get('right_connected')),
                'age_seconds':round(time.monotonic()-received,1) if received else None}
    def emit(data):print(json.dumps(data,ensure_ascii=False),flush=True)
    def on_status(message):
        nonlocal latest,received
        try:data=json.loads(message.data)
        except ValueError:return
        if not isinstance(data,dict):return
        latest=data;received=time.monotonic()
        request_id=data.get('test_request_id')
        if isinstance(request_id,str) and request_id in pending:
            pending.pop(request_id);emit({**snapshot(),'ok':True,'request_id':request_id})
    sub=node.create_subscription(String,'/sirius/ear_led_status',on_status,10)
    pub=node.create_publisher(String,'/sirius/ear_led_test',10)
    def reader():
        nonlocal running
        for line in sys.stdin:
            try:
                data=json.loads(line)
                if isinstance(data,dict):commands.put(data)
            except ValueError:pass
        running=False
    threading.Thread(target=reader,daemon=True).start()
    previous=0
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.05)
            while not commands.empty():
                data=commands.get_nowait();request_id=data.get('request_id');state=snapshot()
                reason=None
                if not state.get('gateway_available'):reason='BLE Gatewayの状態を受信できません'
                elif not state.get('web_test_supported'):reason='BLE Gatewayを更新・再起動してください'
                elif not state.get('enabled'):reason='BLE Gatewayで耳LEDを有効にしてください'
                elif data.get('pattern')!='auto' and state.get('stop'):reason='実機の非常停止表示を優先しています'
                elif data.get('pattern')!='auto' and not state.get('ready'):reason='左右の耳LEDの接続を確認してください'
                elif not pub.get_subscription_count():reason='耳LEDテストの受信先がありません'
                if reason:emit({**state,'ok':False,'error':reason,'request_id':request_id});continue
                pending[request_id]=time.monotonic()+2
                pub.publish(String(data=json.dumps(data)))
            now=time.monotonic()
            for request_id in list(pending):
                if now>pending[request_id]:
                    pending.pop(request_id);emit({**snapshot(),'ok':False,'error':'Gatewayの受付応答を確認できません','request_id':request_id})
            if now-previous>=.5:emit(snapshot());previous=now
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
