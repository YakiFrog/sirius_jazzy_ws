"""One explicitly requested transition. No retries, goals, motor or E-stop writes."""
import json
import sys
import time

from lifecycle_protocol import ACTIONS,validate_command


def execute(command):
    import rclpy
    from lifecycle_msgs.srv import GetState,GetAvailableTransitions,ChangeState
    command=validate_command(command)
    rclpy.init();node=rclpy.create_node('sirius_manual_lifecycle')
    prefix='/'+command['node'];sent=False
    clients={suffix:node.create_client(typ,prefix+'/'+suffix) for typ,suffix in (
        (GetState,'get_state'),(GetAvailableTransitions,'get_available_transitions'),(ChangeState,'change_state'))}
    def call(typ,suffix,request,timeout):
        client=clients[suffix]
        if not client.wait_for_service(timeout_sec=6.):
            raise RuntimeError(prefix+'/'+suffix+' が応答しません')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('サービスの応答待ち期限を超えました')
        return future.result()
    try:
        state=call(GetState,'get_state',GetState.Request(),4).current_state
        transition,required,target,label=ACTIONS[command['action']]
        if state.id!=required:
            return {'ok':False,'sent':False,'error':'状態が変わっています。更新して再操作してください',
                    'state':{'id':state.id,'label':state.label}}
        available=call(GetAvailableTransitions,'get_available_transitions',GetAvailableTransitions.Request(),4)
        if transition not in [t.transition.id for t in available.available_transitions]:
            return {'ok':False,'sent':False,'error':'現在この遷移は許可されていません'}
        request=ChangeState.Request();request.transition.id=transition
        sent=True
        response=call(ChangeState,'change_state',request,70)
        after=call(GetState,'get_state',GetState.Request(),4).current_state
        ok=bool(response.success and after.id==target)
        return {'ok':ok,'sent':True,'state':{'id':after.id,'label':after.label},
                'error':None if ok else '操作後の目的状態を確認できません。履歴・ログを確認してください'}
    except Exception as error:
        return {'ok':False,'sent':sent,'uncertain':sent,'error':str(error)+('。要求は継続中の可能性があります。再送せず状態を確認してください' if sent else '')}
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    try:result=execute(json.load(sys.stdin))
    except Exception as error:result={'ok':False,'sent':False,'error':str(error)}
    print(json.dumps(result,ensure_ascii=False),flush=True)
