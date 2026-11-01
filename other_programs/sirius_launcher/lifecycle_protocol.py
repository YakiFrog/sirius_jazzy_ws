"""Shared, explicit scope for Nav2 lifecycle inspection and manual operations."""
NODES = {
    'map_server':'地図配信', 'amcl':'自己位置推定',
    'global_costmap/global_costmap':'グローバルコストマップ',
    'planner_server':'経路計画', 'local_costmap/local_costmap':'ローカルコストマップ',
    'controller_server':'速度制御', 'smoother_server':'経路平滑化',
    'behavior_server':'リカバリー動作', 'bt_navigator':'ゴール受付',
    'velocity_smoother':'速度平滑化', 'collision_monitor':'衝突監視',
    'waypoint_follower':'Nav2ウェイポイント', 'docking_server':'ドッキング',
}
# action: (ROS transition ID, required stable state, target state, Japanese label)
ACTIONS = {'configure':(1,1,2,'設定'), 'activate':(3,2,3,'有効化'),
           'deactivate':(4,3,2,'無効化'), 'cleanup':(2,2,1,'設定解除')}


def validate_command(data):
    if not isinstance(data,dict) or data.get('node') not in NODES or data.get('action') not in ACTIONS:
        raise ValueError('対象ノードまたは操作が不正です')
    expected=data.get('expected_state')
    if isinstance(expected,bool) or not isinstance(expected,int) or expected!=ACTIONS[data['action']][1]:
        raise ValueError('現在状態と操作が一致していません。状態を更新してください')
    return {'node':data['node'],'action':data['action'],'expected_state':expected}
