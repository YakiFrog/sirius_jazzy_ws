const {test}=require('node:test');
const assert=require('node:assert/strict');
const {lifecycleState,lifecycleCostmap,lifecycleGraphState,lifecycleHealth,lifecycleStartup}=require('../static/lifecycle_model.js');
const e=value=>({fresh:true,value});
test('fresh lifecycle states are distinct, including errors and transitions',()=>{
 for(const [id,tone,label] of [[1,'waiting','未設定'],[2,'waiting','待機中'],[3,'ok','稼働中'],[13,'transition','遷移中'],[15,'error','エラー処理中'],[4,'unknown','終了']]){
  assert.deepEqual(lifecycleState(e({id}),true),{tone,label});
 }
});
test('stale or unavailable state is never reported as active',()=>{
 assert.equal(lifecycleState({fresh:false,value:{id:3}},true).tone,'unknown');
 assert.equal(lifecycleState(e({id:3}),false).tone,'unknown');
 assert.equal(lifecycleState(e({active:true}),true).tone,'unknown');
});
test('costmap receipt is independent of lifecycle status',()=>{
 assert.equal(lifecycleCostmap(e({valid:true}),true).label,'受信中');
 assert.equal(lifecycleCostmap(e({valid:false}),true).label,'空・不正');
 assert.equal(lifecycleCostmap({fresh:false,value:{valid:true}},true).label,'受信未確認');
});
test('input graph reflects current TF, AMCL scan, mux and driver data',()=>{
 const n={available:true,domain:'53',readiness:{ready:true},navigation:{stages:{lock:e(false),driver:e(false)},preflight:{scan:e({valid:true,samples:100,stamp_current:true}),routing:e({connected:true}),communication:e(true)}}};
 assert.deepEqual(Object.values(lifecycleGraphState(n,'53',()=>true)).map(x=>x.tone),['ok','ok','ok','ok']);
 assert.deepEqual(Object.values(lifecycleGraphState(n,'56',()=>true)).map(x=>x.tone),['unknown','unknown','unknown','unknown']);
 n.navigation.stages.lock.value=true;n.navigation.stages.driver.value=true;
 const stopped=lifecycleGraphState(n,'53',()=>true);assert.equal(stopped.mux.label,'ロック中');assert.equal(stopped.driver.label,'停止中');
});
test('unknown sensors and unverified connections are not green',()=>{
 const n={available:true,readiness:{ready:false},navigation:{stages:{lock:e(false)},preflight:{routing:e({connected:false})}}};
 const x=lifecycleGraphState(n,'53',()=>false);assert.equal(x.tf.tone,'waiting');assert.equal(x.laser.tone,'unknown');assert.equal(x.mux.tone,'waiting');assert.equal(x.driver.tone,'unknown');
});
test('inactive alone is waiting, but failed activation remains an error',()=>{
 const payload={available:true,telemetry:{nodes:{planner_server:{...e({id:2}),observed_seconds:100}}}};
 assert.equal(lifecycleHealth(payload).tone,'waiting');
 payload.telemetry.logs={planner_server:{fresh:true,age:0,value:{level:40,text:'有効化に失敗'}}};
 assert.deepEqual(lifecycleHealth(payload).errors,[{node:'planner_server',reason:'有効化に失敗'}]);
});
test('incomplete state history does not misclassify a recovered node as a failure',()=>{
 const payload={available:true,telemetry:{nodes:{controller_server:e({id:3}),waypoint_follower:e({id:2})},history:[{node:'controller_server',id:3,time:20},{node:'controller_server',id:14,time:1},{node:'waypoint_follower',id:2,time:20},{node:'waypoint_follower',id:11,time:1}]}};
 assert.equal(lifecycleHealth(payload).errors.length,0);
});
test('unresolved startup error persists after log freshness expires and clears on recovery',()=>{
 const payload={available:true,telemetry:{nodes:{global_costmap:{...e({id:2}),observed_seconds:300}},logs:{global_costmap:{fresh:false,age:300,value:{level:40,text:'Failed to activate global_costmap'}}}}};
 assert.equal(lifecycleHealth(payload).errors.length,1);
 payload.telemetry.nodes.global_costmap={...e({id:3}),observed_seconds:1};
 assert.equal(lifecycleHealth(payload).errors.length,0);
 payload.telemetry.nodes.global_costmap={...e({id:2}),observed_seconds:1};
 assert.equal(lifecycleHealth(payload).errors.length,0);
});
test('latest operation and current observation determine whether a failure is unresolved',()=>{
 const payload={available:true,telemetry:{nodes:{amcl:{...e({id:2}),observed_seconds:20}}},operations:[{node:'amcl',started:80,finished:90,status:'failed'}]};
 assert.equal(lifecycleHealth(payload,100).errors.length,1);
 payload.operations.push({node:'amcl',started:95,finished:96,status:'done'});
 assert.equal(lifecycleHealth(payload,100).errors.length,0);
 assert.equal(lifecycleHealth({...payload,available:false},100).tone,'unknown');
 payload.telemetry.nodes.amcl.fresh=false;
 assert.equal(lifecycleHealth(payload,100).tone,'unknown');
});
test('OK requires active primary nodes and permits inactive optional features',()=>{
 const payload={available:true,nodes:{map_server:'map',planner_server:'planner',docking_server:'dock'},telemetry:{nodes:{map_server:e({id:3}),planner_server:e({id:3}),docking_server:e({id:2})}}};
 assert.equal(lifecycleHealth(payload).tone,'ok');
 payload.telemetry.nodes.planner_server.value.id=13;
 assert.equal(lifecycleHealth(payload).tone,'waiting');
 payload.telemetry.nodes.planner_server.fresh=false;
 assert.equal(lifecycleHealth(payload).tone,'unknown');
});
test('TF polling messages are waiting and a terminal error becomes a timeout notice',()=>{
 const name='global_costmap/global_costmap';
 const payload={available:true,telemetry:{nodes:{[name]:{...e({id:13}),observed_seconds:23.8}},logs:{[name]:{fresh:true,age:0,value:{level:20,text:'Timed out waiting for transform from base to map'}}}}};
 assert.equal(lifecycleHealth(payload).tone,'waiting');
 assert.equal(lifecycleStartup(payload)[0].label,'TF接続待ち 23秒経過');
 payload.telemetry.nodes[name]={...e({id:2}),observed_seconds:300};
 payload.telemetry.logs[name]={fresh:false,age:300,value:{level:40,text:'Failed to activate global_costmap because transform did not become available before timeout'}};
 assert.equal(lifecycleHealth(payload).tone,'error');
 assert.equal(lifecycleStartup(payload)[0].label,'TF待機タイムアウト');
 assert.equal(lifecycleStartup(payload)[0].parent,'planner_server');
 payload.telemetry.nodes[name]={...e({id:3}),observed_seconds:1};
 payload.telemetry.logs[name].fresh=true;
 assert.equal(lifecycleHealth(payload).tone,'ok');
 assert.deepEqual(lifecycleStartup(payload),[]);
});
