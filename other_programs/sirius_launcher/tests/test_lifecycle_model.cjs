const {test}=require('node:test');
const assert=require('node:assert/strict');
const {lifecycleState,lifecycleCostmap,lifecycleGraphState}=require('../static/lifecycle_model.js');
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
