const {test}=require('node:test');
const assert=require('node:assert/strict');
const {navigationPreflightState}=require('../static/navigation_diagnostics.js');
global.navigationPreflightState=navigationPreflightState;
const {operationsSummary}=require('../static/operations_home.js');
function ready(){
 const e=value=>({fresh:true,age:0,value});
 return {available:true,edges:[{parent:'map',child:'sirius3/base_footprint'}],readiness:{ready:true},
 battery:{details:{state:'live',age_seconds:0,output_states:{ac:'on',dc:'on'},ports:{ac_output:0,dc_output:0}}},
 navigation:{stages:{driver:e(false),lock:e(false)},preflight:{nodes:Object.fromEntries(['bt_navigator','planner_server','controller_server','amcl','map_server','velocity_smoother','collision_monitor'].map(n=>[n,e({active:true})])),scan:{...e({valid:true,samples:100,stamp_current:true}),topic:'/scan3'},communication:e(true),routing:e({connected:true})}}};
}
test('emergency stop has a reason without falsely failing preflight',()=>{
 const n=ready();n.navigation.stages.driver.value=true;n.navigation.stages.lock.value=true;
 const s=operationsSummary(n,{driver:{state:'live',age_seconds:0,emergency_stop:true}});
 assert.equal(s.tone,'blocked');assert.match(s.reason,/電子緊急停止/);assert.equal(s.ready.tone,'ok');
});
test('physical stop inferred from voltage is explicitly an estimate',()=>{
 const s=operationsSummary(ready(),{driver:{state:'live',age_seconds:0,physical_emergency_stop:true}});
 assert.match(s.reason,/電圧による推定/);assert.equal(s.stop,'停止中');
});
test('otherwise ready is distinct from actual driving',()=>{
 const s=operationsSummary(ready(),{});assert.equal(s.title,'自律移動の準備OK');assert.match(s.reason,/ゴール/);
});
test('unknown and stale stops do not become a current stop or OK',()=>{
 assert.equal(operationsSummary(null,null).tone,'unknown');
 const n=ready();n.navigation.stages.driver.fresh=false;
 const s=operationsSummary(n,{driver:{state:'stale',age_seconds:20,emergency_stop:true}});
 assert.equal(s.tone,'unknown');assert.doesNotMatch(s.reason,/ON/);
});
test('stop does not hide another blocker from the explanation',()=>{
 const n=ready();n.navigation.preflight.nodes.amcl.value.active=false;
 const s=operationsSummary(n,{driver:{state:'live',age_seconds:0,emergency_stop:true}});
 assert.equal(s.reasons.length,2);assert.match(s.reasons[1],/自己位置推定/);
});
