const {test} = require('node:test');
const assert = require('node:assert/strict');
const {navigationDiagnosticState: classify} = require('../static/navigation_diagnostics.js');
const entry = value => ({value, fresh:true, age:0.1});
function healthy() {
  return {available:true,readiness:{state:'ok',ready:true},edges:[{parent:'map',child:'sirius3/odom'},{parent:'sirius3/odom',child:'sirius3/base_footprint'}],navigation:{mismatch:false,stages:{driver:entry(false),lock:entry(false),nav:entry({moving:true,x:.2,yaw:0}),safe:entry({moving:true,x:.2,yaw:0}),output:entry({moving:true,x:.2,yaw:0}),motion:entry(true),goal:entry(2)}}};
}
test('green requires fresh release, TF, final command and wheel motion', () => {
  assert.equal(classify(healthy()).tone,'ok');
  for (const key of ['lock','driver']) { const data=healthy();data.navigation.stages[key].fresh=false;assert.equal(classify(data).tone,'unknown'); }
  const missing=healthy();missing.navigation.stages.motion.fresh=false;assert.equal(classify(missing).tone,'attention');
});
test('actual stop dominates previously observed nonzero commands', () => {
  for (const key of ['lock','driver']) { const data=healthy();data.navigation.stages[key]=entry(true);assert.equal(classify(data).tone,'blocked');assert.equal(classify(data).badge,'停止'); }
});
test('recorded stop mismatch is a visible red fault', () => {
  const data=healthy();data.navigation.mismatch=true;assert.match(classify(data).title,/食い違/);assert.equal(classify(data).tone,'blocked');
});
test('safe input without final nonzero output is not OK', () => {
  const data=healthy();data.navigation.stages.output=entry({moving:false,x:0,yaw:0});assert.equal(classify(data).tone,'attention');
});
test('a disconnected map edge is insufficient even if map exists', () => {
  const data=healthy();data.edges=[{parent:'map',child:'other'}];assert.equal(classify(data).tone,'attention');
});
test('TF error does not claim that motor e-stop is active', () => {
  const data=healthy();data.readiness={state:'error',ready:false};assert.equal(classify(data).badge,'TFエラー');
});
test('a stationary robot waiting for a goal is blue, not a red fault', () => {
  const data=healthy();for(const k of ['nav','safe','output'])data.navigation.stages[k]=entry({moving:false,x:0,yaw:0});data.navigation.stages.motion=entry(false);data.navigation.stages.goal=entry(4);assert.equal(classify(data).tone,'idle');
});
test('a stale receiver cannot show green', () => { const data=healthy();data.available=false;assert.equal(classify(data).tone,'unknown'); });
const {navigationPreflightState:preflight,navigationPowerStates:powerStates} = require('../static/navigation_diagnostics.js');
function preparedButStopped() {
 const data=healthy();
 data.navigation.stages.driver=entry(true);data.navigation.stages.lock=entry(true);data.navigation.stages.motion=entry(false);data.navigation.stages.output={fresh:false,value:null};
 data.navigation.preflight={nodes:{},scan:{...entry({samples:360,valid:true,stamp_current:true}),topic:'/scan3'},communication:entry(true),routing:entry({connected:true,missing:[]})};
 for(const name of ['bt_navigator','planner_server','controller_server','amcl','map_server','velocity_smoother','collision_monitor'])data.navigation.preflight.nodes[name]=entry({active:true,label:'active'});
 data.battery={details:{state:'live',age_seconds:.1,stale_after_seconds:5,output_states:{ac:'on',dc:'on'},ports:{ac_output:72,dc_output:0}}};
 return data;
}
test('preflight ignores held emergency stops and missing post-mux motion',()=>{
 const data=preparedButStopped();assert.equal(preflight(data).tone,'ok');assert.equal(classify(data).tone,'blocked');
});
test('0W with reported ON is ON, not OFF',()=>{
 const data=preparedButStopped();assert.equal(powerStates(data).dc.value,'on');assert.equal(preflight(data).tone,'ok');
});
test('power consumption does not substitute for an unknown output switch',()=>{
 const data=preparedButStopped();delete data.battery.details.output_states.dc;data.battery.details.ports.dc_output=20;assert.equal(powerStates(data).dc.value,null);assert.equal(preflight(data).tone,'attention');
});
test('AC or cigar output explicitly OFF blocks readiness',()=>{
 for(const key of ['ac','dc']){const data=preparedButStopped();data.battery.details.output_states[key]='off';assert.equal(preflight(data).tone,'blocked');}
});
test('an inactive action server is caught even when a path was published',()=>{
 const data=preparedButStopped();data.navigation.preflight.nodes.bt_navigator=entry({active:false,label:'inactive'});assert.equal(preflight(data).tone,'blocked');assert.match(preflight(data).detail,/ゴール受付/);
});
test('stale scan, missing telemetry or stale battery cannot produce readiness OK',()=>{
 for(const key of ['scan','communication']){const data=preparedButStopped();data.navigation.preflight[key].fresh=false;assert.notEqual(preflight(data).tone,'ok');}
 const data=preparedButStopped();data.battery.details.age_seconds=6;assert.notEqual(preflight(data).tone,'ok');
});
test('a collision-monitor stop remains visible independently of emergency stops',()=>{
 const data=preparedButStopped();data.navigation.stages.safe=entry({moving:false,x:0,yaw:0});assert.equal(preflight(data).tone,'attention');
});
test('missing configured command route cannot show ready',()=>{
 const data=preparedButStopped();data.navigation.preflight.routing=entry({connected:false,missing:['/cmd_vel_nav_safe 購読先 twist_mux']});assert.equal(preflight(data).tone,'blocked');
});
