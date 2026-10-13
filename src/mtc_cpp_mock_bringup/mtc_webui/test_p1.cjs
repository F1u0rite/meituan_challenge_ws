const test=require('node:test');
const assert=require('node:assert/strict');
const {perceptionView,plannerView,toolView,stamp}=require('./static/p1.js');
const q=s=>s?.fresh?'正常':s?.available?'过期':'缺失';
const sample=data=>({data,fresh:true,available:true,age_ms:0,source:'fixture'});
const pose={valid:true,offset_ms:0,position:[.1,.2,.3],orientation:[0,0,0,1],stamp_ns:'1791871800123456789',issues:[]};
function state(){return {mode:'ros',freshness_limit_ms:3000,task:{task_id:'task-1',status:'EXECUTING',feedback_fresh:true,feedback_age_ms:0,current_index:0,total_count:3,current_object_id:'b1'},task_state:sample({task_id:'task-1',state:'EXECUTE_ITEM'}),
  perception:sample({frame_id:'camera',detections:[{object_id:'b1',identity_valid:true,color:2,confidence:.9,ring_pose_valid:true,body_pose:pose,ring_pose:pose}]}),
  tool:sample({tool_type:'passive_hook_v1',state:3,verified:false,evidence_level:1,object_id:'b1'}),
  tool_verification:sample({verification_id:'v1',task_id:'task-1',object_id:'b1',tool_type:'passive_hook_v1',test_lift:{verified:true,evidence_level:2},seated:null})};}
test('target requires current feedback ID, never matching color or duplicate ID',()=>{
  const s=state();assert.equal(perceptionView(s,q).rows[0].target,true);
  s.task.current_object_id='other';assert.equal(perceptionView(s,q).rows[0].target,false);
  s.task.current_object_id='b1';s.perception.data.detections[0].identity_valid=false;assert.equal(perceptionView(s,q).rows[0].target,false);
});
test('stale detections, completed/foreign tasks and paused transport clear highlights',()=>{
  for(const change of [s=>s.task.status='SUCCEEDED',s=>s.task.feedback_fresh=false,s=>s.task_state.data.task_id='other']){
    const s=state();change(s);assert.equal(perceptionView(s,q).rows[0].target,false);
  }
  const s=state();assert.equal(perceptionView(s,q,false).rows[0].target,false);
  s.perception.fresh=false;assert.deepEqual(perceptionView(s,q).rows,[]);
});
test('fresh array cannot refresh an old/invalid pose or invent validity',()=>{
  const s=state();s.perception.data.detections[0].ring_pose={...pose,offset_ms:9000};
  const v=perceptionView(s,q);assert.equal(v.rows[0].ring_pose_valid,true);assert.equal(v.rows[0].ring.usable,false);
  s.perception.data.detections[0].ring_pose={...pose,valid:false};assert.equal(perceptionView(s,q).rows[0].ring.usable,false);
  s.perception.data.detections[0].ring_pose=pose;assert.equal(perceptionView(s,q,true,3100).rows[0].ring.usable,false);
});
test('zero detections and unavailable perception differ',()=>{
  const s=state();s.perception.data.detections=[];assert.ok(perceptionView(s,q).data);
  s.perception.fresh=false;assert.equal(perceptionView(s,q).data,null);
});
test('planner preserves watchdog reason and marks foreign task reports',()=>{
  const s=state();s.planner=sample({task_id:'other',state:'FAILED',reason:'EXCEEDS_STATE_WATCHDOG',collision_free:false});
  const v=plannerView(s,q);assert.equal(v.foreign,true);assert.equal(v.data.reason,'EXCEEDS_STATE_WATCHDOG');assert.equal(v.data.collision_free,false);
  s.planner.fresh=false;assert.equal(plannerView(s,q).data,null);
});
test('command-only Attached and verified Detached are not independent mounting',()=>{
  const s=state();assert.equal(toolView(s,q).mounted,false);
  s.tool.data.verified=true;assert.equal(toolView(s,q).mounted,false);assert.equal(toolView(s,q).inconsistent,true);
  s.tool.data.evidence_level=2;assert.equal(toolView(s,q).mounted,true);
  s.tool.data.state=1;assert.equal(toolView(s,q).mounted,false);
  s.tool.fresh=false;assert.equal(toolView(s,q).mounted,null);
});
test('trial lift does not imply seated; proof identity and freshness are mandatory',()=>{
  const s=state();assert.equal(toolView(s,q).testLift.verified,true);assert.equal(toolView(s,q).seated,null);
  for(const change of [s=>s.tool_verification.data.task_id='old',s=>s.tool_verification.data.object_id='b2',s=>s.tool_verification.data.tool_type='magnetic_latch_v2',s=>s.task.current_object_id='b2',s=>s.tool_verification.fresh=false,s=>s.tool.fresh=false]){
    const x=state();change(x);assert.equal(toolView(x,q).matched,false);assert.equal(toolView(x,q).testLift,null);
  }
  const stale=state();stale.task_state.fresh=false;stale.task.feedback_fresh=false;assert.equal(toolView(stale,q).matched,false);
  assert.equal(toolView(state(),q,false).matched,false);
});
test('nanoseconds preserve exact precision',()=>assert.equal(stamp('1791871800123456789'),'1791871800.123456789 s (ROS)'));
