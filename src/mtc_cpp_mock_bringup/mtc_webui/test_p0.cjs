/* Node's built-in runner checks display semantics independently of DOM layout. */
const assert = require("node:assert/strict");
const {test} = require("node:test");
const P0 = require("./static/p0.js");
const sample = data => ({available:true, fresh:true, age_ms:0, data});
const quality = s => s?.available && s.fresh ? "正常" : "过期";
const base = () => ({mode:"ros",freshness_limit_ms:3000,task:{task_id:"t",status:"EXECUTING",goal:{mode:2,ordered_colors:[2,1,3]},current_index:1,total_count:3,completed_count:0,current_slot:"P2",current_object_id:"battery-44",feedback_age_ms:100,feedback_fresh:true,state:"EXECUTE_ITEM",substate:"SEAT"}});

test("input order, battery index and target come from separate observed fields",()=>{
  const v=P0.taskView(base(),quality,true,0);
  assert.deepEqual(v.colors,[2,1,3]);assert.equal(v.index,1);assert.equal(v.color,1);
  assert.equal(v.object,"battery-44");assert.equal(v.slot,"P2");assert.equal(v.completed,0);
});
test("expired feedback and paused browser clear current targets and graph highlight",()=>{
  const s=base(); const v=P0.taskView(s,quality,true,3100);
  assert.equal(v.index,null);assert.equal(v.taskState,"");assert.equal(v.object,null);
  s.task_state=sample({task_id:"t",state:"EXECUTE_ITEM",substate:"LIFT"});
  const paused=P0.taskView(s,quality,false,0);
  assert.equal(paused.taskState,"");assert.equal(paused.pickState,"");
});
test("external task never borrows this session's command or target",()=>{
  const s=base();s.task_state=sample({task_id:"foreign",state:"EXECUTE_ITEM",substate:"ENGAGE",completed_count:1,total_count:3});
  const v=P0.taskView(s,quality);
  assert.equal(v.foreign,true);assert.equal(v.id,"foreign");assert.equal(v.colors,null);assert.equal(v.object,null);
});
test("ABORTED 520 remains a failed final result even after Cancel ACK",()=>{
  const s=base();s.task.status="ABORTED";s.task.cancel_ack={request_delivered:true};s.task.result={success:false,error_code:520,completed_count:0,message:"stop unconfirmed"};
  const v=P0.taskView(s,quality);
  assert.equal(v.finalStatus,"FAILED");assert.equal(v.task.status,"ABORTED");assert.equal(v.result.error_code,520);
  const safety=P0.safetyView(s,quality,true);
  assert.equal(safety.cancel.request_delivered,true);assert.equal(safety.final.status,"ABORTED");assert.equal(safety.confirmed,false);
});
test("FSM TRANSPORT and successful task do not manufacture motion or progress",()=>{
  const s=base();s.task.substate="TRANSPORT";
  const v=P0.motionView(s,quality);
  assert.equal(v.physical,"未知");assert.equal(v.progress,null);assert.equal(v.elapsed,null);
});
test("FJT feedback time is displayed without guessing expected duration",()=>{
  const s=base();s.fjt_feedback=sample({goal_id:"fjt-1",elapsed_sec:2.25});
  const v=P0.motionView(s,quality);
  assert.equal(v.elapsed,2.25);assert.equal(v.expected,null);assert.equal(v.progress,null);
});
test("joint values follow names and missing velocities are not zero",()=>{
  const s=base();s.joints=sample({name:["joint_6","joint_1"],position:[1,2],velocity:[]});
  const v=P0.jointView(s,quality);
  assert.equal(v.rows[0].name,"joint_6");assert.equal(v.rows[0].actual,1);assert.equal(v.rows[0].velocity,null);
});
test("target comparison uses one frame and does not combine unrelated joint samples",()=>{
  const s=base();s.joints=sample({name:["joint_1","joint_6"],position:[99,98],velocity:[4,5]});
  s.controller=sample({joint_names:["joint_6","joint_1"],actual_position:[1,2],actual_velocity:[.1,.2],desired_position:[1.5,3]});
  const v=P0.jointView(s,quality);
  assert.equal(v.rows[0].actual,1);assert.equal(v.rows[0].target,1.5);assert.equal(v.rows[0].error,.5);
});
test("mismatched executor UUID cannot label another controller's motion",()=>{
  const s=base();s.motion=sample({command_id:"old",fjt_goal_id:"a",progress:.9});s.fjt_feedback=sample({goal_id:"b",elapsed_sec:1});
  const v=P0.motionView(s,quality);assert.equal(v.mismatch,true);assert.equal(v.execution,null);assert.equal(v.progress,null);
});
test("new FJT status quarantines old telemetry before the first feedback arrives",()=>{
  const s=base();s.motion=sample({command_id:"old",fjt_goal_id:"a",primitive:"lift",progress:.9});
  s.fjt_status=sample({goal_id:"b",status:"EXECUTING"});
  const v=P0.motionView(s,quality);assert.equal(v.mismatch,true);assert.equal(v.execution,null);assert.equal(v.progress,null);
});
test("Stop Request reply, task CANCELED and old true flag are not independent confirmation",()=>{
  const s=base();s.stop_request={request_delivered:true};s.robot=sample({stop_confirmed:true,motion_active:false,simulation_mode:false});
  s.stop_evidence={confirmed:false,post_request:false};
  assert.equal(P0.safetyView(s,quality,true).confirmed,false);
  s.stop_evidence={confirmed:true,post_request:true};assert.equal(P0.safetyView(s,quality,true).confirmed,true);
  s.robot.fresh=false;assert.equal(P0.safetyView(s,quality,true).confirmed,false);
  s.robot.fresh=true;s.robot.data.simulation_mode=true;assert.equal(P0.safetyView(s,quality,true).confirmed,false);
});
test("C++ Mock cannot claim hardware stop; a busy slot is not a fault lock",()=>{
  const s=base();s.mode="cpp_mock";s.cpp_task=sample({admission_open:false,locked:false});s.cpp_pick=sample({admission_open:false,locked:false});
  s.robot=sample({stop_confirmed:true,motion_active:false});s.stop_evidence={confirmed:true,post_request:true};
  let v=P0.safetyView(s,quality,true);assert.equal(v.confirmed,false);assert.equal(v.fault,false);
  s.cpp_pick.data.locked=true;v=P0.safetyView(s,quality,true);assert.equal(v.fault,true);
});
