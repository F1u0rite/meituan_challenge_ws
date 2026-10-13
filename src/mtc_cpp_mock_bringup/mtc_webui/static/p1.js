/* P1 read-only observations; all fields retain their source and evidence boundary. */
(function (root) {
  "use strict";
  const P0 = typeof module !== "undefined" && module.exports ? require("./p0.js") : root.P0;
  const colors = {0:"未知", 1:"红", 2:"蓝", 3:"黄", 4:"绿"};
  const evidence = ["0 · 无证据", "1 · 仅命令", "2 · 几何 / 机械反馈", "3 · 传感器 / 视觉"];
  const toolNames = {passive_hook_v1:"V1 Passive Hook", magnetic_latch_v2:"V2 Magnetic Latch"};
  const toolStates = ["Unknown", "Detached", "Engaging", "Attached", "Releasing", "Fault"];
  const data = (sample, quality) => quality(sample) === "正常" ? sample.data : null;
  const stamp = value => /^\d+$/.test(String(value)) && BigInt(value)>0n
    ? `${BigInt(value)/1000000000n}.${String(BigInt(value)%1000000000n).padStart(9,"0")} s (ROS)` : "时间戳未提供";
  function poseView(pose, sample, limit, delta=0) {
    const age = typeof sample?.age_ms === "number" && typeof pose?.offset_ms === "number"
      ? Math.max(0, sample.age_ms + pose.offset_ms + delta) : null;
    return {pose, age, usable: Boolean(pose?.valid && age !== null && age < limit)};
  }
  function perceptionView(s, quality, live=true, delta=0) {
    const d = data(s.perception, quality);
    const target = P0.taskView(s,quality,live,delta).object;
    const rows = (d?.detections || []).map(row=>({...row,
      target:Boolean(target && row.identity_valid && row.object_id===target),
      body:poseView(row.body_pose,s.perception,s.freshness_limit_ms,delta),
      ring:poseView(row.ring_pose,s.perception,s.freshness_limit_ms,delta)}));
    return {data:d, target, rows};
  }
  function plannerView(s, quality, live=true, delta=0) {
    const d = data(s.planner, quality);
    const task = P0.taskView(s,quality,live,delta);
    return {data:d, foreign:Boolean(d?.task_id && task.id && d.task_id!==task.id),
      associated:Boolean(d?.task_id && task.id && d.task_id===task.id)};
  }
  function toolView(s, quality, live=true, delta=0) {
    const d = data(s.tool, quality);
    const proof = data(s.tool_verification, quality);
    const task = P0.taskView(s,quality,live,delta);
    const taskKnown = task.targetLive || (task.id && data(s.task_state,quality)?.task_id===task.id);
    const matched = Boolean(live && d && proof && taskKnown && task.id && proof.task_id===task.id && proof.object_id===d.object_id
      && proof.tool_type===d.tool_type && (!task.object || task.object===proof.object_id));
    const mounted = d ? d.state===3 && d.verified===true && [2,3].includes(d.evidence_level) && Boolean(d.object_id) : null;
    return {data:d, proof, matched, mounted,
      inconsistent:Boolean(d?.verified===true && ![2,3].includes(d.evidence_level)),
      testLift:matched?proof.test_lift:null, seated:matched?proof.seated:null};
  }
  function render(s, options) {
    const {quality,live,delta=0}=options;
    const $=id=>document.getElementById(id);
    const text=(id,value,fallback="未提供")=>{$(id).textContent=value??fallback;};
    const el=(tag,value,cls)=>{const n=document.createElement(tag);if(value!==undefined)n.textContent=value;if(cls)n.className=cls;return n;};
    const source=sample=>sample?.source?`${sample.source} · ${quality(sample)} · ${stamp(sample.source_stamp)} · 龄期 ${Math.round((sample.age_ms??0)+delta)} ms`:"尚未收到可信来源的反馈";
    const badge=(id,label,cls="")=>{text(id,label);$(id).className="tag "+cls;};
    const vision=perceptionView(s,quality,live,delta);
    badge("perception-quality",quality(s.perception)==="正常"?`${vision.rows.length} 块电池`:quality(s.perception),vision.data?"good":"");
    text("perception-target",vision.target?`准备抓取 · ${vision.target}`:"等待任务目标 ID");
    text("perception-source",source(s.perception));
    text("perception-frame",vision.data?.frame_id||null);
    const list=$("detections"), context=JSON.stringify([s.session_id,s.perception?.source]);
    const opened=new Set(list.dataset.context===context?[...list.querySelectorAll("details[open]")].map(n=>n.dataset.key):[]);
    list.dataset.context=context;list.replaceChildren();
    function poseCell(v, key) {
      const cell=el("td");
      cell.append(el("span",v.usable?`[${v.pose.position.map(x=>x.toFixed(3)).join(", ")}] m`:"位姿不可用",v.usable?"pose-values":"muted"));
      if(v.pose) {
        const details=el("details");details.append(el("summary",v.pose.frame_id||"坐标系未提供"));
        details.dataset.key=key;details.open=opened.has(key);
        details.append(el("p",stamp(v.pose.stamp_ns)),el("p",`四元数 xyzw · ${v.pose.orientation.map(x=>x===null?"?":x.toFixed(4)).join(", ")}`));
        if(v.pose.issues.length)details.append(el("p",v.pose.issues.join("；"),"warning-text"));
        if(v.age!==null)details.append(el("p",`位姿龄期 ${Math.round(v.age)} ms`));cell.append(details);
      }
      return cell;
    }
    for(const row of vision.rows){
      const tr=el("tr",undefined,row.target?"target-detection":"");tr.dataset.objectId=row.object_id;
      const id=el("td");id.append(el("strong",row.object_id||"ID 缺失"));
      if(row.target)id.append(el("small","当前目标","target-label"));
      if(!row.identity_valid)id.append(el("small","ID 缺失或重复","warning-text"));
      const color=el("td",colors[row.color]||`未知 (${row.color})`);color.dataset.color=String(row.color);
      tr.append(id,color,el("td",row.confidence===null?"未提供":`${(row.confidence*100).toFixed(1)}%`),poseCell(row.body,JSON.stringify([row.object_id,"body"])),poseCell(row.ring,JSON.stringify([row.object_id,"ring"])));
      const valid=el("td",String(row.ring_pose_valid),row.ring_pose_valid&&row.ring.usable?"good-text":"warning-text");
      if(row.ring_pose_valid&&!row.ring.usable)valid.append(el("small","发布值 true；位姿不可用","warning-text"));tr.append(valid);list.append(tr);
    }
    $("perception-empty").hidden=vision.rows.length>0;
    text("perception-empty",vision.data?"当前有效检测帧中没有电池。":"等待有效检测帧；缺失或过期时不保留目标高亮。");
    text("perception-target-note",vision.target&&vision.data&&!vision.rows.some(x=>x.target)?"当前检测帧中没有可唯一对应的目标 ID。":"列表高亮来自任务反馈 ID；检测与图像按各自时间戳显示，尚未做帧同步叠图。");

    const plan=plannerView(s,quality,live,delta),p=plan.data;
    const states={WAITING:"等待",PLANNING:"规划中",SUCCEEDED:"成功",FAILED:"失败"};
    badge("planner-state",p?`${states[p.state]} · ${p.state}`:quality(s.planner),p?.state==="FAILED"?"bad":p?.state==="SUCCEEDED"&&!plan.foreign?"good":"");
    for(const [id,key] of [["planner-primitive","primitive"],["planner-id","plan_id"],["planner-task","task_id"],["planner-command","command_id"],["planner-trajectory","trajectory_id"],["planner-points","point_count"],["planner-code","error_code"]])text(id,p?.[key]);
    text("planner-duration",p?.expected_duration_sec!=null?`${p.expected_duration_sec.toFixed(3)} s`:null);
    for(const [id,key] of [["planner-limits","joint_limits_ok"],["planner-collision","collision_free"],["planner-start","start_state_consistent"]]){
      text(id,p?.[key]===true?"通过":p?.[key]===false?"未通过":"未报告");
      $(id).className=p?.[key]===false?"bad-text":p?.[key]===true&&!plan.foreign?"good-text":"muted";
    }
    text("planner-reason",p?.reason);$("planner-reason").className="result "+(p?.state==="FAILED"?"failure":"neutral");
    text("planner-association",plan.foreign?"其他任务的规划反馈；不作为当前任务结论":plan.associated?"任务 ID 与当前任务一致":"独立规划反馈；未建立当前任务归属");
    text("planner-source",source(s.planner));

    const tool=toolView(s,quality,live,delta),t=tool.data;
    badge("tool-quality",quality(s.tool),t?.state===5?"bad":"");
    text("tool-type",t?toolNames[t.tool_type]||t.tool_type||null:null);
    text("tool-estimate",t?toolStates[t.state]||`Unknown (${t.state})`:null);
    text("tool-object",t?.object_id||null);text("tool-independent",t?String(t.verified):null);
    text("tool-evidence-level",t?evidence[t.evidence_level]||`未知 (${t.evidence_level})`:null);
    text("tool-mounted",tool.mounted===true?"已独立确认挂载":tool.mounted===false?"未确认挂载":"未知");
    $("tool-mounted").className=tool.mounted?"good-text":"warning-text";
    text("tool-detail",tool.inconsistent?"verified 与证据等级矛盾，不能认定独立挂载。":t?.detail||"工具估计与独立挂载确认分别显示。");
    // Same evidence boundary in the P0 safety summary; verified may confirm Detached.
    text("tool-verified",tool.inconsistent?"证据矛盾":t?`${t.verified?"当前状态已验证":"当前状态未验证"} · ${tool.mounted?"挂载已确认":"挂载未确认"}`:"未知");
    $("tool-verified").className=tool.mounted?"good-text":"muted";
    for(const [prefix,v] of [["test-lift",tool.testLift],["seated",tool.seated]]){
      text(prefix+"-verified",v?.verified===true?"已验证":v?.verified===false?"未验证":"未提供");
      $(prefix+"-verified").className=v?.verified===true?"good-text":"muted";
      text(prefix+"-source",v?`${evidence[v.evidence_level]||"等级未提供"} · ${v.source||"来源未提供"} · ${stamp(v.stamp_ns)}${v.reason?" · "+v.reason:""}`:"等待独立验证证据");
    }
    text("tool-proof-context",tool.matched?`验证 ${tool.proof.verification_id} · 任务 ${tool.proof.task_id} · 电池 ${tool.proof.object_id}`:tool.proof?"验证记录与当前任务 / 电池 / 工具不匹配，未用于确认。":"尚未收到试提与落座验证记录。");
    text("tool-source",source(s.tool));text("tool-proof-source",source(s.tool_verification));
  }
  const api={stamp,poseView,perceptionView,plannerView,toolView,render};
  if(typeof module!=="undefined"&&module.exports)module.exports=api;else root.P1=api;
})(typeof window!=="undefined"?window:globalThis);
