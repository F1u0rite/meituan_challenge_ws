/* P0 observations. Authored with OpenAI Codex; no motion inference or command API. */
(function (root) {
  "use strict";
  const colors = {1: "红", 2: "蓝", 3: "黄", 4: "绿"};
  const terminal = new Set(["SUCCEEDED", "ABORTED", "CANCELED", "REJECTED"]);
  const taskFlow = ["INIT", "SELF_CHECK", "WAIT_TASK", "VALIDATE_TASK", "SCAN_SCENE", "EXECUTE_ITEM", "RECORD_RESULT", "FINISHED"];
  const pickFlow = ["RESOLVE_TARGET", "PLAN_ACQUIRE", "MOVE_PRE_ALIGN", "ENGAGE", "VERIFY_ENGAGEMENT", "TEST_LIFT", "VERIFY_ATTACHED", "LIFT", "TRANSPORT", "MOVE_PRE_SEAT", "SEAT", "VERIFY_SEATED", "UNLOAD", "DISENGAGE", "RETREAT", "VERIFY_PLACED", "SUCCESS"];
  const exceptions = ["RECOVERY", "CANCEL_PENDING", "CANCELLED", "TASK_FAILED", "FAILED", "FAULT"];
  const names = {INIT:"初始化", SELF_CHECK:"自检", WAIT_TASK:"等待任务", VALIDATE_TASK:"校验口令", SCAN_SCENE:"观察场景", EXECUTE_ITEM:"执行单块", RECORD_RESULT:"记录结果", FINISHED:"任务结束", RESOLVE_TARGET:"确认目标", PLAN_ACQUIRE:"规划抓取", MOVE_PRE_ALIGN:"预对准", ENGAGE:"接合", VERIFY_ENGAGEMENT:"验证接合", TEST_LIFT:"试提", VERIFY_ATTACHED:"验证附着", LIFT:"抬升", TRANSPORT:"搬运", MOVE_PRE_SEAT:"放置前对准", SEAT:"落座", VERIFY_SEATED:"验证承托", UNLOAD:"卸载", DISENGAGE:"分离", RETREAT:"退离", VERIFY_PLACED:"验证放置", SUCCESS:"单块成功", RECOVERY:"恢复处理", CANCEL_PENDING:"取消处理中", CANCELLED:"已取消", TASK_FAILED:"任务失败", FAILED:"单块失败", FAULT:"故障锁定"};
  const number = value => typeof value === "number" && Number.isFinite(value);
  const valid = (sample, quality) => quality(sample) === "正常";
  const data = (sample, quality) => valid(sample, quality) ? sample.data : null;

  function taskView(s, quality, live = true, delta = 0) {
    const state = data(s.task_state, quality);
    const foreign = Boolean(state?.task_id && (!s.task || state.task_id !== s.task.task_id));
    const task = foreign ? null : s.task;
    const done = terminal.has(task?.status);
    const feedbackLive = live && Boolean(task) && (s.mode === "mock" ||
      (task.feedback_fresh && number(task.feedback_age_ms) && task.feedback_age_ms + delta < s.freshness_limit_ms));
    const targetLive = feedbackLive && !done;
    const goal = task?.goal;
    let taskState = state?.state || (targetLive ? task.state : "");
    let pickState = state?.substate || (targetLive ? task.substate : "");
    if (done) {
      if (!exceptions.includes(taskState) && !["FINISHED", "WAIT_TASK"].includes(taskState)) taskState = "";
      if (!exceptions.includes(pickState) && pickState !== "SUCCESS") pickState = "";
    }
    if (!live) { taskState = ""; pickState = ""; }
    const index = targetLive && Number.isInteger(task.current_index) && task.current_index >= 0
      && task.current_index < task.total_count ? task.current_index : null;
    const result = task?.result;
    const finalStatus = result ? ({ABORTED:"FAILED", SUCCEEDED:"SUCCEEDED", CANCELED:"CANCELED"}[task.status] || "UNKNOWN") : null;
    return {task, foreign, id: state?.task_id || task?.task_id || null, mode: goal?.mode,
      colors: goal?.ordered_colors || null, index,
      object: targetLive ? task.current_object_id || null : null,
      slot: targetLive ? task.current_slot || null : null,
      color: index !== null ? goal?.ordered_colors?.[index] : null,
      taskState, pickState, targetLive, done, result, finalStatus,
      completed: result ? result.completed_count : state?.completed_count ?? (targetLive ? task.completed_count : null),
      total: state?.total_count || task?.total_count || null};
  }

  function jointView(s, quality) {
    // Actual and reference values always come from the same controller sample.
    const candidates = [s.fjt_feedback, s.controller].filter(x => valid(x, quality));
    const sample = candidates.sort((a,b) => a.age_ms - b.age_ms)[0] || (valid(s.joints, quality) ? s.joints : null);
    if (!sample) return {sample:null, rows:[]};
    const d = sample.data;
    const observedNames = d.joint_names || d.name || [];
    if (new Set(observedNames).size !== observedNames.length) return {sample:null, rows:[], issue:"关节名重复，无法对应数值"};
    const rows = observedNames.map((name, i) => {
      const actual = (d.actual_position || d.position || [])[i];
      const velocity = (d.actual_velocity || d.velocity || [])[i];
      const target = (d.desired_position || [])[i];
      return {name, actual: number(actual) ? actual : null, velocity: number(velocity) ? velocity : null,
        target: number(target) ? target : null, error: number(actual) && number(target) ? target - actual : null};
    });
    return {sample, rows};
  }

  function motionView(s, quality) {
    let execution = data(s.motion, quality);
    const feedback = data(s.fjt_feedback, quality);
    const status = data(s.fjt_status, quality);
    const robot = data(s.robot, quality);
    const observedGoal = feedback?.goal_id || status?.goal_id;
    const mismatch = Boolean(execution?.fjt_goal_id && observedGoal && execution.fjt_goal_id !== observedGoal);
    if (mismatch) execution = null;
    const result = s.fjt_result;
    return {execution, feedback, status, result, mismatch,
      physical: typeof robot?.motion_active === "boolean" ? (robot.motion_active ? "运动中" : "未在运动") : "未知",
      elapsed: execution?.elapsed_sec ?? feedback?.elapsed_sec ?? null,
      expected: execution?.expected_duration_sec ?? null, progress: execution?.progress ?? null};
  }

  function safetyView(s, quality, live) {
    const robot = data(s.robot, quality);
    const state = data(s.task_state, quality);
    const cpp = [s.cpp_task, s.cpp_pick].map(x => data(x, quality)).filter(Boolean);
    const execution = data(s.motion, quality);
    const locks = [...cpp.map(x => x.locked), execution?.fault_locked].filter(x => typeof x === "boolean");
    const fault = locks.includes(true) || state?.state === "FAULT" || execution?.state === "FAULT";
    const confirmed = Boolean(live && s.mode === "ros" && s.stop_evidence?.confirmed
      && robot?.stop_confirmed === true && robot.motion_active === false && robot.simulation_mode === false
      && s.stop_evidence.post_request);
    const task = s.task;
    const canceled = Boolean(task?.cancel_requested_at || task?.cancel_ack || task?.status === "CANCELING");
    return {robot, confirmed, fault, lockKnown: fault || locks.length > 0,
      stop: s.stop_request, cancel: canceled ? task?.cancel_ack : null, canceled,
      final: canceled && task?.result ? {status: task.status, ...task.result} : null};
  }

  function render(s, options) {
    const {quality, live, delta} = options;
    const $ = id => document.getElementById(id);
    const text = (id, value, fallback="未提供") => { $(id).textContent = value ?? fallback; };
    const el = (tag, value, cls) => { const n=document.createElement(tag); if(value!==undefined)n.textContent=value; if(cls)n.className=cls; return n; };
    const source = sample => sample?.source ? `${sample.source} · ${quality(sample)} · ${sample.age_ms ?? "?"} ms` : "尚未收到可信来源的反馈";
    const v = taskView(s, quality, live, delta);
    text("task-mode", v.mode === 1 ? "基础任务" : v.mode === 2 ? "序列任务" : null);
    text("task-current-item", v.index !== null ? `第 ${v.index + 1} 块 / 共 ${v.total} 块` : null, v.done ? "任务已结束" : "等待执行反馈");
    text("task-object", v.object); text("task-target-color", colors[v.color] ? colors[v.color] + "色（口令）" : null);
    text("task-slot", v.slot);
    text("task-feedback-quality", v.foreign ? "外部任务；未取得原始口令" : v.done ? "已保存终态" : v.targetLive ? "执行反馈有效" : "执行反馈缺失或过期");
    $("task-colors").replaceChildren();
    if (!v.colors) $("task-colors").append(el("span", "未取得口令", "muted"));
    else v.colors.forEach((color,i) => {
      const n=el("span", `${i+1}. ${colors[color] || "未知"} → ${v.mode === 1 ? "T0" : "P"+(i+1)}`, "command-chip");
      n.dataset.color=String(color); if(i===v.index)n.classList.add("current"); $("task-colors").append(n);
    });
    text("task-id", v.id, "—");
    if(v.foreign){
      text("task-status","外部任务");text("elapsed",null,"未取得");
      text("progress-label","外部任务 / 本会话未取得原始 Goal");
    }
    text("progress-count", `${v.completed ?? "—"} / ${v.total ?? "—"}`);
    $("progress").value = v.completed ?? 0;
    $("progress").max = v.total || 1;
    text("fsm-state", v.taskState || "未知 / 反馈不可用");
    text("pick-state", v.pickState || "未知 / 反馈不可用");
    function flow(id, list, current) {
      const container=$(id);
      if (!container.children.length) [...list, ...exceptions.filter(x=>!list.includes(x))].forEach(key=>{
        const n=el("li", undefined, exceptions.includes(key) ? "fsm-node exception-node" : "fsm-node");
        n.dataset.state=key; n.append(el("span", names[key]),el("code",key)); container.append(n);
      });
      for(const node of container.children){
        const active=node.dataset.state===current;
        node.classList.toggle("current",active);
        if(active)node.setAttribute("aria-current","step");else node.removeAttribute("aria-current");
      }
    }
    flow("task-flow",taskFlow,v.taskState); flow("pick-flow",pickFlow,v.pickState);
    const known = [...taskFlow,...pickFlow,...exceptions];
    text("fsm-flow-note", [v.taskState,v.pickState].filter(x=>x&&!known.includes(x)).length
      ? "收到未映射状态："+[v.taskState,v.pickState].filter(x=>x&&!known.includes(x)).join(" / ")
      : "黄色表示当前反馈阶段；其余阶段不表示已完成。序列任务逐块重复观察、执行与记录。");
    $("task-result").hidden=!v.result;
    if(v.result){
      $("task-result").hidden=false;
      text("task-result", `${v.finalStatus}${v.taskState === "FAULT" ? " · FSM FAULT" : ""} · ROS ${v.task.status} / ${v.result.error_code} · ${v.result.message}`);
      $("task-result").className="result"+(v.finalStatus==="FAILED"?" bad":v.finalStatus==="SUCCEEDED"?"":" neutral");
    }

    const joints=jointView(s,quality);
    $("joints").replaceChildren();
    const unit = value => number(value) ? (value*180/Math.PI).toFixed(2) : "—";
    if(!joints.rows.length){
      const row=el("tr");const cell=el("td",joints.issue||"尚未收到关节反馈；不以 0 填充缺失值","muted");cell.colSpan=5;row.append(cell);$("joints").append(row);
    }else joints.rows.forEach(r=>{
      const row=el("tr");[r.name,unit(r.actual),unit(r.velocity),unit(r.target),unit(r.error)].forEach(value=>row.append(el("td",value)));$("joints").append(row);
    });
    text("joints-quality",joints.sample ? `${joints.rows.length} 个关节 / 有效反馈` : "无有效反馈");
    text("joints-note",joints.sample ? source(joints.sample)+"。目标与误差使用同帧数据，误差 = 目标 − 实际。" : "按消息中的关节名对应位置与速度；缺失速度不由位置差分推测。");

    const m=motionView(s,quality);
    text("motion-physical",m.physical);
    text("motion-state",m.execution?.state, "执行端未提供");
    text("motion-primitive",m.execution?.primitive);text("motion-command",m.execution?.command_id);
    text("motion-trajectory",m.execution?.trajectory_id);
    text("motion-goal",m.feedback?.goal_id || m.status?.goal_id);
    text("motion-native-state",m.status?.ambiguous ? "多个活动 Goal，归属不明确" : m.status?.status ||
      (s.fjt_status?.data?.status ? `上次 ${s.fjt_status.data.status} · ${quality(s.fjt_status)}` : null));
    text("motion-elapsed",number(m.elapsed)?m.elapsed.toFixed(2)+" s":null);
    text("motion-expected",number(m.expected)?m.expected.toFixed(2)+" s":null);
    text("motion-progress-value",number(m.progress)?(m.progress*100).toFixed(1)+"%":null);
    $("motion-progress").hidden=!number(m.progress);$("motion-progress").value=m.progress??0;
    text("motion-reason",m.execution?.reason||m.execution?.detail);
    text("motion-execution-result",typeof m.execution?.success === "boolean" ? (m.execution.success ? "成功" : "未成功") : null);
    text("motion-error-code",m.execution?.error_code);
    text("motion-result",m.result?`${m.result.status} / ${m.result.error_code} · ${m.result.message || "无附加说明"}（${m.result.goal_id}）`:null,"尚无执行结果");
    $("motion-result").className="result"+(!m.result?" neutral":!(m.result.status==="SUCCEEDED"&&m.result.error_code===0)?" bad":"");
    text("motion-source",m.mismatch?"执行端遥测与 FJT Goal UUID 不一致，已隔离两路数据。":source(s.motion)+"；FJT："+source(s.fjt_feedback));

    const safety=safetyView(s,quality,live);
    const graphAge=s.connection?.graph_age_ms;
    const graphLive=live&&s.connection?.connected&&number(graphAge)&&graphAge+delta<s.freshness_limit_ms;
    text("safety-ros",graphLive?"ROS 图发现正常":live?"ROS 图未确认":"浏览器断线或暂停");
    text("safety-lock",safety.fault?"FAULT / 锁定":safety.lockKnown?"已报告未锁定":"未报告锁定位");
    $("safety-lock").className=safety.fault?"bad-text":"muted";
    text("safety-stop-request",safety.stop ? safety.stop.request_delivered===true?"已送达（服务回执）":safety.stop.request_delivered===false?"未送达 / 被拒绝":"等待回执 / 未知":"未请求");
    text("safety-stop-confirmed",safety.confirmed?"已确认 · 独立 RobotState":"未确认");
    $("safety-stop-confirmed").className=safety.confirmed?"good-text":"warning-text";
    text("safety-stop-detail",s.mode!=="ros"?"演练环境不提供实机独立停稳证据":s.stop_evidence?.reason||"等待独立反馈");
    text("safety-cancel-ack",safety.cancel?safety.cancel.request_delivered===true?"已受理":safety.cancel.request_delivered===false?"未受理":"未知":safety.canceled?"等待回执":"未请求取消");
    text("safety-cancel-final",safety.final?`${safety.final.status} / ${safety.final.error_code} · ${safety.final.message}`:safety.canceled?"尚无最终结果":"未请求取消");
    $("safety-cancel-final").className=safety.final?.status==="ABORTED"?"bad-text":"";
    text("safety-stop-source",source(s.robot));

  }
  const api={taskView,jointView,motionView,safetyView,render};
  if(typeof module!=="undefined"&&module.exports)module.exports=api;else root.P0=api;
})(typeof window!=="undefined"?window:globalThis);
