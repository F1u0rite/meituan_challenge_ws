/* P2 evidence health and history views. Authored with OpenAI Codex. */
(function(root){
  "use strict";
  const labels={task:"Task Server",pickplace:"PickPlace FSM",vision:"Vision",calibration:"Calibration / TF",planner:"MoveIt Planner",execution:"FJT Execution",tool:"Tool Manager",robot:"Robot / Controller",system:"系统"};
  const show=v=>v===true?"是":v===false?"否":"未知";
  function modules(s,live,quality,delta=0){
    const fresh=k=>!!live&&s.mode!=="mock"&&quality(s[k])==="正常";
    const data=k=>fresh(k)?s[k].data:null;
    const graph=live&&s.connection?.connected&&s.connection.graph_age_ms!=null&&s.connection.graph_age_ms+delta<s.freshness_limit_ms;
    const service=key=>graph?(s.connection[key]?.length===1?"服务可见":s.connection[key]?.length>1?"服务冲突":"未发现服务"):"服务未知";
    const availability=(key,sample)=>{
      if(!graph)return "在线情况未知";
      if(key==="task"||key==="pickplace")return service(key==="task"?"action_servers":"pick_servers");
      if(key==="execution"){
        const servers=s.connection.fjt_servers?.[s.connection.fjt_action]||[];
        return servers.length===1?"服务可见 / 等待反馈":servers.length>1?"服务冲突":"未发现 FJT 服务";
      }
      const publisher=s.connection.publishers?.[sample];
      return publisher?.selected?"发布源可见 / 等待有效反馈":publisher?.candidates?.length>1?"发布源冲突":"尚无有效反馈";
    };
    const row=(key,sample,details,bad,known=true,extra="")=>({key,label:labels[key],state:fresh(sample)?bad?"abnormal":known?"normal":"unknown":"unknown",online:fresh(sample)?"反馈在线":availability(key,sample),details,source:s[sample]?.source,age:s[sample]?.age_ms,extra});
    const t=data("task_state"),p=data("pick_status"),pf=data("pick_feedback"),cp=data("cpp_pick"),v=data("perception"),tf=data("tf"),c=data("calibration"),pl=data("planner"),f=data("fjt_status"),tool=data("tool"),r=data("robot"),m=data("motion");
    const task=row("task","task_state",t?`任务 ${t.task_id||"无"} · ${t.state||"未知"}`:"当前任务 / FSM 未知",["FAULT","FAILED"].includes(t?.state),!!t?.state&&t.state!=="UNKNOWN"&&graph&&s.connection?.action_ready);
    if(graph&&s.connection?.action_servers?.length>1)task.state="abnormal";
    const pickIdentity=s.pick_status?.available&&!s.pick_status?.data?.ambiguous?s.pick_status.data?.goal_id:null;
    const pickTerminal=["SUCCEEDED","ABORTED","CANCELED"].includes(s.pick_status?.data?.status);
    const currentPick=pf&&pf.goal_id===pickIdentity&&!pickTerminal?pf:null;
    const sub=currentPick?.substate||cp?.substate||cp?.state;
    const pick=row("pickplace",cp?"cpp_pick":currentPick?"pick_feedback":"pick_status",`Goal ${currentPick?.goal_id||p?.goal_id||"未知"} · ${p?.status||"状态未知"} · 子状态 ${sub||(pickTerminal?"未提供（Goal 已终结）":"未知")}`,p?.status==="ABORTED"||p?.ambiguous===true||sub==="FAULT",!!sub&&sub!=="UNKNOWN"||!!p&&pickTerminal);
    if(graph&&s.connection?.pick_servers?.length>1)pick.state="abnormal";
    const cal=row("calibration","calibration",`TF 可查询 ${show(tf?.available)} · 标定有效 ${show(c?.valid)}${c?.calibration_id?" · "+c.calibration_id:""}`,c?.valid===false,!!c&&c.valid===true&&tf?.available===true);
    if(tf?.available===false)cal.state="abnormal";
    cal.extra=(tf?.pairs||[]).map(p=>`${p.target} ← ${p.source}: ${p.reason}`).join("；")||"未配置需检查的 TF 坐标链";
    if(c?.reason)cal.extra+="；"+c.reason;
    const ff=data("fjt_feedback"),feedbackFjt=ff&&s.fjt_status?.available&&!s.fjt_status.data?.ambiguous&&ff.goal_id===s.fjt_status.data?.goal_id;
    const fault=m?.fault_locked;
    const robot=row("robot","robot",`通信 ${show(r?.communication_ok)} · Ready ${show(r?.robot_ready)} · Fault ${show(fault)} · StopConfirmed ${r&&s.stop_evidence?.confirmed?"独立确认":"未确认"}`,
      r?.communication_ok===false||r?.robot_ready===false||r?.safety_normal===false||fault===true,
      r?.simulation_mode===false&&r?.communication_ok===true&&r?.robot_ready===true&&r?.safety_normal===true&&fault===false&&(r?.motion_active===true||s.stop_evidence?.confirmed===true));
    return [task,pick,row("vision","perception",v?`${v.detections?.length??0} 个检测 · 数据 ${s.perception.age_ms} ms`:"检测数据未知 / 过期",false),cal,
      row("planner","planner",pl?`${pl.state} · ${pl.primitive||"原语未知"}${pl.reason?" · "+pl.reason:""}`:"规划状态未知 / 尚未接入",pl?.state==="FAILED"||[pl?.joint_limits_ok,pl?.collision_free,pl?.start_state_consistent].includes(false)),
      row("execution",!f&&feedbackFjt?"fjt_feedback":"fjt_status",f?`${f.idle?"没有活动 Goal":f.status} · ${f.goal_id||"UUID 未知"}`:feedbackFjt?`${ff.goal_id} · 实际反馈在线；Status 事件已过期`:"Goal 执行状态未知 / 尚未接入",f?.ambiguous===true||f?.status==="ABORTED"||(s.fjt_result?.goal_id===f?.goal_id&&s.fjt_result?.error_code!=null&&s.fjt_result.error_code!==0),!!f&&(f.idle===true||f.status!=="UNKNOWN")),
      row("tool","tool",tool?`${({0:"Unknown",1:"Detached",2:"Engaging",3:"Attached",4:"Releasing",5:"Fault"})[tool.state]||"未知"} · 电池 ${tool.object_id||"未提供"} · verified ${show(tool.verified)}`:"工具状态未知 / 尚未接入",tool?.state===5,tool?.state>0&&tool?.state<=5),robot];
  }
  function render(s,live,quality,delta=0){
    const $=id=>document.getElementById(id),el=(tag,value,cls)=>{const n=document.createElement(tag);if(value!=null)n.textContent=value;if(cls)n.className=cls;return n;};
    const rows=modules(s,live,quality,delta),body=$("system-modules");body.replaceChildren();
    const names={normal:"正常",abnormal:"异常",unknown:"未知 / 尚未接入"};
    for(const row of rows){
      const tr=el("tr"),state=el("td"),detail=el("td"),source=el("td");
      state.append(el("span",names[row.state],"tag "+({normal:"good",abnormal:"bad",unknown:""})[row.state]),el("small",row.online));
      detail.append(el("span",row.details));if(row.extra)detail.append(el("small",row.extra));
      source.append(el("span",row.source||"未提供"),el("small",row.age==null?"没有有效时间戳":`最近快照数据龄期 ${row.age} ms`));
      tr.dataset.module=row.key;tr.dataset.state=row.state;tr.append(el("th",row.label),state,detail,source);body.append(tr);
    }
    $("system-summary").textContent=`正常 ${rows.filter(r=>r.state==="normal").length} · 异常 ${rows.filter(r=>r.state==="abnormal").length} · 未知 ${rows.filter(r=>r.state==="unknown").length}`;
    const fail=s.diagnostics?.latest_failure;
    $("latest-failure").textContent=fail?`${fail.at} · ${labels[fail.module]||fail.module} · ${fail.task_id||"任务未关联"} · ${fail.error_code??"无错误码"} · ${fail.message}`:"本会话尚未记录失败";
    $("latest-failure").className="result "+(fail?"bad":"neutral");
  }
  const api={modules,labels,render};if(typeof module!=="undefined"&&module.exports)module.exports=api;else root.P2=api;
})(typeof window!=="undefined"?window:globalThis);
