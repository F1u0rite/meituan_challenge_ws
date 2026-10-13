"""ROS observations and owned Action lifecycle. No simulated progress or motion logic.

Authored with OpenAI Codex; mtc_interfaces reference and licenses in provenance.json.
The transport invokes these methods with actual ROS messages converted to dictionaries.
"""

from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time
import uuid

from mtc_webui.config import RosConfig
from mtc_webui.mock_backend import ApiError, MockBackend, iso_time
from mtc_webui.telemetry import ACTION_STATES
from mtc_webui.diagnostics import Journal, goal_key, trace_log, MODULES


ACTIVE = {"SUBMITTING", "ACCEPTED", "EXECUTING", "CANCELING", "UNKNOWN"}
TERMINAL = {"SUCCEEDED", "ABORTED", "CANCELED", "REJECTED"}


class RosBackend:
    mode = "ros"
    scope_note = "实机操作台启动；只接受 ROS 原始反馈，不自动提交或恢复运动"
    def __init__(self, config=None, output_dir=None, clock=time.monotonic, wall_clock=time.time):
        self.config = config or RosConfig()
        self.clock, self.wall_clock = clock, wall_clock
        self.lock = threading.RLock()
        self.session_id = str(uuid.uuid4())
        self.tasks, self.events, self.samples, self.frames = {}, [], {}, {}
        self.current_id = None
        self.seq = self.snapshot_seq = 0
        self.transport = None
        self.connection = {"connected": False, "reason": "等待 ROS 连接", "action_ready": False,
                           "stop_ready": False, "publishers": {}, "image_topics": []}
        self.stop_request = None
        self.stop_boundary = None
        self.fjt_current_id = None
        self.fjt_result = None
        self.pick_current_id = None
        self.pick_result = None
        self.output_dir = Path(output_dir) if output_dir else None
        if self.output_dir:
            self.output_dir.mkdir(parents=True, exist_ok=False)
            (self.output_dir / "connection.json").write_text(json.dumps(asdict(self.config), indent=2) + "\n")
        self.journal = Journal(self.output_dir)
        self._event("SESSION_STARTED", self.scope_note)

    def _public(self, task):
        result = deepcopy({k: v for k, v in task.items() if not k.startswith("_")})
        end = task["_ended"] if task["_ended"] is not None else self.clock()
        result["elapsed_ms"] = round((end - task["_start"]) * 1000)
        received = task.get("_feedback_received")
        result["feedback_age_ms"] = None if received is None else round((self.clock() - received) * 1000)
        result["feedback_fresh"] = (self.connection["connected"] and received is not None
                                    and result["feedback_age_ms"] < self.config.freshness_ms)
        return result

    def _persist(self, task):
        if self.output_dir:
            path = self.output_dir / (task["task_id"] + ".json")
            temp = path.with_suffix(".tmp")
            temp.write_text(json.dumps(self._public(task), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            temp.replace(path)

    def _event(self, kind, message, task=None, level="info", **extra):
        self.seq += 1
        module = extra.pop("module", "task" if task else "system")
        identity = extra.pop("goal_id", task["goal_id"] if task else None)
        task_id = extra.pop("task_id", task["task_id"] if task else None)
        if task:
            self.journal.node(task["goal_id"], "ExecuteTask", task["task_id"])
        event = dict(seq=self.seq, at=iso_time(self.wall_clock()), kind=kind, message=message,
                     level=level, mode=self.mode, module=module, task_id=task_id,
                     goal_id=identity, **extra)
        self.journal.append(event)
        live_event = dict(event, message=event['message'][:3000], preview_truncated=True) if len(event['message']) > 3000 else event
        self.events.append(live_event)
        self.events = self.events[-500:]
        if task:
            task["events"].append(live_event)
            task["events"] = task["events"][-300:]
            self._persist(task)
        if self.output_dir:
            with (self.output_dir / "events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(event, ensure_ascii=False) + "\n")
        return event

    def connection_update(self, **fields):
        with self.lock:
            previous = (self.connection["connected"], self.connection["reason"])
            self.connection.update(fields)
            if "nodes" in fields or "graph" in fields:
                self._graph_received = self.clock()
            if previous != (self.connection["connected"], self.connection["reason"]):
                self._event("ROS_CONNECTION", self.connection["reason"])

    def observe(self, kind, data, source, stamp_ns, source_age_ms):
        """Only increasing source timestamps refresh a sample, even after DDS retransmit."""
        with self.lock:
            old = self.samples.get(kind)
            if old and old["source"] == source and stamp_ns <= old["stamp_ns"]:
                return
            self.samples[kind] = dict(data=deepcopy(data), source=source, stamp_ns=stamp_ns,
                                      source_age_ms=source_age_ms, _received=self.clock(),
                                      received_at=iso_time(self.wall_clock()))
            state_fields = {
                "perception": ("frame_id", "detections"),
                "task_state": ("task_id", "state", "substate", "completed_count", "last_error_code"),
                "robot": ("communication_ok", "robot_ready", "motion_active", "safety_normal", "stop_confirmed", "simulation_mode"),
                "tool": ("state", "object_id", "verified", "evidence_level"),
                "planner": ("task_id", "plan_id", "state", "error_code", "reason", "joint_limits_ok", "collision_free", "start_state_consistent"),
                "motion": ("task_id", "fjt_goal_id", "command_id", "state", "fault_locked", "error_code", "reason"),
                "pick_status": ("goal_id", "status", "ambiguous"), "pick_feedback": ("goal_id", "substate"),
                "fjt_status": ("goal_id", "status", "ambiguous"),
                "calibration": ("calibration_id", "valid", "evidence", "reference", "reason"),
                "tf": ("available",), "tool_verification": ("task_id", "object_id", "tool_type", "verification_id", "test_lift", "seated"),
            }
            keys = state_fields.get(kind)
            usable = stamp_ns > 0 and source_age_ms is not None and -500 <= source_age_ms < self.config.freshness_ms
            if keys and usable:
                current = {k: data.get(k) for k in keys}
                previous = {k: old['data'].get(k) for k in keys} if old and old['source'] == source else None
                if kind == "perception":
                    def detection_state(value):
                        return dict(frame_id=value.get("frame_id"), detections=[{k: d.get(k) for k in ("object_id", "color", "identity_valid", "ring_pose_valid")} for d in value.get("detections", [])])
                    current = detection_state(data)
                    previous = detection_state(old["data"]) if old and old["source"] == source else None
                if current != previous:
                    module = {"task_state": "task", "motion": "execution", "fjt_status": "execution", "pick_status": "pickplace",
                              "pick_feedback": "pickplace", "tool_verification": "tool", "tf": "calibration", "perception": "vision"}.get(kind, kind)
                    goal = data.get("goal_id") or data.get("fjt_goal_id")
                    task_id = data.get("task_id")
                    failure = (data.get("state") in ("FAILED", "FAULT") or data.get("status") == "ABORTED"
                               or kind == "tool" and data.get("state") == 5
                               or kind == "calibration" and data.get("valid") is False
                               or kind == "robot" and any(data.get(k) is False for k in ("communication_ok", "robot_ready", "safety_normal")))
                    self._event("STATE_TRANSITION", data.get("reason") or data.get("detail") or json.dumps(current, ensure_ascii=False),
                                level="error" if failure else "info", module=module, goal_id=goal, task_id=task_id,
                                transition=dict(previous=previous, current=current), data=deepcopy(data),
                                source=source, source_stamp=str(stamp_ns), error_code=data.get("error_code", data.get("last_error_code")))
            if kind == "trace" and usable and (not old or old["data"] != data):
                self.declare_trace(data, source, str(stamp_ns))
            if (kind == "task_state" and stamp_ns > 0 and source_age_ms is not None
                    and -500 <= source_age_ms < self.config.freshness_ms):
                task = self.tasks.get(self.current_id)
                if task and data.get("task_id") == task["task_id"] and task["status"] in ACTIVE:
                    changed = any(task.get(k) != data.get(k) for k in ("state", "substate", "completed_count", "last_error_code"))
                    for key in ("state", "substate", "completed_count", "total_count", "last_error_code", "detail"):
                        task[key] = data[key]
                    if changed:
                        self._event("TASK_STATE", data.get("detail") or data["state"], task)

    def invalidate(self, kind):
        with self.lock:
            self.samples.pop(kind, None)
            self.frames.pop(kind, None)
            if kind == "pick_status":
                self.pick_current_id = None
                self.pick_result = None
                self.samples.pop("pick_feedback", None)
            if kind == "fjt_status":
                self.fjt_current_id = None
                self.fjt_result = None
                self.samples.pop("fjt_feedback", None)

    def _sample(self, kind):
        sample = self.samples.get(kind)
        if not sample:
            return dict(available=False, data=None, source=None, received_at=None, age_ms=None)
        delta = max(0, (self.clock() - sample["_received"]) * 1000)
        source_age = sample["source_age_ms"]
        valid_stamp = sample["stamp_ns"] > 0 and source_age is not None and source_age >= -500
        age = max(0, source_age) + delta if valid_stamp else None
        return dict(available=True, data=deepcopy(sample["data"]), source=sample["source"],
                    received_at=sample["received_at"], source_stamp_ns=sample["stamp_ns"],
                    source_stamp=str(sample["stamp_ns"]),
                    age_ms=round(age) if age is not None else None,
                    fresh=self.connection["connected"] and age is not None and age < self.config.freshness_ms)

    def _reasons(self, include_active=True):
        reasons = []
        if self.config.read_only:
            reasons.append("当前为只读模式")
        if not self.connection["connected"]:
            reasons.append(self.connection["reason"])
        if not self.connection["action_ready"]:
            reasons.append("ExecuteTask Action 服务不可用或存在多个服务端")
        for kind, label in (("task_state", "任务"), ("robot", "机器人"), ("tool", "工具")):
            sample = self._sample(kind)
            if not sample.get("fresh"):
                reasons.append(label + "状态缺失、过期或时间戳无效")
        robot = self._sample("robot")
        if robot.get("fresh"):
            data = robot["data"]
            if data.get("simulation_mode") is not False:
                reasons.append("机器人发布源报告 simulation_mode=true，实机模式禁止提交")
            if not all(data.get(k) is True for k in ("communication_ok", "robot_ready", "safety_normal", "stop_confirmed")) or data.get("motion_active") is not False:
                reasons.append("机器人尚未就绪、联锁异常或未确认停稳")
        state = self._sample("task_state")
        if state.get("fresh") and state["data"].get("state") != "WAIT_TASK":
            reasons.append("FSM 未处于 WAIT_TASK")
        tool = self._sample("tool")
        if tool.get("fresh") and tool["data"].get("state") not in (1,):
            reasons.append("工具尚未报告已分离")
        current = self.tasks.get(self.current_id)
        if include_active and current and current["status"] in ACTIVE:
            reasons.append("当前 Goal 尚未结束或结果未知")
        if current and current.get("_ended") is not None:
            # Never use a pre-result idle sample to enable another physical task.
            if any(self.samples.get(k, {}).get("_received", -1) <= current["_ended"] for k in ("robot", "task_state")):
                reasons.append("等待 Result 之后的新任务与停稳状态")
        if self.stop_request and self.stop_request["status"] == "PENDING":
            reasons.append("软件停止请求等待回执")
        if self.stop_boundary and not self.stop_evidence()["confirmed"]:
            reasons.append("等待请求之后的新独立停稳反馈")
        return list(dict.fromkeys(reasons))

    @staticmethod
    def validate(payload):
        if not isinstance(payload, dict) or "scenario" in payload:
            raise ApiError(400, "INVALID_TASK", "实机 Goal 不接受 Mock 场景参数")
        # Share only field validation with the demo; no fixture state or timers are used.
        MockBackend.validate({**payload, "scenario": "success"})

    def submit(self, payload):
        with self.lock:
            self.validate(payload)
            if payload["task_id"] in self.tasks:
                raise ApiError(409, "DUPLICATE_TASK", "任务 ID 已使用；不会重发")
            reasons = self._reasons()
            if reasons or not self.transport:
                raise ApiError(409, "NOT_READY", "；".join(reasons) or "ROS 未连接")
            task = dict(task_id=payload["task_id"], goal_id=str(uuid.uuid4()), mode=self.mode, owned=True,
                        goal=deepcopy(payload), status="SUBMITTING", state="", substate="", detail="等待 ROS Goal 受理回执",
                        completed_count=0, total_count=len(payload["ordered_colors"]), completed_slots=[],
                        current_index=None, current_slot="", current_object_id="", last_error_code=0,
                        created_at=iso_time(self.wall_clock()), finished_at=None, elapsed_ms=0, result=None,
                        cancel_ack=None, events=[], recordings=[], _start=self.clock(), _ended=None)
            self.tasks[task["task_id"]] = task
            self.current_id = task["task_id"]
            self._event("GOAL_REQUESTED", task["detail"], task)
            self.transport.enqueue("submit", task["goal_id"], deepcopy(payload))
            return dict(task_id=task["task_id"], goal_id=task["goal_id"], status=task["status"])

    def _owned(self, goal_id):
        task = self.tasks.get(self.current_id)
        return task if task and task["goal_id"] == goal_id else None

    def dispatch_allowed(self, goal_id):
        with self.lock:
            task = self._owned(goal_id)
            return bool(task and task["status"] == "SUBMITTING" and not self._reasons(include_active=False))

    def accepted(self, goal_id, accepted, reason="ROS Action 服务端拒绝 Goal"):
        with self.lock:
            task = self._owned(goal_id)
            if not task or task["status"] not in ACTIVE:
                return
            if accepted:
                if task["status"] in {"SUBMITTING", "UNKNOWN"}:
                    task["status"] = "EXECUTING" if task.get("_feedback_seen") else "ACCEPTED"
                task["detail"] = "ROS Action 服务端已受理"
                self._event("GOAL_ACCEPTED", task["detail"], task)
            else:
                task.update(status="REJECTED", detail=reason,
                            finished_at=iso_time(self.wall_clock()), _ended=self.clock())
                self._event("TASK_REJECTED", task["detail"], task, "warning")

    def uncertain(self, goal_id, message):
        with self.lock:
            task = self._owned(goal_id)
            if task and task["status"] in ACTIVE:
                task.update(status="UNKNOWN", detail=message)
                self._event("ACTION_UNKNOWN", message, task, "error")

    def feedback(self, goal_id, data):
        with self.lock:
            task = self._owned(goal_id)
            if not task or task["status"] not in ACTIVE:
                return
            changed = any(task.get(k) != v for k, v in data.items())
            previous = {k: task.get(k) for k in ("state", "substate")}
            task.update(data)
            task["_feedback_seen"] = True
            task["_feedback_received"] = self.clock()
            task["feedback_received_at"] = iso_time(self.wall_clock())
            if task["status"] in {"ACCEPTED", "EXECUTING"}:
                task["status"] = "EXECUTING"
            if changed:
                self._event("FEEDBACK", f"{task['state']} / {task['substate']}", task,
                            data=deepcopy(data), transition=dict(previous=previous, current={k: task.get(k) for k in previous})
                            if any(previous[k] != task.get(k) for k in previous) else None)

    def result(self, goal_id, ros_status, result):
        with self.lock:
            task = self._owned(goal_id)
            if not task or task["status"] not in ACTIVE:
                return
            status = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(ros_status)
            if status is None or (status == "SUCCEEDED") != result["success"]:
                self.uncertain(goal_id, f"ROS 终态与 Result 不一致：status={ros_status}, success={result['success']}")
                task["result"] = deepcopy(result)
                self._persist(task)
                return
            task.update(status=status, result=deepcopy(result), detail=result["message"],
                        completed_count=result["completed_count"], completed_slots=list(result["completed_slots"]),
                        last_error_code=result["error_code"], finished_at=iso_time(self.wall_clock()), _ended=self.clock())
            self._event("RESULT", result["message"], task, "error" if status == "ABORTED" else "info",
                        error_code=result["error_code"], status=status, data=deepcopy(result))

    def cancel(self, task_id, goal_id):
        with self.lock:
            task = self._owned(goal_id)
            if self.config.read_only or not self.connection["connected"] or not task or task["task_id"] != task_id or task["status"] not in {"ACCEPTED", "EXECUTING"}:
                raise ApiError(409, "NOT_CANCELABLE", "只能取消本会话持有的活动 Goal")
            task.update(status="CANCELING", detail="取消已排队，等待 ROS Cancel 回执")
            task["cancel_requested_at"] = iso_time(self.wall_clock())
            self._stop_boundary()
            self._event("CANCEL_REQUESTED", task["detail"], task)
            self.transport.enqueue("cancel", goal_id, None)
            return dict(request_queued=True, request_delivered=None, goal_id=goal_id)

    def cancel_reply(self, goal_id, delivered, message):
        with self.lock:
            task = self._owned(goal_id)
            if not task:
                return
            task["cancel_ack"] = dict(request_delivered=delivered, message=message, at=iso_time(self.wall_clock()))
            if task["status"] == "CANCELING" and delivered is False:
                task["status"] = "EXECUTING"
            self._event("CANCEL_REPLY", message + ("；完成以 C++ Action 最终结果为准" if self.mode == "cpp_mock" else "；停稳以 RobotState 为准"), task, "info" if delivered else "warning")

    def request_stop(self):
        with self.lock:
            if self.config.read_only or not self.connection["connected"] or not self.connection["stop_ready"] or (self.stop_request and self.stop_request["status"] == "PENDING"):
                raise ApiError(409, "STOP_UNAVAILABLE", "软件停止服务不可用或请求仍在等待回执")
            request_id = str(uuid.uuid4())
            self.stop_request = dict(request_id=request_id, status="PENDING", request_delivered=None,
                                     requested_at=iso_time(self.wall_clock()),
                                     message="等待 RequestStop 回执；不代表实体急停或已停稳")
            self._stop_boundary()
            self._event("STOP_REQUESTED", self.stop_request["message"], self.tasks.get(self.current_id))
            self.transport.enqueue("stop", request_id, None)
            return deepcopy(self.stop_request)

    def stop_reply(self, request_id, delivered, message):
        with self.lock:
            if self.stop_request and self.stop_request["request_id"] == request_id:
                self.stop_request.update(status="REPLIED" if delivered is not None else "UNKNOWN",
                                         request_delivered=delivered, message=message,
                                         replied_at=iso_time(self.wall_clock()))
                self._event("STOP_REPLY", message + "；停稳以 RobotState 为准", self.tasks.get(self.current_id))

    def declare_trace(self, data, source, source_stamp=None):
        self.journal.declare(data, source, self.seq + 1)
        self._event("GOAL_LINK", "显式 Goal 关联", module="system", goal_id=data["execute_goal_id"],
                    task_id=data.get("task_id"), data=deepcopy(data), source=source, source_stamp=source_stamp)

    def log(self, name, level, message, source_stamp=None):
        with self.lock:
            normalized = name.strip("/").replace("/", ".")
            mapping = {"task_state": "task", "pick_status": "pickplace", "cpp_task": "task", "cpp_pick": "pickplace",
                       "perception": "vision", "planner": "planner", "calibration": "calibration", "motion": "execution",
                       "fjt_status": "execution", "tool": "tool", "robot": "robot"}
            candidates = []
            for kind, info in self.connection.get("publishers", {}).items():
                for entry in info.get("candidates", []):
                    if entry["node"].strip("/").replace("/", ".") == normalized:
                        candidates.append(mapping.get(kind, "system"))
            fallback = {"task_executor": "task", "tool_manager": "tool", "motion_executor": "execution", "aubo_driver": "robot"}
            module = next((m for m in candidates if m != "system"), fallback.get(normalized, "system"))
            expected = self.config.task_publisher.strip("/").replace("/", ".")
            declaration = None
            if normalized == expected:
                module = "task"
                try:
                    declaration = trace_log(message)
                except (ValueError, TypeError):
                    pass
            self._event("ROS_LOG", message, level="error" if level >= 40 else "warning" if level >= 30 else "info",
                        module=module, logger=name, source="/rosout · " + name, source_stamp=source_stamp,
                        goal_id=declaration.get("execute_goal_id") if declaration else None,
                        task_id=declaration.get("task_id") if declaration else None)
            if declaration:
                self.declare_trace(declaration, "/rosout 日志声明 · " + name, source_stamp)

    def observe_pick_status(self, entries, source, stamp_ns, source_age_ms):
        with self.lock:
            old = self.samples.get("pick_status")
            if old and old["source"] == source and stamp_ns <= old["stamp_ns"]:
                return None
            active = [e for e in entries if e["status"] in (1, 2, 3)]
            if len(active) > 1:
                self.pick_current_id = None
                self.pick_result = None
                self.invalidate("pick_feedback")
                selected = None
            elif active:
                selected = active[0]
                if selected["goal_id"] != self.pick_current_id:
                    self.invalidate("pick_feedback")
                    self.pick_result = None
                self.pick_current_id = selected["goal_id"]
            else:
                selected = next((e for e in entries if e["goal_id"] == self.pick_current_id), None)
            for entry in entries:
                self.journal.node(entry["goal_id"], "PickPlace")
            data = dict(goal_id=selected["goal_id"] if selected else None,
                        status=ACTION_STATES.get(selected["status"], "UNKNOWN") if selected else "UNKNOWN",
                        active_count=len(active), ambiguous=len(active) > 1, idle=not entries)
            self.observe("pick_status", data, source, stamp_ns, source_age_ms)
            return selected["goal_id"] if selected and selected["status"] in (4, 5, 6) else None

    def observe_pick_feedback(self, goal_id, data, source, stamp_ns, source_age_ms):
        with self.lock:
            if goal_id == self.pick_current_id and not self.pick_result:
                self.observe("pick_feedback", dict(data, goal_id=goal_id), source, stamp_ns, source_age_ms)

    def record_pick_result(self, goal_id, status, result, source):
        with self.lock:
            if goal_id != self.pick_current_id or status not in (4, 5, 6):
                return
            self.pick_result = dict(goal_id=goal_id, status=ACTION_STATES[status], **result, source=source, at=iso_time(self.wall_clock()))
            self._event("PICK_RESULT", result["message"], level="error" if status == 6 else "info", module="pickplace",
                        goal_id=goal_id, error_code=result["error_code"], status=ACTION_STATES[status], data=deepcopy(result), source=source)

    def frame(self, kind, data, jpeg, source, stamp_ns, source_age_ms):
        with self.lock:
            old = self.samples.get(kind)
            self.observe(kind, data, source, stamp_ns, source_age_ms)
            if self.samples.get(kind) is not old:
                self.frames[kind] = bytes(jpeg)

    def camera_frame(self, kind):
        with self.lock:
            if kind not in {"color", "depth"} or not self._sample(kind).get("fresh") or kind not in self.frames:
                raise ApiError(404, "NO_FRAME", "没有有效的新鲜相机帧")
            return self.frames[kind]

    def camera_packet(self, kind):
        """Image bytes and metadata from one locked sample, including exact ns stamp."""
        with self.lock:
            jpeg = self.camera_frame(kind)
            metadata = self._sample(kind)
            metadata.update(session_id=self.session_id, stream=kind)
            return jpeg, metadata

    def _stop_boundary(self):
        sample = self.samples.get("robot")
        self.stop_boundary = dict(received=self.clock(), wall_ns=round(self.wall_clock() * 1e9),
                                  stamp_ns=sample["stamp_ns"] if sample else 0)

    def stop_evidence(self):
        sample = self._sample("robot")
        post_request = True
        if self.stop_boundary:
            raw = self.samples.get("robot", {})
            post_request = (raw.get("_received", -1) > self.stop_boundary["received"]
                            and raw.get("stamp_ns", 0) > max(self.stop_boundary["stamp_ns"],
                                                           self.stop_boundary["wall_ns"]))
        value = sample.get("data") or {}
        confirmed = (sample.get("fresh", False) and post_request
                     and value.get("stop_confirmed") is True and value.get("motion_active") is False
                     and value.get("simulation_mode") is False)
        return dict(confirmed=bool(confirmed), sample=sample, post_request=post_request,
                    reason="独立 RobotState 已确认" if confirmed else
                    "等待请求之后的新独立停稳反馈" if not post_request else
                    "独立反馈缺失、过期、来自模拟、未确认或仍在运动")

    def observe_fjt_status(self, entries, source, stamp_ns, source_age_ms):
        """Track only an observed active UUID, never pick an old terminal Goal."""
        with self.lock:
            old = self.samples.get("fjt_status")
            if old and old["source"] == source and stamp_ns <= old["stamp_ns"]:
                return None
            active = [e for e in entries if e["status"] in (1, 2, 3)]
            if len(active) > 1:
                self.fjt_current_id = None
                self.fjt_result = None
                self.invalidate("fjt_feedback")
                selected = None
            elif active:
                selected = active[0]
                if selected["goal_id"] != self.fjt_current_id:
                    self.invalidate("fjt_feedback")
                    self.fjt_result = None
                self.fjt_current_id = selected["goal_id"]
            else:
                selected = next((e for e in entries if e["goal_id"] == self.fjt_current_id), None)
            for entry in entries:
                self.journal.node(entry["goal_id"], "FollowJointTrajectory")
            data = dict(goal_id=selected["goal_id"] if selected else None,
                        status=ACTION_STATES.get(selected["status"], "UNKNOWN") if selected else "UNKNOWN",
                        active_count=len(active), ambiguous=len(active) > 1, idle=not entries)
            self.observe("fjt_status", data, source, stamp_ns, source_age_ms)
            return selected["goal_id"] if selected and selected["status"] in (4, 5, 6) else None

    def observe_fjt_feedback(self, data, source, stamp_ns, source_age_ms):
        with self.lock:
            if data["goal_id"] != self.fjt_current_id or self.fjt_result:
                return
            self.observe("fjt_feedback", data, source, stamp_ns, source_age_ms)

    def record_fjt_result(self, goal_id, status, code, message, source):
        with self.lock:
            if goal_id != self.fjt_current_id or status not in (4, 5, 6):
                return
            self.fjt_result = dict(goal_id=goal_id, status=ACTION_STATES[status], error_code=code,
                                   message=message, source=source, at=iso_time(self.wall_clock()))
            self._event("MOTION_RESULT", message, level="error" if status == 6 else "info",
                        motion_result=self.fjt_result, module="execution", goal_id=goal_id, error_code=code, source=source)

    def snapshot(self):
        with self.lock:
            self.snapshot_seq += 1
            task = self.tasks.get(self.current_id)
            if task:
                task["elapsed_ms"] = round(((task["_ended"] if task["_ended"] is not None else self.clock()) - task["_start"]) * 1000)
                # An absent response is unknown, never a fabricated rejected/canceled result.
                if task["status"] == "SUBMITTING" and task["elapsed_ms"] > 10000:
                    self.uncertain(task["goal_id"], "Goal 回执超过 10 秒仍未知；不会重发，请检查 ROS")
            reasons = self._reasons()
            state = self._sample("task_state")
            observed = state["data"] if state.get("fresh") else None
            foreign = bool(observed and observed["task_id"] and (not task or observed["task_id"] != task["task_id"]))
            connection = deepcopy(self.connection)
            graph_received = getattr(self, "_graph_received", None)
            connection["graph_age_ms"] = None if graph_received is None else round((self.clock() - graph_received) * 1000)
            return dict(schema_version=1, mode=self.mode, session_id=self.session_id, snapshot_seq=self.snapshot_seq,
                        event_seq=self.seq, generated_at=iso_time(self.wall_clock()), freshness_limit_ms=self.config.freshness_ms,
                        task=self._public(task) if task else None, task_state=state, external_task=observed if foreign else None,
                        robot=self._sample("robot"), tool=self._sample("tool"), joints=self._sample("joints"),
                        camera={"color": self._sample("color"), "depth": self._sample("depth")},
                        perception=self._sample("perception"), planner=self._sample("planner"),
                        tool_verification=self._sample("tool_verification"), calibration=self._sample("calibration"), tf=self._sample("tf"),
                        pick_status=self._sample("pick_status"), pick_feedback=self._sample("pick_feedback"), pick_result=deepcopy(self.pick_result),
                        diagnostics=dict(latest_failure=self.journal.query(limit=1)["latest_failure"],
                                         trace=self.journal.graph(task_id=task["task_id"] if task else None)),
                        controller=self._sample("controller"), motion=self._sample("motion"),
                        fjt_status=self._sample("fjt_status"), fjt_feedback=self._sample("fjt_feedback"),
                        fjt_result=deepcopy(self.fjt_result), stop_evidence=self.stop_evidence(),
                        connection=connection, config=asdict(self.config), stop_request=deepcopy(self.stop_request),
                        operations=dict(submit=not reasons and self.transport is not None,
                                        cancel=bool(task and task["status"] in {"ACCEPTED", "EXECUTING"} and not self.config.read_only and self.connection["connected"]),
                                        stop=bool(self.connection["connected"] and self.connection["stop_ready"] and not self.config.read_only and not (self.stop_request and self.stop_request["status"] == "PENDING")),
                                        confirm_mock_stop=False, reasons=reasons),
                        history=[{k: t[k] for k in ("task_id", "goal_id", "status", "created_at", "completed_count", "total_count", "result")} for t in reversed(list(self.tasks.values()))],
                        events=deepcopy(self.events[-200:]))

    def task_record(self, task_id):
        with self.lock:
            if task_id not in self.tasks:
                raise ApiError(404, "NOT_FOUND", "此会话没有该任务")
            self.snapshot()
            record = self._public(self.tasks[task_id])
            full = self.journal.query(task_id=task_id, all_rows=True)
            record.update(events=list(reversed(full["events"])), trace=full["trace"], coverage=full["coverage"], latest_failure=full["latest_failure"])
            return record

    def query_events(self, **filters):
        with self.lock:
            return self.journal.query(**filters)

    def export_session(self, **filters):
        with self.lock:
            # snapshot() may record a newly timed-out Goal; include it in this export.
            snapshot = self.snapshot()
            data = self.journal.query(**filters, all_rows=True)
            events = list(reversed(data["events"]))
            selected = {node["goal_id"] for node in data["trace"]["nodes"]}
            tasks = []
            for task in self.tasks.values():
                if goal_key(task["goal_id"]) not in selected:
                    continue
                record = self._public(task)
                # Retain the v1 task.events field, including all matching recorded
                # events rather than its bounded live preview. Filters apply here too.
                record["events"] = [event for event in events if event.get("task_id") == task["task_id"]
                                    or task["task_id"] in event.get("related_task_ids", [])]
                tasks.append(record)
            data.update(schema_version=2, session_id=self.session_id, config=asdict(self.config), snapshot=snapshot,
                        tasks=tasks, events=events,
                        note="本会话已记录事件按时间顺序导出；筛选同时应用于 events 和 tasks[].events，不受内存窗口限制")
            return data

    def close(self):
        if self.transport:
            self.transport.close()
