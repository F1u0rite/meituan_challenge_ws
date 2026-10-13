"""P1 observation adapters; no FSM, planner or tool control.

Native IDL and optional producer contracts are documented in p1-contract.md.
Authored with OpenAI Codex; non-finite perception values remain unknown.
"""

from mtc_webui.telemetry import number


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def pose_sample(message, array_stamp):
    stamp = stamp_ns(message.header)
    position = [number(getattr(message.pose.position, k)) for k in "xyz"]
    orientation = [number(getattr(message.pose.orientation, k)) for k in "xyzw"]
    finite = all(x is not None for x in position + orientation)
    unit = finite and abs(sum(x*x for x in orientation) - 1) <= .02
    issues = []
    if not finite: issues.append("位姿含非有限值")
    elif not unit: issues.append("四元数无效")
    if not message.header.frame_id: issues.append("缺少坐标系")
    if stamp <= 0: issues.append("位姿时间戳无效")
    # Each pose has its own Header. An updated array cannot freshen an old pose.
    if stamp > array_stamp + 500_000_000: issues.append("位姿晚于检测帧超过 500 ms")
    return dict(position=position, orientation=orientation, frame_id=message.header.frame_id,
                stamp_ns=str(stamp), offset_ms=(array_stamp-stamp)/1e6,
                valid=not issues, issues=issues)


def perception_sample(message):
    if len(message.detections) > 128:
        raise ValueError("检测数组超过 128 个对象")
    stamp = stamp_ns(message.header)
    ids = [d.object_id for d in message.detections]
    rows = []
    for d in message.detections:
        confidence = number(d.confidence)
        confidence = confidence if confidence is not None and 0 <= confidence <= 1 else None
        rows.append(dict(object_id=d.object_id, color=int(d.color), confidence=confidence,
                         ring_pose_valid=bool(d.ring_pose_valid),
                         identity_valid=bool(d.object_id) and ids.count(d.object_id) == 1,
                         body_pose=pose_sample(d.body_pose, stamp), ring_pose=pose_sample(d.ring_pose, stamp)))
    return dict(frame_id=message.header.frame_id, detections=rows)


def _base(data, kind):
    if not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ValueError(kind + " schema_version 必须为 1")
    if type(data.get("stamp_ns")) is not int or data["stamp_ns"] <= 0:
        raise ValueError(kind + " 缺少有效源时间戳")


def _text(data, key, required=False):
    value = data.get(key)
    if value is not None and (not isinstance(value, str) or len(value) > 2000):
        raise ValueError(key + " 必须为字符串（最多 2000 字符）")
    if required and (not value or not value.strip()):
        raise ValueError(key + " 不能为空")
    return value or None


def _check(data, key):
    value = data.get(key)
    if value is not None and type(value) is not bool:
        raise ValueError(key + " 必须为布尔值或 null")
    return value


def planner_sample(data):
    _base(data, "planner")
    if data.get("state") not in {"WAITING", "PLANNING", "SUCCEEDED", "FAILED"}:
        raise ValueError("planner state 无效")
    result = dict(state=data["state"], plan_id=_text(data, "plan_id", True))
    for key in ("primitive", "task_id", "command_id", "trajectory_id", "reason"):
        result[key] = _text(data, key)
    count = data.get("point_count")
    if count is not None and (type(count) is not int or count < 0):
        raise ValueError("point_count 必须为非负整数")
    duration = data.get("expected_duration_sec")
    if duration is not None and (number(duration) is None or duration < 0):
        raise ValueError("expected_duration_sec 必须为非负有限数")
    code = data.get("error_code")
    if code is not None and (type(code) not in (int, str) or len(str(code)) > 2000):
        raise ValueError("error_code 必须为整数或字符串")
    result.update(point_count=count, expected_duration_sec=duration, error_code=code)
    for key in ("joint_limits_ok", "collision_free", "start_state_consistent"):
        result[key] = _check(data, key)
    # Every sample is a complete snapshot of one plan, never merge previous checks.
    return result


def tool_verification_sample(data):
    _base(data, "tool verification")
    result = {key: _text(data, key, True) for key in ("verification_id", "task_id", "object_id", "tool_type")}
    for key in ("test_lift", "seated"):
        value = data.get(key)
        if value is None:
            result[key] = None
            continue
        if not isinstance(value, dict):
            raise ValueError(key + " 必须为对象或 null")
        verified = _check(value, "verified")
        level = value.get("evidence_level")
        if level is not None and (type(level) is not int or level not in range(4)):
            raise ValueError(key + " evidence_level 无效")
        stamp = value.get("stamp_ns")
        if type(stamp) is not int or not 0 < stamp <= data["stamp_ns"]:
            raise ValueError(key + " 需要不晚于遥测的证据时间戳")
        source = _text(value, "source")
        if verified is True and (level not in (2, 3) or not source):
            raise ValueError(key + " true 需要独立证据（等级 2/3 和来源）")
        result[key] = dict(verified=verified, evidence_level=level, source=source,
                           stamp_ns=str(stamp), reason=_text(value, "reason"))
    return result
