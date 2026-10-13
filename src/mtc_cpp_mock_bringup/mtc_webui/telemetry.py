"""Read-only execution telemetry conversion. Never derive motion from Task FSM.

Authored with OpenAI Codex. Native message references: provenance.json.
The optional JSON contract is documented in telemetry-contract.md; it is not
an upstream interface that is assumed to exist.
"""

import math


MOTION_STATES = {"IDLE", "EXECUTING", "CANCELING", "STOPPING", "STOP_CONFIRMED",
                 "SUCCEEDED", "FAILED", "CANCELED", "FAULT", "UNKNOWN"}
ACTION_STATES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
                 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


def number(value):
    return value if type(value) in (int, float) and math.isfinite(value) else None


def vector(values):
    return [number(value) for value in values]


def seconds(duration):
    value = duration.sec + duration.nanosec / 1e9
    return value if math.isfinite(value) and value >= 0 else None


def controller_sample(message):
    """Jazzy JTC uses reference/feedback; preserve missing vector entries."""
    return dict(joint_names=list(message.joint_names),
                actual_position=vector(message.feedback.positions),
                actual_velocity=vector(message.feedback.velocities),
                desired_position=vector(message.reference.positions),
                desired_velocity=vector(message.reference.velocities),
                position_error=vector(message.error.positions))


def fjt_sample(message):
    feedback = message.feedback
    return dict(goal_id=bytes(message.goal_id.uuid).hex(),
                joint_names=list(feedback.joint_names),
                actual_position=vector(feedback.actual.positions),
                actual_velocity=vector(feedback.actual.velocities),
                desired_position=vector(feedback.desired.positions),
                desired_velocity=vector(feedback.desired.velocities),
                position_error=vector(feedback.error.positions),
                elapsed_sec=seconds(feedback.actual.time_from_start))


def execution_sample(data):
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("motion telemetry schema_version 必须为 1")
    if type(data.get("stamp_ns")) is not int or data["stamp_ns"] <= 0:
        raise ValueError("motion telemetry 缺少有效源时间戳")
    if data.get("state") not in MOTION_STATES:
        raise ValueError("motion telemetry 执行状态无效")
    if not isinstance(data.get("command_id"), str) or not data["command_id"]:
        raise ValueError("motion telemetry 缺少 command_id")
    result = {"state": data["state"], "command_id": data["command_id"]}
    for key in ("trajectory_id", "primitive", "fjt_goal_id", "task_id", "detail", "reason"):
        value = data.get(key)
        if value is not None and (not isinstance(value, str) or len(value) > 2000):
            raise ValueError("motion telemetry " + key + " 无效")
        result[key] = value or None
    if result["fjt_goal_id"]:
        import uuid
        result["fjt_goal_id"] = uuid.UUID(result["fjt_goal_id"]).hex
    for key in ("progress", "elapsed_sec", "expected_duration_sec"):
        value = data.get(key)
        if value is not None and (number(value) is None or value < 0 or (key == "progress" and value > 1)):
            raise ValueError("motion telemetry " + key + " 无效")
        result[key] = value
    for key in ("success", "fault_locked"):
        value = data.get(key)
        if value is not None and type(value) is not bool:
            raise ValueError("motion telemetry " + key + " 无效")
        result[key] = value
    code = data.get("error_code")
    if code is not None and type(code) is not int:
        raise ValueError("motion telemetry error_code 无效")
    result["error_code"] = code
    # stop_confirmed is deliberately not consumed here. Safety uses the
    # independent, pinned RobotState source and a post-request timestamp.
    return result
