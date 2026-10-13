"""Explicit ROS connection settings; defaults are names from the pinned repository."""

from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import re


@dataclass(frozen=True)
class RosConfig:
    domain_id: int | None = None
    read_only: bool = False
    task_action: str = "/mtc/task/execute"
    task_topic: str = "/mtc/task/state"
    task_publisher: str = "/task_executor"
    pick_action: str = "/mtc/manipulation/pick_place"
    robot_topic: str = "/mtc/robot/state"
    robot_publisher: str = "/mtc_aubo_bridge"
    tool_topic: str = "/mtc/tool/state"
    tool_publisher: str = ""
    joint_topic: str = "/joint_states"
    joint_publisher: str = ""
    stop_service: str = "/mtc/bridge/request_stop"
    color_topic: str = ""
    depth_topic: str = ""
    perception_topic: str = "/mtc/perception/batteries"
    perception_publisher: str = ""
    planner_topic: str = ""
    planner_publisher: str = ""
    tool_verification_topic: str = ""
    tool_verification_publisher: str = ""
    controller_topic: str = ""
    controller_publisher: str = ""
    fjt_action: str = ""
    fjt_publisher: str = ""
    motion_topic: str = ""
    motion_publisher: str = ""
    pick_publisher: str = ""
    calibration_topic: str = ""
    calibration_publisher: str = ""
    trace_topic: str = ""
    trace_publisher: str = ""
    tf_pairs: tuple = ()
    freshness_ms: int = 3000

    def __post_init__(self):
        if self.domain_id is not None and (type(self.domain_id) is not int or not 0 <= self.domain_id <= 232):
            raise ValueError("ROS domain 必须为 0–232 的整数")
        if type(self.read_only) is not bool:
            raise ValueError("read_only 必须为布尔值")
        if type(self.freshness_ms) is not int or not 500 <= self.freshness_ms <= 5000:
            raise ValueError("freshness_ms 必须为 500–5000 毫秒")
        if self.motion_topic and not self.motion_publisher:
            raise ValueError("执行端遥测必须配置 motion_publisher，不能从 FSM 推测运动")
        for kind in ("planner", "tool_verification", "calibration", "trace"):
            if getattr(self, kind + "_topic") and not getattr(self, kind + "_publisher"):
                raise ValueError(kind + " 必须配置完整发布节点名")
        if not isinstance(self.tf_pairs, (list, tuple)) or len(self.tf_pairs) > 16:
            raise ValueError("tf_pairs 必须为最多 16 组 target/source 坐标系")
        pairs = []
        for pair in self.tf_pairs:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2 or any(not isinstance(v, str) or not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_/]*", v) for v in pair) or pair[0] == pair[1]:
                raise ValueError("tf_pairs 每项为两个不同坐标系 [target, source]")
            pairs.append(tuple(pair))
        object.__setattr__(self, "tf_pairs", tuple(dict.fromkeys(pairs)))
        for key, value in asdict(self).items():
            if key.endswith(("_topic", "_publisher", "_action", "_service")):
                if not isinstance(value, str) or (value and not re.fullmatch(r"/[a-zA-Z_][a-zA-Z0-9_]*(/[a-zA-Z_][a-zA-Z0-9_]*)*", value)):
                    raise ValueError(f"{key} 必须为完整 ROS 名称")
                if not value and key in {"task_topic", "robot_topic", "tool_topic", "task_action", "stop_service", "robot_publisher"}:
                    raise ValueError(f"{key} 不能为空")

    @classmethod
    def load(cls, path=None, domain_id=None, read_only=False):
        data = json.loads(Path(path).read_text(encoding="utf-8")) if path else {}
        if not isinstance(data, dict):
            raise ValueError("配置必须是 JSON 对象")
        if domain_id is not None:
            data["domain_id"] = domain_id
        elif data.get("domain_id") is None and os.environ.get("ROS_DOMAIN_ID"):
            data["domain_id"] = int(os.environ["ROS_DOMAIN_ID"])
        if read_only:
            data["read_only"] = True
        return cls(**data)
