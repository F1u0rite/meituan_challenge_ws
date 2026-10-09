#!/usr/bin/env python3
"""sim_bringup.launch.py —— 仿真组合**骨架**（状态：NOT RUN）。

为什么是骨架而不是可运行文件
----------------------------
1. 本机只有 ROS 2 **Humble**，而目标基线是 **Jazzy**（``ROS_DISTRO=humble``）；
2. 本机**没有 Gazebo / ros_gz / mtc_simulation 场景包**（``src/mtc_simulation`` 仅为
   迁移占位 README，无 ``package.xml``）；
3. ``mtc_motion_planning`` 为迁移占位包（源缺失），仿真所需规划器尚未迁入；
4. 按项目约定本次**不执行 colcon build**，因此不存在可运行的仿真产物。

因此本文件**只声明参数组合**，不启动任何节点、不 spawn 任何实体。直接运行它不会
产生任何动作，只会打印 NOT RUN 说明。

用法（仅用于查看参数组合）::

    ros2 launch mtc_bringup sim_bringup.launch.py            # 打印 NOT RUN
    ros2 launch mtc_bringup sim_bringup.launch.py show_only:=true
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

SIM_NOT_RUN_REASON = (
    "NOT RUN：sim_bringup 为骨架。缺少 ROS 2 Jazzy、Gazebo/ros_gz 依赖、"
    "mtc_simulation 场景包与 mtc_motion_planning 规划器（均为迁移占位）。"
    "本文件不启动任何节点、不 spawn 任何实体，也不产生任何运动。"
)

#: 仿真参数组合声明（仅文档性；不会被实际下发）
SIM_PARAMETER_MATRIX = {
    "backend": "mock",              # 仿真骨架阶段仍不启用任何真实/仿真后端
    "tool_type": "passive_hook_v1",  # 与 manipulation.yaml 对齐；待标定
    "controller_mode": "position",   # 占位值
    "simulation_mode": "true",       # 期望为仿真模式（由 InterlockGuard 校验）
    "placement_stability_sec": "3.0",  # 待实物标定
    "perception_max_age_sec": "0.3",   # 待实物标定
    "spawn_batteries": "3",            # 蓝/红/黄各一
    "world": "warehouse_placeholder",  # 占位：真实世界文件未迁入
}


def _report(context, *args, **kwargs):
    lines = ["=" * 78, SIM_NOT_RUN_REASON, "=" * 78, "参数组合声明（NOT RUN，不会下发）："]
    for key, value in SIM_PARAMETER_MATRIX.items():
        lines.append("  %-26s = %s" % (key, value))
    lines.append("如需仿真：先在 ROS 2 Jazzy 机器上迁入 mtc_simulation / mtc_motion_planning，"
                 "再单独评审并补充 Gazebo 启动项。")
    return [LogInfo(msg="\n".join(lines))]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("show_only", default_value="true",
                              description="只打印 NOT RUN 说明（本骨架当前唯一行为）"),
        DeclareLaunchArgument("world", default_value=SIM_PARAMETER_MATRIX["world"],
                              description="占位世界名（真实世界文件未迁入）"),
        DeclareLaunchArgument("tool_type", default_value=SIM_PARAMETER_MATRIX["tool_type"],
                              description="工具类型（占位）"),
        DeclareLaunchArgument("backend", default_value="mock",
                              description="后端；仿真骨架阶段仍固定 mock"),
        OpaqueFunction(function=_report),
    ])
