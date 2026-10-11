# mtc_moveit_planner —— MoveIt 2 规划适配层（PlannerPort 实现）

> **状态：Mock / Fake Hardware 专用，只规划、不执行。** 输出默认 `simulation_only=true`；
> TCP、工位、提环位姿均为 PLACEHOLDER，禁止用于真实运动。负责人：常建烁。

## 1. 它做什么

把 `PickPlaceFsm` 的规划请求（`mtc::manipulation::PlanTarget`）交给 MoveIt 2，
逐原语返回**完整时间参数化轨迹**（`PlanResult.commands[i].trajectory`），满足
`PickPlaceFsm::accept_plan()` 的全部检查：

| 检查 | 本包保证 |
|---|---|
| 每个原语一条命令，按 `primitive_name` 对应 | 是；任一原语失败则整体 `ok=false`、`commands` 为空 |
| `trajectory.joint_names == PickPlaceConfig::joint_names` | 按**名称**重排（MoveIt 顺序不可信） |
| `target_rad` 为空、`header_frame_id` 非空 | 是（frame = MoveIt planning frame `world`） |
| `velocity/acceleration_scaling ∈ (0, 0.2]` | 默认 0.1，配置超 0.2 直接拒绝 |
| `timeout_ms >= 轨迹时长` | `ceil(时长×1.5)+1000 ms`；时长超过对应 FSM 状态看门狗 80% 时拒绝 |
| 失败可追溯 | `error_code=PLANNING_FAILED(200)`，`message` 以 `[原因] 原语: 详情` 开头，不静默重试 |

```text
PickPlaceFsm ──PlanTarget──▶ MoveItPlannerPort (纯 C++，可离线测试)
                                 │ 原语表 / TCP 补偿 / 工位查表 / 起点串接 / 结果校验
                                 ▼
                           PlanningBackend 接口
                                 ├─ MoveGroupBackend：MoveGroupInterface + tf2（ROS 外壳）
                                 └─ FakeBackend：GTest 用玩具运动学
```

## 2. 原语规划规则

| 原语 | 执行状态（看门狗） | 方式 | 目标 | 起点 |
|---|---|---|---|---|
| `side_insert`(V1) / `top_insert`(V2) | MOVE_PRE_ALIGN (30 s) | OMPL | 提环位姿沿插入轴 (+Z) **后退 0.03 m**（预对准位） | 当前状态 |
| `test_lift` | TEST_LIFT (15 s) | 笛卡尔直线 | +Z 0.01 m | **假设** ToolPort 插入 0.03 m 后 |
| `lift` | LIFT (30 s) | 笛卡尔直线 | +Z 0.05 m | 当前状态（试提已执行） |
| `transport` | TRANSPORT (60 s) | OMPL | 工位上方 0.10 m，保持抓取姿态 | 上一段终点 |
| `pre_seat` | MOVE_PRE_SEAT (30 s) | 笛卡尔直线 | 工位上方 0.02 m | 上一段终点 |
| `seat_contact` | SEAT (20 s) | 笛卡尔直线 | 工位高度（**接触意图，仅仿真**） | 上一段终点 |
| `retreat` | RETREAT (30 s) | 笛卡尔直线 | +Z 0.10 m | **假设** ToolPort 退出 0.03 m 后 |
| `hook_engage` / `auto_lock` / `unload` / `side_withdraw` / `withdraw` | — | **拒绝**（ToolPort 负责） | — | — |

OMPL 行的原语（FREE_SPACE）**先尝试末端直线**（完整、无碰撞、满足关节约束、无构型翻转才采用），
不可行再用 OMPL；控制器限位（foreArm ±2.41、wrist1 ±2.66、wrist3 ±2.23）以 MoveIt 路径约束下发并在输出前复核。

距离来自 `config/motion_profiles.yaml` 与 `config/manipulation.yaml`（均为占位值）；
末端线/角速度上限取原语自身的 `max_linear_speed` / `max_angular_speed`，
在 TOTG 时间参数化后按需**均匀拉伸**时间（路径不变）。

## 2A. 规划场景与起点一致性

- **静态障碍物**：`scene.boxes`（默认占位桌面），工具启动时经 `/apply_planning_scene` 写入。
- **电池载荷**：`carries_payload` 原语（test_lift → seat_contact）在起点 RobotState 上附着长方体，不改全局场景。
- **起点检查**：执行侧发送前调用
  ```cpp
  #include "mtc_moveit_planner/start_consistency.hpp"
  auto check = mtc::moveit_planner::check_trajectory_start(*cmd.trajectory, names, positions);  // 0.01 rad
  if (!check.ok) { /* 拒绝；用 MoveItPlannerPort::replan_primitive(target, cmd.primitive_name, measured) 重规划 */ }
  ```

## 3. 构建与测试

```bash
bash src/mtc_moveit_planner/scripts/build_driver_ws.sh   # AUBO 驱动 5 包（独立目录）
bash src/mtc_moveit_planner/scripts/build_planner.sh     # FSM Core 4 包 + 本包 + GTest
```

## 4. 运行（需要 Fake 控制 + move_group 已启动）

```bash
source src/mtc_moveit_planner/scripts/env.sh
ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_fixed_pose
ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_battery_sequence
```

或一键：启动 Fake + move_group → 只读采集（ROS graph / TF 图 / URDF / SRDF）→ 规划 → 退出：

```bash
bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh
```

证据写入 `docs/planning/evidence/<时间戳>/`。

## 5. 已知限制（详见 docs/planning/PLANNING_HANDOFF.md）

1. 碰撞场景只有机器人自身；无桌面、电池、工具、相机模型。
2. TCP = 单位变换（PLACEHOLDER），轨迹实际把 `ee_link` 送到提环位置。
3. `test_lift` / `retreat` 起点依赖对 ToolPort 动作的假设；执行端必须核对起点偏差。
4. 只读 Fake 环境中验证；不调用 execute()，不发送 FollowJointTrajectory。
