# meituan_challenge_ws

**美团第四届低空经济与具身智能挑战赛｜AUBO S3 机械臂 ROS 2 工程**

**FSM Core v0.1 已完成第一阶段离线安全验收，并合并到 `main`。** 合并提交为
`c2be85724111f6fb4dfcc62a2c974f7be1233cba`，本地 Git tag `fsm-core-v0.1.0`
指向该提交，作为团队的 FSM Core 开发基线。

**独立 ROS 2 Jazzy 驱动与 MoveIt 2 Mock 验证已完成；完整 FSM + MoveIt 2 集成仍在进行。**
第二阶段的新 C++ ROS 2 Action 外壳、MoveIt 2 C++ Planner Adapter、FollowJointTrajectory
与 Mock Hardware 抓放闭环尚未完成。独立 Mock 成功不代表 FSM 已接入，也不代表
AUBO S3 实机抓放验收通过。

开发环境：Ubuntu 24.04、ROS 2 Jazzy、C++17。当前只允许 Mock / Fake / 离线开发；
构建和测试不授权连接控制柜、调用真实 RPC/RTDE、发送实机轨迹或触发实体工具 IO。

## 1. 从这里开始

- [FSM 开发者使用指南](README_FSM.md)：四个 C++ 包、主要 API、Port 契约、示例、构建与测试。
- [ROS 2 驱动与 MoveIt 2 部署手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)：已验证环境、Mock 启动、控制器/TF 检查和 10 月 12 日团队 SOP。
- [第一阶段最终安全验收报告](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)：缺陷复现、修复、318/269/15 测试证据及 TSan 限制。
- [第二阶段 ROS 2 集成计划](docs/migration/PHASE2_ROS2_INTEGRATION_PLAN.md)：后续 Action / Planner / Mock 集成工作。
- [原版本开发指南](README_FSM_v0.1.md)：保留原入口，当前使用说明以 `README_FSM.md` 为准。

## 2. 项目目标与已完成范围

项目面向固定工位电池识别、提环抓取和按颜色顺序摆放：基础任务将指定颜色电池
放入 T0；序列任务按口令将三块电池依次放入 P1、P2、P3。
V1 使用被动舌规；V2 使用机械锁止和电磁解锁策略。两种策略均已有核心逻辑及
Fake 测试，实体机构、工具 IO、TCP/负载和工位标定尚未验收。

四个 C++ 核心包均已合入 `main`，核心逻辑不依赖 `rclcpp`：

| 包 | 职责 | 主要入口 |
|---|---|---|
| [mtc_core_cpp](src/mtc_core_cpp/README.md) | 共享错误码、Pose6D、MotionCommand、TimedTrajectory、Clock 与证据类型 | [公共头文件](src/mtc_core_cpp/include/mtc_core_cpp/) |
| [mtc_task_cpp](src/mtc_task_cpp/README.md) | 整轮 Task FSM、任务校验、动作意图、重试和故障锁定 | [TaskFsm](src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp) |
| [mtc_manipulation_cpp](src/mtc_manipulation_cpp/README.md) | 单块电池 PickPlace FSM、V1/V2 ToolStrategy、四类 Port | [PickPlaceFsm](src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp) |
| [mtc_motion_execution_cpp](src/mtc_motion_execution_cpp/README.md) | 单在途命令、执行校验、关联取消、停稳证据、Mock/Disabled Adapter | [MotionExecutorCore](src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/executor_core.hpp) |

```text
TaskFsm → ActionRequest → PickPlaceFsm
                             ├─ PerceptionPort
                             ├─ PlannerPort
                             ├─ ToolPort / ToolStrategy
                             └─ MotionPort → MotionExecutorCore → MockMotionAdapter

待完成：ROS 2 Action 外壳 → MoveIt 2 Planner → FollowJointTrajectory → Mock Hardware
```

Task FSM 返回动作意图，外部事件循环负责执行和反馈。PickPlace FSM 通过注入的 Port
获取感知、规划、运动和工具证据。两类 FSM 采用单一事件线程所有权；运动执行器
单独处理执行/取消竞争。旧 Python FSM、接口定义和迁移记录均保留。

## 3. 编译与全量离线测试

唯一开发仓库：`/home/aaet/meituan_challenge_ws`。
已具备 Jazzy、colcon、ament 和各包依赖后，在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/aaet/meituan_challenge_ws
colcon list

# 使用独立目录，保留历史 build/install/log
colcon --log-base log/final_gate build \
  --build-base build/final_gate --install-base install/final_gate
source install/final_gate/setup.bash
colcon --log-base log/final_gate test \
  --build-base build/final_gate --install-base install/final_gate
colcon test-result --test-result-base build/final_gate --all

# 核验原290项及当前全部C++测试身份，并显式运行未注册的离线E2E
python3 tests/verify_cpp_audit_results.py --build-base build/final_gate
python3 tests/test_mock_end_to_end.py
```

仅构建四个 C++ 包、定向 GTest、消费者 CMake 和 sanitizer 方法见
[开发者指南](README_FSM.md)。这些命令无需真实机器人；不要为测试启动真实驱动。

## 4. 最终验收记录

以下为 [2026-10-11 Final Gate 报告](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)
记录的实际执行结果；本次 README 更新没有重新运行全量回归。

| 验证项 | 结果 |
|---|---|
| Jazzy 工作空间构建 | 12/12 包 PASS |
| C++ GTest | 318 PASS；16个可执行文件，原290项全部保留，新增28项 |
| Python pytest | 269 PASS |
| 显式离线 E2E | 15 PASS / FAIL 0 / SKIP 0；纯 Python Mock |
| XML manifest、UBSan、git diff --check | PASS |
| TSan 动态竞态回归 | 未完成，NOT RUN；运行时启动报 `unexpected memory mapping` |
| 第二阶段 C++ ROS Action / MoveIt Planner / Mock Hardware 闭环 | 尚未完成、未验收 |
| AUBO S3 实机抓放与实体 IO | NOT RUN，未授权 |

318 + 269 + 15 为602个测试场景。报告中的 colcon `608 tests` 包含 CTest 包装和
lint 条目，不能当作608个独立业务用例。离线 E2E 使用脚本自带 MockPlanner，
不代表真正 MoveIt Planner 或 C++ ROS Action 已接入。

## 5. 安全契约

- Goal-scoped Cancel 使用 `cancel_goal(command_id, reason)`；错ID直接拒绝且后端停止调用为0。`supervisory_stop(reason)` 是明确的监督停止，不能替代旧 Goal 的取消。
- `request_delivered`、Action accepted/canceled 都不能证明停稳；需要关联命令、控制器状态、有效反馈、持续静止及队列清空等独立证据。软件 Stop 不是安全急停。
- 执行超时或运动状态未知时禁止自动重发。FAULT 结束当前执行但继续锁定，不能通过普通 RESET 或自动流程伪造人工恢复。
- 放置成功须由新鲜、有效、可区分的观测支撑。`placement_stability_sec` 必须有限且大于0；默认3.0秒是工程参数，不是比赛官方最低时间。
- 规划输出必须进入执行命令；缺失规划结果时拒绝，不能执行默认全零目标。多点轨迹不得拆成多次 ExecuteJointMove。
- Mock Adapter 和执行器共享同一个 ManualClock 实例；虚拟时间不用于真实后端。未配置关节限位的 Mock 成功不代表实机安全。

## 6. ROS 2 驱动与 MoveIt 2 环境

官方驱动参考工作空间 `/home/aaet/aubo_driver_validation_ws` **只读**。其
`install_no_symlink` 环境已有 GenericSystem、200Hz controller、FollowJointTrajectory
和 MoveIt Plan → Execute 的实际成功日志。该环境包含一处已归档的 Fake Hardware
Xacro 修复，纯上游 clone 不能直接当作同一已验证环境。

[AUBO S3 ROS 2 Jazzy 驱动部署与运行手册](docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)
列出准确包名、Launch 参数、部署前置、三终端 Mock SOP，以及历史起点不匹配和
MoveIt 退出异常。手册明确区分独立 Mock、待完成的 FSM 集成、需要现场单独授权的
真实硬件操作；不提供实机运动命令。新机器修复快照和完整部署仍需独立复验。

## 7. 仓库目录与参考资料

```text
meituan_challenge_ws/
├── README.md / README_FSM.md     # 项目入口与开发者指南
├── src/
│   ├── mtc_*_cpp/               # 四个已完成的C++核心包
│   ├── mtc_interfaces/          # 既有Action/Message/Service契约
│   ├── mtc_task/、mtc_manipulation/、mtc_motion_execution/
│   ├── mtc_tool/、mtc_safety/、mtc_aubo_bridge/、mtc_bringup/
│   └── mtc_description/、mtc_motion_planning/、mtc_simulation/
├── config/、scripts/、tests/
├── docs/                        # 架构、参考、迁移及验收证据
└── build/、install/、log/        # 本地生成产物，不提交Git
```

当前 `colcon list` 注册12个包；description/planning/simulation目录为保留入口，
不计入该12包构建结果。官方驱动参考工作空间 `/home/aaet/aubo_driver_validation_ws`
已有独立的 Jazzy Mock Hardware / FollowJointTrajectory / MoveIt 验证记录，
只读参考；这不等于本仓库的新 C++ FSM 已完成同一集成。

- [Python/C++ 行为映射](docs/migration/FSM_BEHAVIOR_MAPPING.md)
- [C++ 接口兼容审计](docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md)
- [上一轮审计修复报告](docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md)
- [官方驱动 Fake Hardware 修复审计](docs/migration/AUBO_DRIVER_FAKE_HARDWARE_FIX_AUDIT.md)
- [架构设计 v0.1](docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md)：目标设计，以当前代码与验收报告核实完成范围。
- [历史源码审计](docs/migration/SOURCE_AUDIT.md)、[迁移映射](docs/migration/MIGRATION_MAP.md)、[历史工作空间重构提示词](docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md)：保留历史，不作为重新迁移或清空仓库的指令。
- [AUBO S3 交接参考](docs/reference/AUBO_S3.md)：历史记录，不是当前设备状态证明。

仓库路径已于2026-10-09按用户要求从 `/home/meituan_challenge_ws` 迁至当前路径，
保留Git历史。旧工程及官方驱动工作空间只读。开发者从 `main` 建独立分支；
接口或安全语义变化需同步README和验收证据，不将Mock结果宣称为实机许可。
