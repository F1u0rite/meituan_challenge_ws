# PR #14 → AUBO GenericSystem 完整轨迹执行

本包提供 `AuboFjtMotionPort`、原 PickPlace FSM 的依赖注入入口，以及独立 `aubo_fake` Launch/Supervisor。PR #14 原 Gazebo Runner、`/arm_controller` 与 Gazebo 身份检查未修改。

仅允许 `mock_components/GenericSystem`。没有真实 SDK、RPC/RTDE、控制柜或工具 IO 通路。`simulation_only=true` 本身不授予执行权。

## 架构与边界

```mermaid
flowchart LR
  F[原 PickPlaceFsm] --> P[原 PlannerPort / MoveGroupBackend]
  P --> T[完整 MotionCommand / TimedTrajectory]
  T --> A[AuboFjtMotionPort]
  A --> C[唯一 FjtMotionPort Client]
  C --> J[joint_trajectory_controller]
  J --> H[mock_components/GenericSystem]
```

`AuboFjtMotionPort` 实现原 MotionPort：`execute_trajectory()` 返回 nullopt 表示尚未完成；一条轨迹只发一个 FJT Goal。单点 ExecuteJointMove、接触原语及无完整轨迹请求仍拒绝。

新包只创建一个底层 FJT Client，使用 `/joint_trajectory_controller/follow_joint_trajectory`。Planner 在另一 Node/Executor 中工作；MoveIt 使用不提供任何执行 Handle 的 `mtc_aubo_fake/PlanningOnly`，不会产生额外 FJT Client。ROS Graph 是受信任隔离环境的部署证据，不是身份认证或跨进程原子锁；同一 Node 下的多个 Client 也不能单靠节点计数区分。

默认 Launch 为 `mode:=plan`。唯一允许运动的模式由 Supervisor 显式选择，并先核对静态展开 URDF 中唯一插件为 GenericSystem，再核对运行时插件、Active Controller、六关节 Position claim、JointState 和图拓扑。

不能接管已经运行的 Domain 91 控制器。本轮保留现场进程，使用新的空域：启动前只有执行管理者，3 秒空域观察后取得一次性、绑定 Node/Domain/Endpoint 的 ControlledFakeStart 凭据，随后才启动自有控制器和规划器。凭据不可复用；DDS 配置覆盖、预存端点或未核实 RMW 拒绝。原 FJT 默认严格 Status 模式未放宽。

原生 Status 主要事件驱动；新受控模式依赖已建立的启动边界、UUID、Controller 和新鲜 JointState，不伪造空 Status、不预热 Goal。未知 UUID 不认领，外部 Client/直接轨迹 Publisher 使执行权失效。

## 编译

使用独立目录，不覆盖历史构建；官方驱动工作空间始终只读。

```bash
source /opt/ros/jazzy/setup.bash
source /home/aaet/aubo_driver_validation_ws/install_no_symlink/setup.bash
cd /tmp/mtc_pr14_aubo_fake_20261014
colcon build --build-base /tmp/mtc_pr14_aubo_fake_build_20261014/build \
  --install-base /tmp/mtc_pr14_aubo_fake_build_20261014/install \
  --packages-up-to mtc_aubo_fake_integration_cpp mtc_ros2_action_cpp \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_PREFIX_PATH=/tmp/mtc_vision_dependency_20261014/prefix/usr
source /tmp/mtc_pr14_aubo_fake_build_20261014/install/setup.bash
```

本机 JSON 3.11.3 开发依赖仅解包在上述 `/tmp` prefix；正常安装 `nlohmann-json3-dev` 的开发机可省略该 CMake 参数。

## 运行 Fake 模式

先 source 上面的两个环境和本包 Overlay。先规划，再执行；用从未使用且确认为空的独立 Domain、全新证据目录。不要在本轮现场 91 域运行。

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/mtc_pr14_aubo_fake_launch_logs_20261014
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS ROS_LOCALHOST_ONLY
unset FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
# 其他 ROS_DISCOVERY_* / FASTDDS_* / FASTRTPS_* / RMW_FASTRTPS_* 覆盖也会被拒绝。
ros2 launch mtc_aubo_fake_integration_cpp aubo_fake.launch.py \
  mode:=plan domain:=220 output:=/tmp/mtc_aubo_fake_plan_new
# 核对 functional.json、trajectory.json、joint_states.jsonl 和独立 teardown.json。
# 以下仅启动新的 GenericSystem 实例，不复用旧轨迹或旧 Controller。
ros2 launch mtc_aubo_fake_integration_cpp aubo_fake.launch.py \
  mode:=execute domain:=221 output:=/tmp/mtc_aubo_fake_execute_new
```

若这些域/目录已有参与者或证据，换未使用的值，不停止别人的节点、不覆盖目录、不清除 quarantine。Supervisor 不会在参考驱动目录生成模型或改配置，Xacro 结果仅写入新证据目录。

轨迹从本轮实际 JointState 起点出发，按名称映射六关节；目标是测量值的小幅偏移的 FK 位姿。这个 Synthetic 非接触测试不使用旧 Gazebo 初始关节值/轨迹。Planner 约束每轴在测量起点 ±0.15 rad 范围且不超过模型/控制器限位，保留碰撞检查与末端限速。软件速度/加速度能力为 0.2 rad/s、0.2 rad/s²；原 URDF/controller 的位置交集不扩大。

使用原 FREE_SPACE 关节空间规划；不尝试可选 Cartesian 优先优化。实测安装版 MoveIt 2.12.4 有 Cartesian response timeout 后客户端无界等待问题，诊断记录在验收报告。此选择不放宽轨迹、碰撞或起点检查；本轮不支持接触/笛卡尔任务。

`command_from_plan()` 复用原轨迹转换，保留所有路点和导数。显式均匀减速因子 3，时间 ×3、速度 ÷3、加速度 ÷9，原 RawTrajectory 不变；之后再次完整校验 Bernstein 插值边界、限位和 timeout。单独保存规划生成单调时间与显式 immediate ROS stamp=0，不混用时钟。过期规划、过期状态或起点差异拒绝。

原生 Fake 验收使用的 TCP 是明确的 `ee_link` 裸工具 FK，**不是现场已标定 V1 TCP 的验收**。完整 V1 工具模型、TCP 和接触证据须由其原宿主提供；不能将这个非接触探针结果升级为抓放任务 PASS。

## Cancel / Timeout / FAULT

- Goal 发送/取消在 Port mutex 外完成，Session 与完整命令身份固定；同 ID 不同内容拒绝。
- Cancel 仅用本实例实际 Goal Handle/UUID，Pending 时保留取消意图，迟到 Handle 仍只关联原 Session。
- Goal Response/执行超时锁定，迟到 Result 不解锁，不允许自动重发或靠重建 Port 绕过 lease/quarantine。
- ACK 只表示请求收到；停稳还需要 Goal 终态、持续新鲜的 JointState、位置变化率和速度静止窗口。正常成功还需终点按名称一致。
- `request_stop()` 是异步请求；原 FSM 若同步询问时证据尚未就绪，保持其原保守 FAULT 语义。本包没有伪造同步 Stop Bridge。
- GenericSystem 速度字段可能一直为 0；单靠它不能证明停止。执行核心额外检查相邻位置变化率和终态后的持续静止。

异常模式：`mismatch`、`expired`、`foreign`、`cancel`、`identity`、`silence`、`fault`。每种使用全新独立域/目录；fault 的冲突 Publisher 不发布任何轨迹消息，仅验证门禁。不要对故障实例解除锁后继续测试。

## 原 FSM 注入

```cpp
auto motion = std::make_shared<mtc::aubo_fake::AuboFjtMotionPort>(
  owner_node, reviewed_config, controlled_credential, verified_planning_frame);
auto fsm = mtc::aubo_fake::bind_aubo_fake_fsm(existing_perception,
  existing_planner, motion, existing_tool, existing_clock,
  existing_v1_strategy, existing_pick_config);
// 原宿主单一事件线程 step；规划阻塞操作由宿主工作线程承担。
```

该入口不重新实现 FSM、不建立第二个执行者，也不提供 Mock ToolEvidence。原 Gazebo Session 中的 Tool/Perception 依赖物理接触和 Ground Truth；不能把它们直接用于 GenericSystem 并声称抓放成功。本轮原生验证是 MoveGroupBackend → MotionCommand → AuboFjtMotionPort 的独立非接触执行，完整 PickPlace 抓放循环未运行。

## 测试和证据

```bash
export ROS_DOMAIN_ID=219
colcon test --build-base /tmp/mtc_pr14_aubo_fake_build_20261014/build \
  --install-base /tmp/mtc_pr14_aubo_fake_build_20261014/install \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
    mtc_motion_execution_cpp mtc_moveit_planner mtc_ros2_action_cpp \
    mtc_trajectory_execution_cpp mtc_aubo_fake_integration_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py \
  --build-base /tmp/mtc_pr14_aubo_fake_build_20261014/build
```

实际独立 GTest 身份与原生各场景、失败历史、线程栈及子进程退出码见 [验收报告](../../docs/migration/PR14_AUBO_FAKE_FJT_INTEGRATION_20261014.md)。不能根据 Launch 父进程 exit=0 判断验收通过。Supervisor 分别记录 Functional / Teardown / Overall；SIGTERM、SIGKILL 或负退出码不伪装为正常退出，不作为 StopConfirmed。

## 下一步 AUBO Sim

先处理安装版 MoveIt 的正常退出失败，并完成独立 Fake 稳定性回归。若 AUBO Sim 指原 Gazebo/V1 仿真，保留其模型、仿真时钟、`/arm_controller` 与 GazeboSimSystem 身份域，另行通过原 MotionPort 契约对接；不能删除 GenericSystem 或 Gazebo 身份检查来混用两种模式。

若 AUBO Sim 指 SDK/RobotStudio 后端，其插件并非 GenericSystem，本适配器仍拒绝。需要单独授权、独立后端/启动凭据和验收；本包不提供能够误触发实机的启动参数或命令。
