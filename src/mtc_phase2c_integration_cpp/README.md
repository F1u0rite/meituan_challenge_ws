# Phase 2C P0：Single Execution Owner（仅 Fake）

本包提供受控冷启动宿主、planning-only Controller Manager 和显式时间派发适配器。
复用现有 `MoveItPlannerPort` 与 `FjtMotionPort`，不创建第二套 FSM，不连接工具或感知。
实际验收、失败过程和剩余限制见[报告](../../docs/migration/PHASE2C_ON_MAIN_MIGRATION_REPORT.md)。

```mermaid
flowchart LR
    S[受信任冷启动 Supervisor] --> O[唯一执行宿主]
    S --> G[新启动的 GenericSystem / JTC]
    P[MoveIt Planning Only] --> T[完整 TimedTrajectory]
    T --> D[DispatchAdapter]
    D --> F[FjtMotionPort]
    O --> F
    F -->|一个 FJT Goal| G
```

## 执行权与生命周期

- Supervisor 在独立、LOCALHOST、无 static peers/自定义 DDS profile 的 Domain 启动宿主。
  宿主先观察三秒：只有自身 Node、没有既有控制器/Action/JointState/直接轨迹发布者。
  `ControlledFakeStart::verify_empty_domain()` 签发一次性凭据，绑定 Node、Domain 和端点，
  一秒内消费。只有看到 `COLD_START_CREDENTIAL_CONSUMED` 才启动新的 Fake Controller 和规划器。
- 凭据是受信任部署的有限启动证据，不是 DDS 身份认证或跨进程原子锁。不可在任意既有
  ROS 图中取得执行权；不能并入其他团队正在使用的 Domain。更换中间件需重新验证。
- 运行仍要求真实服务查询证明唯一 `mock_components/GenericSystem`、Active Controller、
  新鲜有效 JointState、静止起点、唯一 FJT Client/Server、零直接轨迹发布者及全部限位。
- 默认两参数 FJT 构造函数不变，仍使用 Phase 2B 严格 Status 门禁。三参数构造函数显式
  接收冷启动凭据；该路径用 Controller/Hardware/JointState 与关联 Feedback/Result 维护健康，
  不把原生事件式 Status 当作周期心跳，也不发布或伪造 Status。
- 未取 Handle 为 `GOAL_PENDING`/Unknown。所有观察到的活动 UUID 保留，Handle 到达后
  精确关联；未知/外部 UUID 不被认领，不使用 global cancel。已独立收到的 Own terminal
  Result 可解决该 UUID 的迟到旧 Active Status，不能解决其他 UUID。
- 授权状态：`VERIFYING_ENVIRONMENT → EXCLUSIVE_READY → GOAL_PENDING → GOAL_OWNED`；
  取消为 `STOP_PENDING`，异常为 `FAULT_LOCKED`。故障不自动解除，Port 销毁保留 quarantine；
  凭据不能复用。故障后需停止受控 Fake 部署并由操作者处理，脚本没有自动重启。
- ROS Action 发令/取消通过有界出站队列，在释放 Port mutex 后调用；Session/UUID 先在锁内
  固定，避免与 rclcpp_action 内部 Goal mutex 形成反向锁序。没有 detached thread。
- Cancel ACK 不等于停稳；终态后仍需新的 JointState、位置差分/速度与持续静止窗口。
  软件停止不是实体急停。本包不能证明真实机械臂安全。

## MoveIt 只规划

`planning_only.launch.py mode:=planner` 使用新插件 `mtc_phase2c/PlanningOnlyManager`：
没有控制器、没有 Handle、不能 switch/sendTrajectory、没有 FJT Client。
同时 `allow_trajectory_execution=false`，不管理控制器。

不能把 Controller Manager 字符串直接置空：当前安装版 MoveItCpp 初始化执行管理器时会
异常。也不能移除 `/execute_trajectory`：现有 MoveGroupInterface 构造函数等待此 Action。
保留的是兼容响应入口，执行能力被禁用。宿主用实际规划轨迹验证其返回
`ABORTED / CONTROL_FAILED`，FJT Goal 仍为零，然后才允许自己的 FJT 派发。

位置能力取 S3 URDF 与 ros2_control 指令限位交集：
`±2π, ±π, ±2.41, ±2.66, ±2π, ±2.23 rad`，按 FJT 配置六关节顺序。
Supervisor 校验只读 Xacro 的实际来源；同一交集加载到双方 MoveIt RobotModel。
软件速度上限 0.2 rad/s、加速度上限 0.5 rad/s²；保留 Bernstein 全段校验。
上游部分控制器 min/max 表达式存在解析警告，不能依赖其保护；软件边界不代表实机限位认证。

## 时间契约与 API

`PlanningRecord` 保存原命令、明确的生成时钟域及生成时间。只有 `STEADY`/`ROS` 支持，
未知域、非法枚举、非有限/不合理时间、过期规划拒绝。生成时间不是计划启动时刻。

`DispatchAdapter::prepare()` 返回新的执行副本，原记录不变：

- 必须显式选择 `IMMEDIATE` 或 `ROS_SCHEDULED`；前者将副本启动时间设为 0，后者校验
  ROS 时钟的未来启动时间。不静默改写原始 stamp。
- 独立检查规划年龄（宿主5秒）、当前 JointState 新鲜度/静止/按名称起点一致性。
- 为新副本生成 session/sequence 的 command_id 和 trajectory_id；只 prepare 一次并轮询
  同一不可变命令。FJT 仍拒绝相同 ID 的任何内容变化。
- 默认完整保留时间、关节名和所有路点/导数。宿主显式选用 3 倍均匀减速：`t*=3`、
  `v/=3`、`a/=9`，重算 timeout，再检查位置、速度、加速度、spline、起点和执行能力。
  不拆点、不自动修改空间路径；原始规划及其生成时间保留用于诊断。
- 宿主最多进行三次**未派发的重新规划**，每次完整能力校验。长 IK 分支、过期或其他
  不合能力的结果保持零 Goal；不能在运动状态未知时重发。

Synthetic 目标是当前状态的 FK 小偏移，不是 D435i 观测或标定后的提环。
本轮没有连接 Task/PickPlace Action；原 FSM 的同步 Stop 与异步 FJT 的 Bridge 尚未完成。

## 编译与回归

只构建，不启动硬件：

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp/mtc_phase2c_on_main
colcon build --packages-up-to mtc_phase2c_integration_cpp mtc_ros2_action_cpp
source install/setup.bash
colcon test --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
  mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_moveit_planner \
  mtc_trajectory_execution_cpp mtc_phase2c_integration_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
```

核验固定 main `b418515` 的486项和原 Phase 2C `4a67503` 专有32项身份，比较当前源码、实际注册清单和执行 XML。
本轮命令以独立 `/tmp/mtc_phase2c_on_main` worktree 为例；在其他位置部署须对应替换路径，禁止混用旧 install。
不要将历史 TSan 失败目录混入当前 scoped 结果，也不要删除历史日志。

## 受控 Fake SOP

终端必须只加载以下明确的 Jazzy / 只读驱动安装 / 当前工程环境。选择尚未使用的 Domain，
当前目录不能有隐式 DDS profile；若已有 Node，程序会 BLOCKED，不清理其他成员进程。
仅支持显式 `rmw_fastrtps_cpp`；Discovery Server、Super Client、DDS/RMW profiles、
transport/QoS 覆盖及未评审配置必须未设置（空值也拒绝）。签发与消费均核对实际 Context
的中间件、Domain、LOCALHOST 和零 static peers；single_owner 在 ROS 初始化前先校验。

```bash
source /opt/ros/jazzy/setup.bash
source /home/aaet/aubo_driver_validation_ws/install_no_symlink/setup.bash
source /tmp/mtc_phase2c_on_main/install/setup.bash
cd /tmp/mtc_phase2c_on_main
export ROS_DOMAIN_ID=210
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/phase2c_ros
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE \
  FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT \
  FASTDDS_ENVIRONMENT_FILE FASTDDS_BUILTIN_TRANSPORTS RMW_FASTRTPS_USE_QOS_FROM_XML \
  RMW_FASTRTPS_PUBLICATION_MODE ROS2_EASY_MODE SKIP_DEFAULT_XML_FILE
ros2 run mtc_phase2c_integration_cpp run_fake_gate.py \
  --mode execute --evidence /tmp/my_phase2c_execute
```

保持终端运行。预期冷启动凭据先消费，硬件插件 GenericSystem、控制器 Active、FJT
Server/Client各1、direct publishers=0；MoveIt规划并拒绝执行入口；一个完整FJT Goal完成，
新鲜终态后停稳确认，功能端输出 `PASS execute`；Supervisor 另报 Functional、Teardown、Overall。
只有功能与退出均 PASS 才返回 0；当前 MoveIt 正常退出失败时 Overall=FAIL、返回 1，
不能将其描述为整体验收通过。异常输出 BLOCKED/FAIL，不降低门禁。

其他模式：`plan`（仅规划、零FJT Goal）、`mismatch`、`expired`、`foreign`、`cancel`、
`silence`、`fault`。每次重新受控启动，使用独立证据目录；故障模式不报告已安全停止。

脚本结束或 Ctrl+C 只终止自身创建的进程组；先请求正常退出，必要时终止软件进程。
目前 move_group 退出仍可能升级 SIGTERM，此问题单独记录，不宣称正常退出全部通过。
检查自己的 evidence 日志和退出码，不用清理公共 ROS 图。禁止手动启动真实硬件模式。
证据目录必须全新；已有目录（包括空目录/符号链接）拒绝覆盖。`*.processes.jsonl` 保存
实际 Launch 子进程的原始退出码，`results.json` 分离功能、退出和整体结果。
完整修复验收见[PR #13 Review 修复报告](../../docs/migration/PR13_FINAL_REVIEW_FIX_REPORT.md)。
