# FSM Core 开发者使用指南（美团无人机挑战赛）

> **发布准备状态：Final Gate 源码复审通过，等待 PR 合入 `main`。** 本指南已更新至 `fix/codex-cpp-fsm-final-gate`（`ca28d4c83295e0ac79997571d19a4678cd6b3e19`）的 C++ API。建议合并 README 与代码后发布 `fsm-core-v0.1.0` Git tag；**在 PR 合并并创建标签前，不能将它称为已经发布的版本**。
>
> **适用范围：C++17 FSM Core / 离线 Mock 开发，不是已完成的 ROS 2 / MoveIt 2 / AUBO S3 实机控制系统。** 严禁将本文中的示例改为直接驱动真实机械臂。

## 1. 团队交接：现在完成了什么？

本项目分两层状态机：

```text
比赛任务请求 (ExecuteTask)
    │
    ▼
C++ Task FSM (mtc_task_cpp)          任务口令、顺序、重试、取消与结果记录
    │  输出 CALL_PICK_PLACE 等 ActionRequest（动作意图）
    ▼
C++ PickPlace FSM (mtc_manipulation_cpp)
    ├─ PerceptionPort              感知目标、提环/主体 6D 位姿和放置状态
    ├─ PlannerPort                 规划结果（完整可执行关节命令或时间轨迹）
    ├─ MotionPort                  执行/取消运动，必须确认到位与停稳
    └─ ToolPort                    V1/V2 末端机构操作与独立验证证据
           │
           ▼
     Motion Executor / Mock Adapter (mtc_motion_execution_cpp)
           │
           ▼
     [第二阶段：ROS 2 Action / MoveIt 2 / FollowJointTrajectory / S3 Fake Hardware]
```

### 四个 C++ 包

| 包 | 职责 | 代码入口 |
|---|---|---|
| `mtc_core_cpp` | 共享类型、错误码、6D Pose、TimedTrajectory、时钟与结果契约 | `src/mtc_core_cpp/include/mtc_core_cpp/` |
| `mtc_task_cpp` | 整轮比赛 Task FSM、任务校验和动作意图 | `src/mtc_task_cpp/include/mtc_task_cpp/task_fsm.hpp` |
| `mtc_manipulation_cpp` | 单块电池 PickPlace FSM、V1/V2 ToolStrategy、Port 接口 | `src/mtc_manipulation_cpp/include/mtc_manipulation_cpp/pick_place_fsm.hpp`、`pick_place_types.hpp` |
| `mtc_motion_execution_cpp` | 单在途命令控制、校验、停稳证据、Mock/Disabled 运动适配器 | `src/mtc_motion_execution_cpp/include/mtc_motion_execution_cpp/` |

**已实现：** 核心状态机和共享 C++ API；V1/V2 工具策略；纯逻辑 Fake 与 Mock 单元测试；单命令/整条轨迹的数据表达与保守执行校验。

**未实现/未验收：** 新 C++ FSM 的 ROS 2 Action Server、MoveIt 2 C++ Planner、`FollowJointTrajectory` 端到端适配、完整 S3 Fake Hardware 抓放闭环和真实硬件控制。仓库保留部分历史 Python ROS 节点，但不等于新 C++ FSM 已经接入了这些节点。

## 2. 哪个版本可供队友使用？

**合并前：** Final Gate 已针对过期 Goal 取消误停、Mock 时钟不一致、零时长放置验证和零静止跨度误判完成针对性修复与负面测试。团队可以从此分支开始离线 Port 集成开发，但**不要把尚未合并的分支称为已发布稳定版**。

**发布条件：** 本次 Final Gate 修复已通过源码复审，Codex 报告全量回归通过。请由队友审核 PR，并让此 README 与修复代码一起合入 `main`；合并后才创建 Git tag `fsm-core-v0.1.0`。完整 TSan 仍受容器环境限制，**不得称为已完成 TSan 动态竞态验证或实机验收**。

**发布后：** 队友从 `main` 开分支，使用同一个 tag/commit 作为集成基线：

```bash
# 只在不存在的目标目录中首次克隆；已有工作区禁止重复 clone、git init。
git clone https://github.com/F1u0rite/meituan_challenge_ws.git
cd meituan_challenge_ws
git pull --ff-only origin main
git log -1 --oneline

# 根据负责模块建立独立开发分支
git switch -c feature/my-module-name
```

## 3. 环境、编译与测试

基线：Ubuntu 24.04、ROS 2 Jazzy、C++17、`colcon`、`ament_cmake`、`ament_cmake_gtest`。开发机不必是 NUC；实际依赖以各包 `package.xml` / `CMakeLists.txt` 为准。

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/meituan_challenge_ws

# 可查看 ROS 2 包；不要清空已有 build/install/log
colcon list

# 仅构建四个 C++ 核心包（自动构建它们所依赖的包）
colcon build --packages-up-to \
  mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp

source install/setup.bash

# 对四个包执行已注册单元测试
colcon test --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_motion_execution_cpp mtc_manipulation_cpp
colcon test-result --all --verbose

# 如需验证整个 ROS 2 workspace，请使用：
# colcon build && colcon test && colcon test-result --all

# 单独验证纯 Python 离线 E2E（这不是 C++ ROS 2 E2E）
python3 tests/test_mock_end_to_end.py
```

> **Final Gate 报告记录（2026-10-11，`ca28d4c`）**：ROS 2 Jazzy 12/12 包编译，**318/318 C++ GTest（原 290 项保留，新增 28 项）、269/269 Python pytest、15/15 显式离线 E2E**；XML、UBSan 和 `git diff --check` 通过。完整 TSan 因 `unexpected memory mapping` 无法启动，属于 **NOT RUN**，并非 PASS。以上为 Codex 在 NUC 的报告结果，不是本文作者在 NUC 独立重跑的结果。详见 [`docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md`](docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md)。

### Motion Executor 离线 Mock 最小装配

下面只用于单元测试或隔离的 Mock 程序，不产生 ROS 控制命令。默认 Mock 已具有共享虚拟时钟，但**建议显式传入同一个 `ManualClock` 实例**：

```cpp
#include <memory>
#include "mtc_core_cpp/clock.hpp"
#include "mtc_motion_execution_cpp/mock_motion_adapter.hpp"
#include "mtc_motion_execution_cpp/executor_core.hpp"

auto clock = std::make_shared<mtc::core::ManualClock>(100.0);
auto adapter = std::make_shared<mtc::motion::MockMotionAdapter>(
    mtc::motion::MockMotionAdapterConfig{}, clock);
mtc::motion::MotionExecutorCore executor(
    adapter, mtc::motion::ValidationConfig{}, clock);
// 只有经过验证的完整 MotionCommand 才允许调用 execute_joint_move()。
// Goal-scoped 取消：executor.cancel_goal(command_id, "user canceled");
// 监督停止：       executor.supervisory_stop("operator stop");
```

CMake 中运动执行库的依赖为 `mtc_motion_execution_cpp::mtc_motion_execution_cpp`；完整可运行装配请参照 `src/mtc_motion_execution_cpp/test/test_mock_adapter.cpp`。

### 无设备冒烟示例：运行 Task FSM 产生动作意图

以下代码只推进纯 C++ 状态机，**不会发送 ROS Action 或运动命令**。将其放入你自己的 `ament_cmake` 包 `src/fsm_task_smoke.cpp`：

```cpp
#include <iostream>
#include "mtc_task_cpp/task_fsm.hpp"
#include "mtc_task_cpp/task_validation.hpp"

int main()
{
  using namespace mtc::task;
  TaskFsm fsm;
  fsm.start();
  fsm.report_self_check(true);
  if (!fsm.accepts_tasks()) {
    std::cerr << "FSM is not ready for a task\n";
    return 1;
  }

  TaskRequest goal;
  goal.task_id = "offline-demo-1";
  goal.mode = TaskMode::MODE_BASIC;
  goal.ordered_colors = {1};  // RED
  goal.timeout_ms = 60000;
  goal.placement_stability_sec = 3.0;

  const auto check = validate_task_request(goal);
  if (!check.ok) {
    std::cerr << check.message << '\n';
    return 1;
  }

  fsm.submit_goal(goal);       // WAIT_TASK -> VALIDATE_TASK
  fsm.advance_validation();   // VALIDATE_TASK -> SCAN_SCENE
  const auto step = fsm.report_target("mock-battery-1", 1);
  if (const auto * action = step.find(ActionKind::CALL_PICK_PLACE)) {
    std::cout << "Would call PickPlace: object=" << action->pick_place.object_id
              << ", slot=" << action->pick_place.target_slot << '\n';
  }
  // 这是动作意图，不代表下层已经执行。
  return 0;
}
```

消费者 `CMakeLists.txt` 的关键部分（在一个**独立的** `ament_cmake` 包中）：

```cmake
find_package(ament_cmake REQUIRED)
find_package(mtc_task_cpp REQUIRED)

add_executable(fsm_task_smoke src/fsm_task_smoke.cpp)
target_compile_features(fsm_task_smoke PRIVATE cxx_std_17)
target_link_libraries(fsm_task_smoke PRIVATE mtc_task_cpp::mtc_task_cpp)
install(TARGETS fsm_task_smoke DESTINATION lib/${PROJECT_NAME})
ament_package()
```

消费者 `package.xml` 还需声明 `ament_cmake` 为构建工具依赖、`mtc_task_cpp` 为依赖；这是**代码示例而非一个已提交的独立示例包**，可直接参考 [`src/mtc_task_cpp/test/test_task_fsm_core.cpp`](src/mtc_task_cpp/test/test_task_fsm_core.cpp) 的测试装配方式。`MODE_BASIC` 仅需要一种颜色，对应 T0；`MODE_SEQUENCE` 需要三种不同颜色，按顺序对应 P1/P2/P3。颜色值：1 红、2 蓝、3 黄、4 绿；0 表示未知，不是合法口令。

## 4. 如何对接 Task FSM

入口：`#include "mtc_task_cpp/task_fsm.hpp"`。

| 方法或类型 | 含义 |
|---|---|
| `TaskFsm::start()`、`report_self_check(bool)` | 初始化并完成自检，进入 `WAIT_TASK` |
| `submit_goal(TaskRequest)` | 接收一轮任务；先通过 `validate_task_request()` 验证 |
| `advance_validation()` | 消费 Task FSM 的待验证结果并推进至扫描 |
| `report_target(object_id, color)` | 上报识别结果；可能产出 `CALL_PICK_PLACE` |
| `report_item_result(success, placement_verified, error_code, message)` | 只有下层 PickPlace **真正结束并验证**后才调用 |
| `record_completed_item()` | 完成记录后推进下一项或整个任务结束 |
| `check_timeouts()` / `check_task_deadline()` | 应由外部事件循环周期触发的看门狗 |
| `request_cancel()`、`confirm_child_stopped()` | 取消与确认下层真正停止，是两个不同事件 |
| `StepResult.actions` / `ActionKind` | FSM 输出的**动作请求**：外壳负责转换成 ROS 调用 |
| `action_finished()` 与 `fault_locked()` | **执行结束**与**故障锁定**不能混为一谈 |

**关键契约：** 不要直接在状态机内部做 ROS 发布、网络 I/O 或机械臂动作；外部适配层读取 `StepResult.actions`，执行 `CALL_PICK_PLACE` / `CANCEL_CHILD` / `REQUEST_STOP` / `PUBLISH_STATE` / `LOCK_ACTIONS`，再把异步完成结果反馈给 FSM。`FAULT` 结束当前执行，但不能被普通 `reset()` 或重建对象用来绕过受监督的恢复协议。

## 5. 如何对接 PickPlace FSM

入口：

```cpp
#include "mtc_manipulation_cpp/pick_place_fsm.hpp"
#include "mtc_manipulation_cpp/pick_place_types.hpp"
#include "mtc_manipulation_cpp/tool_strategy.hpp"
```

与 Task FSM 不同，**PickPlace FSM 必须注入四类 Port 和一个 Clock，不能单独靠调用 `step()` 就执行抓放**：

```cpp
// 示意：各 Port 类型由队友的适配模块提供；这里不构造真实设备对象。
// PerceptionPortPtr perception = ...;
// PlannerPortPtr planner = ...;
// MotionPortPtr motion = ...;     // Mock/Disabled only
// ToolPortPtr tool = ...;         // Mock only
// mtc::core::ClockPtr clock = ...;

mtc::manipulation::PickPlaceConfig config;
config.tool_type = mtc::manipulation::TOOL_PASSIVE_HOOK_V1;
config.placement_stability_sec = 3.0;
config.state_timeouts = mtc::manipulation::default_state_timeouts();

// 当以上依赖均已实现并注入后：
// mtc::manipulation::PickPlaceFsm fsm(
//     perception, planner, motion, tool, clock,
//     mtc::manipulation::make_strategy(config.tool_type), config);
// fsm.start("battery_1", 1, "P1", "req-demo-1", 3.0);
// 在同一 FSM 事件线程中逐次 fsm.step() 并读取 fsm.feedback()。
```

这段是**接口装配示意，不是独立可运行的完整抓放程序**；想看到可运行的纯逻辑 Fake，请从 `src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp` 与 `test/fakes.hpp` 复制测试思路。`test/fakes.hpp` 位于测试目录，**不是安装后提供的稳定公共 API**。

### Port 的责任与交付标准

| Port | 队友应该实现什么 | 不能做什么 |
|---|---|---|
| `PerceptionPort::get_detection()` | 返回 `DetectionResult`：目标 ID/颜色、唯一性、新鲜度、完整 `PoseEstimate`；放置后核验需要持续产生带新时间戳的观测 | 丢掉姿态或 `frame_id`；拿旧帧冒充新帧；伪造“物体已稳定” |
| `PlannerPort::plan()` | 为每个 `PlanTarget.primitives` 返回对应的 `PlanResult.commands`，包括关节目标或整条 `TimedTrajectory`；保留 6D 位姿、TF、速度/接触约束 | 只返回 `ok=true`；使用默认全零关节目标；把轨迹拆成多次单点调用 |
| `MotionPort` | 执行一个关节目标或一整条轨迹；支持异步在途语义和命令 ID 去重；取消需要独立停稳证据 | `accepted` 就当作 `arrived`，`request_delivered` 就当作 `stop_confirmed`；状态未知时重发 |
| `ToolPort` | 实现 `engage` / `test_lift` / `seat_verify` / `unload` / `unlock` / `disengage` / `holding_state`，返回独立 `ToolEvidence` | 解锁请求受理就报告已释放；在未落座或未卸载时解除 V2 锁止 |

**异步语义：** Port 方法返回 `std::optional<Result>`，其中 `std::nullopt` 表示尚未完成。FSM 之后可能再次查询同一命令；**端口必须按 command_id 去重/复用结果，不可每次调用都重新发送运动指令**。注意 `VERIFY_PLACED` 需要不同的、新鲜观测，不能一味缓存成功帧。

**V1/V2：** 默认 `TOOL_PASSIVE_HOOK_V1` 为被动舌规，不调用电磁解锁；`TOOL_MAGNETIC_LATCH_V2` 允许在落座、卸载两项独立核验完成后申请解锁。`ToolStrategy` 负责生成意图，不直接操作 IO。

**Final Gate 特别注意（取消接口）：** `MotionExecutorCore` 现在区分 `cancel_goal(command_id, reason)`（只取消指定命令）和 `supervisory_stop(reason)`（明确的监督停止）。**ROS Goal 的取消回调必须调用前者并携带正确 ID，不得使用无 ID 的监督停止替代。** 旧 Goal 取消被明确拒绝后，不应触发后端 Stop；`StopResult::request_delivered` 与 `stop_confirmed` 仍是两种独立状态。可对照 `test_executor_concurrency.cpp` 中的 `GoalCancelGate.*` 测试。

**放置稳定窗口：** `placement_stability_sec` 默认 3.0 秒；**零、负数、NaN 和 Inf 均属非法输入，必须在运动前拒绝**。即使传入极小正数，也至少需要两个不同时间戳且符合新鲜度要求的观测；首帧累计时间为 0 时不得宣告成功。3.0 秒是工程默认值，不是官方赛题给出的最低值。

**并发：** Task FSM 和 PickPlace FSM 对象本身遵循**单一事件线程所有权**。ROS 2 多线程回调应把事件送入串行队列；不能从多个 ROS 回调线程同时调用 `step()/start()/request_cancel()`。`MotionExecutorCore` 独立处理执行/停止竞争，但不能代替 FSM 的事件串行化。

## 6. 与 ROS 2 的关系：接口存在 ≠ C++ Action 已集成

仓库有 `mtc_interfaces` 中的 `ExecuteTask.action`、`PickPlace.action`、`ExecuteJointMove.action` 等契约，旧 Python 包也有 ROS 2 适配代码；但**当前新 C++ FSM 核心库没有实现对应的 C++ `rclcpp_action` Server，也没有自动接入 MoveIt 2**。这是下一阶段的交付范围。

未来集成时：

- `ExecuteTask` Goal → `TaskRequest` → `TaskFsm`，`StepResult.actions` → ROS 2 子操作。
- `PickPlace` Goal → `PickPlaceFsm::start()`；把 `feedback()` 转成 Action Feedback，把 `result()` 转成 Result。
- Perception 的 `geometry_msgs/PoseStamped` → 完整 `Pose6D`，必须核对 TF、坐标系和时间戳。
- MoveIt 2 完整时间参数化轨迹 → 一次 `control_msgs/action/FollowJointTrajectory` Goal；禁止多次 `ExecuteJointMove` 伪造轨迹。
- `Goal accepted` ≠ `motion complete`；`cancel accepted` ≠ `stop confirmed`；`placement_verified` 只有真正连续新观测通过才可置真。

在完成正式 ROS 2 Mock 闭环前，不得宣称新的 C++ FSM 已有“开箱即用的机器人控制节点”。

## 7. 队友开发分工及 PR 验收

推荐以合并后的 `main`/`fsm-core-v0.1.0` 作为共同基线，在独立分支实现：

| 方向 | 建议分支 | 首先交付 |
|---|---|---|
| 感知 | `feature/perception-adapter` | `PerceptionPort` Mock/转换测试、6D Pose、TF/时间戳拒绝场景 |
| 规划 | `feature/moveit-planner` | `PlannerPort` Mock/规划输出测试，`PlanResult.commands` 完整性 |
| 运动执行 | `feature/motion-controller` | Mock `FollowJointTrajectory` 单 Goal + 取消/停稳证据测试 |
| ROS 2 | `feature/ros2-action` | `ExecuteTask`/`PickPlace` C++ Action 外壳，串行 FSM 事件队列 |
| 工具 | `feature/tool-adapter` | V1/V2 `ToolPort` Fake/Mock 与解锁联锁测试 |

**提交 PR 前：** 必须附上相应 `colcon build`、`colcon test` 的实际结果，说明是否触及 IDL、TF、并发/停稳契约；包含负面测试；所有实机能力继续默认禁用。接口变更先讨论，不要各自修改同一 `.action/.msg/.srv` 使契约漂移。

## 8. 常见错误与注意事项

- **找不到包或头文件：** 确认已经 `source /opt/ros/jazzy/setup.bash`，构建后在当前终端 `source install/setup.bash`，消费者在 `package.xml`/CMake 中声明所依赖的 C++ 包。
- **`test/fakes.hpp` 找不到：** 测试 Fake 不属于安装的公共 include 目录；不要把它当作正式后端。
- **FSM 一直等待：** 检查 Port 是否返回 `std::nullopt`、是否由外部事件循环继续 tick、是否向 FSM 投递了对应完成事件；不要绕过看门狗。
- **规划器报告 `ok=true` 但不执行：** `PlanResult.commands` 必须与请求原语一一对应，并满足完整轨迹/关节目标校验；不能只返回 `ok`。
- **取消已请求但结果是 FAULT：** 停止请求被受理不等于后端已确认停稳，检查 StopEvidence 与关联 command_id；不要改写结果掩盖故障。
- **Mock 运行时钟：** Final Gate 后 `MockMotionAdapter` 默认创建 `ManualClock`，`MotionExecutorCore` 若未显式传入时钟，会沿用适配器的同一实例；如显式注入时钟，**必须是同一 `ClockPtr` 实例**。不同 `ManualClock` 即使起点相同也会被拒绝；Mock 不支持 `SteadyClock`。
- **想直接控制 S3：** 此 README 未授权任何实体控制；当前阶段只允许纯逻辑 Mock 和明确使用 `mock_components/GenericSystem` 的隔离仿真。

## 9. 源码和进一步阅读

- `src/mtc_task_cpp/test/test_task_fsm_core.cpp`：Task FSM 从 INIT 到 CALL_PICK_PLACE 的真实测试使用方式。
- `src/mtc_manipulation_cpp/test/test_pick_place_fsm_v1.cpp`：V1 PickPlace 完整 Fake 测试流程。
- `src/mtc_manipulation_cpp/test/test_pick_place_fsm_v2.cpp`：V2 工具策略流程。
- `src/mtc_manipulation_cpp/test/test_pick_place_audit.cpp`：规划输出和新鲜放置观测的安全回归。
- `src/mtc_motion_execution_cpp/test/test_executor_concurrency.cpp`：多线程执行和取消测试。
- `docs/migration/FSM_BEHAVIOR_MAPPING.md`：Python ↔ C++ 行为映射。
- `docs/migration/INTERFACE_COMPAT_AUDIT_CPP.md`：ROS 接口兼容与后续设计缺口。
- `docs/migration/CODEX_CPP_FSM_AUDIT_FIX_REPORT.md`：当前审计分支的修复与测试报告。
- `docs/migration/CPP_FSM_FINAL_SAFETY_GATE.md`：Final Gate 修复、318/269/15 回归及未完成的 TSan 动态验收记录。

---

**维护约定：** PR 合并后，请将文首状态更新为实际合并 commit 和 Git Tag；不得提前填入未创建的标签。每当 Port ABI/API 或 FSM 安全语义发生变化，负责 PR 的开发者必须同步更新本文。`main` 是团队共享的 Mock/集成开发基线，不是实机控制许可。
