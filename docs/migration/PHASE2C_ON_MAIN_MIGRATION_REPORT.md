# Phase 2C P0 — 已合并 Phase 2B 基础上的迁移验收

日期：2026-10-12。范围：受控、隔离、非接触 GenericSystem Fake 执行。本报告的数据全部来自本轮实际编译/测试/原生运行，没有用历史验收结果替代。

## 结论

**PASS：安全迁移、原 main 486 项和旧 Phase 2C 专有 32 项身份、518 项普通及 UBSan 回归、原生 GenericSystem A–J 功能验收。** 可以创建独立 Phase 2C PR 供有限范围评审，继续下一阶段 Fake-only 开发。

**BLOCKED：完整 TSan 回归。FAIL：move_group 的完全正常退出，以及不适用的同进程重复析构夹具实验。** 两类失败原始记录均保留；退出不作停稳证明，析构实验没有清除 quarantine。有效独立进程压力验证 120/120 PASS。

本结论不授权真实机器人、不表示完整 PickPlace 已接通，不包含 V1 接触、ToolArmBridge、FSM Async Stop Bridge、Web UI Topic Publisher 或 PR #4/#9 功能。

## Git / 工作区

| 项目 | 值 |
| --- | --- |
| 基线 origin/main | b4185155383541e1a4161587c4049a162f74b36d |
| 原 Phase 2C 分支 | feature/phase2c-single-execution-owner |
| 原 Phase 2C HEAD | 4a67503492e6bf0f678c6bbca04413e6c670ae30 |
| 新分支 | feature/phase2c-on-main |
| 独立 worktree | /tmp/mtc_phase2c_on_main |
| 实现提交 | ff3b5a97068f16b7534f815e71cb9fc485c87b69 |
| 报告提交 | 包含本报告的后续本地提交；最终 SHA 由 Git 记录及交付输出给出 |

初始原工作区是 `feature/phase2b-trajectory-execution`、HEAD `52288e112aa74cd38ce7ae1d4f43feb2740306e6`，工作树干净。fetch 后 main 与旧 Phase 2C SHA 符合任务基线。新 worktree 直接从 origin/main 创建，没有 merge/rebase/旧分支改写。原预审资料和所有历史构建/测试日志保留。

## 迁移差异及去重

完整审计见[去重报告](PHASE2C_ON_MAIN_DEDUP_AUDIT.md)。

新增 `src/mtc_phase2c_integration_cpp`：PlanningOnlyManager、显式时间契约 DispatchAdapter、最小非接触 single_owner、planning-only Launch/plugin、受控 GenericSystem supervisor、20+1 项测试和 README。

FJT 增量只涉及：ControlledFakeStart 一次性凭据/显式构造、controlled Status 策略、关联 Feedback 新鲜度看门狗、精确 Own terminal UUID 记录、必要 Snapshot 诊断字段；追加 11 项 ControlledStart 测试和独立 FakeController 夹具。

保留 main 的：

- Action mutex 锁外 send/cancel 队列；Session/UUID 固定后再调用 Action API。
- wire dispatch 前首点与静止速度复核。
- Goal Response 捕获 callback group 的生命周期保护。
- 完整命令身份、完成缓存、精确 UUID、Scoped Cancel、无 global cancel、FAULT/Timeout 与停止证据。
- TimedTrajectory 单 Goal、按名称映射、起终点检查、Bernstein 全段保守限位。
- main `test_fjt_action.cpp` 完整原样保留，包括 8 项 mutex 回移并发测试。

默认两参数构造仍要求严格 live Status；新增受控模式只有显式一次性凭据才可用。没有把“缺 Status”普遍等同空闲。受控模式以可信新部署为执行权来源，以 UUID/Result/Feedback/Controller/JointState 跟踪运行；未知 Goal 不认领，外部参与者拒绝或锁定。

ExecutionSnapshot 新诊断追加在原字段之后，原字段保持；所有相关包重编译，不承诺旧二进制 ABI。最小宿主仅增加诊断输出以记录 UUID、计数和停止结论，没有 Web Publisher 或 FSM 接线。

`.gitignore` 新增两条独立 UBSan 生成目录规则，目录和日志实际保留，不提交构建产物。Core/Task/Manipulation/MotionExecutor/Phase 2A/Planner/ROSIDL 均未改动。

## 实际构建与普通回归

只加载 Jazzy，完整工作空间 **16/16 包 PASS，51.6 秒**。新增诊断输出后，仅重新构建 integration 包并执行其 21 项测试；其他源码未改变，不重复全量测试。

完整 colcon test：**16 包完成，约 1 分 15 秒**。本轮 GTest 注册、源码声明和执行 XML 三方核验：

| 包 | 独立 GTest PASS |
| --- | ---: |
| mtc_core_cpp | 23 |
| mtc_task_cpp | 87 |
| mtc_manipulation_cpp | 120 |
| mtc_motion_execution_cpp | 88 |
| mtc_ros2_action_cpp | 40 |
| mtc_moveit_planner | 29 |
| mtc_trajectory_execution_cpp | 110（原 99 + ControlledStart 11） |
| mtc_phase2c_integration_cpp | 21 |
| 总计 | **518 / 29 个可执行文件** |

**FAIL 0 / ERROR 0 / SKIP 0 / 缺失 0 / 重复身份 0 / 未执行 0。** main 固定 486 项 + 原 Phase 2C 专有 32 项全部保留。本次另外新增独立 GTest 身份 0；32 项是移植的已有专有身份，不将重复运行计作新增。

身份脚本固定 main `b418515` 和 Phase 2C `4a67503` 的 Git 对象；核验源码、二进制 `--gtest_list_tests`、实际 `*.gtest.xml` 并要求每个身份执行一次。浅克隆须先补齐这些审计对象。

其他已注册测试：Python **269 PASS**、接口 manifest/校验 **5 PASS**。完整 `colcon test-result` **821 条记录** = 518 GTest + 29 CTest 包装记录 + 269 Python + 5 接口校验。821 不是独立 C++ 数量。独立 Python 离线 E2E、未注册的 Calibration 根目录测试本轮 **NOT RUN**。

证据：[普通身份](evidence/phase2c_on_main/normal_identity.log)、[XML 哈希与完整身份表](evidence/phase2c_on_main/test_identity_results.json)、[构建日志](evidence/phase2c_on_main/normal_build.log)。原始全量结果 `/tmp/phase2c_migration_results.log`、测试输出 `/tmp/phase2c_migration_test.log`，正常 XML 在本 worktree `build/<package>/test_results/`。

## 并发与销毁压力验证

四项：ConcurrentGoalsSingleOwner、ConcurrentPollingAndCancel、ConcurrentSnapshotPolling、DestroyDuringPendingAction。

最初 `--gtest_repeat=30` 同进程运行：前三项均 30/30 PASS；销毁用例第 1 轮 PASS，第 2–30 轮在 SetUp 因“endpoint already owned or quarantined”失败，继而未关联 Node 的 TearDown 抛异常，共 **29 轮 FAIL**。原因可定位：销毁未确认停稳的 Port 必须保留进程内端点隔离，而夹具按测试名重复创建相同端点；不能为了重复实验清除生产隔离锁。这不是取得线程栈证实的死锁，也不是 Goal/Cancel 断言失败。

在保持全部测试源码和 quarantine 逻辑不变的情况下，改为每轮独立 Fake Action 进程/Context 执行同样四项，**30 轮、120 次全部 PASS**。各测试使用真实 Client/Server、四线程 executor，并设置全局及单次有界超时。

原始失败保留：[同进程实验日志](evidence/phase2c_on_main/same_process_repeat_failure.log)、[最后一轮 XML](evidence/phase2c_on_main/same_process_repeat_failure.xml)。GTest repeat XML 只反映最后一轮，29 次失败由完整逐轮日志统计。有效结果：[独立进程压力统计](evidence/phase2c_on_main/fresh_process_repeat_results.json)，完整每轮日志在 `/tmp/phase2c_fresh_concurrency/`。没有删除失败测试或以 120 次运行膨胀 518 项身份。

## 原生 GenericSystem A–J 验收

使用只读参考安装 `/home/aaet/aubo_driver_validation_ws/install_no_symlink`，再加载本 worktree 的正常 install。Domain **201–208** 各场景独立受控新启动，LOCALHOST、无 static peers/discovery server/自定义 DDS profile，PYTHONDONTWRITEBYTECODE=1。

Supervisor 在启动控制器前展开现有 S3 Xacro，必须唯一 `mock_components/GenericSystem`，核对位置能力交集。宿主先完成空域观察、消费 Node/domain/endpoint 绑定凭据，然后才启动新的 Fake Controller 和 planning-only move_group。运行服务验证唯一 GenericSystem、Active JTC/JointState broadcaster、fresh JointState、单发令入口；证明不足不会派发。

| 要求 | 结果 | 实测 |
| --- | --- | --- |
| A Controlled Cold Start | PASS | 无初始 Status：received=0/fresh=0，受控凭据及运行证明建立 ready；无预热 Goal、无伪造 Status |
| B Planning-only | PASS | 15 点计划，FJT Goal 0/Cancel 0；MoveIt 执行能力负面探针 CONTROL_FAILED |
| C Single Execution Owner | PASS，有限可信部署 | 正常图 Server 1/Client 1、direct publishers 0；GenericSystem 和控制器验证成立 |
| D 完整时间轨迹 | PASS | 15 点一次完整提交，Goal 1/Cancel 0；FJT 成功及 named final joints 一致，终态后独立停稳 |
| E Start Mismatch | PASS | 软件计划整条路径偏移产生起点不一致，明确拒绝，Goal 0/Cancel 0，未覆盖实际 JointState |
| F Expired Plan | PASS | 生成时间过期，明确拒绝，Goal 0/Cancel 0 |
| G Foreign Client | PASS | Client 2、ready=0，新命令拒绝，Goal 0/Cancel 0；没有外部 Goal 发令或抢占 |
| H Cancel Own Goal | PASS | Goal 1/Cancel 1，原 UUID；首次 stop_confirmed=false，终态后独立停稳 true |
| I Status Silence | PASS | 成功后 Status 事件数保持 2，接收证据过期；部署授权保持 ready，JointState 持续新鲜 |
| J Fault Lock | PASS | Own Goal 在途增加外部 Client；原 UUID Cancel 1，unknown/FAULT 保持，第二命令拒绝，Goal 总数 1 |

对应八种 supervisor mode 全部返回功能 PASS：plan、execute、mismatch、expired、foreign、cancel、silence、fault。A/C 在各 mode 启动/图证明中覆盖，并非额外 2 个 GTest。

B 的规划本身不调用执行。另有宿主主动发出的 `/execute_trajectory` **拒绝能力探针**；其收到 ABORTED/CONTROL_FAILED，目标 FJT 始终 0 Goal。不能将其表述为没有任何 ROS Action 请求。

### 真实 UUID / 反馈 / 停止记录

| mode | Goal / Cancel | UUID | Feedback | Result / StopConfirmed / FaultLocked |
| --- | --- | --- | ---: | --- |
| execute | 1 / 0 | fa22d38985d6fc449ba8c38950f46abe | 261 | SUCCEEDED / true / false |
| cancel | 1 / 1 | fb19abc7c67e1a0a70754e35312b1c54 | 1 | CANCELED / true / false |
| silence | 1 / 0 | 6aa89437696b64b36ae3946eec609da3 | 261 | SUCCEEDED / true / false |
| fault | 1 / 1 | 226095952834e81d6b5ca84415d1ad31 | 1 | STOP_UNKNOWN，status_unknown=true / false / true |
| plan/mismatch/expired/foreign | 0 / 0 | NONE，无 Goal Handle | 0 | 无 MotionResult；停止结论 UNKNOWN，不伪造 true |

execute 的终态后窗口：45 个新样本、0.219818 秒；cancel：44 个样本、0.214958 秒；silence 初始完成窗口：45 个样本、0.219831 秒。同时检查 JointState 新鲜时间、速度和位置差分，成功还检查终点。**没有把进程退出、Cancel ACK 或 Accepted 当作停稳。** Fault 行保持 false，不根据后续软件进程结束自动升级成功。

原始规划执行时间为 4.36631 秒，受控 Adapter 显式均匀减速 ×3，再校验全部轨迹/导数/timeout/spline/起点；原始计划不修改，空间路径不变、不拆 Goal。目标为 Synthetic FK 小偏移，非感知或实测提环位姿。

原生统计来自 Port 出站计数和匹配 UUID 的控制器 Response/Feedback/Result，不是独立 JTC 累计审计计数器。图仅提供有限可见性，不构成跨进程原子互斥或 DDS 认证。外部活动 Goal/乱序身份反例由真实 Fake Action Server 回归验证；本轮未在原生 JTC 上发送 Foreign 运动 Goal。

证据：[八模式完整摘要](evidence/phase2c_on_main/native_results.json)，各 mode 的 `*_evidence.log`，原始 owner/controller/move_group/展开 URDF/ROS 日志均保留于 `/tmp/phase2c_native_migration/`。展示摘录仅去除行尾空白，原始日志不修改。

明确公开的 8 个参考驱动 Launch/Xacro/SRDF/控制器文件前后指纹 **变化 0**，见[只读核验](evidence/phase2c_on_main/driver_readonly_check.json)。没有扫描凭据、修改驱动或启动真实模式。

## Sanitizer

- **UBSan PASS**：独立 `build_ubsan` / `install_ubsan`，9 个依赖包构建约 1 分 22 秒；8 个 C++ 包实际测试约 1 分 11 秒，**518 / 29 个可执行文件**，FAIL 0 / ERROR 0 / SKIP 0，完整身份保留。
- scoped sanitized test-result 为 **547 条** = 518 GTest + 29 CTest 包装记录。普通 build/install 不受污染。
- 编译 `-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer`，可执行/共享库链接同 sanitizer；UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1。
- **TSan BLOCKED，完整 TSan 回归 NOT RUN**：本轮实际最小无竞态程序退出 66：`FATAL: ThreadSanitizer: unexpected memory mapping ...`。没有改 ASLR、安全设置或忽略报告，原始错误保留。
- 系统 ROS/DDS/MoveIt 动态库未全部重新插桩；UBSan PASS 不表示已证明不存在数据竞争。原生 GenericSystem 验收使用普通候选安装。

证据：[UBSan 身份](evidence/phase2c_on_main/ubsan_identity.log)、[构建](evidence/phase2c_on_main/ubsan_build.log)、[TSan 真实错误](evidence/phase2c_on_main/tsan_probe.log)。原始全量 UBSan 结果 `/tmp/phase2c_ubsan_results.log`，XML 和构建目录均保留。

## 退出行为与剩余边界

**FAIL：正常 move_group teardown。** 各 mode 的 Launch 父进程返回 0，但 move_group 子进程在 SIGINT 后 5 秒未正常结束，升级 SIGTERM、exit -15；相关 `*_shutdown.log` 记录。不能用父进程 0 声称子进程正常结束。其他受控软件进程结束，最终无本轮遗留 owner/move_group/ros2_control 进程。

该已知部署限制未通过修改上游 MoveIt/驱动绕过；在 finite Fake 功能 PR 中必须披露，但没有替代 D/H 的独立停稳证据。涉及 fault 的执行始终报告不确定和锁定。

仍未完成：FSM Async Stop Bridge、Task/PickPlace 联合宿主、ToolArmBridge/V1 接触、Web UI Publisher、PR #4/#9 Scene/CAD 能力。原同步 FSM 不能直接把异步停止 ACK 当作停稳；本轮没有改变它。

真实机械臂、实体接触/IO、SDK/RPC/RTDE、控制柜、实机安全认证：**NOT RUN**。不适用的软件路径保持 disabled/unsupported。

## 复现命令

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp/mtc_phase2c_on_main
colcon --log-base log/migration build --parallel-workers 2
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=229
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONDONTWRITEBYTECODE=1
unset ROS_STATIC_PEERS ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE \
  FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI ROS_DISCOVERY_SERVER
colcon --log-base log/migration_test test --executor sequential
colcon test-result --test-result-base build --all --verbose
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
```

UBSan 使用独立目录，先在新的 Jazzy shell 构建，随后只加载该安装进行测试：

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp/mtc_phase2c_on_main
colcon --log-base log/migration_ubsan_build build \
  --build-base build_ubsan --install-base install_ubsan \
  --packages-up-to mtc_phase2c_integration_cpp mtc_ros2_action_cpp mtc_task_cpp \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined'
source install_ubsan/setup.bash
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
# 同上 LOCALHOST 环境；八个 C++ 包按顺序运行
colcon --log-base log/migration_ubsan_test test \
  --build-base build_ubsan --install-base install_ubsan \
  --packages-select mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp \
  mtc_motion_execution_cpp mtc_ros2_action_cpp mtc_moveit_planner \
  mtc_trajectory_execution_cpp mtc_phase2c_integration_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build_ubsan
```

GenericSystem 环境/运行命令见[新包 README](../../src/mtc_phase2c_integration_cpp/README.md)。本轮执行的 batch wrapper 为 `/tmp/run_phase2c_native_migration.py`，每个 mode 新域、新证据目录，失败即停止，没有预热 Goal。

## 下一阶段与 PR 建议

建议创建独立 Phase 2C PR，base=main，head=feature/phase2c-on-main；限定受信任 GenericSystem Fake、Planning-only、时间适配和非接触执行，显式披露 TSan/退出限制及原始 stress 实验。

可以进入下一阶段 **Fake-only** Async Stop Bridge/Operation Registry 设计与测试。不得据此启用真实设备或 V1 接触；不得自动合并 main。本轮只创建本地提交，没有 push、PR 创建或 merge；原分支与审计记录保留。
