# PR #13 Final Code Review Findings 定向修复验收

日期：2026-10-12。仅修复 ControlledFakeStart DDS 隔离遗漏及 Fake supervisor 退出结果误报；全部动态运动只在 LOCALHOST 的 GenericSystem 软件环境执行。

## 结论及统计

| 项目 | 本轮实际结果 |
| --- | --- |
| 两项定向代码修复及负面回归 | PASS |
| Jazzy 依赖构建 | PASS，9/9 包；普通增量 0.83 秒，独立 UBSan 50.0 秒 |
| 原 518 项 GTest 身份 | PASS，原 main 486 + 原 Phase 2C 专有 32，均注册并实际执行 |
| 新增 GTest | PASS，10 项；合计 528 项、29 个 executable，FAIL/ERROR/SKIP 均 0 |
| 新增验收脚本 pytest | PASS，44 项；普通与 UBSan 构建均实际执行，FAIL/ERROR/SKIP 均 0 |
| UBSan C++ 回归 | PASS，528 项，8 包测试完成约 1 分 15 秒 |
| 原生 GenericSystem Functional | 9 次调用：8 PASS、1 FAIL；8 种模式全部有功能通过记录，但首次 execute 失败保留 |
| 原生 GenericSystem Teardown / Overall | FAIL，9/9 次；move_group 实际退出码 -15，supervisor 返回 1 |
| DDS 覆盖/已有证据目录 CLI 检查 | PASS，均返回 BLOCKED/2，无进程启动；旧目录哈希不变 |
| MoveIt 完全正常退出 | FAIL，仍需单独处理，未修改第三方源码 |
| TSan | NOT RUN，本轮未执行；历史环境受限记录保留 |
| 原 269 项 Python、15 项离线 E2E | NOT RUN，本轮未重复；不能计入本轮 PASS |
| 真实机器人、SDK/RPC/RTDE、工具 IO | NOT RUN，未连接或调用 |

两项 finding 的修复通过代码和测试验收；**完整 Fake gate 仍为 FAIL，不能宣布整体验收 PASS**。MoveIt shutdown 与偶发长规划分支是保留的问题。继续限制在当前非接触 Fake 范围；本报告不授权实机，不表示 FSM Async Stop Bridge、接触运动或完整抓放完成。

## Git 与修改范围

- 原始 PR #13 HEAD：`b6c481003c52c9995babde2ef6e3332c3dc54b73`。
- 分支：`feature/phase2c-on-main`，现有 worktree：`/tmp/mtc_phase2c_on_main`。
- 修复提交：`5346f639ff338ded4ea739809bb61e8f4b561e42`。本报告及证据在后续文档提交；最终 HEAD 见 `git log -1` 和交付输出。
- 本候选与 origin/main 基线为 `b4185155383541e1a4161587c4049a162f74b36d`；本地 main 引用仍停留于旧提交 `a04ac6831dd3963e6b1ec4b9e27214eb564f9f24`，本轮未更新该引用。原 Phase 2C `4a67503492e6bf0f678c6bbca04413e6c670ae30` 未改写。
- 原仓库工作区保持 `feature/phase2b-trajectory-execution` / `52288e112aa74cd38ce7ae1d4f43feb2740306e6`，无未提交修改。

修改文件：

| 所属包 | 文件及用途 |
| --- | --- |
| mtc_trajectory_execution_cpp | `src/fjt_motion_port.cpp`：环境和已初始化 Context 核验；`include/mtc_trajectory_execution_cpp/fjt_motion_port.hpp`：预初始化检查 API；`test/test_controlled_start.cpp`：10 项追加测试；`CMakeLists.txt`、`package.xml`：显式 rcl 依赖 |
| mtc_phase2c_integration_cpp | `src/single_owner.cpp`：ROS 初始化前检查；`scripts/run_fake_gate.py`：结果分离和独占证据目录；新增 `scripts/launch_recorder.py`：真实 Launch 子进程退出事件；新增 `test/test_fake_gate.py`：44 项脚本测试；`CMakeLists.txt`、`package.xml`：测试注册和脚本安装；`README.md`：更新 SOP 与退出契约 |
| docs/migration | 本报告、原迁移报告提示及独立 `evidence/pr13_final_review/` 证据 |

FSM、Core、Task、MotionExecutor、Planner、Phase 2A、ROSIDL、真实驱动均无源码修改。原 `test_fjt_action.cpp` 及 8 项 mutex 并发测试原样保留。Single Owner、凭据绑定/一次消费/过期机制、完整命令身份、Goal UUID、Cancel、StopConfirmed、FAULT、Contact 拒绝与 quarantine 逻辑未放宽。

## Finding 1：DDS 隔离遗漏

旧实现只检查 LOCALHOST、部分 profile/static peers；遗漏 Discovery Server、实际 RMW 及签发后消费前的环境变化。Discovery Server 等配置会改变已审查的发现模型；Fast DDS 也支持环境文件/profile/transport 覆盖，不能仅凭 LOCALHOST 字符串授权。[Fast DDS 官方环境变量说明](https://fast-dds.docs.eprosima.com/en/stable/fastdds/env_vars/env_vars.html)

**修改前运行时复现**：在合法本地 Context 初始化后加入 `ROS_DISCOVERY_SERVER`，分别尝试签发凭据及消费已签发凭据。两个新增负面 GTest 在旧实现上均失败（预期抛出异常，实际未抛出）。没有启动控制器、没有发送 Goal。原始 `/tmp/pr13_findings_before.log` 和 XML 已归档；这不是实机隔离失效演示。

修复契约：

1. 初始化前要求显式 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、Domain 1..232。
2. `ROS_DISCOVERY_*`、`ROS_SUPER_CLIENT`、`FASTDDS_*`、`FASTRTPS_*`、`RMW_FASTRTPS_*`、static peers、legacy localhost、Cyclone 配置、Easy Mode、security/profile selector 等未审查配置存在即拒绝，空值也拒绝。隐式本地 DDS XML 仍拒绝。只检查/报告键名，不读取配置内容或凭据。
3. 签发与消费均核对实际 Context：RMW identifier、实际 domain、LOCALHOST discovery option、static peers 数为 0；环境字符串不能代替实际初始化配置。
4. Supervisor 固定经过验证的环境副本传给全部受控子进程；不自动清理未知配置以获取 PASS。
5. Single Owner 在 rclcpp 初始化前拒绝不合配置。默认两参数 FJT 路径保持原契约；新增检查用于显式受控凭据路径。

新增 `ControlledStart` GTest：

- `DiscoveryServerCannotMintAuthority`（含空值覆盖）
- `SuperClientCannotMintAuthority`
- `MissingRmwCannotMintAuthority`
- `UnverifiedRmwCannotMintAuthority`
- `TransportOverrideCannotMintAuthority`
- `EnvironmentFileCannotMintAuthority`
- `UnknownDdsOverrideCannotMintAuthority`
- `UnverifiedQosOverrideCannotMintAuthority`
- `DomainOverrideAfterInitializationCannotMintAuthority`
- `DiscoveryOverrideBetweenMintAndConsumeRejected`

增量 ControlledStart 21/21 PASS；其中旧 11 项保持，新增 10 项。完整回归再验证全部身份。环境变量属于受信任启动配置，程序不支持运行时并发修改 process environment；这些检查不是 DDS 身份认证或跨进程原子锁，也不证明任意环境配置/中间件都安全。

## Finding 2：功能成功掩盖退出失败

旧脚本只检查 owner 的功能标记和 Launch 父进程退出码。实际 ROS Launch 在子进程失败时仍可能返回 0；因此已知 move_group 的 SIGINT 退出失败被父进程返回码掩盖。

修复后：

- `Functional`：own integration 场景结果。
- `Teardown`：所有受控 launcher 和实际子进程退出证据；缺失事件、非零退出、异常提前退出或 supervisor 强制升级信号都 FAIL。
- `Overall`：仅两者均 PASS 才 PASS/0；功能通过但退出失败为 FAIL/1；启动门禁拒绝为 BLOCKED/2。
- 通过 ROS Launch `ProcessStarted` / `ProcessExited` 事件记录真实 pid、name、returncode、单调观察时间；不从日志字符串推断正常退出。
- 分别保留 supervisor 请求的信号、Launch 内部升级日志、父退出码及子退出码。`-15` 原样保留，不能视为正常 SIGINT。
- 新证据目录独占创建；现存空目录、非空目录和 symlink 都拒绝，已有结果不得覆盖。
- 每个退出等待有界；只操作自己启动的进程。缺退出证据不能证明其已结束，不伪称停稳。

44 项 pytest 覆盖环境负面参数、证据目录拒绝、缺退出证据、-15 保留及功能/退出结果组合。两个用例真正运行 ROS Launch 的普通 Python 子进程：退出 23 和 SIGTERM/-15，父 Launch 均返回 0；新契约均报告 FAIL。pytest 不是 GTest，不能加入 528 的独立 C++ 数量。

## 实际构建、回归和身份核验

普通构建目标 integration / Action / Task 的完整依赖，共 9 包。普通测试选择以下 8 个 C++ 包：

```bash
source /opt/ros/jazzy/setup.bash
cd /tmp/mtc_phase2c_on_main
colcon --log-base log/pr13_findings_full_build build \
  --packages-up-to mtc_phase2c_integration_cpp mtc_ros2_action_cpp mtc_task_cpp \
  --parallel-workers 2
source install/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=229
colcon --log-base log/pr13_findings_full_test test --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp \
  mtc_phase2c_integration_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build
```

| 包 | 本轮 GTest PASS |
| --- | ---: |
| mtc_core_cpp | 23 |
| mtc_task_cpp | 87 |
| mtc_manipulation_cpp | 120 |
| mtc_motion_execution_cpp | 88 |
| mtc_ros2_action_cpp | 40 |
| mtc_moveit_planner | 29 |
| mtc_trajectory_execution_cpp | 120 |
| mtc_phase2c_integration_cpp | 21 |
| 合计 | 528 |

身份脚本未修改，固定 main 486 与旧 Phase 2C 32 的要求未降低；源码声明、实际注册和本轮执行 XML 三方一致。两个构建均没有缺失、重复、未执行或 SKIP。`colcon test-result --all` 包含旧 Python/manifest XML 及 CTest wrapper，不能把其汇总作为本轮独立测试数。完整逐身份及 pytest 参数清单见[身份证据](evidence/pr13_final_review/test_identity.json)。

UBSan 使用独立 `build_ubsan` / `install_ubsan`，没有清空正常目录或旧日志：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/pr13_findings_ubsan_build build \
  --build-base build_ubsan --install-base install_ubsan \
  --packages-up-to mtc_phase2c_integration_cpp mtc_ros2_action_cpp mtc_task_cpp \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS=-fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=undefined' \
  '-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=undefined'
source install_ubsan/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=228
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
colcon --log-base log/pr13_findings_ubsan_test test \
  --build-base build_ubsan --install-base install_ubsan --packages-select \
  mtc_core_cpp mtc_task_cpp mtc_manipulation_cpp mtc_motion_execution_cpp \
  mtc_ros2_action_cpp mtc_moveit_planner mtc_trajectory_execution_cpp \
  mtc_phase2c_integration_cpp --executor sequential
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base build_ubsan
```

44 项 Python 测试在独立 UBSan 构建的 colcon test 中执行，Python 本身未进行 UBSan 插桩。UBSan PASS 不等于证明没有数据竞争。TSan 本轮 NOT RUN，未调整 ASLR、未删除历史错误。

## 原生 GenericSystem 重复验收

只读加载已审核驱动 install，静态 Xacro 确认唯一 `mock_components/GenericSystem` 与限位，运行时 FJT 服务核验 Hardware/Active Controller。隔离 Domain 201、202、211..217，显式 Fast DDS LOCALHOST，无 static peers/profile/discovery override。先消费冷启动凭据才启动 Controller；没有预热 Goal、伪造 Status、清 quarantine 或重启绕过锁定。

| 模式 / Domain | Functional | Goal / Cancel | 终态或独立证据 | Teardown / Overall |
| --- | --- | --- | --- | --- |
| plan / 201 | PASS | 0 / 0 | MoveIt 只规划；执行入口被拒绝 | FAIL / FAIL |
| execute 首次 / 202 | FAIL（能力 BLOCKED） | 0 / 0 | 三次轨迹约 70.3..70.9 秒，重定时 deadline 超过能力，未派发 | FAIL / FAIL |
| mismatch / 211 | PASS | 0 / 0 | 起点不一致拒绝 | FAIL / FAIL |
| expired / 212 | PASS | 0 / 0 | 过期规划拒绝 | FAIL / FAIL |
| foreign / 213 | PASS | 0 / 0 | 外部 Client 拒绝，未抢占 | FAIL / FAIL |
| cancel / 214 | PASS | 1 / 1 | Own UUID；初始 StopConfirmed=0，终态后约 0.220 秒静止，StopConfirmed=true | FAIL / FAIL |
| silence / 215 | PASS | 1 / 0 | Feedback=263，新鲜终态后静止确认；Status 无周期更新不错误撤权 | FAIL / FAIL |
| fault / 216 | PASS | 1 / 1 | Feedback=1，StopConfirmed=false、FaultLocked=1、status_unknown=1；后续发令拒绝 | FAIL / FAIL |
| execute 独立重试 / 217 | PASS | 1 / 0 | Feedback=263，终点按名称验证，约 0.210 秒静止，StopConfirmed=true | FAIL / FAIL |

首次 execute 失败不能从统计中删除，也不能将第二次通过解释为规划稳定性已解决。没有更改路径/限值/Planner；重试在独立新 Domain 和新证据目录进行，不使用 FAULT 后重建 Port 绕过锁定。

所有 9 次 `planning_only` launcher=0，但实际 `move_group` 子退出=-15；日志均显示 SIGINT 后等待 5 秒，ROS Launch 升级 SIGTERM。Supervisor 自身只请求 SIGINT，真实子事件和原日志保留。Controller 子进程与 owner 正常场景退出 0；首次 execute owner=1 也被如实记录。整体均返回 1。

示例成功执行 UUID：`67a27845d31946bbb4d84d1a89f8d2eb`。取消 UUID：`b2d3e8120b0cc66deb0055f5e85c1c73`。故障 UUID：`9f804a29f22343b2d6b903e55f0c5e11`。完整 command_id、UUID、反馈、停稳窗口、子退出码见[原生结果](evidence/pr13_final_review/native_results.json)。停稳证据来自软件模拟 JointState，不是实际机器人安全证明；进程退出未用作停稳证据。

8 个已审核的公开驱动/config/Xacro 文件运行前后 SHA256 一致，参考工作区未写入。全部本轮 owned 进程已结束，没有清理其他成员 ROS 节点。

## 证据与剩余限制

- [普通身份核验输出](evidence/pr13_final_review/pr13_findings_full_identity.log)
- [UBSan 身份核验输出](evidence/pr13_final_review/pr13_findings_ubsan_identity.log)
- [实际 CLI 拒绝及旧证据哈希检查](evidence/pr13_final_review/pr13_native_preflight.json)
- [原始证据包](evidence/pr13_final_review/raw_evidence.tar.gz)：245 份未修改日志/XML/退出事件/生成 Fake URDF/驱动指纹/运行脚本；含修改前两项失败、首次 execute 失败及全部重复验证。
- [逐文件 SHA256 清单](evidence/pr13_final_review/raw_evidence_manifest.json)。原 `/tmp/pr13_*`、worktree `log/pr13_findings_*` 及旧迁移/TSan 日志均保留。

**FAIL：MoveIt 正常退出。BLOCKED：可靠整体 Fake gate PASS，以及本次首次随机长规划的执行能力。NOT RUN：TSan、原 Python/E2E 全量、真实硬件。** 两项修复可供 PR #13 定向评审；不能据此宣布所有阻断已解除。后续处理 MoveIt shutdown 必须另行限定范围；本轮未修改第三方 MoveIt，未开始 P1 或真实运动。
