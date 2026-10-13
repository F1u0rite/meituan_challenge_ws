# C1：正常软件 Mock 延迟取消的停稳采样修复

本独立补丁基于 main `b4185155383541e1a4161587c4049a162f74b36d`，针对 PR #8 原 head `7da43228d602a6b23571bfc599d69529ef7bea23` 的 C1。用户于 2026-10-13 明确要求解决该项；修改范围是软件 Mock 适配器和 AsyncMockMotion 取消前的观测，不接管完整 FSM/Action/FJT，也不改变实机停稳契约。

## 根因与修复边界

MockMotionAdapter 即时完成虚拟运动并保存停稳证据；AsyncMockMotion 的 700 ms 延迟只推迟 Future 结果交付。PickPlace 的 10 ms 定时器每次推进 ManualClock 20 ms，等待 400 ms 后旧反馈已超过 Core 的 0.3 s 新鲜度上限。原 no-in-flight stop 复用旧快照，因此 ACK 之后返回 520 并锁定。新增确定性回归在原代码上记录反馈 1.7 s、当前时刻 2.5 s，与真实父子 Action 的 6/520 对应。

新增 `MockMotionAdapter::sample_stationary_feedback(command_id)`：在适配器锁内读取当前虚拟关节状态；仅对已记录停稳、相同 ID、无在途/队列、位置速度未变、有限且未倒退的采样时间更新反馈。原 `stationary_since_s`、命令身份、证据级别不变。不根据 Cancel ACK、运动结果缓存或“没有命令”新造停稳证明。

AsyncMockMotion 在等待 execute worker 完成后、交由 Core 判定取消前调用该方法；缺失证据场景不采样。Core 的 ID、新鲜度、低速持续区间、通信和队列检查原样保留；直接读取快照不会自动刷新，原 `StaleFeedbackCannotConfirmStop` 仍应拒绝。未知/超时锁定不会自动清除，带载取消后的 admission lock 不变。该采样只适用于适配器完全拥有状态的离线虚拟模型，不是外部机器人反馈，不能复制为 FJT/实机的“刷新时间戳”方案。

## 独立 main 基线复验（2026-10-13）

在新的隔离 Jazzy 容器中从 main b418515 构建六包、测试五个相关 C++ 包，**366 项实际用例通过**（322 Core/Mock + 44 Action；colcon 含 20 个包装项显示 386），无错误、失败或跳过。新增 8 项回归，原有 358 项保留；编译相关源码与容器字节一致，保存二进制哈希和测试身份。

额外新增 `TaskAction.DelayedCancelWaitsForChildStopThenAllowsNextTask`，使用真实父子 Action：结果交付延迟 700 ms，在 MOVE_PRE_ALIGN Feedback 后等 400 ms 再取消，父任务 CANCELED/0、完成数 0、执行和停止各一次；同栈下一任务 SUCCEEDED/0、完成数 1。独立验证不依赖 PR #8 的 Bringup/Web UI。独立补丁只有三个运行源码文件、四个测试文件和两份说明文档。

## 已完成的 PR #8 组合复验（2026-10-13）

以下真实父子 CLI 场景来自 `062811ed2895eadc8b41efe8a8d7b2acff03fe92`（C1 补丁 + 原 PR #8 集成）；这是独立 PR 拆分前的验证记录。用户随后明确“单独提交”“单独推送PR”，故通过普通 revert `8f11750` 将该补丁从 PR #8 撤出，PR #8 源码树恢复到原 7da4322。本分支只承载 C1，三个运行源码文件和原三个回归文件与组合验证逐字相同，另加上述父任务回归；Bringup/Web UI 不纳入本分支。

- 原代码新增回归：2 个延迟取消正例失败、1 个缺证反例通过；独立父子 Action 复现 ACK 后 `ABORTED/520`，下一 Goal 拒绝。原失败日志保留。
- 7 包首次构建、2 个修改包清理重编；五包 **365 项实际 C++ 用例通过**（322 Core/Mock + 43 Action；colcon 含 20 个包装项为 385）。新增 7 项，原有 358 项全部保留；无错误、失败或跳过。
- 独立进程五档取消延迟 0/200/400/600/1000 ms，父子均 `CANCELED/0`、完成数 0；前四档结果交付延迟 700 ms，最后一档 1500 ms。400 ms 档在同栈无重启提交下一任务，`SUCCEEDED/0`、完成数 1。
- `cancel_unconfirmed` 延迟 400 ms 仍父子 `ABORTED/520`，`status_unknown` 仍父子 `ABORTED/230`；下一 Goal 均拒绝。
- 七个修复场景的 supervisor、Task、PickPlace 退出 0，图排空。首次基线 CLI wrapper 没有转发 SIGINT 的退出问题单列保留，修正外部测试启动方式后重新取得完整清理证据。

环境为 x86_64 / Ubuntu 24.04.5 / ROS 2 Jazzy / C++17 的隔离软件 Mock，无设备、外部网络、挂载或端口映射。没有新浏览器、真实 FJT/NUC/实体运动验收，也不关闭其他并发或旧 Python 问题。

## 复现与回归入口

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mtc_ros2_action_cpp --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_CXX_STANDARD=17
source install/setup.bash
colcon test --packages-select mtc_motion_execution_cpp mtc_ros2_action_cpp \
  --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

若从外部归档覆盖已有源码且保留 mtime，必须清理相应构建目标或使用 `--cmake-clean-first`，避免源码时间早于旧对象导致跳过重编译。本次反证与修复都记录实际二进制及源文件哈希。

需要验证 PR #8 的组合入口时，先集成原 PR #8 与本补丁，再按其 Bringup README 启动 `--result-delay-ms 700`，在实际 MOVE_PRE_ALIGN Feedback 后通过 `mock_task --cancel-at MOVE_PRE_ALIGN --cancel-delay-ms 400` 取消。检查 Cancel ACK 与最终父子 `CANCELED/0`、完成数 0；未携带物体的后续任务应可受理并成功。`--profile cancel_unconfirmed` 应仍为 `ABORTED/520` 并拒绝后续任务，`status_unknown` 应保留 `ABORTED/230`。

原失败记录见 [2026-10-12 验收](https://github.com/F1u0rite/meituan_challenge_ws/blob/7da43228d602a6b23571bfc599d69529ef7bea23/docs/migration/CPP_MOCK_WEBUI_REVIEW_20261012.md)，没有覆盖或删除。修改基于本仓库 MIT 根许可证代码；package.xml 既有许可证差异仍按原记录，不重新授权。使用 OpenAI Codex 协助定位、编写最小补丁、正负回归和可追溯记录。未连接硬件，未合并 PR、未推进个人项目正式 pin。
