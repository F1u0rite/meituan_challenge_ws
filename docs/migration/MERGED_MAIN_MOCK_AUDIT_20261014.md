# 2026-10-14 已合入主线的 C++ Mock 接线与核验脚本复验

固定运行源码为 **`80863656368e063d31cf236492d47780f4429126`**，直接父 `b849f9f3c6a4a06f886201a2210e3827b1370559`；PR #8、#15 分别已合入这两个提交。本轮没有拼接未合并分支。当前分支上的修复只改两份 Python 核验脚本、增加外部负向回归入口并纠正 Bringup README 的历史 C1 描述；FSM/Action/Cancel/Motion 内核没有修改。正式项目 pin 仍为 `1d940f7c5f12561fed6641457c0b4b630a282b05`。

## 核验错误与最小修复

原 16 包构建、colcon test/result 成功，汇总 794（含 CTest 包装项）；真实独立 GTest 为 494 项/26 目标，全部执行通过。原 `tests/verify_cpp_audit_results.py` 把宏定义的 `TrajectoryValidator.Name` 当作测试，漏掉 22 个实际 INVALID_TEST 实例，并要求旧的 16 个 XML；Action 包核验脚本有相同宏错误和旧的 20 个 XML 限制。因此两个核验入口在有效证据上失败。

修复明确展开既有宏名，从 CMake 注册读取本来就属于 `mtc_*_cpp` 范围的 GTest 目标，逐项比对 XML 相对路径；不新增 Planner 到这两个入口，不删失败或跳过检查，不修改原 XML。修复后这两者分别报告 **465 项/22 目标，保留正式基线 318 个身份**。既有轨迹核验入口另行包含 Planner，报告 **494 项/26 目标**。源码声明、二进制 `--gtest_list_tests`、XML 三方身份逐项一致。

外部负向回归对原 XML 的临时副本执行：正常、缺 XML、改名 XML（数量和测试身份不变）、额外空 XML、缺用例、重复用例、未知用例、failure、error、skipped、未运行、未完成。两个入口共 **24 个判据全部符合预期**，在宿主离线和隔离 Jazzy 中分别验证。正常通过，另外 22 个拒绝结果不是产品测试失败。原始失败日志保持原样。

复跑：在对应源码版本完成正常 colcon build/test 后执行：

```bash
python3 tests/verify_cpp_audit_results.py --build-base /candidate-build --baseline 1d940f7c5f12561fed6641457c0b4b630a282b05 --baseline-count 318
python3 src/mtc_ros2_action_cpp/test/verify_results.py --build-base /candidate-build
python3 src/mtc_trajectory_execution_cpp/test/verify_results.py --build-base /candidate-build
python3 tests/check_cpp_audit_rejections.py --build-base /candidate-build --output-dir /tmp/audit-rejections-new
```

核验脚本读取结果，不构建、不执行新的 C++ fixture。build-base 必须属于已测源码；不使用 Python `-O` 禁用断言。默认旧290基线参数保留；本轮使用上述显式318参数。

## 实际 HTTP → C++ Task → PickPlace

统一入口、命名空间、参数和清理方式见 [Bringup SOP](../../src/mtc_cpp_mock_bringup/README.md)。新执行的交付 `mtc_cpp_mock_bringup.acceptance` 六场景全符合预期：单任务/三块序列 SUCCEEDED/0、及时取消 CANCELED/0、规划故障 ABORTED/200、无停止证据 ABORTED/520、状态未知 ABORTED/230。后两种保持 admission lock；非法 Goal、并发 Goal 和重复启动被拒绝。没有把 cancel ACK 当作完成。

外部 `delayed_http.py` 运行原安装版，Mock Future 延时700ms，HTTP看到 MOVE_PRE_ALIGN 后等待 **401.948ms**再取消。父UUID **`3135e3b430544d0d8a645fa952c763eb`**，子UUID **`17e684e21eab71dab4193d881907a726`**；Task 的 MTC_TRACE、DDS反馈/状态及子 GetResult 一致为 **CANCELED/0**，completed_count=0。父终态 `2026-10-13T20:28:52.632208+00:00`，子终态 `2026-10-13T20:28:52.631331+00:00`（UTC）。父/子反馈分别49/47条。同一 Task/PickPlace 进程内下一任务 SUCCEEDED/0、完成1；正常停止后使用相同命名空间、Domain219和端口重启，再次 SUCCEEDED/0。

C++ Task 是下游 SendGoal 服务唯一客户端；进程 `/proc/PID/exe` 的 ELF 哈希与固定安装版一致。两个 Action 各一指定 C++ Server，没有旧 Python Task/PickPlace。图记录实际类型/GID/QoS；DDS图对history/depth返回UNKNOWN/0，未冒充测得代码声明的depth5。Web仅桥接Task，未对PickPlace直接发Goal。独立GetResult仅只读观察。

Web 单元39项通过，app.js语法检查通过。首次38通过/1图像解码跳过的日志保留，再用已有离线NumPy/OpenCV缓存补齐后39全过。没有进行新的浏览器渲染/点击测试；未借用未包含的P0/P1/P2可观测分支能力。

## 环境、清理及未关闭项

Ubuntu24.04/Jazzy，Python3.12.3，MoveIt2.12.4/messages2.6.0，驱动53aab837、模型47fa5e02；镜像与详细版本在inputs/environment日志。原固定容器停止后才补验；network none、2CPU/4GiB、pids512、cap-drop ALL、no-new-privileges，无设备/宿主挂载/端口映射。所有8次Bringup生命周期的子进程均exit0；结束前Domain219无残留且docker top只剩keeper，四个相关原/补验容器均停止、pid0、非OOM。

这只支持软件Mock链路。旧Python Task仍ABORTED/220、510、520，PickPlace不可用；独立MoveIt计划/执行4/1成功但move_group退出-11；rosdep缺ament_python定义及驱动lint失败仍在。历史M1 `LateGoalResponseAfterFaultRetainsOriginalSession` 本次通过，源码未变，先前间歇失败仍交孙嘉明，不能因一次通过关闭。完整FSM/FJT、视觉接入、NUC与实机未验收；常建烁继续处理MoveIt退出，集成负责人/包维护人处理剩余依赖和lint缺口。

原[PR #8 C1失败报告](CPP_MOCK_WEBUI_REVIEW_20261012.md)与[PR #15修复来源](C1_DELAYED_CANCEL_FIX_20261013.md)保持历史字节。普通验收档案仍在个人仓库的独立本地副本；本PR只发布完成的核验修复及必要证据，不推进任一main或正式pin。

聚焦冻结[index](evidence/main_mock_audit_20261014/index.json)：**163文件**，SHA-256 **`b37f0fa041abe2d1848f5035d1dc6b06c9cb6a52cdc6da4d4fe6258f3935e735`**。原错误见before/cpp-audit.log、logs/native-verifiers.log；修复结果见logs/fix-verifiers.log、audit-regression/summary.json；父子UUID/时间见delayed-http；原GTest XML在gtest。源码版本和已测试脚本哈希在inputs.json和repaired-script-identity.json；内容只增不改。

使用OpenAI Codex协助审阅、复现、最小核验脚本修复、回归与归档。沿用既有仓库/包许可证，不改写第三方许可。
