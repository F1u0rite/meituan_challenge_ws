# 规划证据误报与旧输出保护修复（2026-10-14）

固定测试源：`b07ccc701eb451d3851c282096ce49ef8bd1eca0`；PR 目标为 `feature/planning-scene-start-v1`。该源是 PR #9 合入此功能分支后的版本；本修复不把整条功能分支引入 main。

交付的采集脚本把任意 `PLAN FAILED` 计为碰撞反证通过，并把没有规划结论的初始化失败写成“目标被接受”。本机原生 Fake 采集复现了两种误报：`collision_probe` 只有 NO_PLAN 却被报 PASS，`tool_collision_probe` 在 apply_scene 失败、没有规划结果时被报为目标接受。10份实际日志的逐项分类见 [evidence-regression.json](evidence/reporting_guards_20261014/evidence-regression.json)。

修复只改变证据门禁：正例必须有唯一成功结论及匹配PID的正常退出；无结论显示 NO_PLANNING_RESULT；规划拒绝不会自动证明碰撞原因，碰撞探针标为 UNVERIFIED 并使采集保持非零。独立碰撞归因仍需规划负责人补证。本PR没有声称解决任何规划失败或MoveIt框架崩溃。

另一个可复现问题是重复指定输出目录会截断原SUMMARY。ROS-free临时夹具中原脚本覆盖哨兵后在域检查退出1；修复在加载ROS前以原子mkdir拒绝已存在路径，exit2，原字节不变。测试也覆盖普通文件和符号链接。真实原始失败目录没有用于破坏性试验。

Ubuntu24.04/Jazzy隔离容器，network none、2CPU/4GiB、无设备/宿主挂载/端口映射。Planner重新构建通过；53项C++、5项原有Python、12项新Python全部通过，零错误/失败/跳过。colcon汇总80含10个CTest包装，不是80个独立用例。修复后完整Fake采集实跑exit1，明确保留场景初始化失败和NO_PLAN的未证实归因；显式插件保留诊断下子进程退出与清理通过。所有本轮ROS进程已停止，Domains221/222空。未接真实硬件。

复现入口（使用已构建的固定Jazzy/Fake依赖，输出必须为新目录）：

```bash
colcon build --packages-select mtc_moveit_planner --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mtc_moveit_planner --return-code-on-test-failure
colcon test-result --all
AUBO_WS=/driver_ws MTC_WS=/repair-source MTC_INSTALL=/repair-install MTC_RMW=rmw_fastrtps_cpp MTC_MOVE_GROUP_PIN_PLUGINS=1 bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh /tmp/new-planning-evidence
```

最后一条插件保留开关只是显式生命周期诊断；默认原生退出仍-11。详细实际命令、日志、XML、源文件哈希及范围在[冻结索引](evidence/reporting_guards_20261014/index.json)，SHA-256 `07e87ba4b0774c66c72c346300113b1a57d7a67a71626ef9103a049f6e1ab69d`，57项证据。完整FSM、Action/Cancel/Motion与实机未在本修复中验收；孙嘉明的内核职责不变。

使用OpenAI Codex协助日志分析、最小脚本修复与隔离验证；原MIT许可声明不变，无占位功能或内核修改。
