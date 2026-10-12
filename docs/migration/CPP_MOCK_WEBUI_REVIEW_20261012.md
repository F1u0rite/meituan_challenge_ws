# C++ Mock / Web UI 集成审阅与内核交接

候选起点 main `f7eaa2c0d2887aeada4fd635112c63c237962ff7`，工作范围为统一启动、参数、只读状态、父子 UUID 日志、网页和 ROS 客户端接入。未合并未交付分支，未更改 `ActionSlot`、Task/PickPlace FSM、Motion Execution 或 StopEvidence 的语义。

[启动 SOP](../../src/mtc_cpp_mock_bringup/README.md)；[个人仓库验收报告与冻结来源](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/main/docs/cpp-mock-webui-acceptance.md)。本 PR 请求 F1u0rite / 孙嘉明审阅，不自动合并、不推进正式 pin。

## 已验证范围

独立 Ubuntu 24.04 / Jazzy，单容器 2 CPU / 4 GiB，无外部网络、设备、宿主挂载或 Docker 端口映射。C++ Task/PickPlace 实际由各自的 ELF 可执行文件运行，无旧 Python Action Server。图快照验证指定命名空间、唯一 Action Server、类型与发布端 QoS；源码只读状态配置 Reliable / Volatile / depth 5，但本环境 DDS 图 API 返回 history=UNKNOWN / depth=0，未把该返回值冒充已实测的队列深度。

- 原 C++ Action 测试 40 个实际用例通过（colcon 总计 44，含 ament 包装项）。UI 离线检查 39 项通过。
- 六个实际 HTTP→rclpy→C++ Task→C++ PickPlace 场景通过其断言：基础任务 T0、三块序列 P1/P2/P3、及时取消 CANCELED/0、注入规划失败 ABORTED/200、取消证据不足 ABORTED/520、运动状态未知 ABORTED/230。最后两项均保持 admission lock。
- 实际 C++ 服务端拒绝非法与并发 Goal；HTTP 自身也拒绝重复/并发提交。HTTP 202 只代表队列，先等 ROS 受理后才可判定并发拒绝。
- 浏览器实际发任务、展示 Feedback 与成功 Result。重复 C++ PickPlace Server 使界面禁用提交，HTTP 返回 409，移除冲突后恢复唯一图。关闭和同一 namespace 的重新启动均检查图排空，Web 新会话不重发旧任务。

## C1：浏览器操作延迟下的取消完成未通过（负责人：孙嘉明）

不是 `stop_evidence_available=false` 故障注入。启动 `success` 配置、`result-delay-ms=700`，浏览器在显示 MOVE_PRE_ALIGN 后点击 Cancel：父 Goal `07750616e1bd41eaba003cdfff9ba0bc`，子 Goal `b82da195d639f268b9f47da27b2dec08`。父 Cancel ACK 受理，但子/父实际 Result 均为 ABORTED/520。网页明确显示失败、完成数 0，并禁止下一任务；没有把 ACK 当作取消完成。

在干净的新进程中，下面最小 CLI 复现也得到同样 520：

```bash
# 终端一，正常 success 软件 Mock；700 ms 是既有 Future 结果交付延时
ros2 run mtc_cpp_mock_bringup mock_stack --namespace /cpp_browser \
  --output-dir /tmp/cpp-cancel-boundary-001 --result-delay-ms 700
# 终端二，source 相同环境 / 相同 ROS_DOMAIN_ID
ros2 run mtc_cpp_mock_bringup mock_task --namespace /cpp_browser \
  --output-dir /tmp/cpp-cancel-boundary-client-001 --cancel-at MOVE_PRE_ALIGN \
  --cancel-delay-ms 400 \
  --goal '{"task_id":"delayed_cancel_001","mode":1,"ordered_colors":[1],"timeout_ms":30000,"placement_stability_sec":3}'
```

CLI 父 UUID `3587bc1fe5b64da7a99405eded248d69`、子 UUID `c861a09931ce7ecf74f2261aca0a6f74`。CLI 退出 0 只表示拿到真实终态，不表示任务/取消成功；`result.json` 是 status=6 / error=520。正常快速取消的 5/0 原始正例同样保留，不能拿它覆盖这里的失败。

静态分析：`AsyncMockMotion` 的 `result_delay_ms` 延迟交付而不延长即时 Mock 运动；PickPlace 定时器继续推进 ManualClock。`MockMotionAdapter` 在完成运动时保存静止证据，`MotionExecutorCore::evidence_valid` 默认要求其龄期不超过 0.3 s，无在途命令的 stop 不刷新证据。该实现与观察到的“晚一点取消变为 520”一致；本轮未增加内核内部探针证明究竟哪一判据失败，确切根因及预期契约请负责人确认。请审阅 Mock 持续反馈/证据新鲜度与 Cancel 的契约，并提供覆盖正常操作时序的回归；不要放宽有效性检查或假造新反馈来让本例变绿。

## 保留的验收工具失败

早期 HTTP 辅助函数与 `http` 模块同名，导致请求未发出；修正了工具命名。早期诊断客户端未等 DDS server/订阅匹配，短任务观测缺失；补了有界匹配检查。一次并发测试在 HTTP 202 后立刻抢发第二 ROS Goal，实际第二个先受理、网页首个被拒绝；这是测试排序错误，保留原始拒绝证据后改为等待 ROS ACCEPTED。浏览器测试先等待“已取消”超时，实际早已返回上述 ABORTED/520；超时及真实终态均保留。没有把这些初次失败删去或改写。

完整 FSM 与视觉/工具、Motion Execution/FJT/MoveIt、故障恢复、NUC/实机不在本轮通过范围。C1 交孙嘉明；张叔阳负责集成与复验。本报告和实现由 OpenAI Codex 协助；来源/许可见 package provenance。
