# Phase 2C on main：重叠修复去重审计

本轮基线为 `b4185155383541e1a4161587c4049a162f74b36d`，迁移来源为
`4a67503492e6bf0f678c6bbca04413e6c670ae30`。实现提交：
`ff3b5a97068f16b7534f815e71cb9fc485c87b69`。

没有复制旧 Phase 2C 的完整 FJT 实现/头文件覆盖 main；以 main 文件为底，逐项加入独有增量。
完整结果见[迁移验收报告](PHASE2C_ON_MAIN_MIGRATION_REPORT.md)。

| 功能 | 来源及处理 | 验证 |
| --- | --- | --- |
| 锁外 Action 出站队列 | 直接保留 main，不重复移植 | issue_cancel/drain 与 main 逐字一致 |
| 不可变命令身份及完成 ledger | 保留 main | same_command 与 main 逐字一致；普通/UBSan 身份与负面回归通过 |
| Scoped Cancel/Stop API | 保留 main | cancel_command/request_stop 与 main 逐字一致 |
| wire dispatch 前首点/速度复核 | 保留 main | 原 guard 完整保留，未用旧版删除 |
| Goal Response callback group 生命周期 | 保留 main | response_group=group 捕获仍存在；析构回归通过 |
| main FjtAction 测试文件 | 原样保留 | 与 main 字节相同，8 项 mutex 回移身份未丢失 |
| ControlledFakeStart | Phase 2C 独有增量 | 一次性 Node/domain/endpoint 凭据及受控启动测试通过 |
| Controlled Status/关联 Feedback 看门狗 | Phase 2C 独有增量 | 默认构造仍严格 Status；受控构造仅用于 Fake 可信部署 |
| ExecutionSnapshot 增量 | 追加诊断字段 | 原字段保留，消费者全部重编译；不承诺旧 ABI |
| PlanningOnlyManager/DispatchAdapter/最小宿主 | 独立新包 | 新增 21 项测试，原生仅规划/执行隔离验证 |
| ControlledStart 测试/夹具 | 只加入新文件 | 新增 11 项测试；没有替换 main 现有夹具 |
| verify_results.py | 扩展核验能力 | main 486 + 旧 Phase 2C 专有 32，注册/XML/源码三方一致 |

可重复核对的源码哈希与断言记录：
[dedup_checks.json](evidence/phase2c_on_main/dedup_checks.json)。

与旧 Phase 2C 实现比较，当前 FJT 额外保留 main 的发送前起点复核及 callback group
生命周期捕获；其余必要受控执行功能迁移成功。默认禁用 Fake execution、接触拒绝、
UUID、FAULT、停止证据与 spline 校验没有被放宽。

以下包相对 main 无跟踪文件差异：`mtc_core_cpp`、`mtc_task_cpp`、`mtc_manipulation_cpp`、
`mtc_motion_execution_cpp`、`mtc_ros2_action_cpp`、`mtc_moveit_planner`、`mtc_interfaces`。
Calibration 与其他成员文件未被旧分支的缺失内容覆盖或删除。

原 Phase 2C 分支没有改写；main 未修改。没有 merge、rebase、push 或 PR 创建。
本轮没有引入 PR #4/#9、V1 接触动作、ToolArmBridge、FSM Async Stop Bridge 或 Web 发布器。
