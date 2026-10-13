# P2 系统状态与执行记录接入

2026-10-13，USER-HW-20261013-04。实现限 Web UI 观察与记录，不改变 FSM / Action / Cancel / Motion 内核，不发布 RobotState 或 ToolState。正式源码 pin 不变。OpenAI Codex 协助实现与检查。

USER-HW-20261013-05 补充[统一接口边界](README.md#接口边界与当前联调范围)：保留现有 HTTP API，未发布状态的模块保留结构和未知值，可选标定/关联格式等待对应模块负责人确认正式接口；当前联合验证仅限 C++ Mock / GenericSystem Fake Hardware，无真实机械臂控制。

## 八个模块的三态

面板分别显示正常、异常、未知/尚未接入，以及在线证据、当前值、来源、数据龄期。服务发现只能证明服务可见。状态消息过期、唯一来源丢失、浏览器暂停或失联后，当前正面结果不保持绿色；历史失败单独保留时间。

| 模块 | 真实读取来源 | 判定边界 |
| --- | --- | --- |
| Task Server | TaskState + ExecuteTask 服务发现 | 当前 task_id / state；FAULT 为异常 |
| PickPlace FSM | 原生 PickPlace FeedbackMessage / GoalStatusArray / GetResult；C++ Mock 的 integration_state | 子状态只归属于相同活动 UUID；仅发现服务时状态未知 |
| Vision | BatteryDetectionArray | 零检测是有效结果；数据过期为未知，不凭节点名宣称视觉正常 |
| Calibration / TF | 可配置 tf2 查询 + 下述独立有效性报告 | TF 可查询与标定有效分别显示；名义/Fake 参数不能证明实测标定 |
| MoveIt Planner | P1 Planner 消费者 | FAILED 或显式检查失败显示异常；缺发布器为未知 |
| FJT Execution | 原生 FJT Status / Feedback / GetResult | 显示 Goal 状态，不推定机械臂运动或停稳 |
| Tool Manager | 原生 ToolState | 状态 5 为 Fault；Attached 与独立挂载证据仍分开 |
| Robot / Controller | 原生 RobotState + 可选执行端 fault_locked | 通信、Ready、Fault、StopConfirmed 分开；没有 Fault 字段证据时显示未知 |

本轮没有 Calibration、Planner、工具验证或跨层 FJT 关联的生产发布器。这些属于可选消费者。当前源码的标定模块是离线检查，Phase 2C snapshot 的 command_id / UUID 未以相应 ROS 遥测发布；不要把测试夹具当成生产接入。

## TF 与标定

配置示例（坐标名只是占位，必须替换为现场确认的名称）：

```json
{
  "tf_pairs": [["confirmed_base_frame", "confirmed_camera_frame"]],
  "calibration_topic": "/your_calibration/status",
  "calibration_publisher": "/your_calibration_node"
}
```

`tf_pairs` 为最多 16 组 `[target, source]`。默认空，显示尚未配置，不假定相机/机械臂坐标名。tf2 只查询最新变换，不下发任何 TF。动态变换检查数值、单位四元数与源时间新鲜度；静态链的零时间戳标为“静态缓存可查询”，不因此宣称发布节点在线或标定有效。此查询不是图像采集时刻的历史 TF 检查，也不替代 Planner/视觉使用正确采集时间。

可选 `std_msgs/msg/String` JSON（唯一发布者、固定节点名，32 KiB 限制，源时间戳需新鲜）：

```json
{
  "schema_version": 1,
  "stamp_ns": 1791878400000000000,
  "calibration_id": "site-calibration-revision",
  "valid": null,
  "evidence": "unknown",
  "reference": null,
  "reason": "尚无实测验收记录"
}
```

`valid` 只接受布尔值或 null。true 必须同时提供 `evidence="measured"` 和可追溯 `reference`；发布者负责确认这份标定覆盖当前部署、工具、相机和所配置的坐标链。Web UI 检查格式与来源，不自行测量或复验标定。设备/装配/参数变化后发布者必须失效旧报告，不能持续刷新旧结论的时间戳。

## Goal 关联

父子关系不按“同一时刻正在执行”拼接。

- 本会话提交的 ExecuteTask 保存真实 UUID 与 task_id。
- 原生 PickPlace/FJT 通过只读 Status、Feedback、终态 GetResult 保存真实 UUID；没有父子字段时保持未关联。Web UI 不新增 PickPlace/FJT 发送或取消客户端。
- 配置的 Task 发布节点在 `/rosout` 中发出的精确 `MTC_TRACE event=task_start parent=... task="..."` 与 `child_response parent=... child=... accepted=1 request="..."` 被标为“日志声明”。保留 logger、源时间与原文；不是独立控制器证据。该格式来自 PR #8 的固定源码。
- 可选 `trace_topic` / `trace_publisher` 接收显式关联，默认不配置。

```json
{
  "schema_version": 1,
  "stamp_ns": 1791878400000000000,
  "link_id": "attempt-1-motion-2",
  "task_id": "task-id",
  "execute_goal_id": "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa",
  "pick_goal_id": "bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb",
  "fjt_goal_id": "cccccccc-cccc-cccc-cccc-cccccccccccc",
  "command_id": "lift-command"
}
```

UUID 必须有效、非零、层级间不同；FJT 可缺省。必须报告实际受理后的 UUID，重试/新原语各用实际新 UUID，不能拿上一轮值补空。重复报告去重；同一个 child 被不同 parent 声明、同一个 UUID 被不同 Action/任务占用时标出冲突，该节点及后代不用于推导任务归属。迟到的明确关联可检索到此前记录的子 Goal 结果。

## 查询、时间线与导出

页面“任务记录”支持 task_id、Goal UUID、模块和状态转换/错误筛选，100 条一页，“加载更早事件”继续读取。Goal UUID 查询含该 Goal 和无冲突的子孙，关系图展示祖先。任务查询含直接 task_id 事件与无冲突关联的各层 Goal；一般 ROS 日志若无显式身份就保留未关联，不附会给当前任务。

- `GET /api/events?task_id=...&goal_id=...&module=...&limit=100&before=...&transitions=1&errors=1`
- `GET /api/export`：本会话执行记录；同样支持筛选，导出不受分页或页面内存窗口限制。
- `GET /api/tasks/<owned-task-id>`：本会话持有任务的参数、真实回执/结果、各层已关联事件、最近失败和关系图。

兼容原会话导出的 `snapshot` / `tasks` / `events` / `note` 和 `tasks[].events`，附件名保留 `webui-session.json`。扩展字段为附加诊断数据，旧任务提交/取消/停止接口不变。筛选同时作用于顶层与任务内的事件数组，两者均按时间先后排列；导出快照检查产生的新 UNKNOWN 事件也包含在同一份记录中（ENV-WEBUI-API-20261013）。

`module` 为 task / pickplace / vision / calibration / planner / execution / tool / robot / system。筛选条件取交集；最多 200 条/页。历史失败保留原始错误码，例如 ABORTED/520 或 EXCEEDS_STATE_WATCHDOG。状态转换记录前后值、来源和可获得的源时间；换源后的第一次观察不补造跨源转换。

会话目录中 `events.jsonl` 逐条追加，`events.sqlite3` 为完整查询索引；内存仍有界。服务重启创建新目录，旧文件不覆盖，当前页面不自动导入旧会话。未指定输出目录的直接 Python 单测使用内存索引并明确标识不落盘；正式 server 默认创建会话目录。

“完整”指本会话**已接收、已记录的事件**全部导出：Task 变更反馈、Action 回执/结果、模块状态转换、检测身份变化、Planner/工具验证变化与接收的 ROS 日志。不是 ROS bag，不包括 UI 启动前或 DDS 丢失的信息、所有高频关节/相机帧/相同状态的逐帧进度；未关联 ROS 日志仍可从整个会话导出查看。SQLite/JSONL 不会把缺失的反馈补成事实。
