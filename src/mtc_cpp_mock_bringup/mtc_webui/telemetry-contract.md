# P0 执行遥测接入约定

本约定属于 Web 桥接层 v1，由 USER-HW-20261013-06 的 P0 要求派生；不是已存在的上游 ROS IDL。2026-10-13 核对 PR #8 `7da4322…` 与 Phase 2C PR #13 `22b0607…`，尚无发布原语名、command_id、trajectory_id、预计总时长和进度的 Web 遥测节点（TEAMMATE-WEBUI-P0-20261013）。客户端已可消费这些字段；发布端需由 Motion Execution 负责人提供，缺失时页面显示“未提供”。

USER-HW-20261013-05 明确[统一接口边界](README.md#接口边界与当前联调范围)：以下可选格式是消费侧预留，正式状态接口待执行端负责人确认。现有 HTTP API 保持兼容，网页仅发送顶层 ExecuteTask Goal；当前联合验证仅限 C++ Mock / GenericSystem Fake Hardware，不控制真实机械臂。

## 直接使用的原生接口

| 数据源 | 页面字段 | 限制 |
| --- | --- | --- |
| ExecuteTask Goal / Feedback / Result、TaskState | 任务 ID、模式、口令顺序、当前块、目标对象/工位、Task/PickPlace 状态、完成数和最终结果 | `current_index` 是零起始，只用于“第几块”；完成数只用 TaskState/Result。目标颜色标注为输入口令，非新一次视觉识别结果。外部任务没有原始 Goal 时不借用本会话口令 |
| JointState | 按消息关节名对应的实际位置、速度 | rad / rad/s 转 ° / °/s；缺失、NaN 不补零，也不差分推算速度 |
| Jazzy JointTrajectoryControllerState | `feedback.positions/velocities` 与同帧 `reference.positions` | `reference` 是采样目标，非最终落点；误差为同帧目标减实际。不能混合另一帧 JointState |
| FollowJointTrajectory FeedbackMessage | 实际/目标关节量、Goal UUID、`actual.time_from_start` | 实际时间来自控制器反馈，不是浏览器秒表；原生 Feedback 不提供预计总时长、原语名或百分比 |
| FJT GoalStatusArray + GetResult | 原生 Action 状态、最终错误码/原因 | 只追踪观察到的唯一活动 UUID，不选任意历史终态。多个活动 UUID、来源变化时撤销归属；旧 Goal 的迟到反馈/结果不能覆盖新 Goal |
| 独立 RobotState | 通信、Ready、Safety Interlock、motion_active、stop_confirmed | 需唯一且与 robot_publisher 匹配的权威节点；运动状态和停稳不从 FSM、FJT Result 或遥测状态推测 |

控制器话题为空时只自动选择唯一 `control_msgs/msg/JointTrajectoryControllerState` 话题；FJT Action 为空时只自动选择唯一对应 Action。多机械臂环境应显式配置 `controller_topic`、`controller_publisher`、`fjt_action`、`fjt_publisher`。每路仍要求唯一发布者，显示实际选择的完整来源。

FJT 观察器只订阅 `/_action/feedback`、`/_action/status`，创建 `/_action/get_result` 客户端读取已观察到的终态，**不创建 FJT send_goal/cancel_goal 客户端**。Phase 2C 的执行者门禁统计 send_goal 客户端；隔离 ROS 回归检查了 Web 节点只有 get_result 客户端。Task ExecuteTask 客户端与定向取消仍保留。

## 可选执行端消息

配置 `motion_topic` 为发布端实际话题，同时必须填写完整 `motion_publisher` 节点名。类型为 `std_msgs/msg/String`，其 `data` 是以下 JSON。空配置默认不订阅，不假定 `/mtc/motion/state` 已经存在。

```json
{
  "schema_version": 1,
  "stamp_ns": 1791867600000000000,
  "command_id": "example-command-42",
  "trajectory_id": "example-trajectory-42",
  "task_id": "example-task",
  "fjt_goal_id": "00112233445566778899aabbccddeeff",
  "state": "EXECUTING",
  "primitive": "side_approach",
  "elapsed_sec": 1.25,
  "expected_duration_sec": 4.0,
  "progress": 0.3125,
  "success": null,
  "error_code": null,
  "reason": null,
  "fault_locked": false
}
```

这是格式样例；时间戳和各值不能直接用于现场发布。`schema_version`、正整数 `stamp_ns`、非空 `command_id`、`state` 必填；其余可省略或为 null。未知值应省略，不用 0/false 占位。消息最大 32 KiB。

`state` 允许 IDLE、EXECUTING、CANCELING、STOPPING、STOP_CONFIRMED、SUCCEEDED、FAILED、CANCELED、FAULT、UNKNOWN。状态必须来自执行端实际执行/取消/停止确认过程；不得将 Task/PickPlace 子状态映射为它。`success`、`fault_locked` 若提供必须为布尔值，`error_code` 为整数。取消完成和成功分别保留原始语义，不能因错误码为 0 就认定成功。

`progress` 为执行端定义的 [0,1] 有限值，发布端应说明分母和口径；本 UI 不根据 elapsed/expected 自行计算。`elapsed_sec`、`expected_duration_sec` 必须是非负有限数；没有总时长时仅显示已有实际时间。`fjt_goal_id` 若提供，需为真实 FJT UUID；与当前 FJT 反馈 UUID 不同的遥测不用于标记该轨迹。`primitive`、`trajectory_id` 和 `command_id` 必须来自执行端分配的标识。

`stamp_ns` 使用与 Web ROS 节点同步的源时钟；重复时间戳不刷新龄期。默认 3 秒失效，500 ms 未来容差。执行期间需周期发布。FJT Status 没有 Header，以 DDS source_timestamp 判龄；其事件式发布可能变旧，页面明确标为“上次状态 / 过期”，不据此断言机器人正在运动。

## 停止证据

独立区域分别显示 Stop Request 服务回执、Cancel ACK、Cancel Final Result 与 Stop Confirmed。遥测里的 `STOP_CONFIRMED` 状态或自带 `stop_confirmed` 字段不能把安全区变绿。

取消/停止请求建立新的证据边界：只有随后收到、源时间戳也晚于请求与此前样本的 RobotState，且样本新鲜、`stop_confirmed=true`、`motion_active=false`、`simulation_mode=false`，才显示独立确认。请求前的 true、延迟到达的旧样本、取消受理、Action CANCELED/SUCCEEDED 均不够。时钟不同步时保留未确认。RobotState 发布端仍负责保证布尔值依据实际独立停稳测量；消息格式与新时间戳本身不证明物理安全。

FAULT 锁定位仅用明确的执行端 `fault_locked`、C++ 集成状态 `locked` 或实际 FAULT 状态；`admission_open=false` 可能只是正忙，不能据此认定故障。未发布锁定位则显示未报告。

PR #8 的正常 Mock 延迟取消 `ABORTED/520` 仍为历史未通过项 C1；页面显示 `FAILED · ROS ABORTED / 520`，如有新鲜 FAULT 状态则一并显示。此次 fixture 注入同一终态只验证 UI 保真，不是 C1 的真实复验或修复。

来源：USER-HW-20261013-06（用户会话）；TEAMMATE-WEBUI-P0-20261013、ROS2-CONTROLMSGS-20261013（2026-10-13 查阅）；ENV-WEBUI-P0-20261013（隔离接口验证）。使用 OpenAI Codex 协助实现和验证；版本、许可与修改范围见 [provenance.json](provenance.json)。
