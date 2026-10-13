# P1 视觉、规划与工具证据接入

范围来自 USER-HW-20261013-02（用户会话）。此文给出已实现的 Web 消费接口及尚待负责人提供的发布端，不修改 FSM/Planner/工具内核。正式基线仍为 `1d940f7c5f12561fed6641457c0b4b630a282b05`。

按 USER-HW-20261013-05 的[统一接口边界](README.md#接口边界与当前联调范围)，可选 Planner / 工具验证格式属于消费侧预留，正式状态接口由各模块负责人提供并确认。通过 ROS Topic 或统一只读状态适配，保留现有 HTTP API，不增加规划调用或工具 IO 按钮；当前联合验证仅在 C++ Mock / GenericSystem Fake Hardware 中进行。

## 原生视觉与图像

默认订阅 `/mtc/perception/batteries`，类型 `mtc_interfaces/msg/BatteryDetectionArray`。可用 `perception_topic` / `perception_publisher` 指定；每路必须唯一发布者，来源变更、重复发布者、过期或无效源时间撤销当前显示。上游要求 RELIABLE + KEEP_LAST(5)；Web 使用兼容的 sensor-data reader，仅观察、不参与运动放行。

检测列表显示 object_id、颜色、confidence、body_pose/ring_pose 的 xyz（m）、四元数、坐标系和各自 ROS 时间戳，以及发布端原始 `ring_pose_valid`。NaN/Inf 变成 null，不补零；零四元数、无坐标系、无效时间戳和旧位姿不作为有效位置展示。即使数组新鲜，位姿仍按其独立 Header 判龄。`ring_pose_valid=true` 也可能伴随“位姿不可用”，UI 不修正成可抓取。ID 缺失或重复不高亮；当前目标只来自本会话新鲜 ExecuteTask Feedback 的 `current_object_id`，不按颜色猜测。零检测帧与未收到检测明确区分。

RGB / Depth 沿用 `color_topic` / `depth_topic`；空值仅自动发现唯一候选。Raw Image / RGB CompressedImage 由 OpenCV 解码；16UC1 深度按 mm、32FC1 按 m，固定 0–2 m 伪彩，无效深度为黑色。系统需安装 `python3-opencv`、`python3-numpy`。这些编码口径不是 D435i 实物单位或标定验收。

JPEG HTTP 响应 `X-MTC-Frame` 为 Base64 编码的 JSON，包含与**返回字节同一锁内取得**的源、坐标系、尺寸、精确十进制字符串 `source_stamp`、龄期、stream、session_id。浏览器用该元数据标注实际显示帧，不借用随后快照的时间；超时、换源、暂停或断线隐藏旧帧。图像上限 5 Hz 编码、页面约 750 ms 刷新，属于操作观察预览。

2026-10-13 核对 PR #5 `dc76a0b…`：原生消息没有二维框，也没有像素提环位置；适配器当前强制 `ring_pose_valid=false`，无有效三维位姿时为 NaN。因此本轮完成检测列表与列表目标高亮，RGB 叠框留待像素坐标、CameraInfo/TF 和图像帧同步契约确定。不会把三维 xyz 直接画成像素框，不声称已接实物相机。

## 可选 Planner 反馈

2026-10-13 核对 PR #4 `943bf37…` 的 `mtc_moveit_planner`：有 PlanResult / PlanReport 和 YAML 报告，含 primitive_name、point_count、duration_s、reason/message，但未发布对应实时 ROS 状态话题（TEAMMATE-WEBUI-P1-20261013）。以下是 Web 桥接层 v1 契约，**不是已存在的上游 IDL**。由 Planner 负责人在真实规划入口/回调提供发布器，Web 只消费。

配置 `planner_topic` 和完整 `planner_publisher`，类型 `std_msgs/msg/String`，data 示例：

```json
{
  "schema_version": 1,
  "stamp_ns": 1791871800123456789,
  "plan_id": "plan-example-1",
  "task_id": "task-example",
  "command_id": null,
  "trajectory_id": null,
  "primitive": "side_approach",
  "state": "FAILED",
  "point_count": 42,
  "expected_duration_sec": 8.7,
  "joint_limits_ok": null,
  "collision_free": null,
  "start_state_consistent": false,
  "error_code": "EXCEEDS_STATE_WATCHDOG",
  "reason": "EXCEEDS_STATE_WATCHDOG: trajectory exceeds state watchdog"
}
```

示例为格式说明，非本次 Planner 运行结果。必填 schema_version=1、正整数 stamp_ns、非空 plan_id，以及 WAITING / PLANNING / SUCCEEDED / FAILED 之一。其余字段可省略/null；未知值不能用 0/false 代替。point_count 为非负整数，预计时长为非负有限秒，错误码可为整数或字符串。三项检查必须是显式布尔或 null；规划成功不自动填“检查通过”，缺省碰撞规避配置也不等于本次检查结果。

每条是同一 plan_id 的完整快照，新计划不沿用上次检查、点数或原因。task_id 与当前任务不匹配时明确标记“其他任务”；没有 task_id 不建立当前任务归属。错误原文和状态变化保存在事件记录中。UI 不解析自由文本日志去制造结构化检查结果；现有 `/rosout` 仍可查看原始日志。

## 原生 ToolState 与可选验证记录

原生 `/mtc/tool/state` 已包含 state、verified、evidence_level、tool_type、object_id、detail。显示 `passive_hook_v1` 为 V1 Passive Hook，`magnetic_latch_v2` 为 V2 Magnetic Latch；这只是实际发布的工具类型，不替团队选择末端。

state=Detached / Engaging / Attached / Releasing / Fault 独立显示。`verified` 验证的是**当前工具状态**，Detached 的 verified=true 不是挂载成功；只有新鲜 ToolState 的 state=Attached、verified=true、evidence_level=2 或 3，且 object_id 非空，才显示“已独立确认挂载”。0=无证据，1=仅命令，2=几何/机械反馈，3=传感/视觉；上游 mtc_tool 定义 2 为最低独立证据等级。矛盾的 verified=true + 命令等级不会显示挂载成功。

原生 ToolState 没有试提/落座字段；ToolManager 内部值及离线占位参数不当作实机验证。配置可选 `tool_verification_topic` / `tool_verification_publisher`，类型同为 String JSON：

```json
{
  "schema_version": 1,
  "stamp_ns": 1791871800123456789,
  "verification_id": "grasp-attempt-example-1",
  "task_id": "task-example",
  "object_id": "battery-example",
  "tool_type": "passive_hook_v1",
  "test_lift": {
    "verified": true,
    "evidence_level": 2,
    "source": "relative_motion_observer",
    "stamp_ns": 1791871800023456789,
    "reason": "independent relative-motion evidence"
  },
  "seated": null
}
```

身份和外层时间戳均必填。verification_id 唯一标识本次抓取尝试，发布端必须在新尝试/重试开始时更换它并清空旧检查；同一 task_id/object_id 不能复用上一尝试证据。外层为当前有效记录的心跳时间，各项 stamp_ns 为实际验证事件时间，必须正数且不晚于外层；不能用发布心跳覆盖事件时间。各检查 verified 可 true/false/null；true 必须有等级 2/3 和非空独立来源，命令回执不够。省略检查表示未提供，试提通过不填充落座验证。

记录仅在自身、ToolState、当前任务归属有效时用于当前显示，task_id/object_id/tool_type 必须匹配；若存在当前目标 ID，也必须一致。不匹配、断线、过期均撤销检查结论。检查事件可早于当前心跳，页面保留事件时间；它们属于该次尝试的历史完成证据，不是实时力传感测量。工具记录不参与 Stop Confirmed；P0 独立停止证据边界不变。

## 通用约束与验证范围

可选 JSON 消息最多 32 KiB，默认不订阅；启用必须显式固定发布节点，仍要求唯一发布者。时间戳使用与 ROS Web 节点同步的源时钟，默认 3 秒过期、500 ms 未来容差；重复时间戳不刷新龄期，周期发布只维持本次实际有效状态。不添加运动、工具 IO、解锁或 Planner 调用入口。

本轮隔离接口验证登记为 ENV-WEBUI-P1-20261013；合成画面、检测、错误与验证记录只证明 UI 接收和展示边界，不代表浩然视觉、建烁 Planner、实机挂载或整体 FSM 已验收。RViz/独立 3D 留在后续。

来源：USER-HW-20261013-02（用户会话）；TEAMMATE-WEBUI-P1-20261013（2026-10-13 查阅）；ENV-WEBUI-P1-20261013（本地/隔离验证）。使用 OpenAI Codex 协助实现、测试和文档整理，许可与范围见 [provenance.json](provenance.json)。
