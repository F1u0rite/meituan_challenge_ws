# 实机操作台

用户澄清“我要的是实机操作台”（USER-HW-20261011-07，2026-10-11 用户会话）。`mtc_webui/` 默认连接 **ROS 2 Jazzy**：真实 ExecuteTask Action、TaskState/RobotState/ToolState、关节、RGB-D 和 ROS 日志。缺少环境、配置或消息时显示离线，不回退到演示数据。早期 Mock 保留为 `--mock`，原始证据不改写（USER-HW-20261011-06、DEMO-WEBUI-MOCK-20261011）。

## 接口边界与当前联调范围

USER-HW-20261013-05（2026-10-13 用户会话）明确：面向实机的操作台目标保留，**现阶段联合验证仅限 C++ Mock / GenericSystem Fake Hardware，不涉及真实机械臂控制**。

- 保留现有 HTTP URL、方法、任务请求字段及核心响应字段。状态扩展以 `GET /api/snapshot` 的可选字段和只读查询提供，现有调用者不需改任务提交格式。
- 网页只发送顶层 `ExecuteTask` Goal。已有的定向任务取消与软件停止请求继续使用原接口；不增加 PickPlace、FJT、工具 IO、关节点动或规划调用按钮。
- 模块通过 ROS 2 Topic 或统一只读状态接口提供事实；Web 负责订阅、适配和显示。PickPlace/FJT 仅观察 Feedback/Status/GetResult，不创建其 Goal/Cancel 客户端。
- 尚未发布状态的模块先保留数据结构与显示区域，值为 null/未知/尚未接入。下面的可选 JSON 是已实现消费者的预留格式，**正式接口由对应模块负责人提供并确认**；接入差异在适配层处理，不要求上游为页面生成正常状态。

| 状态区域 | 已有读取能力 | 待交接内容 |
| --- | --- | --- |
| 任务与安全 | ExecuteTask、TaskState、独立 RobotState | FSM / 执行端负责人提供真实状态与停止证据；ACK、最终结果、StopConfirmed 分开 |
| 运动 | JointState、控制器状态、只读 FJT | 执行端负责人提供原语、command_id/trajectory_id、进度及预计时长，见 [P0](telemetry-contract.md) |
| 视觉 | BatteryDetectionArray、RGB/Depth | 视觉负责人提供正式检测、图像与后续像素框/帧同步约定，见 [P1](p1-contract.md) |
| 规划与工具 | Planner / 工具验证消费者、原生 ToolState | 各模块负责人提供规划检查/失败和独立试提/落座证据，见 [P1](p1-contract.md) |
| 标定与追踪 | TF 查询、标定和 Goal 关联消费者 | 标定负责人确认坐标链/有效性；FSM / 执行端提供显式跨层 UUID，见 [P2](p2-contract.md) |

C++ Mock 任务联调使用下文 `--cpp-mock` 入口；GenericSystem Fake Hardware 的状态观察使用隔离 ROS 域及 `--readonly`。完整 Task→FJT Fake 闭环仍待模块接线，不能通过伪造 `RobotState` / `ToolState` 或放宽实机模式门禁补齐。历史 `--mock` 定时演示不能作为 C++ Mock / Fake 联合验证结果。

## P2 更新（2026-10-13）

按 USER-HW-20261013-04 增加八模块系统状态、独立 TF/标定判断与日志追踪。模块分正常/异常/未知；缺失、过期或尚未接入保持未知。原生 PickPlace 与 FJT 只读接收，不下发它们的 Goal/Cancel。任务记录支持 task_id、Goal UUID、模块、状态转换和错误筛选；SQLite 索引与 JSONL 保留完整已记录事件，导出不受页面窗口限制。

[接入约定](p2-contract.md)列出 TF 坐标链、可选标定/UUID 关联格式、冲突处理与记录范围。已有 C++ Task 的明确父子 UUID 日志可以接入；跨层 FJT 关联和标定有效性还需真实发布者提供，缺失时显示未关联/未知（TEAMMATE-WEBUI-P2-20261013）。验证与边界见 [报告](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/web-ui.md)（ENV-WEBUI-P2-20261013），不代表实机验收。

## P1 更新（2026-10-13）

USER-HW-20261013-02 要求的视觉/规划/工具观察区已加入：默认订阅原生 BatteryDetectionArray，列表显示电池颜色/置信度/主体与提环位姿/有效标志，按新鲜任务反馈 ID 高亮目标。RGB 与深度预览显示返回 JPEG 自身的时间戳和坐标系；新鲜检测数组不会刷新旧位姿。

Planner 面板逐项显示状态、原语、ID、点数、预计时长、关节/碰撞/起点检查、原始错误；工具类型、估计状态、当前状态 verified、独立挂载、证据等级、试提、落座分别显示。可选 Planner/验证消息须配置实际话题和固定发布节点，字段缺失显示未提供。当前上游未提供这两个实时发布器，不能仅启动页面就认为已完成模块联调（TEAMMATE-WEBUI-P1-20261013）。

[P1 接入约定](p1-contract.md)含完整字段、时间/身份边界及给模块负责人的发布格式。上游消息无二维框，本轮不叠画虚构框，不新增 RViz/3D、工具 IO 或规划调用。ENV-WEBUI-P1-20261013 记录 62 Python / 20 Node / 6 P1 ROS 和 5 P0 ROS 检查通过；安装包同一 62/6 项重复检查通过，不累计成新用例。隔离验证与实物验收分别记录，见 [验证报告](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/web-ui.md)。

## P0 更新（2026-10-13）

按用户明确的三项 P0（USER-HW-20261013-06），页面现在提供任务身份/口令/当前电池/完成数、Task 与 PickPlace 双层 FSM 高亮、独立安全与停止区，以及六关节实际位置/速度/同帧目标对比。Task 完成进度与当前处理序号分别显示；失败保留原生 ROS 状态、错误码和原因。

FJT 通过只读 Feedback/Status/GetResult 接入，不增加执行 Goal 客户端。运动原语、command_id/trajectory_id、预计时长和百分比由**可选执行端遥测**提供；当前上游 PR #13 尚无对应发布器，未接入时显示未提供，不能用 FSM 推测。字段与发布约束见 [遥测契约](telemetry-contract.md)。

安全区把 Cancel ACK、最终结果、Stop Request 回执和独立 Stop Confirmed 分开展示。取消/停止请求之前的 true 不作为本次请求的停稳证据；需要请求后的新鲜独立 RobotState，且无运动、非 simulation_mode。PR #8 的 C1 `ABORTED/520` 原样保留，没有修复或关闭。

新增验证 ENV-WEBUI-P0-20261013：54 项 Python、12 项前端语义、5 项隔离 Jazzy 接口检查通过；浏览器检查成功序列、注入 520、暂停与手机布局。测试消息和 execution telemetry 均由隔离夹具生成，**不代表 Phase 2C 或实机闭环**。历史 33/9 项验证保留在原目录，本轮不重新计为新结果。

## 包内启动与配置

从本工作空间根目录构建并 source Jazzy 与安装 overlay；统一 C++ Mock 启动见 [包入口](../README.md)。本阶段联合验证只在隔离 C++ Mock / GenericSystem Fake Hardware 环境执行。

```bash
# 已由 mock_stack 启动同一隔离域的 C++ Task / PickPlace 后，可单独启动网页。
ros2 run mtc_cpp_mock_bringup web_console --cpp-mock \
  --cpp-namespace /mtc_cpp_mock --ros-domain-id <隔离域>

# GenericSystem Fake 状态观察：填写本次隔离环境的实际 domain / 节点 / 话题。
cp src/mtc_cpp_mock_bringup/mtc_webui/connection.example.json /tmp/mtc-webui-connection.json
ros2 run mtc_cpp_mock_bringup web_console --readonly --config /tmp/mtc-webui-connection.json

# 已 source 的环境也可直接运行模块；无 domain 时显示离线，不生成设备数据。
python3 -m mtc_webui.server --readonly
```

完整 Task→FJT Fake 联合接线仍待模块负责人交付，缺状态保留未知。NUC 的实际工作空间、SSH 与实机 domain 尚未确认；SDK 文档中的 domain 91 是 Fake 示例，旧 demo 的 67 是历史仿真域，均不能作为现场实机参数。只启动 Web 不会启动 FSM、驱动、MoveIt 或机械臂使能。

相机使用系统 `python3-opencv`、`python3-numpy`，Ubuntu/Jazzy 环境可用 `sudo apt install python3-opencv python3-numpy` 安装。其余 ROS 依赖为 `rclpy`、`mtc_interfaces`、`sensor_msgs`、`rcl_interfaces`、`unique_identifier_msgs`；P0 轨迹观察另需 `control_msgs`、`action_msgs`，由已构建的 Jazzy 工作空间提供。缺 OpenCV 时任务与状态仍可用，相机错误写入日志；不把依赖缺失当相机正常。前端是原生 HTML/CSS/JavaScript，无 npm、CDN 或外部字体。客户端 API 参考 ROS2-RCLPY-20261011（2026-10-11 官方源码查阅），具体版本以隔离环境实测记录为准。

服务仅监听本机。开发电脑沿用 SSH 隧道方式（USER-HW-20260923-07）：

```bash
ssh -N -L 8767:127.0.0.1:8767 <用户>@<NUC>
# 浏览器打开 http://127.0.0.1:8767/
# 若本地用 8877 转发，NUC 启动参数增加 --tunnel-port 8877。
```

Host/Origin 检查保留；不提供 shell/PTY 路由。端口 8767 与历史监控台 8765 分开。单个 HTTP 服务应对应一个机器人会话；现场仍须避免其他任务客户端并行控制。新服务会话不会恢复、重发或自动取消任何旧 Goal。

## 连接配置与操作语义

默认端点来自固定 `mtc_interfaces` 与节点源码（TEAMMATE-FSM-20261011、TEAMMATE-SDK-20261011，`1d940f7c5f12561fed6641457c0b4b630a282b05`，2026-10-11 核对）：

| 配置 | 默认 | 用途 |
| --- | --- | --- |
| task_action | `/mtc/task/execute` | ExecuteTask Goal / Feedback / Cancel / Result |
| task_topic / task_publisher | `/mtc/task/state` / `/task_executor` | 当前 FSM 状态与外部任务观察 |
| robot_topic / robot_publisher | `/mtc/robot/state` / `/mtc_aubo_bridge` | 通信、就绪、运动、联锁、独立停稳及 simulation_mode |
| tool_topic / tool_publisher | `/mtc/tool/state` / 空 | 工具估计、evidence_level 与 verified 分别显示 |
| joint_topic / joint_publisher | `/joint_states` / 空 | 实际关节位置和速度，rad / rad/s 转 ° / °/s |
| pick_action | `/mtc/manipulation/pick_place` | 发现 PickPlace 服务在线状态；子状态来自 Task Feedback / TaskState |
| controller_topic / controller_publisher | 空 / 空 | 唯一 Jazzy 控制器状态话题；同帧目标与实际对比 |
| fjt_action / fjt_publisher | 空 / 空 | 唯一 FJT 的只读反馈、状态和结果；不是发轨迹入口 |
| motion_topic / motion_publisher | 空 / 空 | 可选执行遥测；配置话题时必须显式填写权威节点名 |
| stop_service | `/mtc/bridge/request_stop` | RequestStop 软件停止，服务回执不等于停稳 |
| color_topic / depth_topic | 空 | 自动发现唯一匹配的相机原始话题；多相机时显式填写 |
| domain_id / read_only / freshness_ms | null / false / 3000 | 未配置 domain 不创建 ROS 节点；只读禁止写操作；时效是工程门槛 |

桥接要求每个状态话题只有一个发布端，再核对完整节点名；图中的 DDS GID 用于识别发布端更换。Jazzy 7.1.12 的 MessageInfo 没有逐消息 publisher_gid，不能在多发布端话题上仅凭节点名筛选消息。因此同名 robot 话题同时由 motion executor 和 aubo bridge 发布时，整路数据无效、禁止提交；须把 aubo bridge remap 到独立状态话题并配置 robot_topic。默认期望节点 `/mtc_aubo_bridge`，不混合布尔值。空 publisher 字段同样要求该话题恰有一个发布端；每条消息与提交前均重查来源。界面的接口页展示发现的发布源、选择结果、Action/服务就绪与图像话题。Action/停止服务有多个服务端时拒绝写入。

接收单调时钟和消息 Header 时间戳同时决定时效；旧/重复时间戳不刷新数据，零时间戳或显著未来时间戳不能用于就绪。3 秒默认阈值和 500 ms 时钟未来容差是本项目工程限制，不是比赛规则或形式化安全证明。ROS 时钟/时间同步异常应现场排查。

提交前要求：可用且唯一的 ExecuteTask 服务、有效 Task/Robot/Tool 样本、FSM 为 WAIT_TASK、机器人 `simulation_mode=false`、通信/就绪/联锁/独立停稳均正常、没有运动、工具已分离，以及没有结果未知的本会话 Goal。发送前在 ROS executor 再核验一次。任务口令由操作员填写：基础一色到 T0，序列三种不同颜色到 P1–P3；稳定观察固定 3 秒（RULES p3–4），超时范围 1–600 秒是界面限制。

HTTP 202 只表示命令入队，**不是 ROS 受理**。真实 Action 回调分别更新 SUBMITTING、ACCEPTED、EXECUTING、CANCELING、SUCCEEDED/ABORTED/CANCELED/REJECTED。完成数来自 TaskState 或 Result，不从时间或 current_index 推算。Goal UUID 与 task_id 共同关联记录；旧 Goal 反馈不能覆盖当前结果。受理超过 10 秒、取消回执异常或终态不一致标为 UNKNOWN，禁止再次提交，保留后续真实 Result 接收；不自动重试。

取消仅针对本会话持有的活动 Goal，精确检查 ROS 返回的取消 Goal UUID。浏览器刷新不丢失服务端句柄；服务重启后只能观察外部任务，不臆造句柄或“取消全部”。任务结束后等待新的 FSM 与独立停稳状态再开放下一任务。RequestStop 单独显示送达回执；**软件停止不是实体急停**，操作台不会把请求送达改写成 `stop_confirmed=true`。

相机优先自动发现唯一 `/color/image_raw`（或 compressed）、`/aligned_depth_to_color/image_raw`（否则 `/depth/image_rect_raw`）后缀。订阅端最多处理约 5 fps，页面随 750 ms 快照更新观察；保留原始 stamp、编码、尺寸与来源，过期后遮掉旧画面。RGB 支持 rgb8/bgr8/rgba8/bgra8/mono8 或 CompressedImage；深度仅支持原始 16UC1（按 ROS 常见约定 mm）/32FC1（m），固定 0–2 m 伪彩，非法深度显示黑色，尊重行步长与端序。实际相机编码、单位、同步与标定仍须现场核对；未提供检测叠图或任务录像。

## C++ Mock 兼容入口

`python3 -m mtc_webui.server --cpp-mock --cpp-namespace /mtc_cpp_mock --ros-domain-id <隔离域>` 可连接 PR #8 已启动的 C++ 软件 Mock；需要 source 含 `mtc_cpp_mock_bringup` 的安装环境。该模式订阅 C++ `integration_state`，只发送 Task Goal / 定向 Cancel，绝不构造 RobotState 或硬件停稳证据。与真实 ROS 模式和历史 `--mock` 演示模式分别标识。`--cpp-mock` 不接受实机 `--config`，避免无声忽略配置。

本轮可应用到 PR #8 固定 head 的包内补丁见 [source.patch](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/experiments/webui-p0-20261013/source.patch)。补丁仅改 Web 模块及其依赖/安装资料，不变更 FSM、Action、Motion 内核或 IDL；未自动合入团队 main。

## HTTP 契约 v1

| 路径 | 语义 |
| --- | --- |
| `GET /api/snapshot` | mode=ros/cpp_mock/mock、当前 Goal、独立 task_state/external_task、robot/tool/joints/camera、controller/motion/fjt_status/fjt_feedback/fjt_result/stop_evidence、connection/config、operations/reasons、history/events |
| `POST /api/tasks` | 精确字段 task_id/mode/ordered_colors/timeout_ms/placement_stability_sec；实机拒绝 scenario；202 返回 Goal UUID 与 SUBMITTING |
| `POST /api/tasks/{task_id}/cancel` | `{"goal_id":"对应 UUID"}`；202 仅排队，快照中的 cancel_ack 是实际 ROS 回执 |
| `POST /api/stop` | `{}`；202 仅排队，stop_request 单独报告回执 |
| `GET /api/camera/color`、`/api/camera/depth` | 新鲜 JPEG；缺失或过期返回 404，不展示旧帧 |
| `GET /api/tasks/{task_id}`、`/api/export` | 单任务 / 会话 JSON 下载 |
| `GET /api/events` | P2 新增的只读日志查询；按任务/Goal/模块/转换/错误筛选，不改变任务写接口 |
| `POST /api/demo/*` | 只存在于显式 --mock 模式；实机返回 404 |

快照保持 `schema_version=1`。ROS 会话导出保留原有 `snapshot`、`tasks`、`events`、`note` 及每个任务的 `events` 字段，默认附件名仍为 `webui-session.json`；P2 的 `schema_version=2` 是扩展导出载荷标识（旧 ROS 导出无此字段），新增 trace/coverage 等诊断信息，HTTP 路由与请求格式不变。事件由窗口预览扩展为全部已记录事件，按先后顺序排列；可选筛选同时作用于顶层 `events` 和 `tasks[].events`。兼容性回归见 ENV-WEBUI-API-20261013。

样本包装字段 source/received_at/source_stamp_ns/age_ms/fresh 属于 Web 桥接层，不是上游 ROS IDL。浏览器另检查自身最后收包龄期；暂停、断线期间禁用任务写操作，恢复只获取快照。相机是观察图，不是机器人位置或碰撞证明。

每次启动新建 `outputs/webui-<时间>-<随机后缀>/`，或指定 `--output-dir <新目录>`；拒绝覆盖。目录含 connection.json、完整 events.jsonl / events.sqlite3、按 task_id 保存的 Goal/Result/事件，保留失败和取消。页面仅展示本次会话；会话事件内存窗口 500 条，单任务窗口 300 条；全量事件查询与导出读取持久索引，磁盘任务 JSON 保留有界预览。ROS /rosout 记录 mtc 与已知业务节点；不是任意系统日志抓取器。录像字段仍为空。

## 验证与边界

```bash
PYTHONPATH=src/mtc_cpp_mock_bringup python3 -m unittest discover -s src/mtc_cpp_mock_bringup/mtc_webui -p 'test_*.py' -v
node --check src/mtc_cpp_mock_bringup/mtc_webui/static/app.js
node --test src/mtc_cpp_mock_bringup/mtc_webui/test_p0.cjs src/mtc_cpp_mock_bringup/mtc_webui/test_p1.cjs src/mtc_cpp_mock_bringup/mtc_webui/test_p2.cjs
# 独立测试容器中，source Jazzy 与接口 overlay 后：
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m mtc_webui.p0_integration_check --isolated-fixture
# 历史相机/任务适配检查单独运行：mtc_webui.ros_integration_check --isolated-fixture
# 严禁把测试发布器在实机 domain 中启动；测试域固定 94，只用于隔离容器。
```

测试发布器模拟消息刺激，经实际 rclpy/DDS/Action 链路核验 Web 适配器；其 `simulation_mode=false` 是门禁测试输入，不是连接实体设备的证据。验证结果见 [实现与验收](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/web-ui.md)。旧 Mock 仅用 `python3 -m mtc_webui.server --mock` 启动；冻结证据仍为 DEMO-WEBUI-MOCK-20261011。

验证记录为 ENV-WEBUI-ROS-20261011：33 项离线/HTTP 检查与 9 项隔离 Jazzy ROS 检查通过，浏览器已通过真实 DDS fixture 交互。发生重复节点名后，删除重复节点可能使存活节点在图中成为 UNKNOWN；应重建可识别的唯一业务来源，操作台不会放宽身份检查。

当前 NUC SSH 地址、现场 domain 与实际运行图尚无核验记录。固定旧 Python 基线的 B03/B05/B06 保留；新 C++ Mock 的 C1 与 Phase 2C 完整接线缺口分别见 [当前状态](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/current-status.md)和 [C++ Mock 报告](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/95bfa6002db3dd5330461efcfcd0cc8112323246/docs/cpp-mock-webui-acceptance.md)。操作台客户端不会修复或接管这些 FSM 内核问题。实机运动、标定、相机真值、最终末端、完整任务录像均不能由此界面或隔离测试宣称完成。

使用 OpenAI Codex 协助实现前端、ROS 客户端、测试与文档；未复制上游 FSM 实现。字段来源、依赖、许可证及修改范围见 [provenance.json](provenance.json)，按 FAQ p6/Q17 披露。
