# meituan_challenge_ws

**美团第四届低空经济与具身智能挑战赛｜AUBO S3 机械臂 ROS 2 工程**

> **项目阶段：架构设计 / 新工作空间迁移准备（尚未完成实机抓放联调）。** 该仓库计划整合旧工程的运动规划与仿真成果，在独立 ROS 2 工作空间中实现比赛任务状态机、机械臂运动执行适配层和两代提环抓放末端策略。
>
> **安全默认值：Mock / 离线模式。** 克隆、构建或运行测试不意味着被授权连接或驱动真实 AUBO S3。

## 1. 项目目标

本项目面向初赛的固定工位多色电池识别、抓取和序列摆放任务：

- **基础任务**：识别指定颜色电池，利用顶部提环抓取，放入 T0。
- **序列任务**：按照当轮颜色口令，将三块电池依次放入 P1、P2、P3。
- **末端执行器 V1**：纯机械舌规，侧向穿环、挂载、放下卸载、退钩。
- **末端执行器 V2（规划中）**：从上方插入提环并机械锁止，放置后电磁解锁，再退出。
- **软件目标**：复用同一套任务与抓放状态机，使 V1/V2、Mock/仿真/实机通过配置及后端适配切换。

当前已知技术基线：**Ubuntu 24.04 + ROS 2 Jazzy**；机械臂为 **AUBO S3**。交接记录中的实机控制使用 `pyaubo-sdk 0.24.1`（Python 3.10）与 RPC/RTDE；**完整 ROS 2 规划轨迹到 S3 实机的执行链路仍需核实和验证**。

## 2. 仓库与工作空间的关系

**仓库名、克隆后的目录名和 ROS 2 Workspace 名统一为 `meituan_challenge_ws`**，固定工作目录为 `/home/aaet/meituan_challenge_ws`：

| 位置 | 用途 | 写入原则 |
| --- | --- | --- |
| GitHub `meituan_challenge_ws` | 设计文档、新工作空间代码与版本历史 | 仅提交已审查、无凭据的内容 |
| `/home/aaet/meituan_challenge_ws` | **唯一新 ROS 2 workspace；也是克隆后的 Git 仓库根目录** | 新开发、迁移、编译、测试均在这里 |
| `/home/chang/meituan_challenge` | 旧 ROS 2/MoveIt/Gazebo 比赛工程 | **只读迁移来源，禁止原地修改** |
| `/home/chang/aubo_s3_nuc_smoke` | 旧 SDK/NUC 冒烟工程（如本机存在） | 只读参考 |
| `/home/aaet/aubo_s3_nuc_smoke/hardware` | 交接记录中的 NUC 实机代码路径（如本机存在） | 禁止未经授权的远程连接或实机操作 |

旧路径来自历史交接记录，**不是当前主机上文件必然存在的证明**。实际迁移前必须逐一确认。

### 2.1 工作空间路径变更记录（2026-10-09，用户指示）

| 项 | 内容 |
|---|---|
| 原定路径 | `/home/meituan_challenge_ws`（`docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md` 与本文档原稿的要求） |
| 现路径 | **`/home/aaet/meituan_challenge_ws`** |
| 变更原因 | 用户反馈 `/home` 根目录下的入口不便操作，明确指示迁回 `~/meituan_challenge_ws` |
| 变更方式 | 同文件系统内整体移动（保留完整 `.git` 历史与全部提交），非重新克隆 |
| 偏离登记 | 该路径**偏离** V3 提示词 §1.1 的“禁止改建到 `~/meituan_challenge_ws`”，属**用户显式豁免**；V3 原文未被修改，仍以 `docs/prompts/` 下原文为准 |
| 原始文档完整性 | `docs/prompts/…_v3.md`（SHA256 `6f058481…`）、`docs/reference/AUBO_S3.md`（`a5cf0980…`）、`docs/architecture/…_v0.1.md`（`802c1986…`）**内容未改动** |

**不受影响的事项：** 旧工程只读边界、实机禁用边界、Git 远端与提交历史均未因路径变更而改变。`git remote -v` 仍指向 `https://github.com/F1u0rite/meituan_challenge_ws.git`。

## 3. 从 GitHub 开始

### 3.1 准备仓库

建议仓库最初只包含以下文件，**不要直接复制旧工程的 `.git/`、`build/`、`install/`、`log/`、本地运行配置和测试日志**：

```text
meituan_challenge_ws/
├── README.md
├── .gitignore
└── docs/
    ├── reference/
    │   └── AUBO_S3.md                                  # 脱敏后的交接文档
    ├── architecture/
    │   └── AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md    # 设计提案
    └── prompts/
        └── DSH_AUBO_S3_独立Workspace重构提示词_v3.md  # GitHub-first 执行提示词
```

**`.gitignore`：**创建 GitHub 仓库时可以直接选择 GitHub 自带的 **ROS** 模板。不过该模板以传统 ROS/catkin 为主，包含 `build/` 和 `logs/`，但未覆盖 ROS 2/colcon 的 `install/`、`log/` 及本机凭据。建库后至少追加：

```gitignore
# ROS 2 / colcon
install/
log/

# Local Python environments and private configuration
__pycache__/
.pytest_cache/
.venv/
.env
.env.*
!.env.example
*.local.*
secrets/
credentials/
```

模板还会忽略名为 `lib/`、`bin/` 的目录与 `*.pcd` 文件。迁移前要用 `git check-ignore -v <路径>` 检查有没有误忽略应提交的源码或测试资产。

**必须先对 `AUBO_S3.md` 脱敏再上传。** 原交接文档包含设备账号、密码和内部网络信息；即使仓库设置为 Private，也建议删除明文凭据，只保留变量名、用途及配置说明。真实凭据仅保存在受控的本地秘密配置中，并列入 `.gitignore`。如果曾意外提交凭据，不要只删除文件后继续使用旧密码：应更换受影响的凭据并处理 Git 历史暴露风险。

### 3.2 克隆到固定路径

将 `<OWNER>` 换成实际 GitHub 用户名或组织名，然后**在目标机器上**执行：

```bash
# 先检查目录，不要覆盖已有文件
ls -ld /home /home/aaet/meituan_challenge_ws 2>&1 || true

# 仅在 /home/aaet/meituan_challenge_ws 尚不存在或为空且有权限时克隆
# 仓库可选 HTTPS 或 SSH；下例为 HTTPS
git clone https://github.com/<OWNER>/meituan_challenge_ws.git /home/aaet/meituan_challenge_ws

cd /home/aaet/meituan_challenge_ws
git status --short
git remote -v
```

注意：普通账户不一定有权限在 `/home` 根目录创建目录。若 `git clone` 因权限失败，**请管理员只对 `/home/aaet/meituan_challenge_ws` 做最小必要的建目录/授权**；不要对 `/home` 整体 `chmod`，也不要**在未经用户明确指示时**把项目改建到其他路径（本仓库当前的 `~/meituan_challenge_ws` 路径系用户指示迁移，见 §2.1）。如果目标目录已存在且非空，先检查里面是否已是预期仓库，不能覆盖。

**克隆后不要再执行 `git init`。** `/home/aaet/meituan_challenge_ws/.git` 就是新工程独立的版本历史。

## 4. 文档阅读顺序

1. [`docs/reference/AUBO_S3.md`](docs/reference/AUBO_S3.md)：理解旧 S3、NUC、SDK/RTDE 与原 ROS 2 项目。**这是历史交接记录，不能直接当作当前设备状态。**
2. [`docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md`](docs/architecture/AUBO_S3_ROS2_状态机与接口技术设计_v0.1.md)：理解双层 FSM、Action/Topic/Service、TF/TCP、V1/V2 末端策略和安全联锁。**V0.1 是设计草案，接口需以代码和编译结果核验。**
3. [`docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md`](docs/prompts/DSH_AUBO_S3_独立Workspace重构提示词_v3.md)：将工作空间迁移、实现与离线验收交给 DSH 时遵循的执行计划。

遇到冲突时：**安全与路径隔离约束优先；源代码代表现存事实；架构文档代表目标设计；差异必须记录，不能默默覆盖。**

## 5. 目标工程结构（迁移后逐步形成）

以下是**计划中的结构**，不是宣称首次克隆就已存在这些功能包：

```text
/home/aaet/meituan_challenge_ws/
├── src/
│   ├── mtc_interfaces/          # ROSIDL：Action、Message、Service
│   ├── mtc_description/         # S3、电池、V1/V2 工具模型
│   ├── mtc_motion_planning/     # 迁移并保留已有规划器
│   ├── mtc_simulation/          # 迁移并改造 Gazebo 场景
│   ├── mtc_task/                # 整轮比赛任务 FSM
│   ├── mtc_manipulation/        # 单块电池抓放 FSM
│   ├── mtc_motion_execution/    # Motion Backend 抽象与 Mock
│   ├── mtc_tool/                # 舌规 / 电磁锁止策略
│   ├── mtc_safety/              # 软件联锁与故障监控
│   ├── mtc_aubo_bridge/         # ROS 2 ↔ AUBO SDK 适配
│   └── mtc_bringup/             # 启动与参数配置
├── config/
├── docs/
│   ├── reference/
│   ├── architecture/
│   ├── prompts/
│   └── migration/               # 源码审计、迁移映射、接口变更、验收报告
├── scripts/
├── tests/
├── build/                        # colcon 生成，不提交 Git
├── install/                      # colcon 生成，不提交 Git
└── log/                          # colcon 生成，不提交 Git
```

### 关键软件链路

```text
ExecuteTask Action  →  Task FSM
                           ↓
PickPlace Action    →  Manipulation FSM
                           ├── Perception / TF
                           ├── PlanMotion（保留原规划契约）
                           ├── Tool Strategy（V1 / V2）
                           └── Motion Execution
                                  ├── Mock / Gazebo
                                  └── AUBO Bridge（实机默认禁用）
```

必须区分 **轨迹规划成功、指令被控制柜接受、运动到位、抓取/放置经验证成功**。不得将 MoveIt 输出的多点轨迹直接拆成高频 `moveJoint` 调用，也不得以夹爪开闭接口代替提环机械接合过程。

## 6. DSH 启动要求

**先由人员克隆仓库，再让 DSH 在仓库根目录执行。** DSH 开始时须：

1. 确认 `realpath /home/aaet/meituan_challenge_ws`、Git remote、现有文件和写权限；将已经克隆的 `.git` 视为仓库初始化完成，**禁止重新 `git init`、重新克隆或清空目录**。
2. 完整阅读上述三份文档，核验旧工程路径与实际源文件（旧工程只读）。
3. 建立 `docs/migration/SOURCE_AUDIT.md`，记录源 Git commit、脏工作树、原始文件哈希与依赖。
4. 定向复制经审计的四个旧 ROS 2 包，不复制生成产物或明文密码；添加新的 ROS 2 接口、Task/PickPlace FSM 和 V1/V2 Tool Strategy。
5. 默认以 `mock` 后端完成 `colcon` 编译、单元测试、端到端测试，并在 `docs/migration/IMPLEMENTATION_REPORT.md` 如实记录 PASS / FAIL / NOT RUN。
6. **不得自行 SSH 机械臂 NUC、连接控制柜、上电、发运动指令、触发电磁铁或更改安全配置。** 任何真实设备操作需获得单独明确授权并在现场执行安全检查。

启动提示：

```text
请先完整阅读仓库根目录 README.md，以及 docs/reference/、
docs/architecture/、docs/prompts/ 下列出的三份文档。
当前工作目录必须是 /home/aaet/meituan_challenge_ws，仓库已经通过 GitHub 克隆，
不得重新初始化 Git 或覆盖已有文件。
请按 DSH V3 的 P0–P6 逐阶段实施，先只读审计、确认迁移来源，
再选择性复制源码、实现状态机和 ROS 2 接口、完成 Mock 离线测试。
旧工程只读，真实 AUBO S3 控制始终禁用；遇到权限、冲突或关键
接口不兼容，记录证据并向我报告，不得擅自绕过。
```

## 7. 构建与测试（功能包迁入后）

在确认已安装 ROS 2 Jazzy / `colcon` 且 `src/` 中确有合法 ROS 2 包后：

```bash
cd /home/aaet/meituan_challenge_ws
source /opt/ros/jazzy/setup.bash
colcon list
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

这些命令是**预期验收入口，并非已在新工作空间验证通过的结果**。缺少依赖时先登记，不得为了通过测试自动控制实机或修改旧目录。

## 8. 版本管理与安全约束

- 新工程提交只发生在 `/home/aaet/meituan_challenge_ws`，旧工程绝对只读；迁移时用源代码快照、哈希和迁移映射记录可追溯性。
- `.gitignore` 至少排除 `build/`、`install/`、`log/`、`.venv/`、Conda 环境、本地 secrets、`.env`、`*.local.*`、运行日志与缓存。
- 实机连接配置、设备凭据、控制柜地址不直接写进仓库的示例代码；示例只保留占位符。
- 所有 `mock`/Gazebo 测试均应明确标记，不将仿真成功写成 AUBO S3 实机抓放成功。
- 不在 GitHub 上公开团队无权重新分发的第三方 CAD、模型、资料或带隐私的比赛素材；必要时以来源和获取说明替代。

## 9. 当前待确认项

S3 在当前控制柜/SDK 下支持的轨迹执行能力、ROS 2 与 Python SDK 的进程兼容方案、V1 舌规 TCP 与入环/退钩路径、V2 电磁解锁反馈、电池和工位坐标标定、视觉接口落地、实机安全验收均须在迁移后逐项确认。

---

**维护约定**：本 README 负责解释项目入口和工程边界；详细接口字段以设计文档及后续 `docs/migration/INTERFACE_CHANGELOG.md` 为准。未经确认的设计不得标为已实现功能。
