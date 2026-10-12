# V1（被动舌规）预对准超时：原因、规划侧能做的、需要 FSM 决定的

> 常建烁，2026-10-12。全部结论来自 Mock / Fake 仿真与离线运动学计算，**未在真机运行**。
> 提环位置、提环朝向、钩子 TCP 均为 PLACEHOLDER，结论中的具体秒数会随实测几何变化。

## 1. 结论

1. **从 home 出发，单个 `side_insert`（0.02 m/s、0.1 rad/s）在物理上就放不进 MOVE_PRE_ALIGN 看门狗。**
   当前占位几何下，home（工具朝下）到预对准位姿要转 **180°**；0.1 rad/s 下最少 **31.4 s**，比 30 s 看门狗本身还长。
   这与规划算法无关，任何规划器都做不到。
2. 规划侧已做到头：末端限速时间最优参数化把 V1 从 39.59 s 降到 **35.04 s**（上限不变），仍超时。
3. **可行方案：把 V1 预对准拆成"快速接近 + 低速预对准"两段**。这正是 Python `mtc_tool` 原来的设计
   （`side_insert_pre_align` 0.05 m/s / 0.2 rad/s + `side_insert`）；C++ 迁移时合并成了一段。
   提案分支实测：V1 单块电池 8 段全部规划成功，MOVE_PRE_ALIGN 合计 **19.68 s < 24 s**，3/3 次结果相同。
   **需要孙嘉明批准**（改动在 `mtc_manipulation_cpp`），本人未合并任何 FSM 改动。

## 2. 为什么慢：先算物理下限

`scripts/analysis/v1_reorientation_feasibility.py`（纯运动学，读展开后的 URDF）：

| 目标姿态 | 需要转动 | 0.1 rad/s 下最少 | 从 home 走"末端直线 + 匀速转动" |
|---|---|---|---|
| 当前占位（提环 quat (0,0.707,0,0.707)，TCP 单位变换） | 180.0° | 31.4 s | 走到 52% 被 foreArm 限位（±2.41）挡住 |
| 钩子绕自身轴转 180° 安装 | 93.8° | 16.4 s | 走到 51% 被 foreArm 限位挡住 |

再换 64 种几何（提环 x∈{0.40,0.50}、y∈{-0.15,0.05}、z∈{0.13,0.25}，插入方向 ±X/±Y，钩口朝上，滚转 0/180°）：
**55 种从 home 根本走不通**（肘或腕关节限位），能走通的 9 种下限也有 16.4–21.8 s（不含加减速），
离 24 s 几乎没有余量。原始输出见 [`evidence/20261012_022117/v1_reorientation_feasibility_grid.txt`](evidence/20261012_022117/v1_reorientation_feasibility_grid.txt)。

结论：**问题出在"让慢速原语去完成大角度转向"这个动作设计**，不在规划算法。

## 3. 规划侧已做的（`feature/planning-v1-feasibility`，只改 `mtc_moveit_planner`）

| 改动 | 效果 |
|---|---|
| 末端限速时间最优参数化（`ee_limited_retiming`）：关节空间直线路径上逐段同时满足关节速度/加速度 × scaling 与末端线/角速度上限，替代"按峰值均匀拉伸" | V1 39.59 s → 35.04 s；上限不变，最终轨迹仍密采样复核（峰值角速度 0.098 ≤ 0.1） |
| 就绪位求解 `solve_ik_max_margin()` + 序列工具 `start_mode: ready` | 能直接回答"如果从工具已对准的位置出发会怎样" |
| `side_approach` / `top_approach` 规划规则（FSM 未发出时不起作用） | 为拆分方案准备 |
| 同一 PickPlace 状态内多个原语**累计**对照看门狗 80% | 拆分后两段之和也受 24 s 约束，不会被分段"绕过" |

从就绪位出发（`config/sequence_v1_ready_override.yaml`，就绪位 = 预对准位后方 2 cm）：
V1 后续 7 段全部成功，`side_insert` 只需 1.95 s；home → 就绪位的参考准备动作按 0.05 m/s / 0.2 rad/s 需 17.74 s。
就绪位关节值确定（3/3 次相同）：shoulder 0.391, upperArm −0.128, foreArm 2.303, wrist1 −0.711, wrist2 −1.180, wrist3 0.000（rad）。
证据：[`evidence/20261012_022117/v1_ready_mode/`](evidence/20261012_022117/v1_ready_mode/)。

## 4. FSM 提案（`proposal/v1-fast-approach`，需孙嘉明批准）

| 改动 | 说明 |
|---|---|
| `ToolStrategy::FREE_APPROACH_LINEAR_SPEED = 0.05`、`FREE_APPROACH_ANGULAR_SPEED = 0.2` | 数值沿用 Python `PassiveHookParams.free_*`（未标定占位值），不改任何已有上限 |
| V1 `make_acquire_plan` → `[side_approach, side_insert]` | `side_approach` 停在预对准位前 2 cm；`side_insert` 仍是 0.02 m/s / 0.1 rad/s |
| `h_move_pre_align` 顺序执行全部接近原语 | 已有结果的原语直接取缓存、不再构造命令，避免替换在途命令或以新 command_id 重发；单原语（V2）行为完全不变 |
| V2 | 不变（从 home 16.46 s 可行） |

Fake MoveIt（V1，从 home，占位几何）：

| 段 | 时长 | 末端峰值 / 上限 |
|---|---|---|
| side_approach | 17.74 s | 0.196 / 0.2 rad/s，0.033 / 0.05 m/s |
| side_insert | 1.94 s | 0.0196 / 0.02 m/s |
| **MOVE_PRE_ALIGN 合计** | **19.68 s** | 24 s（30 s × 80%） |
| test_lift → retreat | 全部成功 | — |

证据：[`evidence/20261012_023217/`](evidence/20261012_023217/)（提案分支，固定位姿 / 场景隔离 / 碰撞反证 / V1 / V2 全部 PASS）；
7 个 C++ 包 534 项测试 0 失败（含新增：V1 两段顺序执行且在途不重发、第二段失败停在 ENGAGE 之前）。

## 5. 仍需注意

- **关节裕度偏小**：占位提环离底座近（0.40 m、高 0.13 m），预对准时 foreArm 离控制器限位只有 0.107 rad（约 6°）。
  真实提环位置确定后必须重跑；若裕度不足，优先调整电池摆放或插入方向，而不是放宽限位。
- **钩子绕自身轴的朝向（TCP 滚转）未标定**：占位值使 home 到目标要转 180°。若钩子按"从朝下到水平只需俯仰 90°"安装，
  所需转动降到约 94°；但在本占位位置下该朝向的各个解都会碰到桌面。需韩明赫给出 TCP 实测后重算。
- 多块电池时，第二块的接近从上一工位出发，距离可能更远；拆分后的快速段同样适用。
- `side_approach` 的速度是否可接受（离提环 5 cm 外以 0.05 m/s / 0.2 rad/s 运动）是安全参数决策，由孙嘉明确定。

## 6. 复现

```bash
python3 src/mtc_moveit_planner/scripts/analysis/v1_reorientation_feasibility.py <aubo_S3_fake_expanded.urdf> --grid
MTC_RMW=rmw_cyclonedds_cpp bash src/mtc_moveit_planner/scripts/run_planning_evidence.sh
ros2 launch mtc_moveit_planner plan_tool.launch.py program:=plan_battery_sequence \
  extra_params_file:=$(ros2 pkg prefix mtc_moveit_planner)/share/mtc_moveit_planner/config/sequence_v1_ready_override.yaml
```
