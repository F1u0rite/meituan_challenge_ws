# 双仓库当前进度与验收边界

核对时间：2026-10-12 16:27（UTC+8）。团队 main 为 `b4185155383541e1a4161587c4049a162f74b36d`，个人既有证据归档为 `6e040096986cdf0ea409943ae209d42507da7424`。这是 GitHub 源码、PR 状态与既有报告的文档核对，本轮没有构建或运行 ROS、Fake、仿真、NUC 或实机验收。后续变动须重新核对对应 SHA。

**当前结论：离线模块及部分 Mock 集成通过；完整 Task/PickPlace → MoveIt → FJT → Hardware 抓放闭环和实机仍未验收。** 团队首页此前停留在 Phase 2A；团队后续代码与报告已推进，个人独立复验则固定在不同历史提交，不能按仓库提交时间或测试总数排出统一的验收先后。

## 已合入团队 main

| 内容 | 上游记录 | 个人独立复验边界 |
| --- | --- | --- |
| Phase 2A C++ Action、规划 PR #3 | 已合并；Task → PickPlace 的 Mock Action 链已有测试，规划端口已交付 | 各固定提交报告保留；单独规划/执行成功不等于完整 FSM 闭环 |
| Calibration / TF / TCP PR #6 | 已合并；[标定报告](https://github.com/F1u0rite/meituan_challenge_ws/blob/b4185155383541e1a4161587c4049a162f74b36d/docs/calibration/TONIGHT_STATUS.md)记录 54 项离线测试，真实外参/TCP/工位仍待测 | 个人既有标定候选复验通过；真实安装与采样未验收 |
| Phase 2B 轨迹执行 PR #7 | 已在 main `b418515` 合并；[mutex 合并门禁报告](https://github.com/F1u0rite/meituan_challenge_ws/blob/b4185155383541e1a4161587c4049a162f74b36d/docs/migration/PHASE2B_ACTION_MUTEX_BACKPORT_GATE.md)记录普通/UBSan各486项通过，另有40轮×9项重复检查 | 个人报告固定在旧版 `681bb64`：首次61项套件超时，同二进制61项补跑及独立478项UBSan通过；本轮未复验新修复 |

486 是该报告所测 Phase 2B 候选的结果，不是本轮重测合并后 main。原生 GenericSystem 严格 Status 冷启动、标准 move_group 第二 FJT 客户端及完整联合执行仍是该 Phase 2B 报告的 BLOCKED；TSan 在测试前因环境错误退出，不能标为通过。报告中的“尚未 push/merge”是报告写作时的历史状态；本表以当前 GitHub 合并状态补充，不回写原报告。

## 待合并 PR 与独立分支

| 候选及核对版本 | 状态与报告范围 | 尚未关闭的边界 |
| --- | --- | --- |
| [PR #4](https://github.com/F1u0rite/meituan_challenge_ws/pull/4) `943bf37bec8fc35b510abe56a6844b3f6280cce4` | OPEN，未合并；上游记录40项规划GTest，V2、碰撞反证与场景隔离通过 | V1从home约39.59秒，超过24秒状态预算；个人未复验该最新版 |
| [PR #5](https://github.com/F1u0rite/meituan_challenge_ws/pull/5) `dc76a0bf0a43f9110f3d24cfa38a2b3478f48dcd` | Draft，未合并；个人已有离线/合成回放部分验收 | colcon零收集、默认零等待可靠性、真实FSM消费与实际相机输入仍未关闭 |
| [PR #8](https://github.com/F1u0rite/meituan_challenge_ws/pull/8) `7da43228d602a6b23571bfc599d69529ef7bea23` | OPEN，未合并；个人已验统一C++ Mock/UI、真实HTTP→ROS→Task→PickPlace、40项Action与39项UI及六项正负场景 | **C1未通过**：正常Mock延迟取消ACK受理，父/子最终ABORTED/520；不能将ACK作为取消完成 |
| [PR #9](https://github.com/F1u0rite/meituan_challenge_ws/pull/9) `9f6860d1edbbbd5a161703fbfe88e3d71caa29f8` | OPEN，叠在PR #4；上游记录V1从就绪位七段规划通过 | 从home仍约35.04秒超预算；就绪位规划成功不等于完整FSM执行，个人未复验 |
| [PR #10](https://github.com/F1u0rite/meituan_challenge_ws/pull/10) `748943efde2ada71a71d4ba49fc86fc6fb0155cc` | Draft提案，叠在PR #9，涉及V1策略/FSM；上游记录Fake规划两段接近合计19.68秒 | 待内核负责人审查安全参数与状态语义；未合并、未获个人独立复验，不能据此关闭main的V1阻断 |
| [PR #11](https://github.com/F1u0rite/meituan_challenge_ws/pull/11) `6c1d966c422ee858c4d23cc809743237c3d20c7f` | OPEN，相机底面贴圆盘端面的侧视试装文档；上游记录54项离线检查 | 当前外参仍UNKNOWN/null；旧安装数值不可用于新试装。本项目仅登记上游选择，现场测量与用户确认链另行补充 |
| [Phase 2C独立分支](https://github.com/F1u0rite/meituan_challenge_ws/tree/4a67503492e6bf0f678c6bbca04413e6c670ae30) `4a67503492e6bf0f678c6bbca04413e6c670ae30` | 未合入main；[报告](https://github.com/F1u0rite/meituan_challenge_ws/blob/4a67503492e6bf0f678c6bbca04413e6c670ae30/docs/migration/PHASE2C_P0_EXECUTION_OWNERSHIP_REPORT.md)记录510项普通/UBSan及受信任Fake冷启动下有限Planner→FJT→GenericSystem执行 | 完整Task/PickPlace、异步Cancel/停稳桥与实机未运行；move_group正常退出仍失败，个人未复验 |

不同分支、普通/UBSan、重复执行及CTest包装不累加为一个“整套系统通过”的测试总数。PR #8代码已在团队仓库，个人仓库保存[验收报告](https://github.com/AfonsoZhang/meituan-robotics-challenge-2026/blob/6e040096986cdf0ea409943ae209d42507da7424/docs/cpp-mock-webui-acceptance.md)与953文件冻结证据，两处是同轮交付，非两次独立通过。

## 环境与终止记录

团队[驱动手册](https://github.com/F1u0rite/meituan_challenge_ws/blob/b4185155383541e1a4161587c4049a162f74b36d/docs/setup/AUBO_ROS2_DRIVER_JAZZY.md)于2026-10-11记录独立Fake验证环境 **Ubuntu 24.04.4 LTS / x86_64 / ROS 2 Jazzy**，历史日志主机名为asusNUC05。它证明该记录对应的环境，不证明所有项目部署完成，也不是本轮对实际NUC的实时检查。个人Mac为ARM64；ARM容器结果不能直接充当x86_64 NUC的部署和时序验收。

用户已要求“终止验收，清理”。本机C1复验准备中止，未执行测试，没有新增PASS/FAIL；本次新增容器、镜像、下载源码、临时报告及相关启动进程已清理，既有镜像与历史证据保留。本轮同步不恢复监测或验收；原C1未通过结论继续有效。

个人正式源码锁定保持 `1d940f7c5f12561fed6641457c0b4b630a282b05`。后续如重新授权验收，应选定候选SHA、核对原生x86_64/Jazzy环境，并分别验证C1、完整FSM执行/停稳及实际硬件。冻结失败、签收回执与旧报告均保持原字节。

使用OpenAI Codex协助核对公开仓库状态、整理验收范围与文档；本轮未修改功能代码、参数或接口。
