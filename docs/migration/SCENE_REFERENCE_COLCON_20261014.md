# PR19 场景参考回归的 colcon 注册补全（2026-10-14）

本次只补一行 `tests_require=['pytest']`。PR19 原提交 `182386a2e896f8b85570b73619b7ec4788a64a4c` 的场景坐标标签修复及 8 项测试均不改动；原已签收报告和冻结集也不修改。

触发条件：当前 `feature/perception-adapter` 的 `4e7ddd4794c28fdd423aaf67477fee5b8f40d2a0` 是 PR17 squash，整树与 `5efbd2d` 相同。直接 pytest 可执行视觉回归，但 `colcon test` 使用 unittest 后执行 0 用例、退出 5。PR18 已完成的同一注册补丁合入了 `codex/yuze-parallel`，没有进入当前目标分支。此次把该注册补入现有 PR19，保证它的新增回归进入标准 colcon 入口，不新建重复 PR。

在 Ubuntu24.04/Jazzy、network none、2 CPU/4 GiB、无设备/宿主挂载/端口映射的独立副本，以同一离线依赖、源码布局和断言验证：

- 注册前正确目录 `/fix-review`：1 包构建成功，colcon 0 用例、退出 5。
- 注册后：1 包重建成功，colcon/test-result 退出 0，**168 项，0 error/failure/skip**；XML 包含全部 8 个 `test_scene_reference_rejects_wrong_destination_frame` 参数实例。
- 独立安装 replay 子进程到真实 DDS 接收器：4 类错误标签均非零退出且两个话题零发布；3 类兼容输入保持正确发布，共 7 场景通过。模块安装哈希等于修复源码。
- 最初外部编排虽显式从 fix 路径构建，test 工作目录仍是 `/candidate`，因此仍发现旧包、0 用例；原失败日志完整保留。v2 同时将前后命令目录改为 `/fix-review`，没有删测试或放宽断言。
- 原/补验容器停止、所有记录 runner PID 不存在，Domain224 空。完整原监测失败在本地保留；本 PR 只纳入与注册修复相关的指定证据。

复跑需准备 Jazzy/mtc_interfaces overlay、Python 依赖和完整源码根目录，再进入根目录运行 build/test。自定义安装位置须同时设置 `MTC_VISION_INSTALL_BASE` 和 `MTC_VISION_ROS_SETUP`；`PYTHONPATH` 包含本轮 vision_core 和离线 Python 依赖。完整已执行命令见本证据中的两个 `registration-*-v2.sh`，避免从另一个源码根运行 colcon。

[冻结索引](evidence/scene_reference_colcon_20261014/index.json)：39 个原件，index SHA-256 `edcfe9ad298be0a5fe4e1500d0b4f9c6e91de9b2e6cd58471ffe4ef4d4009710`。源码父提交、仅一行代码差异、安装身份和依赖版本在 inputs/module-identity；所有原日志按原字节冻结。

仍未修复：手动参考跨 T0 丢失后重用，回交视觉负责人 / BHBAllen；FSM/Action/Cancel/Motion 内核仍由孙嘉明负责，旧 Python 220/510/520、PickPlace 不可用不由本注册补丁关闭。MoveIt 子进程 -11、真实相机/SAM/私有录制、完整 FSM 和实机均未通过本轮。没有推进正式 pin、推 main、合并或发布 Release。

使用 OpenAI Codex 协助最小构建配置修复、隔离回归和证据整理；保留原上游许可，不修改几何算法、IDL、运行内核或测试断言。
