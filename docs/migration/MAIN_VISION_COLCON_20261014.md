# 主线视觉 pytest 的 colcon 登记修复（2026-10-14）

主线 `b25c2cdd9cca58b09e864d44be4383f34542b42b` 合入 PR5 后，`colcon test --packages-select mtc_vision_adapter --return-code-on-test-failure` 调用 unittest，实际执行 **0 项测试并退出 5**。直接 pytest 能运行交付的测试。原因是 setup.py 缺少 pytest 登记；本次只增加一行 `tests_require=['pytest']`，不改变发布器、安全 helper、IDL、测试断言或 FSM 内核。

PR18 的同类补丁只合入 `codex/yuze-parallel`；PR19 面向 `feature/perception-adapter`，包含未交付到本主线的实时/场景功能；PR20 是 C++ 审计工具修复。发布前核对未发现同一视觉问题的主线 PR，因此单独回移这一行；没有为了获取修复而拼接未合并的功能分支。

在同一 Ubuntu24.04 / ROS2 Jazzy 隔离容器对照：network none、2 CPU / 4 GiB、无设备映射、宿主挂载或端口映射。复用23个已缓存依赖，未取外网。原运行和本次补跑分别保留。

| 检查 | 注册前 | 注册后 |
| --- | --- | --- |
| 该包构建 | 通过 | 通过 |
| 标准 colcon test | 0项 / exit5 | **57项 / exit0** |
| error / failure / skipped | 无实际覆盖 | **0 / 0 / 0** |
| 安装模块 | 原主线源码 | 3个Python模块逐SHA与原主线相同 |

57项中包含4个真实安装进程检查：synthetic/replay 自然完成各收到4/4帧，中途SIGINT各收到2帧，四个发布器均退出0；消息字段、原采集时戳、NaN、false-ring和QoS兼容性均按原测试核验。另有输入安全、本地TF Buffer与清理回归。不是相机录像、实测TF或实际FSM接受数据的证据。

复现需完整源码工作空间、Jazzy和mtc_interfaces overlay。进入**被验源码根目录**后运行本证据的 registration-before/after-command.sh。这里使用自定义 `/registration-install`；交付的旧验证器固定寻找 `<workspace>/install`，所以容器内建立 `/fix-review/install -> /registration-install` 符号链接。常规 `colcon build` 使用默认 install 时无需该别名。别名不修改跟踪源码、不是宿主挂载。测试前后使用同一源码和依赖；原始0项失败没有覆盖或删除。

聚焦证据 [index](evidence/main_vision_colcon_20261014/index.json) 包含 32 个成员，index SHA-256 **`33f247d54dc1578b168a3c7cd9f528a0e6d6213881c9f9454b14747d8efa6b1c`**；实际命令、安装身份、57项XML、四组消息与退出日志均已冻结。提交只含此setup行、本报告和这些指定证据。源码父SHA与修复后setup哈希记录在inputs，实际修复提交由Git记录。

仍未完成：私有22帧数据、真实D435i/跨机/标定与抓取精度；C++仍构造MockPerception，未由本修复接入视觉数据。旧Python Task/PickPlace、Cancel受理后ABORTED/520及Motion/FSM问题交孙嘉明；独立MoveIt退出-11交规划负责人。登记修复不能关闭这些问题。原固定运行其他失败保留在验收工作区。本PR不推进正式pin、不直接推main、不自动合并。

使用 OpenAI Codex 协助构建配置修复、隔离回归与证据归档；保留上游许可，未改几何算法、运行内核或任何测试断言。
