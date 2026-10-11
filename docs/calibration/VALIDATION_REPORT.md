# 离线验收记录

2026-10-11，Windows / Python 3.11.9；完整命令、执行时间、返回码与 CLI JSON 见 [results.json](validation/results.json)，逐项测试输出见 [unittest.txt](validation/unittest.txt)。

| 验证项 | 结果 | 证据 / 解释 |
| --- | --- | --- |
| Python 离线测试 | PASS，34/34，无跳过 | unittest discover；包含完整位姿链、非交换旋转、TCP 逆补偿、腕部随动、逆查找、异常拒绝、CLI 以及名义安装几何 |
| 合成 Pose/TF case | PASS，exit 0 | [synthetic.json](validation/synthetic.json)：位置约 [1,1.1,0.05]，完整姿态与 stamp 保留，simulation_only=true、motion_authorized=false |
| UNKNOWN 相机安装外参 | EXPECTED REJECTION，exit 2 | [unknown_extrinsic.json](validation/unknown_extrinsic.json)：UNKNOWN_CALIBRATION，不代入单位变换 |
| 支架设计推导 | PASS，exit 0；ESTIMATED | [nominal_mount.json](validation/nominal_mount.json)：F→depth optical [0.0825,0,0.0358] m；不宣称 deployment_ready |
| 设计 JSON 当作真实/合成观测 case | EXPECTED REJECTION，exit 2 | [design_not_real_case.json](validation/design_not_real_case.json)：INVALID_CONFIG，设计值不冒充实测快照 |
| 超大整数输入 | EXPECTED REJECTION，两个 CLI 均 exit 2 | [overflow_rejection.json](validation/overflow_rejection.json)：10**400 坐标被 INVALID_VECTOR 拒绝 |
| JSON 解析 / CSV 列数 / Python 编译 | PASS | 全部新校验与测量文件 |
| 本地 Markdown 文件链接 | PASS | 官方远程来源已在研究中核查；本项只检查新文档引用的本地文件存在 |
| SVG / 安装候选示意图 | 已渲染目视检查 | camera_link 与 TCP 为末端的独立分支；设计图包含参考系、方向、尺寸和估计状态 |
| 修改范围 | 新文件限定本分工 | 根交接文档及 scripts/tests/docs/config/examples 的 calibration 目录；未改已有 IDL/Core/FSM/驱动 |
| ROS/Jazzy 节点、TF 监听和集成 | NOT RUN | 本机离线工具没有 ROS 依赖；现场只读命令已提供但未执行 |
| 手眼求解、实物外参、V1 TCP、台面/T0 误差 | NOT RUN | 缺现场数据；清单仍为 UNKNOWN；未实现或调用手眼求解器 |
| 自动规划拒绝规则接入 | NOT IMPLEMENTED HERE | 诊断工具输出拒绝原因；实际适配器接入由对应成员完成 |

TDD 起点记录：基础校验模块未实现时，测试门禁失败 1 项、28 项跳过；安装推导脚本未实现时，门禁失败 1 项、4 项跳过。随后实现并执行全部 34 项测试通过。初始失败来自本轮执行记录，没有把跳过的用例称为已经观察到的行为失败。

独立只读审查未发现 Critical/Important；审查提出的超大整数输入问题已先用拒绝子用例复现，再修正转换溢出的处理。修正后本机与审查者均独立复跑 34/34 PASS；两个 CLI 的溢出拒绝证据另存。审查没有修改原工作树或操作实机。

这些验证证明离线数学和输入契约按测试样例工作，不证明机器人运动正确、任务达到比赛精度或团队适配器已消费校验结果。MEASURED evidence 在工具中是引用检查；证据真实性、误差和安装稳定性由现场复核。
