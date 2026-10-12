# 10/12 Calibration / TF / TCP 进度汇报

可以直接转给团队。原交付 PR #6 已合入 main；当前安装说明修订分支为 `fix/calibration-side-mount-trial`，入口为根目录交接文档。

> Calibration / TF / TCP 已完成离线 P0：核对固定版本的官方 AUBO S3 坐标链，整理 D435i 设备内外参读取流程、相机/末端/TCP/台面/工位坐标约定，以及已知/待测表和现场手眼、TCP、T0 测量 SOP。完成无 ROS 依赖的完整位姿转换和 TCP 逆偏置校验；未知外参、缺 TF、异常四元数、单位和时间错误有拒绝测试。Claude 优化后 48 项测试已复跑；本轮再补 6 项 fake 模型/日志回归，总计 54 项通过，证据可用 `run_offline_validation.py` 一键重建；未修改现有 IDL、FSM 或机器人驱动。
>
> 常提供的 URDF、SRDF、三份 TF 日志和 TF 图与 main 归档一致。独立 FK 核验日志，确认 fake planning frame=world、EE=ee_link、wrist3→ee 固定 yaw+90°，不能将 fake 读数转成真实标定。已同步 main 的规划模块，补充参数方向、TCP 补偿所有权、工位目标语义及动态观测时间阻断说明。
>
> 安装选择已更新：用户先试相机带螺孔的窄底面贴蓝色圆盘正面，镜头大致侧向观察，与法兰轴线/舌规长杆垂直。旧玻璃中心 [100,0,40] mm、深度原点 [82.5,0,35.8] mm 和旧旋转均只保留为历史布局的回归示例，不用于当前试装。机身照片已确认标签 D435I，但还需读取实际设备参数。新安装位置、完整朝向、圆盘到金属法兰面的距离及接触层尚未测量，相机外参仍 UNKNOWN/null。详见 [当前试装](CURRENT_MOUNT_TRIAL.md)。
>
> 现场先冻结 V1/相机装配、核对实际 TF 名称和设备参数，再采集手眼与工具数据、台面/T0，并做独立误差验证。真实外参、TCP、台面/T0 和进入方向仍待测；本分支的 ROS 集成及实机验证未运行。当前规划器已负责 TCP 逆补偿，交接时不得重复补偿；其 latest TF 查询还需由视觉/集成/规划明确采集 ROS stamp 的接入，移动相机观测不能直接取最新机器人姿态。

现场需要各成员提供的关键输入：

| 成员职责 | 输入 |
| --- | --- |
| 机械 / 工具 | S3 实物法兰接口尺寸、V1 包络与安装图、相机转接件方案；确认可装与视野 |
| 视觉 | 实际相机 optical frame、颜色/深度是否对齐、设备参数、采集时间、ring 原点与三轴定义 |
| 规划 | 真实部署复核 world / ee_link；T_ee_TCP、工具目标姿态、T0 的落座 TCP / 电池语义、允许接合余量；当前工位朝向未消费 |
| 集成 / 状态机 | 同步机器人位姿与图像记录、TF 历史可用性、ROS/Core 时间适配、拒绝状态消费 |

文件入口：[交接文档](../../TF_AND_CALIBRATION_HANDOFF.md)；[fake 核验及规划交接](FAKE_RUNTIME_AUDIT.md)；[当前试装](CURRENT_MOUNT_TRIAL.md)；[历史安装候选](S3_D435I_MOUNT_DESIGN.md)；[现场 SOP](FIELD_SOP.md)；[复现证据](VALIDATION_REPORT.md)。
