# S3 / D435i 可调整支架：安装候选与设计外参

2026-10-11。用户确认可以按尺寸重新制作或调整支架。本文给出可测量、可复算的安装候选；它是布局提案，不是已核对 V1 干涉的加工定稿或实测手眼结果。

## 1. 官网 / GitHub 查到了什么

| 资料 | 已核验信息 | 能支持的结论 |
| --- | --- | --- |
| AUBO 官方 S3 URDF，固定提交 `47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1` | 机器人运动链、末端 `wrist3_Link`；另由 Xacro 添加 `ee_link`，相对 wrist3 绕 Z +π/2 | 可明确软件坐标关系，不能得到本团队支架尺寸 |
| 同提交 S3 `visual/link6.DAE` | 包含末端视觉网格。按 COLLADA 节点变换，外形约 X ±36 mm、Y -36～39.5 mm、Z -55.61～0 mm | 仅用于初步空间布局与模型参考，不确定孔螺纹、公差、实物版本或定位基准 |
| AUBO 2024 官方目录的搜索索引 | 存在“ AUBO-S3末端法兰 / S3尺寸图”条目；直接 PDF 下载当前失败 | 尺寸图尚未取到并视觉核验，不抄索引碎片中的孔尺寸 |
| 当前 AUBO 开发者 S 系列页面 | 当前列 S2 / S5 | 不把 S2、iS3 或 ES3 的法兰参数代入这台 S3 |
| RealSense D400 2024-10 官方数据手册，337029-017，p146 Figure 10-9 | D435/D435i 外形 90 × 25 × 25.05 mm；背面两个 M3 安装孔，间距 45 mm、最大旋入深度 3 mm | 可定义相机安装位置与外壳包络；图中光学定位的括号尺寸属于参考尺寸 |
| 同手册 p93 Figure 4-4、p94 Table 4-16 / §4.8.1 | 深度原点 X/Y 为左 imager 中心；D435/i 深度零平面相对前玻璃 Z′=-4.2 mm | 可由玻璃基准估计深度光学原点，不用外壳中心代替它 |

已查看的本地官方 PDF、页面截图和网格投影保存在本项目 `报告附件/官方资料`。末端模型源码哈希见 MODEL_FRAME_AUDIT.md；网格与 PDF 哈希见交付回执。

## 2. 候选安装基准与尺寸

定义物理设计坐标系 **F**：原点为末端法兰安装面的中心；+Z 沿法兰轴线向工具外侧；+X 是安装时明确标记的一条径向方向，+Y 按右手系确定。F 目前不直接等同于 `wrist3_Link` 或 `ee_link`，必须与实际部署模型对齐并保存证据。

定义 **H**：D435i 前玻璃平面上的外壳宽/高中心，坐标轴采用光学轴向。由相机背后向前看，+X 向图像右，+Y 向下，+Z 向前。相机顶边因此朝 -Y；不要按面对镜头时的左右反着安装。

候选按以下几何安装，所有尺寸单位 mm：

![候选安装基准与尺寸](mount_design.png)

| 项目 | 候选值 / 要求 |
| --- | --- |
| H 相对法兰中心 F | `[100, 0, 40]` |
| 相机观察方向 | +Z_H 与 +Z_F 平行，朝工具外侧 |
| 图像右向 | +X_H 与 +X_F 平行 |
| 图像下向 | +Y_H 与 +Y_F 平行 |
| 相机宽度方向包络（投影） | F 的 X 约 `[55,145]`；高度 Y 约 `[-12.5,12.5]` |
| 背面 M3 安装孔 | 按官方 45 mm 孔距；X 投影约 77.5 / 122.5 mm，Y≈0；实际 Z 安装面另行定义 |

100 mm 侧偏与 40 mm 玻璃前伸是本次提出的布局尺寸，不是 AUBO 或 RealSense 推荐值。相机侧偏让它避开法兰中心的工具安装区域；没有 V1 实物包络时，这还不能证明视野、碰撞或强度满足比赛要求。视线是否能看到环也取决于观测姿态与 V1 遮挡。

支架可采用法兰转接件加侧伸托板/可调相机座，安装时用机械量具定位玻璃中心 H，调整到上述轴向后锁紧。支架的材料、厚度、法兰孔位、定位销及紧固承载需机械成员依据实物接口和 V1 装配定稿。相机后壳并非前玻璃；不要直接把“40 - 25.05”当成精确的 M3 安装面 Z 尺寸。加工孔位应以安装面基准图/CAD或实测确定。

相机 M3 **旋入壳体深度**不得超过官方图中的 3 mm；螺钉总长还包含支架/垫片厚度。安装时留出 USB 接头与弯线空间。本文没有给出 S3 法兰孔钻孔坐标或支架强度结论。

## 3. 根据候选几何推算外参

`T_A_B` 把 B 系坐标映射到 A 系。由相机背后看，左红外 imager 在外壳中心的 -X 方向；官方图参考偏移为 17.5 mm。深度零平面在前玻璃后方 4.2 mm。本候选将光学中心高度视为外壳中心高度，实际设备/安装偏差仍需验证。

所以本次采用的 **名义** `T_H_D`（D 为 depth optical）平移为 `[-0.0175,0,-0.0042]` m，旋转为单位阵。它包含参考尺寸和对齐假设，精度不能由小数位数推断。

`T_F_D = T_F_H × T_H_D`：

```text
T_F_depth_optical =
[[1,0,0,0.0825],
 [0,1,0,0     ],
 [0,0,1,0.0358],
 [0,0,0,1     ]]

translation_m = [0.0825, 0, 0.0358]
quaternion_xyzw = [0, 0, 0, 1]
status = ESTIMATED
```

这是一组 **法兰设计 F → 深度光学系** 的完整设计外参。只有安装达到约定尺寸/朝向时，它才代表对应名义装配。不能将这些数字直接填成现有相机的实测外参。

官方 ROS Wrapper 的 `camera_link` 名义原点为左 IR，轴向为 X 前/Y 左/Z 上，和 depth optical 的 X 右/Y 下/Z 前不同。若运行内部 TF 与该约定一致，则：

```text
T_F_camera_link_nominal:
  translation_m = [0.0825, 0, 0.0358]
  quaternion_xyzw = [0.5, -0.5, 0.5, 0.5]
```

**不能把光学系的单位四元数复制给 camera_link。** 对彩色图/对齐到彩色的深度输出，还必须按设备内部 TF 接到实际 color optical frame，不能把 depth optical 参数直接当成 color optical 参数。驱动已提供的内部 TF 不重复发布。

若现场证实 F 与实际 `wrist3_Link` 完全重合，上述 camera_link 外参才可解释为 `T_wrist3_camera_link`；否则先使用核验后的 `T_wrist3_F` 合成。若改以 `ee_link` 为父系，则应再左乘官方 `inverse(T_wrist3_ee)`，不能忽略其 +π/2 旋转。

复算命令，输出始终 ESTIMATED / `deployment_ready=false`：

```bash
python3 scripts/calibration/derive_nominal_mount.py examples/calibration/nominal_mount_design.json
```

修改设计 JSON 中玻璃中心坐标或四元数即可重算侧偏/倾斜方案。该 JSON 不属于实测站点配置，也不能作为 `check_calibration.py` 的 REAL case。实际站点清单里的安装外参仍为 UNKNOWN。

## 4. 按方案安装后的最短闭环

1. 机械成员核对 S3 实物法兰接口与 V1 外形，确认本候选可装、视野无挡、接头有空间；不满足时调整 100/40 mm 和朝向并重算。
2. 以明确标记的法兰 F 测量玻璃中心 H 与三轴方向，锁紧并拍照、记录。确定 `T_robotEE_F`，注明采用的 URDF 提交与实际父 frame。
3. 读取这台 D435i 的内参、实际帧名与内部 TF，确认视觉使用 depth 还是 color optical；在仿真/RViz 检查相机姿态与实物方向。
4. 按 FIELD_SOP.md 采集不同姿态的手眼数据，并以未用于求解的点/姿态验证。暂用 CAD 外参时仍记录 ESTIMATED 和验证误差，不能仅因按图安装就标 MEASURED。
5. 误差达到团队根据舌规与提环余量确定的要求后，交规划/集成使用有效标定版本。任务允许的误差当前未知。

1° 安装角误差在约 0.3 m 观测距离上会带来约 `0.3*sin(1°)=5.2 mm` 的横向误差；这是几何估算，说明名义装配应保留验证步骤，不是本设备误差测量。近距离观测能力还随分辨率变化，见数据手册 §4.6；先定位再接近工具的现场姿态由规划成员确定。

## 5. 来源

- [AUBO S3 官方固定 URDF](https://github.com/AuboRobot/aubo_description/blob/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1/urdf/aubo_S3.urdf)
- [末端 Xacro](https://github.com/AuboRobot/aubo_description/blob/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1/urdf/xacro/inc/aubo_ros2_macro.xacro)
- [S3 官方末端视觉网格](https://github.com/AuboRobot/aubo_description/blob/47fa5e02fa873f27f7e812d31f31e3f4cf5e56b1/meshes/aubo_S3/visual/link6.DAE)
- [RealSense 官方数据手册 2024-10](https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf)，已下载并渲染核对 p93/94/146
- [RealSense ROS Wrapper 坐标说明](https://github.com/realsenseai/realsense-ros#ros2robot-vs-opticalcamera-coordination-systems)
- [AUBO 开发者 S 系列页面](https://developer.aubo-robotics.cn/hardware/robot_arm_s.html)
- [AUBO 2024 目录索引线索](https://aubocdn.aubo-robotics.cn/official_website/hardware/AUBO_Robotics_ZH_202410_Catalog.pdf)，本轮直接获取失败，未作为孔位数值依据
