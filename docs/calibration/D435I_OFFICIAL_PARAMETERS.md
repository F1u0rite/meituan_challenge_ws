# 官方提供哪些 D435i 参数

2026-10-11 查阅官方 SDK 与 ROS Wrapper 资料。

## 1. 两类外参分别记录

| 参数 | 官方支持 | 本项目当前状态 |
| --- | --- | --- |
| 彩色/深度流内参、畸变模型 | SDK `get_intrinsics()` 或实际流 `CameraInfo` | UNKNOWN：未读取当前设备/分辨率 |
| 深度、彩色等内部传感器间外参 | SDK `get_extrinsics_to()`；ROS Wrapper 的相应内部 TF/Extrinsics | UNKNOWN：可从当前设备读取，不能照抄官网示例数值 |
| 相机光学系轴向 | X 右、Y 下、Z 前；ROS 相机机体系 X 前、Y 左、Z 上 | 约定已知，实际 frame 名与内部 TF 待读取 |
| 法兰/末端参考系到 D435i | 与本团队支架和安装有关；官方内部传感器外参不能提供这段关系 | UNKNOWN：手眼标定，或 CAD/测量给 ESTIMATED 后复核 |
| V1 工具 TCP | 与舌规装配和工具工作点有关 | UNKNOWN：工具测量/标定 |

SDK 返回的平移单位为 m，旋转是 3×3 正交矩阵；ROS Extrinsics 的九元素旋转按 column-major 存储。与本交付 xyzw 四元数转换时注意矩阵布局和方向，不按 row-major 直接 reshape。本文不提供某台其他 D435i 的外参数值。

官方资料存在年代不同的 IMU 内参描述，本任务不据此开展 IMU 标定；当前任务只需颜色/深度定位与末端安装外参。

## 2. 明天优先读取现有设备参数

先由视觉/集成人员启动已确认版本的 D435i ROS 节点。本交付不启动相机、不更改设备 EEPROM、不重新烧写标定。

```bash
ros2 topic list -t
ros2 node list
```

在实际话题列表中选择彩色/深度 `camera_info`、内部 extrinsics 和图像话题。官方常见例子为 `/camera/camera/color/camera_info`、`/camera/camera/depth/camera_info`、`/camera/camera/extrinsics/depth_to_color`，**这些名称并非本项目已核验值**。设定变量为实际值后执行只读导出：

```bash
# 将变量值换成上一步实际发现的话题，保留证据目录。
export COLOR_INFO_TOPIC=/camera/camera/color/camera_info
export DEPTH_INFO_TOPIC=/camera/camera/depth/camera_info
export INTERNAL_EXTRINSICS_TOPIC=/camera/camera/extrinsics/depth_to_color
mkdir -p runtime_logs/calibration/device
ros2 topic echo "$COLOR_INFO_TOPIC" --once > runtime_logs/calibration/device/color_camera_info.yaml
ros2 topic echo "$DEPTH_INFO_TOPIC" --once > runtime_logs/calibration/device/depth_camera_info.yaml
ros2 topic info "$INTERNAL_EXTRINSICS_TOPIC" --verbose
# transient_local/reliable 应与上一步实际发布者 QoS 匹配；不要盲目套用。
ros2 topic echo "$INTERNAL_EXTRINSICS_TOPIC" --once --qos-durability transient_local --qos-reliability reliable > runtime_logs/calibration/device/depth_to_color.yaml
```

没有消息时保留 NOT RUN/FAIL 原因，不空等认定成功。CameraInfo 需核对分辨率、K/D/R/P、distortion_model、header.frame_id，并记录设备序列号、固件、SDK/Wrapper 版本和视觉是否将深度对齐彩色。使用校正图、对齐深度或 resize 图时必须采用相应投影/内参处理，不能套其他流的矩阵。

可以先复用设备参数并用真实测量点检查质量；只有验证显示问题或输入处理不匹配时，再决定是否额外标定相机内参。当前没有数据，不能宣称已完成该质量验证。

## 3. 官方来源

- [SDK 投影与内外参](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0#extrinsic-camera-parameters)：流间外参、`get_extrinsics_to()`、m 单位。
- [ROS Wrapper](https://github.com/realsenseai/realsense-ros#extrinsics-from-sensor-a-to-sensor-b)：内部外参话题及 column-major。
- [坐标轴约定](https://github.com/realsenseai/realsense-ros#ros2robot-vs-opticalcamera-coordination-systems)：光学系和 ROS 系不同。
- [D435i SDK 说明](https://github.com/realsenseai/librealsense/blob/master/doc/d435i.md)：内部 IMU/深度坐标关系。

读取到设备内参/内部外参后，该项可标 MEASURED，来源注明 DEVICE_READBACK；它不意味着法兰至相机的手眼外参也完成。

支架可重新制作/调整后的布局提案、前玻璃/左 IR 原点区别及设计外参推导见 [S3_D435I_MOUNT_DESIGN.md](S3_D435I_MOUNT_DESIGN.md)。官方 PDF 已下载并渲染核对图页；45 mm 安装孔距可以用于支架规划，实物到法兰的关系仍由具体安装定义。
