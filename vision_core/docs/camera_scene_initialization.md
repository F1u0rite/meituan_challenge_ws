# 相机启动自检与 T0 场地坐标初始化

此功能为可选模式，不改变现有相机坐标、80 mm 顶部/60 mm 孔中心高度约束或机械臂接口。只在 Yuze 工作区运行，不修改 CAD/录制原始数据，不操作机械臂。

## 坐标定义

- `camera_color_optical`：相机 X 向图像右、Y 向下、Z 向前；移动相机后数值会改变。
- `t0_scene`：原点为当前观测的 T0 框桌面中心；X 沿 200 mm 长边的指定正端；Z 为桌面向上的法线；Y = Z × X，为右手系。T0 固定在场地上是本模式的必要前提。
- `p_scene = R_scene_from_camera @ p_camera + t_scene_from_camera`。脚本不会给相机 XYZ 统一加一个固定补偿。
- 高度约束仍由已有算法执行；本脚本只转换已输出的点，不重新推断电池孔、不重复高度补偿。
- T0 落点是放置后的孔中心，理想场地坐标 `(0, 0, 0.060)` m；不是纸面中心，也不是舌规 TCP。
- 场地坐标不等于机械臂基座坐标；本功能不标定 `T_base_from_scene`、TCP 或完整插入姿态。`motion_usable=false` 和绝对误差未测量状态保留。

## 为什么需要方向参考

200×100 mm 矩形有 180° 对称性。仅有边框和桌面时，脚本返回 `DIRECTION_REFERENCE_REQUIRED`，不会自动选图像右侧为永久的场地正 X。输出该状态表示自检发现缺少方向信息，不代表桌面或分色失败。

支持两种显式参考：

1. `--positive-x-pixel U V`：当前图像中，T0 正 X 长边端点附近的像素（例如该端两角的中点）。用户指定这个端的物理含义。该像素仅适用于当前视角；检测到相机移动后立即失效，须退出并重新选择。不把演示数据中的像素当作通用参数。
2. `--aruco-id ID`：固定、平贴桌面且与场地保持不动的 ArUco 标记。标记规范角点 **0→1** 必须沿 T0 的正 X 长边方向（夹角不超过 15°）。默认 `DICT_4X4_50`，可选 `DICT_4X4_100`、`DICT_5X5_50`、`DICT_6X6_50`。标记不必在 T0 内，但必须在同一桌面且完整可见。脚本检查标记内部深度与桌面一致；重复同 ID、缺失、深度不足或明显抬高会拒绝输出。

ArUco 模式提供每次启动自动恢复方向的方式，需要现场先固定一个符合上述方向约定的标记。当前真实录制中未提供这个物理参考，不能宣称已自动完成带方向的场地标定。

## 运行独立脚本

从 Yuze 工作区执行（必须在 Ubuntu，经 SSH 进入后运行）：

```bash
cd /home/shrmeituan/meituan_vision_ws_yuze
source log/yuze_env.bash

# 离线启动自检：不打开相机。没有方向参考时退出码 2 是预期结果。
python -B vision_core/scripts/initialize_camera_scene.py \
  --input log/yuze_reposition_20261013_164751 \
  --geometry /home/shrmeituan/private_data/v44g_cad/geometry.json \
  --frames 6

# 固定好参考标记后，显式采集 10 帧进行初始化。
# 仅在设备空闲时执行，不中断其他相机进程。
python -B vision_core/scripts/initialize_camera_scene.py \
  --live --aruco-id 7 --frames 10 \
  --geometry /home/shrmeituan/private_data/v44g_cad/geometry.json
```

独立脚本不用 SAM，也不加载其模型。它复用分色、RGB-D 配对检查、当前桌面和 T0 检测。默认需要 5 个连续、不同且时间递增的有效样本；桌面 inlier ratio ≥0.6、全图有效深度比 ≥0.5，相对本轮稳定参考的位置变化 ≤3 mm、旋转变化 ≤1°。这些是初始化稳定性门限，不是绝对精度验收标准。

输出放在新的 `log/scene_init_*`：`summary.json`、逐帧 `frames.json`、末帧 `preview.png`。`--out` 只能指定工作区 `log/` 下尚不存在的目录，避免覆盖历史结果或写入 private_data。成功退出 0；未就绪、发生错误或被中断退出 2。相机在退出时释放。

可加 `--coordinates SAVED_PACKETS.json` 在离线初始化中转换已有电池坐标。只转换同一 source frame ID、完整源时间戳和采集时间均匹配的数据；不匹配直接报错。缺失点仍为 null，不从以前帧补点。

## 与现有坐标发送流程一起使用

在原有 `run_automatic_top_coordinates.py` 命令上增加：

```bash
--scene-aruco-id 7 --scene-stable-frames 5
```

仍需原有 `--live` 或 `--input`、`--geometry`、`--checkpoint` 等参数。新增字段发送在已有 JSON topic `/mtc/perception/top_hole_candidates`：

- `scene_reference`：当前帧的 T0 原点、方向、变换、稳定状态与明确源身份。
- `scene_coordinates`：`t0_scene` 中各颜色的 `green_point_scene_m`、`red_point_scene_m`，以及 T0 的 `target_hole_center_scene_m`。

原有相机坐标字段及 `BatteryDetectionArray` 保持相机坐标语义；下游必须明确选择新增场地坐标字段。不启用参数时旧消息行为不变。发送流程只支持固定 ArUco 参考；不接受可过期的手动像素作为连续运行参考。

所有变换从**本次当前帧**重新计算。相机发生超过门限的移动时，停止输出场地点，重新累计稳定样本；标记或 T0 丢失立即清空有效状态。没有读取保存的初始化矩阵这一入口，历史 `summary.json` 只能作为证据，不能直接用来转换新帧。

独立脚本采样快，但坐标发送流程中稳定计数跟随当前慢速 SAM 推理帧（目前约 9–10 秒一帧），5 帧准备可能需要约 50 秒。本功能不解决 SAM 速度、偶发漏点、相机到机器人变换或绝对精度验证。
