# mtc_simulation —— Gazebo 仿真（迁移占位）

本目录是**迁移占位**，不是已迁移完成的功能包。

## 为什么是空的

本包应从旧工程 `/home/chang/meituan_challenge/src/mtc_simulation` 复制而来（含
`src/simulation_system.cpp`、Gazebo 世界与场景）。该路径在本机**不存在**，取证见
`docs/migration/SOURCE_AUDIT.md` §4。

## 刻意不做的事

1. **没有 `package.xml` / `CMakeLists.txt`**：避免空包被当作已完成迁移。
2. **没有安装或伪造 Gazebo 依赖**：本机亦无 ROS 2 Jazzy，仿真回归**无法运行**。
3. **没有用假测试冒充仿真验证**：设计文档 V0.1 §11 明确“Ground Truth 不能伪装为真实感知”。

## 补齐后如何接入

1. 只读审计并复制世界文件、场景与 `simulation_system.cpp`，记录哈希；
2. 确认仿真依赖（Gazebo/ros2_control/gazebo_ros2_control）版本与 Jazzy 匹配；
3. 运行迁移后的仿真回归并与旧结果对照，把真实输出写入
   `docs/migration/IMPLEMENTATION_REPORT.md`；
4. 如缺依赖，如实标记 `NOT RUN`，不得替换为假测试。

## 当前状态

**迁移状态：NOT RUN**（源缺失）；**仿真回归：NOT RUN**（缺依赖与基线）。
