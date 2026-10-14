# 回放验收入口支持 ROS 安装前缀（2026-10-14）

`fc82ffa4dab68437ab4eccdb319048466737c1c9` 已登记 pytest，但监测使用 `--install-base /candidate-install` 时，四个真实回放测试在启动发布器前失败。`ros_replay_validation.py` 把入口写死为源码根目录下的 `install/mtc_vision_adapter/...`，忽略已 source 的安装空间。

本次只用 `ament_index_python.get_package_prefix` 查找当前 overlay 内的 replay_publisher，补充其 test_depend，并在结果中记录实际前缀/可执行文件。未加入目录别名、替代发布器、跳过或放宽原断言；发布器、输入安全逻辑、接口和 FSM/Action/Cancel/Motion 内核均未改。未找到包或可执行文件时仍在启动 ROS 前失败，不回退到其他 workspace/install。

在同一 Ubuntu24.04/Jazzy、network none、2 CPU/4 GiB、无设备/宿主挂载/端口的隔离副本中，原版 **57项/4失败**；修改后自定义隔离、常规 workspace/build+install、合并安装三种布局均 **57项/零错误失败跳过**。每种均运行4个真实安装进程：synthetic/replay自然完成各4帧，中途SIGINT各2帧、exit0。导入模块和实际子进程路径匹配对应前缀，三个运行模块字节匹配原SHA。三轮是相同57项回归，不称171项独立覆盖。

另两项负例：没有登记包、登记存在但可执行文件缺失，均在初始化ROS前明确失败；即使源码旁已有默认安装也不回退。191–194及232域末尾为空，相关进程及原/补验容器已停止。

自定义布局57项先通过，但首次 `colcon test-result` 试写只读源码目录log而失败。首次命令/日志原样保留，改用显式log-base收集同一次XML，没有重跑或篡改测试；默认布局使用容器内root拥有的相同源码副本。后续命令均通过。所有开发过程失败与原始四项失败保留。

复验需先source Jazzy和本次mtc_interfaces，再构建/ source待验adapter安装空间。命令在本目录 `evidence/replay_install_prefix_20261014/commands/`；例如自定义布局：

```bash
colcon build --base-paths src/mtc_vision_adapter --build-base /fix-custom-build --install-base /fix-custom-install --executor sequential
source /fix-custom-install/setup.bash
colcon --log-base /tmp/vision-test-log test --build-base /fix-custom-build --install-base /fix-custom-install --packages-select mtc_vision_adapter --return-code-on-test-failure
```

聚焦冻结[index](evidence/replay_install_prefix_20261014/index.json)：**93成员**，SHA-256 **`946c6666f5689eea2ae9596429af7b74676d887ce5eba151f110fc257587c8f8`**；inputs记录父SHA、两个修复文件和原运行模块哈希。原PR22登记证据保持原字节。此补丁补入已有PR22，不另开重复PR、不推main、不自动合并。

仍未通过：原Python Task/PickPlace/Cancel的220/510/520与C++真实视觉消费（孙嘉明），独立MoveIt关闭-11（常建烁），rosdep/驱动环境（集成），真实相机/私有输入/标定（视觉负责人）。本修复只验证回放测试入口，不扩大为完整FSM/硬件通过。

使用OpenAI Codex协助测试基础设施修复、隔离回归和证据归档，保留原许可声明。
