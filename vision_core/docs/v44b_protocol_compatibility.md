# V4.4-B / LIVE_VISION_PROTOCOL_V1 兼容性说明

协议仍为 V1，字段、类型、颜色枚举、确认门禁、长度前缀均不变。不修改接收端。

- 新的 core 字段 detection_ambiguity / detection_ambiguity_reasons 只描述视觉目标确认、颜色及身份关联证据。
- pose_ambiguity / pose_ambiguity_reasons 描述 CAD、Yaw、提环多解；pose_validation_reasons 单独保留独立三维精度未验证。
- detection_stale / detection_age_ms 使用当前帧 host_utc 与处理完成系统时间；cad_stale 保留 CAD 源帧及年龄判定。
- 协议 ambiguity 对应未解决的目标候选；stale 对应当前目标观察的过期。CAD 多解或过期时所有空间姿态候选字段为 null，避免以 ambiguity=false 发送未解决的空间候选。
- 无时间来源时 detection_stale 为 null，发送端保守按 stale=true 处理。检测有效期工程初值 2.5s，与原有 CAD 新鲜度上限相同；不是精度标准或时钟标定。
- body_pose_valid / ring_pose_valid 始终 false，接收端不得将诊断候选用于抓取。
- 旧 core 记录没有新字段时继续使用旧映射，不静默重解释历史数据。旧 ambiguity 列表和 stale_status 留作兼容的历史诊断别名，不再用作新记录的目标有效性依据。

这是语义修正，不要求 Haoran 修改 V1 接收程序，但联调必须告知其上述映射区别。未记录的原始网络包不补造；回放和历史语义对比均标记为离线诊断。
