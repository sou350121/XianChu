# Data Capture（数据采集 runbook）

## 目标
把示教/回放/评测过程转成**可复现数据**，并能对齐触觉/音频/控制命令时间轴（以控制命令为主时钟）。

SSOT：
- Episode schema：`../../contracts/data_schema/episode_schema.md`
- Music spec：`../../contracts/data_schema/music_spec.md`
- ROS 接口：`../../contracts/ros_interfaces/README.md`

## 1) 采集前（必须确认）

- **demo profile/配置版本**：记录本次运行使用的 `configs/` 版本（建议写入 run 的元数据）
- **校准版本**：记录 `calibration/` 产物版本（或外部指针）
- **时间同步**：确认所有 JSON topic 都包含 `timestamp`，标准消息用 `header.stamp`
- **安全门禁**：确认 `contracts/safety/limits.md` 对应的阈值已加载（或写明默认值）

## 2) 采集内容（v0 最低集合）

必须录制（对齐主时钟与复盘必需）：
- `/violin/music/spec`
- `/violin/state/joint_state`
- `/violin/state/end_effector_pose`
- `/violin/tactile/features`
- `/violin/audio/features`
- `/violin/control/command_raw`
- `/violin/control/command`
- `/violin/control/emergency_stop`

强烈建议额外录制（如存在）：
- 任何驱动 fault / 碰撞检测 topic
- 视觉（若用于定位/标定）：外部相机或头相机（注意体积，外部存储）

> 大文件（raw tactile / raw audio / video）不入库：放外部存储并登记 `../../data_registry/external_artifacts.md`。

## 3) 录制（rosbag2 示例）

```bash
# v0：录制最薄切片所需 topics（按需增减）
ros2 bag record -o v0_run_$(date +%Y%m%d_%H%M%S) \
  /violin/music/spec \
  /violin/state/joint_state \
  /violin/state/end_effector_pose \
  /violin/tactile/features \
  /violin/audio/features \
  /violin/control/command_raw \
  /violin/control/command \
  /violin/control/emergency_stop
```

## 4) 采集后登记（pointers-only）

把本次 run 的信息登记到：
- `../../data_registry/external_artifacts.md`

建议最小字段：
- `run_id`、日期时间、场地/琴弓信息、demo profile
- 硬件 SN（若可得）、校准版本、配置版本
- 结果：`outcome.success`、`failure_mode`、`interventions`（若已人工标注也可后补）
