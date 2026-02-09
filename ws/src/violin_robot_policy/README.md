# violin_robot_policy

策略层：Diffusion/Flow/规则策略等以可替换组件运行。

## 职责

- 推理服务化（可与控制层解耦），输出 action chunks
- 与 `violin_robot_control` 通过明确接口交互（SSOT：`contracts/ros_interfaces/`）

## 关联 SSOT

- Music spec：`../../contracts/data_schema/music_spec.md`
- Action schema：`../../contracts/data_schema/episode_schema.md`
