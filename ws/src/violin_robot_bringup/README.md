# violin_robot_bringup

系统启动与编排（launch）包。

## 职责

- 统一启动 drivers/state/control/policy/eval 等节点
- 加载 demo profile（来自 `../../configs/demo/`）

## 关联 SSOT

- ROS 接口：`../../contracts/ros_interfaces/README.md`
- 安全约束：`../../contracts/safety/limits.md`
