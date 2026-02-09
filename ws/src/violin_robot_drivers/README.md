# violin_robot_drivers

硬件驱动抽象层。

## 职责

- 天工本体、零巧手、触觉、音频等硬件的 ROS 2 driver 封装
- 对外只暴露标准化 topic/service/action（定义见 `contracts/ros_interfaces/`）

## 关联 SSOT

- ROS 接口：`../../contracts/ros_interfaces/README.md`
- 坐标系：`../../contracts/frames/tf_tree.md`
