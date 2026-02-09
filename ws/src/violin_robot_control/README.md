# violin_robot_control

控制核心与安全限幅。

## 职责

- 将上层（skills/policy）输出转换为可执行的末端/关节命令
- 阻抗/导纳控制模式切换（接触相位低刚度）
- 安全约束（限位/速度/力矩/急停）执行以 `contracts/safety/` 为 SSOT

## 现状（v0）

- 已有 `tactile_guard_node.py`：对 `command_raw` 做触觉门控/减速/急停（JSON 通道）

## 关联 SSOT

- 安全约束：`../../contracts/safety/limits.md`
- 数据 schema：`../../contracts/data_schema/episode_schema.md`
