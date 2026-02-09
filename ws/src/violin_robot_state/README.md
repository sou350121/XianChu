# violin_robot_state

状态与感知层：统一关节状态、末端状态、触觉特征、音频特征与时间同步。

## 职责

- 触觉预处理（滑移/接触事件/接触相位/高频能量代理）
- 输出可消费的 state topics
- 时间同步规则以 `contracts/data_schema/` 为 SSOT

## 关联 SSOT

- Episode schema：`../../contracts/data_schema/episode_schema.md`
