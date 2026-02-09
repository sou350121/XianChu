# violin_robot_skills

技能与编排层：把「持琴/持弓/按弦/运弓/换弦/连弓/恢复」定义为可组合 skills。

## 核心约束

- 状态机切换条件以触觉门控为准（SSOT：`contracts/safety/` + `contracts/data_schema/episode_schema.md`）
- 输出必须满足可执行率 ER（由控制层兜底与统计）
