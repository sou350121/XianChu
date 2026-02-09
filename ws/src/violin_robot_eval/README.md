# violin_robot_eval

评估与回放层。

## 职责

- SR/IR/ER 指标计算、失败分布统计、置信区间
- rosbag 录制/回放流程与离线评测脚本
- 输出报告（并登记到 `data_registry/`）

## 关联 SSOT

- 评测协议：`../../docs/engineering/evaluation_protocol.md`
- Episode schema：`../../contracts/data_schema/episode_schema.md`
