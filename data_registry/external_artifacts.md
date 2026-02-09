# External Artifacts Registry

> pointers-only：这里只登记外部位置与元数据，不存大文件。

## 1) 数据集（示教/回放/评测）

- **Dataset ID**：TBD
  - **位置**：TODO
  - **格式**：LeRobot/Parquet 或 RLDS/TFRecord（待定）
  - **统计**：episodes=N, duration=, sensors=
  - **Schema**：`../contracts/data_schema/episode_schema.md`
  - **采集条件**：琴/弓/松香、场地、光照、温湿度（如有）

## 2) 模型权重

- **Model ID**：TBD
  - **位置**：TODO
  - **基线**：TBD
  - **校验**：sha256=TODO

## 3) 评测与日志

- **Eval Run ID**：TBD
  - **位置**：TODO
  - **协议**：`../docs/engineering/evaluation_protocol.md`
  - **结果摘要**：SR/IR/ER（含置信区间）
