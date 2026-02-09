# contracts（SSOT：契约层）

这里是本项目的 **Single Source of Truth（契约层）**：接口、坐标系、数据 schema、安全约束。

## 原则

- **Contract-first**：先改这里，再改实现。
- **可搜索、可链接、可审查**：所有对外接口与关键约定必须落在此处。

## 目录

- `ros_interfaces/`：topic/service/action 规范（人类可读，IDL 指向 `ws/src/violin_robot_interfaces/`）
- `frames/`：tf 树与坐标系约定
- `data_schema/`：episode schema、music_spec schema、时间同步
- `safety/`：限位、速度/力矩上限、急停与异常处理
