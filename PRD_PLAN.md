---
name: XianChu_PRD
overview: PRD 对齐与执行索引（requirements → design → task list）。正文以 `PRD.md` 为准。
links:
  prd: ./PRD.md
  design: ./DESIGN.md
  task_list: ./TASK_LIST.md
  team: ./TEAM.md
  agent: ./AGENT_robotExpert.md
  docs: ./docs/README.md
  contracts:
    - ./contracts/README.md
    - ./contracts/data_schema/episode_schema.md
    - ./contracts/data_schema/music_spec.md
    - ./contracts/frames/tf_tree.md
    - ./contracts/safety/limits.md
  runbooks:
    - ./docs/engineering/data_capture.md
    - ./docs/engineering/evaluation_protocol.md
    - ./docs/engineering/demo_safety.md
---

# XianChu（弦触）PRD_PLAN（对齐索引）

本文件不再承载 requirements 正文与详细任务拆解，只作为**索引与执行顺序**。

## 1) 正文入口

- PRD（requirements 与验收）：`PRD.md`
- 系统设计（闭环与分层）：`DESIGN.md`
- 任务清单（工程执行）：`TASK_LIST.md`
- 团队协作（角色/实习生配置）：`TEAM.md`
- 技术路线（决策原则/里程碑）：`AGENT_robotExpert.md`
- 工程 runbooks（构建/启动/录制/评测/现场安全）：`docs/README.md`

## 2) 推荐执行顺序

- 先读 `PRD.md` 的 **Scope/KPI/FR/NFR**（明确“验收口径”）
- 再读 `DESIGN.md` 的 **弓-弦闭环** 与 **多速率控制栈**（明确“怎么做不会翻车”）
- 最后按 `TASK_LIST.md` 的 **M0→M3** 勾选推进（每项任务带 DoD）

## 3) 变更原则

- Contract-first：任何接口/字段变更先改 `contracts/` 再改实现
- pointers-only：大文件不入库，统一登记到 `data_registry/`
