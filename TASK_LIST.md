# TASK_LIST：XianChu（弦触）— 小提琴人形机器人（触觉闭环）任务清单

> 本文件是工程执行的“可勾选任务清单”，按里程碑 M0–M3 组织。

关联 SSOT：
- PRD：`PRD.md`
- 系统设计：`DESIGN.md`
- 契约：`contracts/`
- runbooks：`docs/engineering/`

---

## 约定

- **DoD（Definition of Done）**：每个任务必须写清“验收口径/产物位置/可复盘证据”。
- **pointers-only**：rosbag/raw 数据/模型权重大文件不入库，登记到 `data_registry/`。

---

## M0：安全与可重复（不掉琴、不伤人）

### M0-1 契约与文档（SSOT 先行）
- [ ] **M0-1.1** 确认 `contracts/safety/limits.md` 的阈值来源与默认策略（含失联超时）
  - **位置**：`contracts/safety/limits.md` + `configs/control/`
  - **DoD**：阈值全部可配置；demo profile 可覆盖；文档写明“保琴优先”的门禁逻辑

- [ ] **M0-1.2** 确认 `contracts/frames/tf_tree.md` 与硬件现有 tf 树的映射（不破坏式扩展）
  - **DoD**：能给出 `base_link/left_ee/right_ee/violin_body/bow_hair/string_*` 的定义与来源（静态标定或推算）

### M0-2 安全门控 v0（vertical slice 必需）
- [ ] **M0-2.1** 完成 e-stop 全链路（latched）与解除流程（受权限/物理按钮约束）
  - **包**：`violin_robot_control`（门控） + `violin_robot_drivers`（驱动）
  - **DoD**：触发后 `/violin/control/emergency_stop=True` 持续；控制输出归零；有 reset 流程说明

- [ ] **M0-2.2** “持琴优先”门禁：`violin_grip` 高风险 → recovery 或 e-stop
  - **包**：`violin_robot_control`
  - **DoD**：可通过模拟/测试注入复现；日志能解释触发原因

### M0-3 现场 runbook
- [ ] **M0-3.1** 完成现场安全 checklist（围栏/安全员/急停/线缆）
  - **位置**：`docs/engineering/demo_safety.md`
  - **DoD**：能在现场照做；包含演示前/中/后

---

## M1：可稳定出声（空弦/单音）

### M1-1 弓-弦闭环的最小实现（teacher 规则）
- [ ] **M1-1.1** 触觉特征输出稳定（`bow_string` 至少有 `contact_phase/slip_risk/contact_mode/hf_energy`）
  - **包**：`violin_robot_state`（建议承载特征提取）或 `violin_robot_drivers`（早期过渡）
  - **DoD**：rosbag 可复盘；字段与 `contracts/data_schema/episode_schema.md` 一致

- [ ] **M1-1.2** 门控策略：`slip_risk`/`hf_energy` 触发减速/减力/恢复
  - **包**：`violin_robot_control`
  - **DoD**：在 mock slip 注入/真机试验中可复现；`command` 与 `command_raw` 有可解释差异

### M1-2 音频自听 v0
- [ ] **M1-2.1** 音频特征提取：`f0_hz/energy/noise_ratio/valid`
  - **包**：`violin_robot_state` 或 `violin_robot_drivers`
  - **DoD**：对无声/噪声场景 `valid` 行为明确；字段写入 rosbag

---

## M2：可辨识旋律（含换弦/简单连弓）

### M2-1 技能层（skills）与恢复原语
- [ ] **M2-1.1** 定义最小技能集合（API/状态机切换条件）
  - **包**：`violin_robot_skills`
  - **DoD**：至少包含 `hold_violin/hold_bow/bow_stroke/string_switch/recover_*` 的接口草案与输入输出

- [ ] **M2-1.2** 换弦策略与失败恢复（`string_miss`）
  - **DoD**：评测中记录换弦成功率；失败可自动撤离并重试（受 retry_policy 限制）

### M2-2 评测闭环
- [ ] **M2-2.1** 输出 SR/IR/ER 与失败分布的评测脚本/流程（可先半自动）
  - **包**：`violin_robot_eval`
  - **DoD**：能从 rosbag + 少量人工标注生成报告；报告登记到 `data_registry/`

---

## M3：1–2 分钟展示级端到端

### M3-1 端到端输入
- [ ] **M3-1.1** MusicXML/MIDI 解析器 → `music_spec`
  - **包**：`violin_robot_music_spec`
  - **DoD**：满足 `contracts/data_schema/music_spec.md`；能驱动完整 demo

- [ ] **M3-1.2** 音频输入路线（可选，若作为本期硬目标）
  - **DoD**：音频→事件对齐策略明确；不稳定时按 PRD cut line 降级

### M3-2 稳定性与可运维
- [ ] **M3-2.1** 连续 N 次 demo 的稳定性评测与报告（含置信区间说明）
  - **DoD**：按 `docs/engineering/evaluation_protocol.md` 产出报告；失败分布可解释

- [ ] **M3-2.2** 一键演示流程（校准→自检→开演→收尾→导出）
  - **包**：`violin_robot_bringup`
  - **DoD**：现场可重复；异常可快速复位
