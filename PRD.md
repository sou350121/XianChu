# XianChu（弦触）PRD：基于触觉反馈的小提琴人形机器人（天工 × 零巧手）

> 本文件聚焦**可验收的 Requirements**（FR/NFR），并与本项目 SSOT（`contracts/`、`configs/`、`calibration/`、`docs/engineering/`）保持可追溯。

- **系统设计**：见 `DESIGN.md`
- **任务清单（工程执行）**：见 `TASK_LIST.md`

---

## 0) 元信息

- **PRD 版本**：v0.1
- **目标演示**：1–2 分钟展示级小提琴演奏
- **硬件前提**：天工人形机器人 + 零巧手（带触觉）
- **实现原则**：高层做规划与编排；低层用触觉/力控闭环（System0）兜底（见 `DESIGN.md`）

---

## 1) 一句话目标（TL;DR）

让天工人形机器人搭配零巧手触觉感知，**在无人工介入**的情况下完成 **1–2 分钟展示级**小提琴演奏（含运弓、换弦、简单连弓），并支持从 **乐谱或音频** 端到端输入到自主演奏。

---

## 2) 范围（Scope）

### 2.1 In-scope（本期必须交付）

- **端到端链路**：`MusicXML/MIDI/音频` → `music_spec` → 动作生成 → 触觉闭环执行 → 输出评测报告
- **双手任务**：机器人同时**持琴**与**持弓**
- **演奏内容**：短曲目/片段，至少包含：
  - 运弓（上弓/下弓）
  - 换弦（G/D/A/E 中至少 2 条弦；最终目标 4 条弦）
  - 简单连弓（slur/legato）
- **触觉闭环**：至少覆盖 4 类接触源（见 `contracts/data_schema/episode_schema.md`）：
  - `bow_string`（弓-弦）
  - `finger_string`（按弦）
  - `violin_grip`（持琴）
  - `bow_grip`（握弓）
- **可复盘与可验收**：SR/IR/ER + 失败分布 + 日志/rosbag 指针登记（`data_registry/`）

### 2.2 Out-of-scope（本期不承诺）

- 专业演奏级音色与风格化处理（以“可辨识、稳定、不翻车”为主）
- 即兴、多首曲目编排、多机协奏
- 视谱即弹（real-time sight reading）

---

## 3) 使用情境与场景约束（必须可落地）

- **场地**：室内/舞台；光照、噪声、空间、地面平整度
- **人机共处**：观众距离、是否需要围栏/隔离线、安全员职责
- **道具**：小提琴型号（4/4 等）、弓、松香、肩托/腮托是否允许改造
- **可选增强传感**（不作为硬前提）：外部相机/动捕、腕部 F/T、弓杆振动传感器

> 现场安全流程以 `docs/engineering/demo_safety.md` 为准。

---

## 4) 系统边界与接口（I/O）

### 4.1 输入

- **主要输入**：`MusicXML` / `MIDI` / `音频`
- **统一中间表征**：`music_spec`（SSOT：`contracts/data_schema/music_spec.md`）

### 4.2 输出

- **演示输出**：演奏音频（现场扩声/录音可选）
- **工程输出**：
  - rosbag（外部存储）+ `data_registry/external_artifacts.md` 登记
  - 评测报告（可入库小文件），指标口径见 `docs/engineering/evaluation_protocol.md`

---

## 5) 成功指标（KPI）与验收（Acceptance）

> 必须同时包含“可量化”与“可复盘”。指标与计算口径以 runbook 为准。

### 5.1 音乐层（输出质量，展示可辨识）

- **KPI-M1 曲目完成度**：在指定片段中不中断完成率 ≥ **X%**
- **KPI-M2 旋律可辨识度**：盲听人评 ≥ **X/5** 或 AB 辨识率 ≥ **X%**
- **KPI-M3 节奏稳定度**：onset 偏差 P95 ≤ **X ms**（或以人评为主，保留客观记录）
- **KPI-M4 音高稳定度**：`|f0 - f0_target|` 的 P95 ≤ **X Hz**（按曲目定义容忍区间）

### 5.2 控制层（接触与操控）

- **KPI-C1 弓-弦接触稳定**：`bow_string` 在安全窗口（例如 `slip_risk<threshold`）内时间占比 ≥ **X%**
- **KPI-C2 换弦成功率**：指定换弦点成功率 ≥ **X%**
- **KPI-C3 持琴稳定**：`violin_grip` 的 `slip_events_per_min` ≤ **X**，且无掉落
- **KPI-C4 恢复有效**：进入 recovery 后恢复成功率 ≥ **X%**

### 5.3 系统层（可靠性与可运维）

- **KPI-S1 冷启动到可演示时间**：≤ **X 分钟**（含校准/自检）
- **KPI-S2 单次演示重置时间**：≤ **X 分钟**（含故障恢复）
- **KPI-S3 连续演示能力**：连续 N 次 demo 成功 ≥ **X%**

### 5.4 评测基本盘（避免剪视频式成功）

- **SR / IR / ER**：见 `docs/engineering/evaluation_protocol.md`
- **失败分布**：至少区分：`drop_risk / excessive_force / collision / no_sound / squeal / string_miss / unstable_audio / other`

---

## 6) 功能需求（FR）

> 规范：每条 FR 必须可测试（Given/When/Then 或量化门槛），并可追溯到实现模块与 contracts。

### FR-001 输入解析与统一表征
- **需求**：系统应支持输入 `MusicXML/MIDI/音频`，并转换为统一 `music_spec`。
- **验收**：对同一段音频/乐谱，能生成符合 `contracts/data_schema/music_spec.md` 的 JSON，并可被策略层消费。

### FR-002 music_spec 事件驱动执行
- **需求**：系统应按 `music_spec.events` 的时间推进执行，并在事件切换点同步更新控制目标（弓向/弓速/力度等）。
- **验收**：录制 rosbag 后能复盘每个事件的开始/结束与控制模式切换（通过 `command_raw`/`command` 日志）。

### FR-003 触觉特征多源输出（四接触源）
- **需求**：系统应输出 `obs.tactile.features.sources`，至少包含：`bow_string/finger_string/violin_grip/bow_grip`。
- **验收**：在 rosbag 中可见 `/violin/tactile/features`，其 schema 与 `contracts/data_schema/episode_schema.md` 一致。

### FR-004 弓-弦闭环门控（System0）
- **需求**：当 `bow_string` 的 `slip_risk` 或 `hf_energy` 进入异常区间时，系统应触发减速/减力/恢复动作。
- **验收**：在注入 slip 的场景下（mock 或真机），`command` 相对 `command_raw` 出现可解释的门控与模式切换，并有日志记录。

### FR-005 持琴安全优先门禁
- **需求**：当 `violin_grip` 的滑移风险超过阈值且不可恢复时，系统必须以“保琴”为最高优先级进入安全态（recovery 或 e-stop）。
- **验收**：人为触发/模拟 `violin_grip` 高风险时，`/violin/control/emergency_stop` 被 latched，并能执行 `reset_safe_pose`（若实现）。

### FR-006 接触相位控制模式切换
- **需求**：系统应至少支持 `free_space / in_contact / recovery / e_stop` 四种 `control_mode`，并在接触相位切换到低刚度/高阻尼策略（见 `contracts/safety/limits.md`）。
- **验收**：在 rosbag 中能看到控制模式切换；`e_stop` 时弓速/弓压归零。

### FR-007 端到端可复盘数据链路
- **需求**：系统应能录制最小 topics 切片并登记外部资产指针（pointers-only）。
- **验收**：按 `docs/engineering/data_capture.md` 录制后，能在 `data_registry/external_artifacts.md` 找到登记条目。

### FR-008 评测报告生成
- **需求**：系统应能输出 SR/IR/ER 与失败分布（至少 v0 先支持手工标注 + 汇总）。
- **验收**：存在可复现的评测产物（报告/脚本输入输出口径见 `docs/engineering/evaluation_protocol.md`）。

---

## 7) 非功能需求（NFR）

### NFR-001 安全（必选）
- **需求**：在人机共处场景下，碰撞/夹伤/掉落风险必须在 **X ms** 内进入安全状态（减速/停止/撤离/e-stop）。
- **验收**：按 `docs/engineering/demo_safety.md` 执行 checklist，触发条件能正确进入安全态。

### NFR-002 可观测性（可除错）
- **需求**：系统必须记录：触觉/音频特征、控制模式、门控原因、e-stop 触发原因与时间戳。
- **验收**：rosbag + 日志可重建关键事件时间线。

### NFR-003 可重现性
- **需求**：相同 demo profile + 相同校准版本下，关键指标波动应可控（用 SR/IR/ER 与触觉/音频统计量表达）。
- **验收**：A/B 交替评测（`evaluation_protocol.md`）能复现趋势，且报告版本信息齐全。

### NFR-004 可维护性（pointers-only）
- **需求**：大文件不得入库；所有外部资产必须登记指针与元信息。
- **验收**：仓库内无大体积 raw 数据；`data_registry/` 有完整登记。

---

## 8) 风险与降级策略（Cut line）

> 你选择了最高难度（持琴+持弓 + 端到端 + 1–2 分钟展示级），必须写明触发条件与最晚决策点。

- **R1 持琴不稳（掉琴风险）**
  - 触发：`violin_grip` 滑移不可恢复或 e-stop 频繁
  - 降级：允许引入辅助支撑/半固定（若允许）或缩短演示时长

- **R2 音频端到端不稳**
  - 触发：音频解析导致节奏/音高偏差持续累积
  - 降级：先只支持 `MusicXML/MIDI` 输入

- **R3 1–2 分钟稳定性不足**
  - 触发：SR 低、IR 高、失败分布集中在某类不可恢复失败
  - 降级：缩短曲目、降低速度、减少弓法种类、减少换弦频次

---

## 9) 里程碑（Milestones）与交付物

- **M0 安全与可重复（不掉琴、不伤人）**
  - 交付物：安全门禁、e-stop、复位流程、基础握持

- **M1 可稳定出声（空弦/单音）**
  - 交付物：弓-弦接触闭环最小实现（减速/减力/恢复）

- **M2 可辨识旋律（含换弦/简单连弓）**
  - 交付物：技能编排与恢复策略可用；评测报告能反映失败分布

- **M3 1–2 分钟展示级端到端**
  - 交付物：端到端输入→演奏；SR/IR/ER 报告（含样本量与置信区间）

---

## 10) 任务清单（工程执行）

- 见 `TASK_LIST.md`（按 M0–M3 拆分，含 DoD、对应 ROS2 包与 contracts）。

---

## 11) 未决问题（Open questions）

- Demo 曲目：哪一首/哪一段？是否允许简化/改编？
- 琴/弓可否改造：是否允许加标记、保护套、触觉贴片或振动传感器？
- 是否可用外部感测：外部相机/动捕/麦克风阵列？
- 安全要求：是否必须围栏/安全员/物理急停按钮？
- 目标数值：上述 X/N 等阈值由硬件额定参数与试运行统计确定（写入 `configs/`）。
