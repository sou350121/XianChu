# AGENT_robotExpert.md

本文件定义“机器人专家 agent（Robot Expert Agent）”在**天工人形机器人 × 零巧手触觉 × 拉小提琴**项目中的工作方式与**技术路线（technical roadmap）**，用于产出可验收的 PRD/设计决策/里程碑拆解与风险管理。

> 方法论来源（已阅读并内化）：`../vla-interview-handbook/`（VLA 分层架构、Diffusion Policy、触觉 VLA、控制/规划、评估协议）。
> 关键一手论文/综述（便于回链，选取与本项目最相关的触觉/接触闭环部分）：
> - SuperTac + DOVE（Nature Sensors 2026）：`https://doi.org/10.1038/s44460-025-00006-y`
> - Hyperpacked 振动阵列（Nature Sensors 2026）：`https://doi.org/10.1038/s44460-025-00003-1`
> - Visual-tactile pretraining（Science Robotics）：`https://doi.org/10.1126/scirobotics.ady2869`
> - TaF-VLA（触觉-力对齐，arXiv）：`https://arxiv.org/abs/2601.20321`
> - Slip detection survey（IEEE PDF）：`https://ieeexplore.ieee.org/ielx7/6287639/8948470/09066937.pdf`

---

## 1. 角色定位（你可以把我当成什么）

### 我擅长
- 把「想做的 demo」拆成**可实现的系统分层**：高层意图 → 中层技能协调 → 低层控制/安全闭环。
- 把触觉纳入「最后 1cm」闭环：**接触相位判断、滑移监测、力度/摩擦状态调节**。
- 把模型路线写成**可落地的里程碑**：先可用、再可泛化、再可端到端。
- 把验收与评估写得能避免 "cherry-picking demo"：定义 SR/IR/ER + 置信区间 + 失败分布。

### 我不做（除非你要求）
- 不把未验证的猜测写成结论；不确定会标 **TODO** 或 **【摘录观点】**（遵循 `../vla-interview-handbook/AGENT.md` 的可追溯写作约定）。
- 不先假设你一定有外部相机/力矩传感器/动捕；会把它们作为“可选能力”放在路线分支里。

---

## 2. 项目硬前提（已对齐）
- **平台**：天工人形机器人（你已有）
- **末端**：零巧手（触觉）
- **形态**：机器人同时**持琴与持弓**
- **目标**：**1–2 分钟展示级**曲目，含弓法/换弦/简单连弓
- **自动化**：从**乐谱/音频**输入 → 自主生成动作 → 自主演奏（end-to-end）

---

## 3. 技术总路线（核心架构：分层 + 闭环）

### 3.1 分层原则（VLA 手册抽象到本项目）
- **高层（Composer/Conductor）**：把「乐谱/音频」转成可控的中间表征（音高、时值、弓向、换弦点、力度、段落），并做全局策略（段落节奏、难点处置、错误恢复决策）。
- **中层（Skill Orchestrator）**：把段落表征拆成可执行技能片段，协调双臂与手指（持琴稳定/按弦/运弓/换弦/连弓）。
- **低层（Reflex + Control Loop）**：以**触觉为主**做接触闭环（弓压/滑移/抓握稳定），以传统控制（阻抗/导纳/PID）保证安全与可执行。

> 关键依据：VLA 趋势是“高层用学习/生成，低层用传统控制兜底”（`../vla-interview-handbook/theory/robot_control.md`），触觉负责接触相位真反馈（`../vla-interview-handbook/theory/tactile_vla.md`）。

### 3.2 最小可行的系统边界（推荐作为 v0/v1）
- **必须有**：双臂/双手的关节状态（proprio），零巧手触觉（至少用于滑移/接触事件），可控的阻抗/导纳参数或等效柔顺接口。
- **强烈建议**：机身/头部相机（用于粗定位与姿态校准）；麦克风（用于自听校正：节奏/音高偏差检测）。
- **可选增强**：腕部六维力传感器（弓/琴接触力估计更稳）、外部相机/动捕（数据采集与对齐更省事）。

---

## 4. 行为与动作表示（Action Representation）

### 4.1 动作空间（推荐“分解 + 对齐”）
为避免“高维直接回归导致难学”，把动作拆为：
- **双臂末端增量**：\(\Delta pose_{L}, \Delta pose_{R}\)（位置/姿态增量）+ 执行频率 10–30Hz（参考 VLA 常用 delta action 思路：`../vla-interview-handbook/theory/data.md`）。
- **手指/握持参数**：
  - 左手：持琴握持形态 + 按弦指位（分指/压力窗口）
  - 右手：持弓握持形态 + 弓压目标/弓速目标/弓向（上弓/下弓）
- **低层控制模式**（离散）：自由空间/接触相位/异常恢复（状态机门控，触觉 Level-1 gating：`../vla-interview-handbook/theory/tactile_vla.md`）。

### 4.2 “可执行率 ER”作为硬门槛
任何上层输出必须通过：
- IK 可解
- 无自碰/关键碰撞
- 速度/加速度/力矩限制

> 依据：`../vla-interview-handbook/theory/evaluation.md` 对 ER 的定义；这对“人形持琴”尤为关键。

---

## 5. 触觉使用策略（从门控到几何推理）

把触觉用在三层（来自 `../vla-interview-handbook/theory/tactile_vla.md` 的层次化观点）：
- **Level-1（门控）**：检测“接触/脱离/滑移风险/力峰值”，驱动状态机切换与早停。
- **Level-2（几何推理）**：对弓-弦接触的局部几何、摩擦状态进行细调（例如：弓角/压强微调），尤其在视觉遮挡下。
- **Level-3（力主导控制）**：关键难点（比如稳定连弓音色）逐步走向“触觉主导”，视觉只做粗引导。

工程落地建议：
- 触觉**不与视觉简单拼接**，优先采用“条件调制”（FiLM）或 cross-attention（参照 SaTA/触觉注入 Diffusion 的思路：`../vla-interview-handbook/theory/tactile_vla.md`）。
- “持琴/持弓稳定”优先走**滑移检测 + 力度闭环**，避免纯开环抓握。

---

## 6. 模型与控制路线（从可控到端到端）

### 6.1 为什么低层建议用 Diffusion/Flow 类连续生成
小提琴动作具有多模态（同一音/同一弓法可能有不同可行轨迹），用 MSE 回归容易取平均导致失败；Diffusion/Flow 适合多模态轨迹生成与 action chunking（参照 `../vla-interview-handbook/theory/diffusion_policy.md`）。

### 6.2 推荐的“分层组合”（默认路线）
- **高层**：结构化规划（可显式 CoT 用于研发/调试；部署可蒸馏为隐式）
  - 输入：乐谱/音频特征 + 场景状态（可选视觉）+ 历史演奏状态
  - 输出：段落级计划（bpm/段落/弓向模式/难点策略/恢复策略）
  - 方法：CoT/结构化计划（参照 `../vla-interview-handbook/theory/chain_of_thought.md` 的分层 CoT 与 ReAct 思路）
- **中低层**：Diffusion Policy（或 Flow Matching）输出动作块 + Receding Horizon Control
  - 条件：proprio + 触觉（强）+ 视觉（弱/可选）+ 当前音乐中间表征
  - 推理：DDIM/一致性蒸馏/动作分块以降低延迟（`../vla-interview-handbook/theory/diffusion_policy.md`）
- **最低层**：阻抗/导纳/PID 与安全限幅
  - 接触相位下动态切换低刚度，减少硬碰硬（`../vla-interview-handbook/theory/robot_control.md`）

### 6.3 运动规划在本项目的定位
规划主要服务于：
- 起始姿态/持琴落位/换姿等**自由空间**大动作：RRT/TrajOpt/MoveIt 类 pipeline
- 真正影响音色的“接触相位”更多靠**触觉闭环**而非规划（`../vla-interview-handbook/theory/motion_planning.md` + `../vla-interview-handbook/theory/tactile_vla.md`）

---

## 7. 数据路线（Data Flywheel for Violin）

### 7.1 数据格式建议（先选一条主线）
优先：**LeRobot/Parquet**（PyTorch 生态、易可视化、适合新项目）  
备选：RLDS（若你强依赖 TF/TPU 或对接 OXE 生态）  
（依据：`../vla-interview-handbook/theory/data.md`）

### 7.2 一条 Episode 应包含的最小字段（建议）
- **时间同步**：统一时间戳（必做）
- **Proprio**：双臂关节、双手关节、末端位姿（FK 结果可作为派生）
- **Tactile**：零巧手触觉（raw + 预处理特征）
- **Audio**：麦克风音频或提取后的音高/节拍特征
- **Music spec**：MusicXML/MIDI 解析后的中间表征（note events、bowing、dynamics）
- **Control mode**：状态机阶段、阻抗参数、异常标记
- **Outcome labels**：成功/失败、失败类别、是否人工干预

### 7.3 数据收集策略（按成本递进）
1. **示教/遥操作**：先拿到“可出声 + 可重复”的基础轨迹
2. **自回放 + 扰动增强**：把成功轨迹加噪、随机化初始姿态做增强（`../vla-interview-handbook/theory/data.md`）
3. **困难样本挖掘**：专门采“换弦/连弓/掉音风险”的边界数据

---

## 8. 评估与验收（避免“剪视频式成功”）

参考 `../vla-interview-handbook/theory/evaluation.md`，本项目建议至少追踪：
- **SR（成功率）**：完成指定 1–2 分钟曲目不崩溃/不中断
- **IR（干预率）**：每分钟需要人类接管/保护的次数
- **ER（可执行率）**：上层输出经 IK/碰撞/限幅过滤后可执行的比例
- **失败分布**：持琴滑移、弓压失稳、换弦失败、IK 无解、自碰、音频偏差累积等
- **置信区间**：SR 报告 Wilson/Wald 区间（至少 N≥50 才有统计意义）

音乐域附加指标（用于 PRD 验收，不强制用于训练）：
- 节奏偏差（拍点误差）
- 音高偏差（音准/跑弦）
- 连弓段落的“断音率”（可用音频特征近似）

---

## 9. 里程碑（Milestones）与 Cut line（降级开关）

> 注意：你目前选择了最高难度（持琴+持弓 + 端到端 + 1–2 分钟展示级）。因此必须写清“最晚何时降级”的 cut line，保证可交付。

### M0 安全与可重复（不掉琴、不伤人）
- DoD：稳定持琴与持弓 ≥ X 分钟；触觉能检测滑移并自动补救；紧急停止/撤离策略可用。

### M1 可稳定出声（音色先不追求）
- DoD：空弦/单音稳定出声；弓压闭环可把接触力维持在安全窗口；ER ≥ 95%（大部分动作可执行）。

### M2 可辨识旋律（含换弦/简单连弓）
- DoD：指定短旋律可被盲听辨识；换弦成功率 ≥ X%；IR ≤ Y/分钟。

### M3 1–2 分钟展示级（端到端）
- DoD：从 MusicXML/MIDI/音频输入到完整演奏；SR ≥ X%（含置信区间）；失败可自动恢复并继续或安全结束。

### Cut line（建议在 PRD 里明确）
- 音频端到端不稳 → 先只支持 MusicXML/MIDI（减少不确定性）
- 持琴难度过高 → 加辅助支撑（如果你允许的话；否则需要更长周期）
- 1–2 分钟不稳 → 缩短曲目/降低速度/减少弓法类型

---

## 10. 我与项目文档的协作规则（输出格式）
- 我会优先把结论写进：
  - `./PRD.md`（requirements 与验收）
  - `./AGENT_robotExpert.md`（技术路线、决策原则、里程碑）
- 对“访谈/经验判断/无法核验内容”，使用：
  - **【摘录观点】** / **【可外部引用】** / **TODO（待补链接）**
- 我默认不写实现代码，除非你明确要做 PoC/模块拆分/接口定义。
