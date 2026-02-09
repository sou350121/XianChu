# XianChu（弦触）：触觉与声音的二重奏（天工 × 零巧手）

> 这里既是一个作品的工作台，也是本项目的工程仓库（ROS 2）与单一真实来源（SSOT）。

```
   ┌──────────┐                 ┌───────────┐
   │  Robot   │─── bow / grip ─▶│  XianChu  │
   │  hands   │                 │ strings   │
   └──────────┘                 └───────────┘
       ▲   │                         ▲
 touch │   │ proprio                 │ sound
       │   ▼                         │
       └────────── closed loop ──────┘
```

小提琴是个很“刁钻”的机器人任务：它不是拼力气，也不是拼精度标定，而是拼**接触**——弓毛和弦之间那点微妙的摩擦（stick-slip），决定了你听到的是旋律、噪声还是啸叫。

我们做这个项目的动机很简单也很好玩：
- **把“不可控的手感”变成可工程化的闭环**：让触觉/振动把“将要打滑/将要啸叫/力不在窗口里”提前变成信号。
- **把“演奏”拆成可复盘的系统工程**：每一次成功/翻车都能用 SR/IR/ER + 失败分布解释，而不是“剪视频式成功”。
- **做一个观众能秒懂的具身智能 demo**：机器人不是在“画轨迹”，而是在“听自己、摸自己、及时收手”。

## 从“作品”开始（更艺术、更少技术）

- **艺术声明 / 展演动机 / 三幕脚本**：`ARTISTIC_STATEMENT.md`

## 从“工程”开始（入口索引）

- **PRD（验收口径）**：`PRD.md`
- **DESIGN（系统设计）**：`DESIGN.md`
- **任务清单（按 M0–M3 执行）**：`TASK_LIST.md`
- **团队协作（角色/实习生配置）**：`TEAM.md`
- **技术路线与决策原则**：`AGENT_robotExpert.md`
- **PRD 对齐计划（索引）**：`PRD_PLAN.md`
- **工程 runbooks（构建/启动/录制/评测/现场安全）**：`docs/README.md`

<details>
<summary><strong>研究背景（工程版：像论文写作一样“句内引用”，默认折叠）</strong></summary>

弓弦系统的“好听/难听”，不是几何轨迹一个变量决定的，而是**摩擦学 + 波动传播 + 控制稳定性**的耦合结果。

在物理层面，理想的稳定发声常被描述为 *Helmholtz motion*：弦上存在一个“拐角/波前”周期性往返，带来相对稳定的基频与谐波结构；但这种状态对**弓速、弓压、落弓点（距琴码）**高度敏感。经典的可演奏区间分析将其形式化为 **Schelleng diagram**，用“允许区间”解释为什么“压太重会吵、压太轻会飘”（见 [Schelleng, 1973](https://pubs.aip.org/asa/jasa/article-abstract/53/1/26/627402/The-bowed-string-and-the-player)）。更进一步的弓弦动力学建模把“捕获-释放、非线性摩擦、音高下沉/不稳定”等机制写成可计算模型（参见 McIntyre & Woodhouse 的经典建模论文 PDF：[*Modelling the bowed string*](https://www.ioa.org.uk/system/files/publications/ME%20MCINTYRE%20J%20WOODHOUSE%20MODELLING%20THE%20BOWED%20STRING.pdf)），而更系统的教材级总结可参考 Fletcher & Rossing 的《The Physics of Musical Instruments》（[Fletcher & Rossing, 1998](https://link.springer.com/book/10.1007/978-0-387-21603-4)）。

对机器人来说，这意味着一个很反直觉的结论：**“轨迹对了就会响”是错的**。同一条几何轨迹，只要弓压/弓速/接触点有细小漂移，就可能跨出允许区间，触发双滑移、噪声、啸叫等失败模式（Schelleng, 1973；McIntyre & Woodhouse）。因此，小提琴把控制问题从“几何误差”升级成“接触状态误差”。

这也是为什么本项目把闭环设计为多模态、分层、可复盘：
- **触觉/振动是快反馈**：用于接触相位、预滑/滑移风险的早期预警与 200Hz–1kHz 级别门控（滑移检测综述可参考 [Romeo & Zollo, 2020](https://ieeexplore.ieee.org/ielx7/6287639/8948470/09066937.pdf)）。近两年更“系统工程化”的传感路线包括多模态触觉皮肤与触觉语言模型（例如 [SuperTac + DOVE, Nature Sensors 2026](https://doi.org/10.1038/s44460-025-00006-y)，并可回链到 [手册笔记](../vla-interview-handbook/deployment/perception/supertac_dove_biomimetic_multimodal_tactile_sensing.md)），以及覆盖 80–5,000 Hz 的高保真振动阵列（例如 [Hyperpacked, Nature Sensors 2026](https://doi.org/10.1038/s44460-025-00003-1)，并可回链到 [手册笔记](../vla-interview-handbook/deployment/perception/hyperpacked_piezocapacitive_vibration_sensor_array.md)）。
- **音频是慢反馈**：用于最终听感的校验与段落级自我修正；但音频存在估计窗长与延迟，因此更适合做“慢纠偏”，而不是替代接触快环。
- **VLA 更适合放在上层**：上层（乐谱→意图→动作编排）天然适合 VLA/生成式策略学习，例如开源 VLA 训练生态的代表作 [OpenVLA, 2024](https://arxiv.org/abs/2406.09246) 与跨机器人泛化的 [UniVLA, IJRR 2024](https://journals.sagepub.com/doi/full/10.1177/02783649241227559)（手册综述见 [Literature Review](../vla-interview-handbook/theory/literature_review.md)）。在动作表示与高频精度方面，近两年的一个关键工程问题是动作 token 化与效率，例如 [FAST, 2025](https://arxiv.org/abs/2501.09747)（见 [手册笔记](../vla-interview-handbook/theory/fast.md)）。
- **触觉要“对齐力语义”而不是“多一张图”**：近期工作已经把触觉从“视觉补充”推向“力语义通道”，例如 [TaF-VLA, 2026](https://arxiv.org/abs/2601.20321)（见 [手册笔记](../vla-interview-handbook/theory/frontier/taf_vla_tactile_force_alignment_2026.md)）与跨传感器可迁移力感知 [GenForce, Nat Commun 2026](https://doi.org/10.1038/s41467-026-68753-1)（见 [手册笔记](../vla-interview-handbook/theory/tactile/genforce_tactile_force_transfer_2026.md)）。同时，“低成本触觉也能显著增强遮挡下灵巧性”的范式例子是 Science Robotics 2026 的视触觉预训练 + 在线多任务学习（[DOI:10.1126/scirobotics.ady2869](https://doi.org/10.1126/scirobotics.ady2869)，见 [手册笔记](../vla-interview-handbook/theory/frontier/visual_tactile_pretraining_online_multitask_learning_2026.md)）。

最后，**可复盘**是把研究变成工程迭代的必要条件：我们将 SR/IR/ER 与失败分布写进 `PRD.md`，把“每次翻车到底是哪类接触相位问题”变成可定位、可对比、可回归的指标。更完整的一手论文/综述回链仍以 `DESIGN.md` 文末 References 为准。

</details>

## 工程目录（你应该从这里开始）

- **ROS 2 Workspace**：`ws/`
  - `ws/src/`：ROS 2 packages（包边界就是物理导轨）

- **SSOT：契约层**：`contracts/`
  - `contracts/ros_interfaces/`：topic/service/action 规范（人类可读）
  - `contracts/frames/`：坐标系与 tf 树规范（人类可读）
  - `contracts/data_schema/`：数据/episode schema 与时间同步规范
  - `contracts/safety/`：限位/速度/力矩/急停策略（默认约束）

- **SSOT：配置层**：`configs/`
  - `configs/hardware/`：硬件与序列号绑定配置（driver 参数）
  - `configs/control/`：控制器与限幅配置
  - `configs/demo/`：demo profile（场地/琴/弓/曲目）配置

- **SSOT：校准层**：`calibration/`
- **工程文档（runbooks）**：`docs/`（入口：`docs/README.md`）
- **工具与脚本**：`tools/`
- **数据注册表（pointers-only）**：`data_registry/`（只存外部位置/版本/统计，不存大文件）

## SSOT 规则（最小但强约束）

- **物理导轨**：新增功能必须放进正确的 ROS2 package；禁止“到处扔脚本”。
  - packages 的依赖关系以 `contracts/ros_interfaces/` 与包职责为准。
- **契约优先（contract-first）**：先改 `contracts/` 再改实现；实现必须对齐契约。
- **pointers-only**：视频/音频/触觉原始流/模型权重不入库，只登记在 `data_registry/`。

## GitHub 联动（索引层）

- Issue/PR 只做索引与协作，不承载正文；正文以本目录内 canonical 文档为准。
- Issue 必须链接到本目录内的 canonical 文档（PRD/设计/契约/决策/复盘）。
