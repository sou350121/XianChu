# XianChu（弦触）DESIGN：基于触觉反馈的小提琴人形机器人系统设计（Violin-Playing Humanoid with Tactile Feedback）

> 目标：把“小提琴演奏”当作 **contact-rich 双手协作** 系统工程问题来做。
> 核心打法（来自 `vla-interview-handbook`，关键一手论文/综述见文末 References）：**高层负责意图与协作编排；低层用可解释的触觉/力控闭环（System0/反射层）兜底**。

## 0. TL;DR（一句话可复述）
- **弓-弦是核心难点**，本质是控制 stick-slip（近似 Helmholtz 运动）并抑制啸叫/噪声：
  - 直接可控量：**弓压（法向力）**、**弓速（切向速度）**、**落弓点（距琴码）**、**弓毛倾角/弓杆姿态**。
  - 快反馈：触觉/振动（微滑、高频能量）
  - 慢反馈：音频（f0、谐波结构、噪声比）
- 系统必须是**多速率闭环**：
  - **≥1kHz（反射/安全/限幅）**：防撞、防掉琴、触觉失联门禁、接触相位阻抗切换。
  - **200–500Hz（接触执行层）**：弓-弦法向力控 + 切向速度控 + 小姿态微调。
  - **10–30Hz（策略/技能）**：从 `music_spec` 产出动作块与模式切换（free_space/contact/recovery）。

---

## 1. 系统分层与包边界（与 `ws/src/*` 对齐）

### 1.1 三层控制栈（S2/S1/S0）
- **S2：Composer/Conductor（慢，~1–5Hz）**
  - 输入：曲目、段落结构、难点策略、失败恢复策略
  - 输出：段落级计划（tempo、bowing 模式、换弦点、允许的降级策略）
  - 位置：文档先定义，不要求 v0 上线

- **S1：Skill Orchestrator / Policy（中，10–30Hz）**
  - 输入：`music_spec` 当前事件 + 状态（proprio/触觉特征/音频特征）
  - 输出：动作块（action chunk）：`ee_delta` + `hand_params` + `control_mode`
  - 对应包：`ws/src/violin_robot_policy`（可替换：Diffusion/Flow/规则策略）

- **S0：Reflex + Real-time Control（快，≥1kHz）**
  - 输入：高频 proprio、触觉/力/振动特征（可能是 50–200Hz 但需时间戳对齐）
  - 输出：硬件可执行命令（关节/末端），并执行限幅/急停/撤离
  - 对应包：`ws/src/violin_robot_control` + 驱动/硬件接口在 `ws/src/violin_robot_drivers`

### 1.2 v0 最薄切片（你当前 repo 已具备雏形）
- `violin_robot_music_spec` 发布 `/violin/music/spec`
- `violin_robot_policy` 发布 `/violin/control/command_raw`
- `violin_robot_control`（当前已有 `tactile_guard_node`）门控后发布 `/violin/control/command` 与 `/violin/control/emergency_stop`
- `violin_robot_drivers` 发布 state/tactile/audio 特征
- `violin_robot_bringup` 用 launch 编排启动

> 重要：v0 允许 JSON topic（`std_msgs/String(JSON)`）先跑通链路，但 **contracts 是 SSOT**，字段/单位/频率必须可追溯。

---

## 2. 四类关键接触对：观测 → 特征 → 控制 → 失败模式 → 恢复

下面每类都用同一模板写清“闭环长什么样”，并把 **可观测与可验收** 做成第一等公民。

### 2.1 接触对 A：弓-弦（Bow-String Friction Control）——系统第一优先级

#### A1. 目标（控制层可量化）
- **稳定出声**：连续段落内“非预期噪声/啸叫”占比低
- **stick-slip 稳定**：避免持续 slip（无声/尖噪）与过度 stick（卡顿/断音）

#### A2. 观测（优先级从必需到可选）
- **必需**
  - 右臂/右手 proprio：关节状态、右端执行器位姿（`right_ee`）
  - **音频特征（自听）**：`f0_hz`、能量、噪声比（或谱相关代理）
  - 触觉特征（至少门控级）：`contact_event`、`slip_risk`、`contact_phase`

- **强烈建议（弓-弦最值钱的增强）**
  - **腕部 6D F/T 或等效力估计**：得到弓压法向力的近似（`bow_normal_force_est`）
  - **弓杆振动/加速度计或高带宽振动阵列**：用 `hf_energy` 作为微滑/啸叫前兆

- **可选**
  - 外部相机/头相机：用于粗定位与落弓点标定（接触相位不强依赖视觉）

#### A3. 特征（建议先规则 teacher，后学习 student）
- **接触几何代理**：落弓点（近似）、弓毛倾角、弓速方向
- **摩擦状态代理**：
  - `hf_energy_band`：例如 80–5000Hz（或按硬件采样率裁剪）的能量
  - `contact_mode`：`stick / preslip / slip`（由状态机产生）
- **音频质量代理（慢反馈）**：
  - `f0_hz` 与目标音高一致性
  - `harmonic_to_noise_ratio`（可先用简化的“高频噪声能量占比”替代）

#### A4. 控制结构（多速率 + 混合力位）
- **控制变量（推荐最小集合）**
  - 法向：目标弓压 `F_n*`（或等效的“压入量/位置偏置”）
  - 切向：目标弓速 `v_t*`
  - 姿态：弓毛倾角 `theta_tilt*`（影响接触面积与力分布）
  - 位置：落弓点 `x_contact*`（距琴码；先做粗控制）

- **执行层（200–500Hz）**
  - **n 轴（法向）**：力控/导纳（用 `F_n` 误差调节微位移）
  - **t 轴（切向）**：速度控（跟随 `v_t*`），并对 `slip_risk` 做限速/加阻尼
  - 姿态微调：当 `hf_energy` 上升时，降低 `F_n`/降低 `v_t`/微调倾角以回到稳定窗口

- **反射层（≥1kHz）**
  - 触觉失联/状态异常 → 进入 `recovery` 或 `e_stop`
  - 力矩/速度/加速度硬限幅（见 `contracts/safety/limits.md`）

#### A5. 失败模式字典（必须可记录、可复盘）
- `no_sound`：接触但音频能量/谐波不足（空擦/滑过）
- `squeal`：高频噪声能量占比持续高（啸叫/刺耳）
- `chatter`：断续 stick-slip（听感“抖/断”），可能来自姿态/弓压波动
- `string_miss`：换弦失败/落弓偏离（弓落到相邻弦或空中）

#### A6. 恢复动作（Recovery primitives）
- `recover_bow_contact`：减速 → 退弓 → 低力重新接触 → 再加力
- `recover_squeal`：降低 `F_n` 与 `v_t`，调整倾角与落弓点（更远离琴码），再渐进回目标
- `recover_string_plan`：若 string miss，撤离到安全姿态，再按 `string_plan` 重新落弓

---

### 2.2 接触对 B：按弦指尖-弦/指板（Finger-String / Fingerboard）

#### B1. 目标
- **音高正确**：`f0_hz` 接近目标音高（允许 vibrato/短时误差）
- **压弦稳定**：避免“半按（打品/嘶嘶）”或“过力损伤/卡顿”

#### B2. 观测
- 左手手指 proprio +（若有）指尖触觉特征（接触面积、法向代理、滑移风险）
- 音频特征（`f0_hz`）作为最终闭环校验（慢反馈）

#### B3. 控制
- 快环（200–500Hz）：压弦法向力窗口控制（触觉/effort 代理）
- 慢环（10–30Hz）：根据 `f0` 偏差做指位微调（沿弦方向）

#### B4. 失败模式
- `pitch_low/high`：按弦位置偏差或弦张力/环境漂移
- `buzzing`：压弦不足或接触不稳（可用音频噪声代理 + 触觉微滑代理）

#### B5. 恢复
- `recover_finger_press`：短撤离→重新压弦→逐步加力
- `recover_pitch`：以 `f0` 误差驱动的“微扫寻优”（小步搜索）

---

### 2.3 接触对 C：左手持琴（虎口/拇指-琴颈）

#### C1. 目标
- 不掉琴、不滑移；姿态稳定（便于按弦与换把）

#### C2. 观测与特征
- 手掌/拇指触觉：接触面积、CoP 速度、`slip_risk`
- effort 代理：电流/扭矩异常作为卡死/碰撞代理

#### C3. 控制
- 反射层：`slip_risk` 上升 → 增加握持法向/调整接触斑（先“铺开面积”再加力）
- 恢复：撤离到安全姿态（避免摔琴）

---

### 2.4 接触对 D：右手握弓（弓蛙/拇指-手指）

#### D1. 目标
- 握持稳定、力传递一致（避免手内滑移导致弓压/弓速不可控）

#### D2. 观测与特征
- 触觉：握持接触面积/CoP 漂移 + 高频微滑特征

#### D3. 控制
- 快环：防滑移（加握持/降弓压/限速）
- 慢环：握持姿态重整（回到 reference grasp）

---

## 3. 触觉/振动特征流水线（从 raw 到可闭环信号）

> 参考（工程口径总览）：`../vla-interview-handbook/deployment/perception/tactile_array_algorithms_capacitive_piezoresistive.md`
> - Slip detection survey（IEEE）：`https://ieeexplore.ieee.org/ielx7/6287639/8948470/09066937.pdf`
> - 摩擦/预滑综述（review）：`https://www.semanticscholar.org/paper/Tactile-Sensors-for-Friction-Estimation-and-Slip-A-Chen-Khamis/6dd6bd45f8de70e69fcae1aa0e658d2685997615`
> - NIST（触觉滑移检测标定与分析）：`https://www.nist.gov/publications/calibration-and-analysis-tactile-sensors-slip-detectors`
> - DIGIT（Lambeta et al., 2020）：`https://arxiv.org/abs/2005.14679`

### 3.1 统一输出（写入 `obs.tactile.features`）
每个接触源（建议至少 4 个：`bow_string`, `finger_string`, `violin_grip`, `bow_grip`）输出：
- `contact_event: bool`
- `contact_phase: enum`（`free_space / pre_contact / in_contact / recovery / e_stop`）
- `slip_risk: float[0,1]`
- `contact_mode: enum`（`stick / preslip / slip / unknown`）
- `hf_energy: float`（或 `hf_energy_band: {low, mid, high}`）
- （可选）`area`, `cop`, `cop_vel`

### 3.2 规则 teacher（v0 就能上）
- 基线/增益：无接触时 EMA 更新；接触时冻结基线
- slow/fast 分离：`p_slow = LPF(p)`；`p_fast = p - p_slow`
- 高频能量：`hf_energy = sum(BPF(p_fast)^2)`
- 状态机：`no_contact -> stick -> preslip -> slip`（带滞回与 debounce）

### 3.3 学习 student（v1+）
- 输入：短时窗（0.2–0.5s）的 raw/特征图序列
- 标签：teacher 输出 + 少量人工校验
- 验收：绑定闭环 KPI（掉琴率、啸叫占比、恢复成功率、延迟分布），不看纯离线准确率

---

## 4. 坐标系与标定（tf + 校准资产）

### 4.1 必须明确的轴向（为了“混合力位控制”可复现）
在 `contracts/frames/tf_tree.md` 中补齐：
- `violin_body`：琴体参考
- `string_g/d/a/e` 或 `string_plane`
- `bow_hair`：弓毛接触参考

建议额外定义一个派生 frame（可在控制节点内部算，不一定发布 tf）：
- `bow_string_contact`：
  - `t_hat`：运弓切向（沿弓速方向）
  - `n_hat`：弓压法向（压向弦）
  - `s_hat`：沿弦方向（用于按弦/落点参考）

> 这样才能把控制写成：“n 轴力控 + t 轴速度控 + 姿态微调”，而不是含糊的 world XYZ。

### 4.2 v0 标定最小流程（写入 `calibration/`）
- 静态标定：
  - 机器人 `base_link` 到 `violin_body` 的相对位姿（持琴姿态下）
  - `bow_body/bow_hair` 相对 `right_ee`
- 运行时校验：
  - 触觉/音频通道在线（心跳）
  - 轻触弦（低力）确认接触事件与音频能量响应

---

## 5. 安全与乐器保护（必须 contract-first）

在 `contracts/safety/limits.md` 增加乐器保护软限位（示例字段，数值 TBD）：
- **接触相位速度上限**：运弓/按弦/换弦时不同上限
- **弓压上限**：超限 → 立即减力/撤离
- **触觉/状态失联超时**：视为高危 → `e_stop` 或 `reset_safe_pose`
- **掉落风险门禁**：`slip_risk` 持续高且不可恢复 → 进入安全姿态，优先“保琴”

> 现有 `tactile_guard_node` 已有“slip_risk 触发减速/急停”的雏形；后续应升级为“多接触源 + 超时 + 乐器保护策略”。

---

## 6. 数据记录与评测（避免剪视频式成功）

### 6.1 最小指标（与 `docs/engineering/evaluation_protocol.md` 对齐）
- **SR**：完整跑完指定片段/曲目且不中断
- **IR**：每分钟人工接管次数
- **ER**：上层输出通过 IK/碰撞/限幅后可执行比例
- **失败分布**：`drop_risk/excessive_force/ik_fail/collision/no_sound/squeal/unstable_audio/...`

### 6.2 音乐域可复现指标（先定义口径，不追求专业音色评测）
- **节奏偏差**：onset 误差（ms）分布
- **音高偏差**：`|f0 - f0_target|` 的统计量
- **噪声占比**：高频噪声能量占比（啸叫/擦弦的代理）

### 6.3 触觉域可复现指标
- `time_in_contact_window`：弓压/滑移风险在安全窗口内的占比
- `preslip_events_per_min`、`slip_events_per_min`
- `recovery_success_rate`

---

## 7. 实施路线（与 PRD Milestones 对齐）
- **M0（安全）**：持琴/持弓稳定 + 触觉门控/急停可靠
- **M1（出声）**：空弦单音稳定出声（弓-弦闭环跑通）
- **M2（可辨识旋律）**：含换弦/简单连弓（恢复策略开始发挥作用）
- **M3（1–2 分钟展示级端到端）**：从 `music_spec` 到完整演奏；报告 SR/IR/ER 与置信区间

---

## 8. 关联 SSOT
- PRD：`./PRD.md`
- 技术路线：`./AGENT_robotExpert.md`
- 契约：`./contracts/`
- runbooks：`./docs/engineering/`

---

## 9. References（来自 vla-handbook 的一手论文/综述）

> 这些链接对应 `vla-interview-handbook` 中已沉淀的笔记/工程口径，便于“方案 → 一手来源”回链。

- SuperTac + DOVE（Nature Sensors 2026）：`https://doi.org/10.1038/s44460-025-00006-y`
  - 代码：`https://github.com/wut19/DOVE`
- Hyperpacked 自供能电容振动阵列（Nature Sensors 2026）：`https://doi.org/10.1038/s44460-025-00003-1`
- Visual-tactile pretraining & online multitask learning（Science Robotics）：`https://doi.org/10.1126/scirobotics.ady2869`
  - Focus：`https://doi.org/10.1126/scirobotics.aee5782`
  - 数据/代码（Zenodo）：`https://doi.org/10.5281/zenodo.17986310`
- TaF-VLA（触觉-力对齐，arXiv 2601.20321）：`https://arxiv.org/abs/2601.20321`
- Slip detection survey（IEEE PDF）：`https://ieeexplore.ieee.org/ielx7/6287639/8948470/09066937.pdf`
- DIGIT（Lambeta et al., 2020，arXiv）：`https://arxiv.org/abs/2005.14679`
- 预滑/摩擦综述线索（review）：`https://www.semanticscholar.org/paper/Tactile-Sensors-for-Friction-Estimation-and-Slip-A-Chen-Khamis/6dd6bd45f8de70e69fcae1aa0e658d2685997615`
- NIST（触觉滑移检测标定与分析）：`https://www.nist.gov/publications/calibration-and-analysis-tactile-sensors-slip-detectors`
