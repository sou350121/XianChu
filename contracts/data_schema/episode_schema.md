# Episode Schema（数据与时间同步 SSOT）

本文件定义**一条示教/回放/评测 episode** 的最小字段集合与时间同步规则。

- **目标**：支持「触觉闭环 + 音频自听 + 双臂协同」的可复现训练与评测。
- **原则**：以控制命令为主时钟；所有多模态数据必须可对齐、可复盘。

## 1) 时间戳与同步规则（强制）

- 全部数据必须包含 `timestamp`（推荐：ROS time）
  - 标准 ROS 消息：用 `header.stamp`
  - JSON：必须显式携带 `timestamp`（秒）
- 录制时必须保存以下元数据：
  - 控制频率（Hz）与实际周期统计（P50/P95 jitter）
  - 关节/触觉/音频的实际采样频率与丢帧率
  - 时钟源（sim/real、`use_sim_time`）

**对齐优先级（同一时间轴）**：
1. 控制命令（ground truth 时间轴）
2. 关节状态 / 末端状态（proprio/kinematics）
3. 触觉特征（tactile features）
4. 音频特征（audio features）

> 说明：触觉 raw/音频 raw 体积大且可能低频，但其特征必须与控制命令同轴对齐，否则无法做接触闭环复盘。

## 2) 数据落盘与仓库规则

- 推荐落盘格式：Parquet / RLDS / rosbag2（按工程阶段选择）
- 本 repo 采用 **pointers-only**：raw 触觉/音频/视频/模型权重不入库
  - 外部资产登记：`../../data_registry/external_artifacts.md`

## 3) 最小字段（v0）

### 3.1 Observation（观测）

#### 3.1.1 Proprio / Kinematics
- `obs.proprio.joint_state`
  - 双臂 + 双手：`position(rad) / velocity(rad/s) / effort(N·m 或硬件单位)`
- `obs.kinematics.ee_pose`
  - `left/right` 末端位姿（推荐 frame：`base_link`）

#### 3.1.2 Tactile（触觉）
- `obs.tactile.raw`（可选）
  - raw 图像/阵列/marker 等（体积大，建议外部存储）

- `obs.tactile.features`（必需，v0 即要可复盘）
  - `sources`: 映射（key 为接触源名称）
    - 建议最小接触源集合：
      - `bow_string`：弓-弦接触
      - `finger_string`：按弦指尖-弦/指板
      - `violin_grip`：左手持琴（虎口/拇指-琴颈）
      - `bow_grip`：右手握弓

每个 source 的最小字段（全部可缺省为 unknown，但建议尽快补齐）：
- `contact_event: bool`
- `contact_phase: enum`
  - `free_space | pre_contact | in_contact | recovery | e_stop | unknown`
- `slip_risk: float`（范围 [0,1]；无数据则填 0 并标注 `valid=false`）
- `contact_mode: enum`（建议由状态机输出）
  - `stick | preslip | slip | unknown`
- `hf_energy: float`（高频能量代理；无振动通道时可缺省）
- （可选但很常用）`area`, `cop`, `cop_vel`

弓-弦相关的可选增强字段（建议写入 `bow_string`）：
- `bow_normal_force_est: float`（N，来自腕 F/T 或估计器）
- `bow_speed_est: float`（归一化或 m/s，需在实现侧明确）
- `bow_contact_point_est_mm: float`（距琴码 mm，粗估即可）

#### 3.1.3 Audio（音频自听）
- `obs.audio.features`（必需）
  - `f0_hz: float`（音高估计；无声/不可信可用 0 或 NaN，但需有 `valid` 标记）
  - `bpm_est: float`（可选）
  - `beat_phase: float`（可选）
  - `energy: float`（归一化或 dBFS，需在实现侧明确）
  - （可选）`noise_ratio: float`（高频噪声占比 / HNR 代理）

#### 3.1.4 Music spec（目标事件）
- `obs.music_spec`
  - 当前目标 `event_id`（或直接嵌入该事件结构）
  - schema 见：`music_spec.md`

### 3.2 Action（动作）

- `action.ee_delta`
  - `left/right`: `Δpos(m)` + `Δrot(rad)`（推荐 axis-angle 或 rpy 增量，但需在实现侧固定一种）

- `action.hand_params`（允许不同策略实现不同子集，但字段语义要稳定）
  - 右手（弓相关）：
    - `bow_speed`（归一化或 m/s）
    - `bow_force`（N，或等效压入量）
    - `bow_direction`（`up/down/stop`）
    - （可选）`bow_tilt_rad`、`bow_contact_point_mm`
  - 左手（按弦/持琴）：
    - `finger_press`（归一化或 N）
    - `finger_target`（例如弦上位置/把位提示）
    - `grip_strength`（持琴握持强度，归一化）

- `action.control_mode`
  - `free_space | in_contact | recovery | e_stop`

### 3.3 Outcome（结果与标签）

- `outcome.success: bool`
- `outcome.failure_mode: enum`
  - `drop_risk | excessive_force | ik_fail | collision | no_sound | squeal | string_miss | buzzing | unstable_audio | other`
- `outcome.interventions`
  - 人工接管计数与时间戳列表

## 4) 关联规范
- 坐标系：`../frames/tf_tree.md`
- 安全/限位：`../safety/limits.md`
- 音乐中间表征：`./music_spec.md`
