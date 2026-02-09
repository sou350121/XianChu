# Safety Limits（安全约束 SSOT）

本文件定义默认安全约束与急停/异常处理策略。所有执行层（`violin_robot_control`）必须以此为准。

> TODO：等你补充天工/零巧手的接口与额定参数后，把数值从占位改成真实值，并绑定到 `configs/hardware/` 的硬件 SN 与配置版本。

## 1) 硬限位（Hard Limits）

硬限位来自厂商/URDF/驱动能力，**不得突破**：
- 关节位置：URDF 限位
- 关节速度：额定上限
- 关节力矩/电流：额定上限
- 通信与驱动故障：驱动返回 fault 必须立刻进入安全态

## 2) 软限位（Soft Limits，默认更保守）

软限位是 demo/研发默认启用的保守约束（可由 demo profile 覆盖）：
- `soft_joint_margin_ratio`：硬限位内留余量（建议 ≥ 5%）
- 末端速度上限（分相位）：
  - `ee_speed_free_space_max`
  - `ee_speed_in_contact_max`
- 末端角速度/加速度上限（分相位）

## 3) 接触相位控制门禁（关键）

- 接触相位必须切换到**低刚度/高阻尼**（阻抗/导纳参数以 `configs/control/` 为准）
- 触觉/力/振动的异常应触发：
  - **减速**（slowdown）
  - **进入恢复**（recovery primitive）
  - 或 **急停**（e-stop）

### 3.1 多接触源（multi-source）门禁规则（建议 v0 就实现）

以 `obs.tactile.features.sources` 为准（见 `contracts/data_schema/episode_schema.md`）。不同接触源的策略不同：

- `bow_string`（弓-弦）：优先“保音色/保稳定”，一般不直接 e-stop
  - `slip_risk` 高 → 限速、减弓压、进入 `recover_bow_contact`
  - `hf_energy` 持续高（啸叫代理）→ 降力/降速/调整倾角/落弓点

- `finger_string`（按弦）：优先“保琴/保手指机构”
  - `slip_risk` 高或 `contact_mode=slip` → 重新压弦/减速/微调位置
  - 压弦力超限（若可估）→ 立即减力并撤离

- `violin_grip`（持琴）：最高安全优先级（掉琴不可接受）
  - `slip_risk` 超阈值 → 立即进入握持增强/撤离到安全姿态
  - 若持续不可恢复 → e-stop（保琴优先）

- `bow_grip`（握弓）：其次安全优先级
  - `slip_risk` 高 → 降弓压/限速/重整握持

> 阈值建议放在 `configs/control/`，不同 demo profile 可配置。

## 4) 传感器/状态失联超时（必须）

任何关键通道失联都视为高危（尤其持琴）：
- `state_timeout_ms`：关节/末端状态超时
- `tactile_timeout_ms`：触觉特征超时
- `audio_timeout_ms`：音频特征超时（可降级，但需明确策略）

超时策略（建议）：
- 触觉失联：直接进入 `recovery`，若涉及 `violin_grip` 则升级为 e-stop
- 状态失联：e-stop
- 音频失联：允许降级（继续执行短窗口），但必须限制风险（降低速度/降低弓压）

## 5) 乐器保护（Instrument Protection，软限位）

这部分是“demo 能不能持续跑”的关键：

- 弓压上限（接触相位）：`bow_force_soft_max_n`（TBD）
- 指尖压弦上限：`finger_press_soft_max_n`（TBD）
- 接触相位最大运弓速度：`bow_speed_in_contact_max`（TBD）
- 接触相位最大冲击（加速度/jerk）：`ee_jerk_in_contact_max`（TBD）

触发策略：
- 超限 → 立刻减速/减力并撤离；重复触发 → 进入 `reset_safe_pose` 或 e-stop

## 6) 急停（E-Stop）

任何以下条件触发 e-stop（latched）：
- 碰撞检测或自碰风险不可控
- 力矩/电流超限
- 掉落风险（`violin_grip` 滑移不可恢复）
- 触觉/状态失联超过超时

急停后动作：
- 若硬件支持：进入安全姿态 `reset_safe_pose`
- 否则：保持当前位置并释放危险输出（例如弓速/弓压归零）

## 7) 失败恢复（Recovery）

- `reset_safe_pose`：撤离到安全姿态（必须提供）
- `retry_policy`：允许的重试次数与退避策略（例如退避时间、降低力度/速度后重试）
