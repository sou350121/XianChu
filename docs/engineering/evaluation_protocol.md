# Evaluation Protocol（评估协议）

本协议用于避免“剪视频式成功”，让 SR/IR/ER、触觉闭环稳定性与失败分布**可复现**。

SSOT 参考：
- `../../contracts/data_schema/episode_schema.md`
- `../../contracts/ros_interfaces/README.md`
- `../../contracts/safety/limits.md`

## 1) 指标（v0 最小集合）

- **SR（Success Rate）**：完成指定曲目/片段且不中断（或按 PRD 定义）
- **IR（Intervention Rate）**：单位时间人工接管次数（次/分钟）
- **ER（Executable Rate）**：上层输出经 IK/碰撞/限幅过滤后可执行比例
- **Failure Distribution**：失败类别占比
  - `drop_risk / excessive_force / ik_fail / collision / no_sound / squeal / string_miss / buzzing / unstable_audio / other`

## 2) 音乐域辅助指标（v0 可用简化口径）

> 目的不是做专业音色评测，而是提供“可验收、可对比”的客观信号。

- **音高偏差**：对每个目标 note，统计 `|f0_hz - f0_target|` 的分布（P50/P95）
- **节奏偏差**：note onset 与目标 `t_start` 的偏差（ms）
- **噪声占比（啸叫/擦弦代理）**：`noise_ratio`（若暂未实现，可用高频能量占比替代）

## 3) 触觉/接触域指标（闭环 KPI）

从 `obs.tactile.features.sources` 计算：
- `slip_events_per_min`：每分钟 slip 次数（分 source）
- `preslip_events_per_min`
- `time_in_contact_window`：在安全窗口内的时间占比（例如 `slip_risk<threshold` 且 `contact_phase=in_contact`）
- `recovery_success_rate`：进入 recovery 后成功回到稳定接触的比例

## 4) 安全与可靠性指标

- `estop_count`：e-stop 触发次数
- `sensor_timeout_count`：状态/触觉/音频超时次数（若有）
- `jitter_p95_ms`：关键 loop 的周期抖动统计（若可从日志推断）

## 5) 样本量与置信区间

- 建议 **N≥50** 才有统计意义；否则必须报告置信区间并注明样本不足
- SR 建议报告 Wilson 区间（实现侧可后补脚本）

## 6) A/B 评测

- 交替测试（A,B,A,B...）避免环境随时间漂移导致偏差
- 每次 run 必须记录 demo profile / 配置版本 / 校准版本（见 data_capture）

## 7) 产物

- 原始日志/rosbag：外部存储 + `data_registry` 登记
- 汇总报告：可入库（小文件，例如 `report.json` + `report.md`）
