# ros_interfaces（接口规范：人类可读）

本目录定义本项目的 ROS 2 接口“规范视图”（topics/services/actions、字段语义、频率、单位）。

真实可执行接口定义（IDL）应位于：
- `../../ws/src/violin_robot_interfaces/`

## 0) v0 接口策略（先跑通链路，再升级类型）

- v0 阶段优先使用：**标准消息** + `std_msgs/String(JSON)` 承载复杂结构
- 先保证：时间同步、字段语义、单位、失败可复盘
- v1+ 再升级为自定义 `.msg/.srv/.action`

SSOT：
- 数据字段：`../data_schema/episode_schema.md`
- 音乐表征：`../data_schema/music_spec.md`
- 坐标系：`../frames/tf_tree.md`
- 安全约束：`../safety/limits.md`

## 1) Topics（最薄切片 v0）

### 1.1 Music

- **`/violin/music/spec`**（发布：`violin_robot_music_spec`）
  - **类型**：`std_msgs/String`（JSON）
  - **频率**：事件驱动（加载/更新发布）
  - **schema**：`contracts/data_schema/music_spec.md`

### 1.2 State / Sensors

- **`/violin/state/joint_state`**（发布：`violin_robot_drivers`）
  - **类型**：`sensor_msgs/JointState`
  - **频率**：≥ 100 Hz（mock 默认 100 Hz）
  - **单位**：rad, rad/s, N·m（或 effort 的硬件单位，需记录）

- **`/violin/state/end_effector_pose`**（发布：`violin_robot_drivers`）
  - **类型**：`geometry_msgs/PoseStamped`
  - **频率**：30–100 Hz（mock 默认 30 Hz）
  - **frame**：`base_link`

- **`/violin/tactile/features`**（发布：`violin_robot_drivers`）
  - **类型**：`std_msgs/String`（JSON）
  - **频率**：30–100 Hz（mock 默认 50 Hz）
  - **schema**：`contracts/data_schema/episode_schema.md`（`obs.tactile.features` 子集）

建议 JSON 最小结构（示意）：

```json
{
  "timestamp": 1730000000.123,
  "sources": {
    "bow_string": {
      "contact_event": true,
      "contact_phase": "in_contact",
      "slip_risk": 0.12,
      "contact_mode": "stick",
      "hf_energy": 0.03,
      "bow_normal_force_est": 0.9
    },
    "violin_grip": {
      "contact_event": true,
      "contact_phase": "in_contact",
      "slip_risk": 0.02,
      "contact_mode": "stick",
      "hf_energy": 0.00
    }
  }
}
```

- **`/violin/audio/features`**（发布：`violin_robot_drivers`）
  - **类型**：`std_msgs/String`（JSON）
  - **频率**：10–50 Hz（mock 默认 20 Hz）
  - **schema**：`contracts/data_schema/episode_schema.md`（`obs.audio.features` 子集）

建议 JSON 最小结构（示意）：

```json
{
  "timestamp": 1730000000.123,
  "f0_hz": 440.2,
  "energy": 0.37,
  "noise_ratio": 0.08,
  "valid": true
}
```

### 1.3 Commands

- **`/violin/control/command_raw`**（发布：`violin_robot_policy`）
  - **类型**：`std_msgs/String`（JSON）
  - **频率**：10–30 Hz（mock 默认 20 Hz）
  - **说明**：未经安全门控的原始命令
  - **schema**：`contracts/data_schema/episode_schema.md`（`action` 子集）

建议 JSON 最小结构（示意）：

```json
{
  "timestamp": 1730000000.123,
  "ee_delta": {
    "left": {"dpos_m": [0, 0, 0], "drot_rad": [0, 0, 0]},
    "right": {"dpos_m": [0, 0, 0], "drot_rad": [0, 0, 0]}
  },
  "hand_params": {
    "bow_speed": 0.4,
    "bow_force": 1.0,
    "bow_direction": "down",
    "finger_press": 0.3,
    "grip_strength": 0.5
  },
  "control_mode": "in_contact"
}
```

- **`/violin/control/command`**（发布：`violin_robot_control`）
  - **类型**：`std_msgs/String`（JSON）
  - **频率**：10–30 Hz
  - **说明**：经过触觉门控/限幅后的命令（可以附带 guard 字段）

> 兼容性约定：控制层允许追加 `guard` 字段（例如 slip_risk、contact_phase），上层策略必须忽略未识别字段。

- **`/violin/control/emergency_stop`**（发布：`violin_robot_control`）
  - **类型**：`std_msgs/Bool`
  - **触发**：latched（触发后保持 True，直到 reset）

## 2) Services / Actions（占位，后续 contract-first 扩展）

- **`/violin/system/calibrate`**：执行校准流程（输出校准产物路径与版本）
- **`/violin/system/run_demo`**：启动 demo（选择 demo profile）
- **`/violin/system/reset_safe_pose`**：回到安全复位姿态
- （建议新增）`/violin/system/reset_estop`：解除 e-stop latch（必须受权限/物理按钮约束）

## 3) 命名与硬约束

- topic 前缀统一为 `/violin/`
- 所有可对齐数据都必须带 `timestamp`
  - 标准消息：`header.stamp`
  - JSON：显式 `timestamp`（秒）
- 单位必须显式：m / rad / N·m / Hz / mm / N

## 4) v1+ 类型升级路线（建议）

- 将 `/violin/tactile/features`、`/violin/audio/features`、`/violin/control/command*` 从 JSON 升级为自定义 `.msg`
- 保持字段语义不变，只升级承载类型（避免破坏数据/训练/评测工具链）
