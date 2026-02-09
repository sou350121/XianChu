# Music Spec（中间表征 SSOT）

本文件定义从 MusicXML/MIDI/音频解析后的**统一中间表征**。

- **目标**：让策略层/技能层不直接依赖输入格式；同一套 `music_spec` 可由不同解析器产生。
- **原则**：字段语义、单位、时间基准必须明确；新增字段必须向后兼容。

## 1) 顶层字段（v0 必需）

- `music_spec_version`：字符串或整数版本号（例如 `"0.1"`），用于向后兼容
- `piece_id`：曲目/片段标识
- `time_base`：时间基准（必须二选一）
  - `"seconds"`：`t_start/t_end` 以秒计（float）
  - `"ticks"`：`t_start/t_end` 以 tick 计（int）；需要 `ticks_per_quarter`
- `ticks_per_quarter`：每四分音符 tick 数（仅当 `time_base=ticks` 必填）
- `tempo_bpm`：目标 BPM（v0 可先只支持常速；变速见可选字段）
- `time_signature`：拍号（建议结构化）
  - `beats_per_bar`：每小节拍数（如 4）
  - `beat_unit`：以几分音符为一拍（如 4 表示四分音符）
- `events[]`：按时间排序的事件序列（v0 以 note 事件为主）

## 2) `events[]`：事件字段（v0 推荐 schema）

### 2.1 通用字段
- `event_id`：事件唯一 ID（字符串/整数均可）
- `type`：事件类型（v0 最小集合：`"note"`）
- `t_start` / `t_end`：事件起止时间（单位由 `time_base` 决定）

### 2.2 `type="note"` 字段
- `pitch_midi`：MIDI pitch（整数，例如 A4=69）
- `string`：可选，目标弦（`"G" | "D" | "A" | "E"`）；未知可为空
- `finger`：可选，指法（v0 允许为空）
  - `finger_index`：1/2/3/4（不含拇指）
  - `position_hint`：可选，例如 `"1st" | "2nd" | "3rd"`（把位提示，非硬约束）

#### 2.2.1 弓法（`bow` 子结构，v0 推荐但可为空）
- `bow`：对象
  - `bow_direction`：`"up" | "down"`
  - `bowing_style`：`"detache" | "legato" | "slur"`（v0 最小集合）
  - `slur_group_id`：可选，同一连弓组 ID
  - `bow_speed`：可选，运弓速度目标（v0 建议先用归一化 0–1；后续可升级为 m/s）
  - `bow_force`：可选，弓压目标（单位 N；若硬件暂不可力控，可作为参考）
  - `bow_contact_point_mm`：可选，落弓点（距琴码的距离，mm；用于音色/稳定性策略）
  - `bow_tilt_rad`：可选，弓毛倾角（rad；用于接触面积/力分布策略）

#### 2.2.2 力度（`dynamics`）
- `dynamics`：可选
  - `mark`：`"pp"|"p"|"mp"|"mf"|"f"|"ff"`
  - `target_energy`：可选，目标能量（归一化 0–1 或 dBFS；v0 可先不用）

## 3) 可选字段（v1+ 扩展，v0 可忽略）
- `tempo_map[]`：支持变速的 tempo 变化点
- `string_plan`：显式换弦计划（当 `string` 未指定时用于约束）
- `articulation`：更细分的演奏技法（staccato/spiccato 等）

## 4) 向后兼容约定
- 未识别字段必须忽略（不应导致系统崩溃）。
- 解析器可以不填可选字段；策略层必须能在字段缺失时降级运行。

## 5) 示例（JSON，概念示意）

```json
{
  "music_spec_version": "0.1",
  "piece_id": "demo_scale_C_major",
  "time_base": "seconds",
  "tempo_bpm": 80,
  "time_signature": {"beats_per_bar": 4, "beat_unit": 4},
  "events": [
    {
      "event_id": "n1",
      "type": "note",
      "t_start": 0.0,
      "t_end": 0.5,
      "pitch_midi": 69,
      "string": "A",
      "finger": {"finger_index": 1, "position_hint": "1st"},
      "bow": {
        "bow_direction": "down",
        "bowing_style": "detache",
        "bow_speed": 0.4,
        "bow_force": 1.0,
        "bow_contact_point_mm": 25.0,
        "bow_tilt_rad": 0.0
      },
      "dynamics": {"mark": "mf"}
    }
  ]
}
```
