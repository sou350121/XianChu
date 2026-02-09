# frames（tf 树与坐标系 SSOT）

本文件定义本项目必须遵守的坐标系与 tf 树约定，用于保证多源数据（视觉/触觉/音频/关节状态）对齐与可复现。

> 注意：人形机器人可能已有既定 tf 树；本文件采用“**扩展而不破坏**”策略：在既有 base/torso/arm/hand 基础上添加 violin/bow/string 等语义 frame。

## 1) 基础 frame（最小集合）

- `map`：可选（若有外部定位/动捕）
- `odom`：可选（若有里程计）
- `base_link`：机器人基座（推荐作为全局参考 frame）
- `torso_link`：躯干参考
- `left_arm_base` / `right_arm_base`
- `left_ee` / `right_ee`：左右手末端执行器 frame（握持参考）

## 2) 道具相关 frame（本项目新增）

### 2.1 小提琴（Violin）

- `violin_body`：琴体参考 frame（建议作为所有弦 frame 的父 frame）
- `violin_neck`：琴颈参考 frame
- `string_plane`：可选，四根弦近似共面的参考 frame
- `string_g` / `string_d` / `string_a` / `string_e`：四根弦的参考 frame

### 2.2 弓（Bow）

- `bow_body`：弓体参考 frame
- `bow_hair`：弓毛接触参考 frame（用于弓角/弓压/落弓点建模）

> 备注：这些 frame 在 v0 可来自“静态标定 + 抓握姿态推算”；后续可引入视觉跟踪/在线校正更新。

## 3) 轴向约定（必须明确，否则混合力位控制不可复现）

### 3.1 `violin_body`（推荐约定）

右手系：
- `+X_v`：沿弦方向（从琴码 bridge 指向琴头 scroll / nut）
- `+Y_v`：跨弦方向（从 G 弦指向 E 弦）
- `+Z_v`：从面板向外的法向（离开琴体的“上”方向）

`violin_body` 的原点（v0 建议）：琴码处弦平面上的参考点（例如 D/A 两弦之间的琴码中点投影）。

### 3.2 `string_*`（推荐约定）

- `string_g/d/a/e` 与 `violin_body` 共用同一轴向（X/Y/Z 同向），但原点位于各自弦在琴码处的接触参考点。

这样可以把“换弦”写成：沿 `+Y_v`（跨弦）方向的离散切换/连续插值，而不是依赖世界坐标。

### 3.3 `bow_hair`（推荐约定）

右手系：
- `+X_b`：沿弓毛方向（从弓蛙 frog 指向弓尖 tip）
- `+Y_b`：弓毛横向（朝向演奏者的左侧为正，具体按安装定义并固定）
- `+Z_b`：从弓毛指向弓杆 stick 的法向（用于定义弓毛倾角）

> 若硬件/模型不便提供 `Y_b/Z_b`，至少要稳定提供 `X_b` 与 `Z_b`，否则弓毛倾角无法定义。

## 4) 派生 frame（可在控制节点内部计算，不强制发布 tf）

为实现“弓-弦混合力位控制”，推荐在控制层内部构造接触坐标：

- `bow_string_contact`
  - `s_hat`：沿弦方向（≈ `+X_v`）
  - `n_hat`：弓压法向（压向弦；≈ `-Z_v`，并随弓毛倾角微调）
  - `t_hat`：运弓切向（与 `s_hat/n_hat` 正交；可由弓速方向投影得到）

控制可写成：
- n 轴力控（弓压） + t 轴速度控（运弓） + 姿态微调（倾角/落弓点）

## 5) 对齐规则（数据可复盘要求）

- 任何触觉特征（滑移风险、接触相位、contact_mode）必须能映射到当时的 `left_ee/right_ee` 姿态（至少时间戳对齐；更理想是附带 `frame_id`）。
- 音频特征不需要 frame，但必须与控制命令在同一时间轴对齐（见 `../data_schema/episode_schema.md`）。
