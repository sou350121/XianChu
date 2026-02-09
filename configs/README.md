# configs（SSOT：配置层）

本目录存放硬件/控制/demo profile 的配置。

## 规则

- 任何运行时参数必须可追溯：配置文件要带版本/日期/适用硬件（SN）。
- 校准结果引用必须指向 `calibration/` 内的版本化文件。

## 子目录

- `hardware/`：drivers 参数、端口、SN 绑定
- `control/`：控制器参数、限幅、阻抗/导纳 profile、触觉阈值与超时
- `demo/`：demo 场地/琴弓/曲目 profile
