# Build & Run（ROS 2）

> 本文是“能跑起来”的最小 runbook。具体 driver/launch 细节会随着硬件接口落地而补齐。

## 1) 前置
- 已安装 ROS 2（发行版与 OS 版本请在此补充：TODO）
- 已安装 `colcon` 与常用工具链

## 2) 构建

```bash
cd Violin_Robot_PRD/ws
colcon build --symlink-install
```

## 3) 环境加载（按你的终端选择其一）

### 3.1 Linux/macOS（bash）

```bash
cd Violin_Robot_PRD/ws
source install/setup.bash
```

### 3.2 Windows（PowerShell）

```powershell
cd .\Violin_Robot_PRD\ws
.\install\local_setup.ps1
```

> 备注：有的 ROS 2/colcon 版本会生成 `setup.ps1` 或 `local_setup.ps1`；以 `install/` 目录下实际脚本为准。

## 4) 启动（占位）
启动入口建议统一由 `violin_robot_bringup` 提供。

### 4.1 v0 mock（最薄垂直切片）

```bash
# mock driver + music + policy stub + tactile guard
cd Violin_Robot_PRD/ws
source install/setup.bash
ros2 launch violin_robot_bringup v0_mock.launch.py
```

```powershell
cd .\Violin_Robot_PRD\ws
.\install\local_setup.ps1
ros2 launch violin_robot_bringup v0_mock.launch.py
```

## 5) 常见问题
- **找不到 package**：确认已加载环境（bash：`source install/setup.bash`；PowerShell：`.\install\local_setup.ps1`）
- **参数散落**：所有运行参数必须放入 `configs/`，不要硬编码
