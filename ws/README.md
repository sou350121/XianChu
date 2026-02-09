# ROS 2 Workspace（colcon）

本目录是 ROS 2 的开发工作区（workspace）。使用 `colcon` 构建，源码位于 `src/`。

## 约定
- `src/`：所有 ROS 2 packages
- `build/`, `install/`, `log/`：构建产物（已在 `.gitignore` 排除）

## 快速命令（示例）
> 具体命令以 `../docs/engineering/build_and_run.md` 为准。

### Linux/macOS（bash）

```bash
cd Violin_Robot_PRD/ws
colcon build --symlink-install
source install/setup.bash
```

### Windows（PowerShell）

```powershell
cd .\Violin_Robot_PRD\ws
colcon build --symlink-install
.\install\local_setup.ps1
```

