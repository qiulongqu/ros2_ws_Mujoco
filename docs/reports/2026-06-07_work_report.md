# GoFa MoveIt2 + MuJoCo 联合仿真 — 工作报告

> 日期: 2026-06-07 | 状态: 基础链路打通，待端到端验证

---

## 一、目标回顾

构建 `ros2 launch gofa_moveit_config demo.launch.py` 一键启动，实现：
- MuJoCo 物理仿真 (MujocoSystemInterface)
- ros2_control 桥接 (joint_state_broadcaster + joint_trajectory_controller)
- MoveIt2 运动规划 (move_group + OMPL)
- RViz 可视化 (MotionPlanning 插件可规划可执行)

---

## 二、当前状态

### 已验证成功

| 组件 | 状态 | 证据 |
|------|------|------|
| robot_state_publisher | OK | 解析全部 8 个 segment |
| ros2_control_node (MujocoSystemInterface) | OK | 6 个关节全部注册为 position actuator |
| abb_controller (JTC) | OK | loaded → configured → activated |
| joint_state_broadcaster | OK | loaded → configured → activated |
| move_group | OK | moveit_simple_controller_manager 识别 abb_controller |
| RViz | OK | 加载模型，连接 MoveGroup |

### 已知问题

| 问题 | 严重程度 | 状态 |
|------|----------|------|
| Octomap 无 3D 传感器插件 | 低 (不影响规划) | 预期行为，无深度相机 |
| gripper end-effector parent group 未识别 | 低 | SRDF 中未定义 gripper group |
| 未验证端到端规划+执行 | **高** | 需要在 RViz 中手动测试 Plan & Execute |

---

## 三、Bug 与修复历程

### Bug 1: 控制器加载失败 — YAML 结构不匹配

**现象**: spawner 报 `Failed loading controller` / `no controller with this name exists`

**根因**: 控制器 YAML 结构错误。将 `type` 放在 `controller_manager.ros__parameters` 内部（参考了 abb_gofa_battery 项目的模式），但 `mujoco_ros2_control` 要求每个控制器有自己独立的 `ros__parameters` 块，`type` 放在各自块内。

**修复**: 参照官方 `/opt/ros/humble/share/mujoco_ros2_control_demos/config/controllers.yaml` 重写 `gofa_controllers.yaml`:

```yaml
# 错误 (参考项目模式，不兼容 mujoco_ros2_control):
controller_manager:
  ros__parameters:
    update_rate: 100
    abb_controller:
      type: joint_trajectory_controller/...

# 正确 (官方 mujoco_ros2_control 模式):
controller_manager:
  ros__parameters:
    update_rate: 100

abb_controller:
  ros__parameters:
    type: joint_trajectory_controller/...
```

### Bug 2: Launch 文件参数加载方式

**现象**: 早期版本用字符串路径传参，不稳定

**修复**: 采用官方 `ParameterFile()` 封装 + spawner `--param-file` 参数:

```python
# ros2_control_node
parameters=[
    ParameterFile(controllers_cfg),
],
# spawner
arguments=["abb_controller", "--param-file", controllers_cfg],
```

### Bug 3: TF 树为空，RViz 模型堆叠在原点

**现象**: `/tf` 无发布，所有 link 显示在原点

**根因**: `robot_state_publisher` 只在收到外部 `/joint_states` 时才发布 TF。没有 joint_state_broadcaster 运行 → 无 joint_states → 无 TF。

**修复**: 修复 Bug 1 后，joint_state_broadcaster 正常加载，TF 树恢复。

### Bug 4 (历史): controller_manager 服务不可达

**现象**: 切换为 ParameterFile 后 `ros2 control list_controllers` 超时

**状态**: 已在当前版本修复。最终 launch 输出显示两个 spawner 均成功 `Configured and activated`。

---

## 四、文件变更清单

| 文件 | 变更 | 原因 |
|------|------|------|
| `gofa_mujoco_bringup/config/gofa_controllers.yaml` | 重写 | 匹配官方 mujoco_ros2_control 参数结构 |
| `gofa_moveit_config/launch/demo.launch.py` | 重写 | 采用官方 ParameterFile + --param-file 模式 |
| `gofa_moveit_config/config/moveit_controllers.yaml` | 新增 `moveit_controller_manager` key | moveit_simple_controller_manager 识别 abb_controller |
| `gofa_moveit_config/config/moveit.rviz` | Fixed Frame world→base_link | TF 树的根帧是 base_link |
| `gofa_moveit_config/package.xml` | 添加 `moveit_configs_utils` 依赖 | MoveItConfigsBuilder 需要 |

---

## 五、待办

1. **端到端验证**: 在 RViz MotionPlanning 面板中 Plan & Execute 一个目标姿态，确认机械臂在 MuJoCo 中真实移动
2. **关闭 MuJoCo 渲染窗口**: 添加 headless 选项避免 WSL2 黑屏窗口
3. **清理 CLAUDE.md**: 沉淀本次修复的经验教训
4. **更新 README**: 记录 launch 命令和预期行为
