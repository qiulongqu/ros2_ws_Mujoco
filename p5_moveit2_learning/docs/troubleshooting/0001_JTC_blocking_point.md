# Troubleshooting #0001: P5 MoveIt2 baseline 3 阻塞点修复

**日期**: 2026-06-09
**Phase**: P5 baseline (B → A 顺序, 先打通 L1↔L3 链路, 再补概念文档)
**作者**: Claude (Sonnet 4.6) + qiulongqu

---

## 背景

P5 README 路线图已写, 7 个子目录都是 .gitkeep 占位。`src/gofa_moveit_config/` 和
`src/gofa_mujoco_bringup/` 已有基础配置, 但从未端到端跑通过。盘点发现 3 个阻塞点
会让 `demo.launch.py` 启动必崩。

---

## 阻塞点 1: L1 → L3 链路断裂 (核心)

### 现象
MoveIt2 启动后, Plan 成功 → Execute 失败。`/abb_controller/follow_joint_trajectory`
action server 没人接, MoveIt2 action client 报 "Action server not available"。

### 根因
| 组件 | 期望 (CLAUDE.md 5-02/5-03 沉淀) | 修复前现状 |
|------|------------------------------|-----------|
| MoveIt2 想发 | `/abb_controller/follow_joint_trajectory` (action) | ✅ moveit_controllers.yaml 配好 |
| ros2_control 实际接 | `JointTrajectoryController` | ❌ **只配了 `position_controller` (ForwardCommandController)** |

### 修复
1. **`src/gofa_mujoco_bringup/config/gofa_controllers.yaml`** — 添加 `abb_controller` 块:
   ```yaml
   abb_controller:
     ros__parameters:
       type: joint_trajectory_controller/JointTrajectoryController
       joints: [joint_1, ..., joint_6]
       command_interfaces: [position]
       state_interfaces: [position, velocity]
       allow_partial_joints_goal: true
       open_loop_control: true
   ```

2. **`src/gofa_moveit_config/launch/demo.launch.py`** — spawner 列表更新:
   ```python
   controllers_to_spawn = ["joint_state_broadcaster", "abb_controller"]
   ```

### 修复中踩的坑 (1) — 类名错位
**错误**: 第一版用了 `position_controllers/JointTrajectoryController`
**报错**:
```
Loader for controller 'abb_controller' (type 'position_controllers/JointTrajectoryController') not found.
Available classes: ... joint_trajectory_controller/JointTrajectoryController ...
```
**教训**: 查 ros2_controllers 源码包名, 不是 `position_controllers`(那是 GripperActionController 用的) 而是 `joint_trajectory_controller`。

### 修复中踩的坑 (2) — 资源冲突
**错误**: 第二版把 `position_controller` 和 `abb_controller` 都 spawn 了
**报错**:
```
Resource conflict for controller 'abb_controller'. Command interface 'joint_1/position' is already claimed.
```
**根因**: ros2_control 硬约束 — 同一 hardware interface (joint_X/position) 只能被一个 controller 持有
**解决**: `demo.launch.py` (P5) 只 spawn `abb_controller`; `gofa_mujoco.launch.py` (P4) 只 spawn `position_controller`。两个 launch 文件互补, 不要在同一个进程里共存。

---

## 阻塞点 2: move_group.launch.py 缺 ompl_planning.yaml

### 现象
`ros2 launch gofa_moveit_config move_group.launch.py` 启动后, move_group 用默认
OMPL 参数, 规划超时或路径质量差。

### 根因
`src/gofa_moveit_config/config/ompl_planning.yaml` 写了 5 个规划器配置 (RRT, RRT*, PRM, EST, BiTRRT),
但 `move_group.launch.py` 没把它传给 move_group 节点。

### 修复
在 move_group 节点的 parameters 列表添加:
```python
ompl_yaml = load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml"))
# ... 并入 parameters=[...]
```

---

## 阻塞点 3: move_group.launch.py xacro 解析风格不一致

### 现象
`move_group.launch.py` 用 `subprocess.run(["xacro", path])` 同步解析, 第一次跑有 1-2 秒延迟,
且和 `demo.launch.py` 用的 `Command([xacro, ...])` launch 风格不一致。

### 根因
历史上 move_group.launch.py 写的时候没参考 demo.launch.py 的 launch 风格。

### 修复
替换为 launch_ros 风格的 xacro 解析:
```python
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([bringup_share, "urdf", "gofa.ros2_control.urdf.xacro"]),
    " headless:=true",
])
robot_description = {"robot_description": robot_description_content.perform(context)}
```

---

## 验证证据 (2026-06-09)

启动命令:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_abb/mujocoONLY/install/setup.bash
ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false
```

### 关键日志
```
[spawner_joint_state_broadcaster] Configured and activated joint_state_broadcaster
[spawner_abb_controller] Configured and activated abb_controller
[abb_controller] Command interfaces are [position] and state interfaces are [position velocity].
[abb_controller] Using 'splines' interpolation method.
[abb_controller] Goals with partial set of joints are allowed
[MujocoSystemInterface] Joint joint_1: position control enabled (velocity, effort disabled)
... (joint_2 ~ joint_6 同样)
```

### ros2 control list_controllers
```
abb_controller         joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster        active
```

### action server
```
$ ros2 action list
/abb_controller/follow_joint_trajectory
```

### /joint_states (50Hz)
```
$ ros2 topic echo /joint_states --once
header: stamp: {sec: 17, nanosec: 852000000}, frame_id: base_link
name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
position: [8.27e-25, 0.0118, ...]  (零位附近)
```

---

## 沉淀到 CLAUDE.md 的新教训

### 5-06: ros2_control controller 资源互斥

同一 `joint_X/position` command interface 只能被一个 controller 持有。两个 controller
(ForwardCommandController + JointTrajectoryController) 不能在同一个 controller_manager
进程里同时 active。解决: 用不同的 launch 文件分流 (P4 → ForwardCommandController, P5 → JTC)。

### 5-07: JointTrajectoryController 类名位置

类名是 `joint_trajectory_controller/JointTrajectoryController` (包名=joint_trajectory_controller),
不是 `position_controllers/JointTrajectoryController` (那是占位, 实际不存在)。
`ros2 control list_controller_types` 是验证唯一可靠方式。

### 5-08: 启动 move_group 前必须先启动 controllers

move_group 不负责 spawn controllers。它假设 `/controller_manager/list_controllers` 服务可用,
`abb_controller` 处于 active 状态, `/abb_controller/follow_joint_trajectory` action server 在线。
正确顺序:
1. `ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py` 或 `demo.launch.py` (启动 controllers)
2. 验证 `ros2 control list_controllers` 看到 `abb_controller active`
3. `ros2 launch gofa_moveit_config move_group.launch.py` (启动规划节点)
4. `rviz2 -d <path>/moveit.rviz` (可视化)

### 5-09: 配置文件散落 → launch 文件要拉通加载

moveit_config 包下 5 个 yaml (srdf/joint_limits/kinematics/ompl/moveit_controllers) 缺一不可。
launch 文件不加载某个 = 走默认值, 大概率 fail。模板:
```python
parameters=[
    robot_description,
    robot_description_semantic,
    kinematics_yaml,
    ompl_yaml,                  # 别漏!
    joint_limits_yaml,          # 别漏!
    moveit_controllers_yaml,
    {"use_sim_time": True},
]
```

---

## 下一步 (Phase A 概念补全)

- [ ] Phase 1: 读 5 篇 MoveIt2 官方文档, 写 `docs/concept_notes/*.md`
- [ ] Phase 2: 跑官方 MoveIt2 Python tutorial (在 `moveit2_tutorials` 包里, **目前未装**, 需要 `apt install ros-humble-moveit2-tutorials`)
- [ ] Phase 3: 启动完整 `demo.launch.py` + `move_group.launch.py` + RViz, 在 RViz 中 Plan + Execute 一次, 截图存 `resources/p5/`
