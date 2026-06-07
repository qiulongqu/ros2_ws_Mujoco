# P3 Step3 机械臂运动控制 — 实现方案 v2

> **更新时间戳**: 2026-06-06
> **状态**: 待用户审核
> **变更**: v1→v2 从"正弦轨迹跟踪"调整为"点到点运动 → ROS2桥接 → MoveIt2集成"三阶段架构

---

## 0. 前期调研结论

### 0.1 官方生态现状

| 组件 | 来源 | 版本/状态 |
|------|------|-----------|
| MuJoCo (conda) | mujoco 3.8.1 | ✅ 已安装 |
| ROS2 Humble | apt (ros-humble-*) | ✅ 已安装 |
| mujoco_ros2_control | apt `ros-humble-mujoco-ros2-control` | ✅ 可直接安装 |
| mujoco_ros2_control_demos | apt `ros-humble-mujoco-ros2-control-demos` | ✅ 可直接安装 |

### 0.2 标准控制架构 (MoveIt2 → MuJoCo)

```
MoveIt2 (运动规划: RRT/OMPL)
  │
  ▼ FollowJointTrajectory Action
joint_trajectory_controller (轨迹插值: 梯形/S曲线)
  │
  ▼ position command interface
mujoco_ros2_control::MujocoSystemInterface (硬件桥接)
  │
  ▼ data.ctrl[:] = target
MuJoCo position actuator (内置PD: kp,kv)
  │
  ▼ mj_step()
Physics Simulation
```

### 0.3 MuJoCo Menagerie 参考模式

Franka FR3 / Panda / PiPER / Lite6 / Kinova Gen3 统一使用 position/general actuator:

```xml
<!-- pattern A: position actuator -->
<position name="joint1" joint="joint1" kp="4500" kv="450"
          ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>

<!-- pattern B: general actuator (等价) -->
<general name="joint1" joint="joint1" dyntype="none" biastype="affine"
         gainprm="4500" biasprm="0 -4500 -450"
         ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>
```

两种都在 C 引擎层实现相同的 PD 伺服: `torque = kp*(ctrl - q) - kv*qvel`

### 0.4 关键发现

- **我们的 MJCF XML 已经是 mujoco_ros2_control 需要的格式**，不需要 URDF→MJCF 转换
- **position actuator 的 ctrl 就是目标角度**，MoveIt2 的 `joint_trajectory_controller` 输出的也是位置指令，接口天然对齐
- mujoco_ros2_control 的 Humble apt 包包含 vendor MuJoCo 3.4.0，与我们 conda 的 3.8.1 独立，不冲突

---

## 1. 三阶段实现计划

### Step3-1: MuJoCo 纯仿真 — 点到点运动 (PTP)

**目标**: `python3 move_to.py --joints "0, -0.5, 0.3, 0, 0, 0"` 机械臂平滑到达

**改动文件**:

1. `gofa_crb15000.xml` — actuator 替换

```xml
<!-- 删除: motor + gear -->
<!-- 替换为: position actuator (Menagerie 模式) -->
<actuator>
  <position name="act_joint_1" joint="joint_1"
            kp="3500" kv="350"
            ctrlrange="-4.7124 4.7124"
            forcerange="-330 330"/>
  <position name="act_joint_2" joint="joint_2"
            kp="3500" kv="350"
            ctrlrange="-2.4435 1.5708"
            forcerange="-330 330"/>
  <position name="act_joint_3" joint="joint_3"
            kp="2500" kv="250"
            ctrlrange="-3.9270 1.4835"
            forcerange="-200 200"/>
  <position name="act_joint_4" joint="joint_4"
            kp="1500" kv="150"
            ctrlrange="-3.4907 3.4907"
            forcerange="-50 50"/>
  <position name="act_joint_5" joint="joint_5"
            kp="1000" kv="100"
            ctrlrange="-2.2689 2.2689"
            forcerange="-30 30"/>
  <position name="act_joint_6" joint="joint_6"
            kp="500"  kv="50"
            ctrlrange="-4.7124 4.7124"
            forcerange="-15 15"/>
</actuator>
```

2. `step3_ptp_move.py` — 点到点运动脚本

- 梯形速度曲线轨迹生成器
- `data.ctrl[:] = interpolated_target` (每步插值)
- `--joints` 参数接收目标角度
- 三阶段验证: 预热保持 → 小幅度移动 → 全范围移动

### Step3-2: ROS2 桥接

**目标**: MuJoCo 仿真通过 ros2_control 接入 ROS2 生态

**步骤**:
1. `sudo apt install ros-humble-mujoco-ros2-control ros-humble-mujoco-ros2-control-demos`
2. 创建 `gofa_ros2_bringup/` 包 (URDF + controller yaml + launch)
3. 配置硬件接口: 指定我们的 MJCF 路径
4. 验证: `/joint_states` 发布, `/joint_trajectory_controller/follow_joint_trajectory` action 可用

**涉及文件**:
- `gofa_ros2_bringup/urdf/gofa.ros2_control.xacro` — 硬件接口配置
- `gofa_ros2_bringup/config/gofa.controllers.yaml` — 控制器参数
- `gofa_ros2_bringup/launch/gofa_mujoco.launch.py` — 启动文件
- `gofa_ros2_bringup/description/gofa_crb15000.xml` — 我们的 MJCF (软链接)

### Step3-3: MoveIt2 运动规划

**目标**: MoveIt2 规划路径 → MuJoCo 仿真执行

**步骤**:
1. 用 MoveIt Setup Assistant 生成 `gofa_moveit_config/`
2. 配置 Planning Pipeline (OMPL, RRTConnect)
3. 通过 `/follow_joint_trajectory` action 连接
4. RViz 可视化 + MuJoCo 同步执行

**涉及包**: `gofa_moveit_config/` (MoveIt Setup Assistant 自动生成)

---

## 2. 验证流程

### Step3-1 验证

| # | 测试 | 验收标准 |
|---|------|---------|
| 1 | 零位保持 500 步 | qpos 偏离 < 0.01° |
| 2 | 单关节移动 ±10° | 稳态误差 < 0.1°, 无过冲 |
| 3 | 6 关节同时移动 ±30° | 最大误差 < 1°, 2 秒内到达 |
| 4 | 全范围往返 (j2: 0→-90°→0) | 最大误差 < 2°, 无振荡 |

### Step3-2 验证

| # | 测试 | 验收标准 |
|---|------|---------|
| 1 | `ros2 topic list` | `/joint_states` 存在 |
| 2 | `ros2 topic echo /joint_states` | qpos 值正确 |
| 3 | `ros2 action send_goal` 发目标 | 机械臂移动 |

### Step3-3 验证

| # | 测试 | 验收标准 |
|---|------|---------|
| 1 | RViz 中拖拽目标位姿 → Plan & Execute | MuJoCo 中机械臂到达 |
| 2 | 避障规划 | 路径避开碰撞物体 |

---

## 3. 风险与缓解

| 风险 | 概率 | 缓解 |
|------|------|------|
| mujoco_ros2_control vendor 3.4.0 与 conda 3.8.1 冲突 | 低 | 两套独立环境，apt 包自带 vendor |
| position actuator kp 选择不当 | 中 | 参考 Panda FR3，按关节力矩等比例缩放 |
| ros2_control YAML 配置复杂 | 中 | 参考 demos 包的模板 |
| MoveIt2 SRDF 配置耗时 | 中 | 用 Setup Assistant 自动生成 |

---

## 4. 参考资料

- [mujoco_ros2_control (ros-controls)](https://github.com/ros-controls/mujoco_ros2_control)
- [mujoco_ros2_control (moveit)](https://github.com/moveit/mujoco_ros2_control)
- [MuJoCo Menagerie - Panda](https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/panda.xml)
- [ros2_control 文档](https://control.ros.org/humble/index.html)
- [MoveIt2 文档](https://moveit.ai/)

---

> **文档作者**: Claude Code
> **审核状态**: 待用户确认
> **下一步**: 用户确认后 → 执行 Step3-1
