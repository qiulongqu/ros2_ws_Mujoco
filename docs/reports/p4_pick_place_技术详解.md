# P4: Pick-and-Place 拣放操作 (纯 MuJoCo, 无 ROS2/MoveIt2)

使用 8 阶段状态机 + 数值 IK + teleport 吸附实现完整的 pick-and-place 流程。

## 目录结构

```
p4_pick_place/
├── gofa_table_block.xml                  # MuJoCo 模型 (GoFa 臂 + 桌子 + 方块 + 吸盘 + 标记)
├── step1_build_scene.py                  # 场景构建与验证 (7 项测试 + 渲染)
├── step2_manual_pick_place.py            # 手动 pick-and-place (8 阶段状态机 + IK)
└── README.md
```

## 模型信息

| 属性 | 值 |
|------|-----|
| 机械臂 | ABB GoFa CRB15000-10/1.52 (6-DOF) |
| 执行器模式 | Position actuator (Menagerie 模式, kp=1500-4000, kv=kp/10) |
| 积分器 | implicit (dt=0.002s) |
| 场景元素 | 桌子 (0.50, 0, 0.45), 红色方块 (2.5cm³), 绿色拣取标记, 蓝色放置标记 |
| 抓取方式 | Teleport 吸附 (方块位置每步同步到 gripper_site) |
| 逆运动学 | 数值 IK (有限差分 Jacobian + DLS) |

### 场景布局

```
机械臂底座: (0, 0, 0)
桌子:       x=0.50, y=0,    z=0.45 (台面)
拣取位:     x=0.50, y=-0.15, z=0.475 (绿色球)
放置位:     x=0.50, y=0.15,  z=0.475 (蓝色球)
```

### 运动学链

```
world → base_link
  → joint_1(Z) → link_1
    → joint_2(Y) → link_2
      → joint_3(Y) → link_3
        → joint_4(X) → link_4
          → joint_5(Y) → link_5
            → joint_6(X) → link_6 → tool0 → gripper_site (吸盘站点)
```

零位末端位置: `[0.888, 0.000, 1.297]`（垂直向上伸展）

## 运行

### Step1: 场景构建与验证（自动测试，无需显示器）

验证模型结构、body/site 名称、方块位置、桌子高度、吸盘站点、抓取约束，并保存渲染截图。

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p4_pick_place/step1_build_scene.py
```

输出：
- 终端：7 项测试结果（模型结构、body 名称、吸盘站点、方块位置、桌子高度、零位 tool0、抓取约束）
- 图片：`resources/p4/p4_step1_scene_zeropose.png`, `p4_step1_scene_reach.png`

交互查看器模式：

```bash
python3 tutorials/p4_pick_place/step1_build_scene.py --view
```

### Step2: 手动 Pick-and-Place（8 阶段状态机）

纯 MuJoCo 实现，无需 ROS2/MoveIt2。使用数值 IK 求解关节角，通过 8 阶段状态机完成拣放操作。

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY

# 验证模式（自动运行 + 轨迹图）
python3 tutorials/p4_pick_place/step2_manual_pick_place.py

# 交互查看器模式（观察全过程）
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view
```

输出：
- 终端：IK 求解结果、各阶段转换日志、最终验证（方块位置误差）
- 图片：`resources/p4/p4_step2_trajectory.png`（关节轨迹 + 方块位置 + 阶段标记）

### 8 阶段状态机

| 阶段 | 名称 | 动作 | 触发条件 |
|------|------|------|---------|
| 1 | PRE_GRASP | 移动到拣取位上方 10cm | 关节到达目标 |
| 2 | APPROACH | 下降到拣取位 | 关节到达目标 |
| 3 | GRASP | 激活 teleport 吸附 | 立即进入下一阶段 |
| 4 | LIFT | 提升到安全高度 (50cm 余量) | 关节到达目标 |
| 5 | PRE_PLACE | 移动到放置位上方 (同安全高度) | 关节到达目标 |
| 6 | DESCEND | 下降到放置位 | 关节到达目标 |
| 7 | RELEASE | 停用吸附，方块自由下落 | 立即进入下一阶段 |
| 8 | RETREAT | 撤退到安全高度 | 关节到达目标 |

### 关键实现细节

**Teleport 吸附** (替代 weld constraint)：

MuJoCo 3.8.1 中 `<weld>` equality constraint 在 free body 和 kinematic body 之间无法正常工作。替代方案：在抓取阶段每步将方块 qpos 同步到 gripper_site xpos，qvel 清零。

```python
if sm._grasp_active:
    data.qpos[6:9] = data.site('gripper_site').xpos.copy()
    data.qvel[6:12] = 0.0
```

**数值 IK**：有限差分 Jacobian (ε=0.0005) + DLS (λ=0.05)，关节限位约束，收敛阈值 1mm。

**放置高度**：`PLACE_SITE_XYZ Z=0.49`（比台面高 1cm），释放后方块掉落到桌面，避免穿透。

## 前置条件

- `mujoco` conda 环境: `conda create -n mujoco -y && conda activate mujoco && pip install mujoco numpy pillow matplotlib`
- Step2 交互查看器需要 X server (WSLg 或 VcXsrv)

## 技术要点

### Position Actuator (Menagerie 模式)

MuJoCo 内置 PD 伺服: `torque = kp*(ctrl - qpos) - kv*qvel`

```xml
<position name="act_joint_1" joint="joint_1"
          kp="4000" kv="400"
          ctrlrange="-4.7124 4.7124"
          forcerange="-400 400"/>
```

Python 侧只需 `data.ctrl[:] = target_q`，无需手动 PD 计算。

### implicit 积分器

高 kp + 低惯量手腕关节组合下，显式 Euler 积分数值不稳定 (dt × ω_n > 1)。`integrator="implicit"` 是解决该问题的关键配置。

### 全局控制回调

交互查看器模式使用 `mj.set_mjcb_control(callback)` 设置全局控制器，然后调用 `viewer.launch(model, data)`。注意 `viewer.launch()` 不接受 `controller` 关键字参数。

## 模型来源

基于 P3 的 `gofa_crb15000.xml` 扩展而来，添加了桌子、方块、吸盘站点、视觉标记和抓取约束。URDF/XACRO 源文件来自 `abb_omnicore_ros2` 包。

启动命令
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_abb/learning_project/install/setup.bash
ros2 launch gofa_moveit_config demo.launch.py
```