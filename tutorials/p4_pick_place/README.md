# P4: Pick-and-Place · 拣放操作
**Pure MuJoCo · 纯 MuJoCo 实现**

![status](https://img.shields.io/badge/status-passed-brightgreen) ![runtime](https://img.shields.io/badge/runtime-3.04s-blue) ![accuracy](https://img.shields.io/badge/place_error-7mm-green) ![deps](https://img.shields.io/badge/deps-mujoco%20%7C%20numpy-orange)

---

## What is this | 这是什么

**EN** — A complete **pick-and-place** pipeline for the **ABB GoFa CRB15000** arm running entirely in **MuJoCo**.
The arm picks a red block off a table, lifts it across, and places it at a target spot — all driven by an 8-phase finite state machine with smooth quintic S-curve interpolation between waypoints.

**Why pure MuJoCo (no ROS2/MoveIt2 here):**
- Isolate and verify the **control + planning logic** before adding any middleware
- Master the foundational patterns: state machines, IK, contact, grasping
- Future P5 will layer MoveIt2 on top (already partially integrated at workspace root)

**CN** — 基于 **MuJoCo** 物理引擎的 **ABB GoFa CRB15000** 机械臂**完整拣放流程**。
机械臂从桌面拾取红色方块，搬运至对侧，放置到目标位置——全流程由 **8 阶段状态机** 驱动，相邻 waypoint 之间用 **quintic S-curve** 平滑插值。

**为何纯 MuJoCo (无 ROS2 / MoveIt2):**
- 在引入中间件前, 先**隔离验证控制+规划逻辑**
- 打好基础: 状态机、IK、接触、抓取
- P5 会在此之上叠加 MoveIt2 (workspace 根目录已部分集成)

---

## Quick Start | 快速开始

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY

# 1. Verify scene (7 tests, no GUI) | 场景验证 (7 项, 无需 GUI)
python3 tutorials/p4_pick_place/step1_build_scene.py

# 2. Run pick-and-place (auto + trajectory plot) | 运行拣放 (自动 + 轨迹图)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py

# 3. Interactive viewer (auto-loop, Space to pause) | 交互查看器 (自动循环, Space 暂停)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view
```

**EN** — Total time-to-first-cycle: **< 30 seconds** (after `mujoco` env is installed).
**CN** — 首次跑通时间: **< 30 秒** (前提是 `mujoco` 环境已装好).

---

## Results (2026-06-09) | 验收结果

| Metric / 指标 | Target / 目标 | Actual / 实际 | Status / 状态 |
|--------|--------|--------|--------|
| Cycle time / 单次循环 | < 5 s | **3.04 s** | ✅ |
| Place position error / 放置误差 | < 50 mm | **7.0 mm** | ✅ |
| Max joint tracking error / 关节跟踪误差 | < 5° | **1.7°** | ✅ |
| Phase transitions / 阶段切换 | 8/8 | **8/8** | ✅ |
| Scene validation tests / 场景验证 | 7/7 | **7/7** | ✅ |

**EN** — Output artifacts: `resources/p4/p4_step1_scene_*.png`, `p4_step2_trajectory.png`
**CN** — 输出产物: `resources/p4/p4_step1_scene_*.png`, `p4_step2_trajectory.png`

---

## 8-Phase State Machine | 8 阶段状态机

```
┌──────────┐    ┌──────────┐    ┌────────┐    ┌────────┐
│PRE_GRASP │ →  │APPROACH  │ →  │GRASP   │ →  │LIFT    │
│预抓取    │    │下降接近  │    │吸附    │    │提升    │
│above+10cm│    │descend   │    │suction │    │lift up │
└──────────┘    └──────────┘    └────────┘    └────────┘
                                                ↓
┌──────────┐    ┌──────────┐    ┌────────┐    ┌────────┐
│RETREAT   │ ←  │RELEASE   │ ←  │DESCEND │ ←  │PRE_PLACE│
│撤退      │    │释放      │    │下降放置│    │预放置  │
│back up   │    │release   │    │descend │    │above   │
└──────────┘    └──────────┘    └────────┘    └────────┘
```

| # | Phase / 阶段 | Action / 动作 | Trigger / 触发 |
|---|-------|--------|---------|
| 1 | `PRE_GRASP` | Move above pick point (+10 cm) / 移动到拣取位上方 10cm | Joints at target / 关节到位 |
| 2 | `APPROACH`  | Descend to pick point / 下降到拣取位 | Joints at target / 关节到位 |
| 3 | `GRASP`     | Activate suction (teleport) / 激活吸附 (teleport) | Immediate / 立即 |
| 4 | `LIFT`      | Move to safe altitude (+50 cm) / 提升到安全高度 | Joints at target / 关节到位 |
| 5 | `PRE_PLACE` | Move above place point / 移动到放置位上方 | Joints at target / 关节到位 |
| 6 | `DESCEND`   | Descend to place point / 下降到放置位 | Joints at target / 关节到位 |
| 7 | `RELEASE`   | Deactivate suction / 停用吸附, 方块自由下落 | Immediate / 立即 |
| 8 | `RETREAT`   | Return to safe altitude / 撤退到安全高度 | Joints at target / 关节到位 |

---

## Architecture | 架构分层

```
┌─────────────────────────────────────────────────────┐
│  Layer 4 — Physics: MuJoCo engine (dt=2ms, implicit)│
│  第 4 层 — 物理: MuJoCo 引擎 (隐式积分)              │
├─────────────────────────────────────────────────────┤
│  Layer 3 — Servo: Position actuator (kp=1500~4000)  │
│  第 3 层 — 伺服: 位置执行器 (Menagerie 模式)         │
├─────────────────────────────────────────────────────┤
│  Layer 2 — Trajectory: Quintic S-curve interpolation│
│  第 2 层 — 轨迹: 五次多项式 S 曲线插值                │
├─────────────────────────────────────────────────────┤
│  Layer 1 — Planning: 8-phase state machine          │
│  第 1 层 — 规划: 8 阶段状态机                         │
└─────────────────────────────────────────────────────┘
```

| Layer / 层 | Component / 组件 | Implementation / 实现 |
|-------|-----------|----------------|
| 4 | Physics / 物理 | MuJoCo with `integrator="implicit"`, contact solver / 接触求解 |
| 3 | Servo / 伺服 | `<position>` actuators, `kp*err - kv*vel` (Menagerie pattern) |
| 2 | Trajectory / 轨迹 | `6t⁵-15t⁴+10t³` per phase, 0.4 s ramp / 每阶段 0.4s 渐变 |
| 1 | Planning / 规划 | `PickPlaceStateMachine` class, IK + waypoint cache / IK + waypoint 缓存 |

**EN** — Industrial 4-layer separation. P5 will add Layer 0: MoveIt2 L1 planning on top.
**CN** — 工业 4 层架构分离. P5 将在顶层叠加 Layer 0: MoveIt2 L1 规划.

---

## Files | 文件结构

```
tutorials/p4_pick_place/
├── gofa_table_block.xml        # MuJoCo model: arm + table + block + suction
│                                 # MuJoCo 模型: 臂 + 桌子 + 方块 + 吸盘
├── step1_build_scene.py        # 7 scene tests + render
│                                 # 7 项场景验证 + 渲染
├── step2_manual_pick_place.py  # 8-phase state machine + IK + viewer
│                                 # 8 阶段状态机 + IK + 交互查看器
└── README.md                   # this file / 本文件
```

| File / 文件 | Lines / 行数 | Role / 角色 |
|------|-------|------|
| `gofa_table_block.xml` | 130 | Scene composition (extends P3 arm) / 场景组合 (扩展 P3 臂) |
| `step1_build_scene.py` | 210 | Scene validation, 7 tests, PNG render / 场景验证 + 渲染 |
| `step2_manual_pick_place.py` | 460 | FSM, IK, viewer controller, plotting / 状态机 + IK + 查看器 + 绘图 |

---

## Key Design Decisions | 核心设计决策

**1. Position actuator (Menagerie pattern) / 位置执行器 (Menagerie 模式)**
- **EN** — Instead of writing a manual PD loop, leverage MuJoCo's built-in servo: `torque = kp*(ctrl - qpos) - kv*qvel`. Python side just sets `data.ctrl[:] = target_q`. Less code, more robust.
- **CN** — 不手写 PD 循环, 复用 MuJoCo 内置伺服: `torque = kp*(ctrl - qpos) - kv*qvel`. Python 端只需 `data.ctrl[:] = target_q`. 代码更少, 鲁棒性更高.

**2. Implicit integrator / 隐式积分器**
- **EN** — High kp + low-inertia wrist joints → explicit Euler becomes unstable (dt × ω_n > 2). `integrator="implicit"` is mandatory for numerical stability.
- **CN** — 高 kp + 低惯量手腕关节 → 显式 Euler 不稳定 (dt × ω_n > 2). `integrator="implicit"` 是数值稳定的必要条件.

**3. Quintic S-curve between phases / 阶段间五次 S 曲线**
- **EN** — Without interpolation, every state transition is an instant target step → PD overshoots → end-effector shakes. The 0.4 s quintic ramp makes the motion smooth and the tracking error < 2°.
- **CN** — 无插值时, 每次阶段切换都是目标阶跃 → PD 过冲 → 末端抖动. 0.4s 五次曲线渐变使运动平滑, 跟踪误差 < 2°.

**4. Teleport-based grasp (replacing `<weld>`) / Teleport 吸附 (替代 `<weld>`)**
- **EN** — MuJoCo 3.8.1's `<weld>` equality constraint between a free body (block) and a kinematic body (link_6) does not activate reliably. Workaround: each step, set `qpos[6:9] = gripper_site.xpos` and `qvel[6:12] = 0`. Physically equivalent to a perfect suction gripper.
- **CN** — MuJoCo 3.8.1 中 `<weld>` 约束在 free body (方块) 和 kinematic body (link_6) 之间无法可靠激活. 替代方案: 每步 `qpos[6:9] = gripper_site.xpos`, `qvel[6:12] = 0`. 物理上等价于完美吸盘.

**5. Numerical IK with damping / 阻尼数值 IK**
- **EN** — 6-DOF analytical IK is awkward for non-spherical-wrist designs like GoFa. Finite-difference Jacobian (ε=5e-4) + damped least squares (λ=0.05) + joint-limit clamping → 1 mm convergence in 100-500 iterations.
- **CN** — GoFa 是非球腕设计, 6-DOF 解析解复杂. 有限差分 Jacobian (ε=5e-4) + 阻尼最小二乘 (λ=0.05) + 关节限位 clamp → 100-500 次迭代达 1mm 精度.

---

## Interactive Viewer Keys | 交互查看器快捷键

| Key / 按键 | Action / 动作 |
|-----|--------|
| `Space` | Pause / resume (inspect joints) / 暂停 / 恢复 (检查关节角) |
| `Tab` | Toggle debug panel (qpos / qvel / ctrl) / 切换调试面板 |
| `Esc` | Quit viewer / 退出 |
| Left drag / 左键拖拽 | Rotate camera / 旋转视角 |
| Right drag / 右键拖拽 | Pan camera / 平移视角 |
| Scroll / 滚轮 | Zoom / 缩放 |

**EN** — In `--view` mode, the script **auto-loops**: complete cycle → hold 2 s → reset → next cycle. Press Space at any moment to freeze and inspect.
**CN** — `--view` 模式下脚本**自动循环**: 完成一次 → 暂停 2s → 复位 → 下一次. 任意时刻按 Space 冻结检查.

---

## Dependencies | 依赖

```bash
conda create -n mujoco -y
conda activate mujoco
pip install mujoco numpy pillow matplotlib
```

**EN** — No ROS2 required for P4. (ROS2 + MoveIt2 integration is P5, see workspace root.)
**CN** — P4 无需 ROS2. (ROS2 + MoveIt2 集成在 P5, 见 workspace 根目录.)

---

## Documentation | 文档索引

| Doc / 文档 | Purpose / 用途 |
|-----|---------|
| [README.md](README.md) | This file / 本文件 — project entry point / 项目入口 |
| [docs/reports/p4_pick_place_技术详解.md](../../docs/reports/p4_pick_place_技术详解.md) | Full technical notes / 完整技术详解 (XML 结构, IK 推导, FSM 细节, 排错) |
| [docs/reports/脚本启动命令.md](../../docs/reports/脚本启动命令.md) | All launch commands / 全部启动命令 |
| [CLAUDE.md](../../CLAUDE.md) | Lessons learned / 经验教训, debugging tips / 调试技巧 |
| [../../README.md](../../README.md) | Workspace overview / 工作区概览, P1-P5 status / 阶段状态 |

---

## Next Steps → P5 | 下一步

**EN** — P4 is the **pure-MuJoCo baseline**. P5 layers MoveIt2 on top:
- Replace position actuator + manual IK with `JointTrajectoryController` + MoveIt2 `FollowJointTrajectory` action
- Use MoveIt2 L1 planning (RRT*/OMPL) instead of hand-written waypoints
- Verify L3→L4 end-to-end (joint trajectory → ros2_control → MuJoCo)

See [`p5_moveit2_learning/`](../../p5_moveit2_learning/) for the 6-week roadmap.

**CN** — P4 是**纯 MuJoCo 基线**. P5 在此之上叠加 MoveIt2:
- 位置执行器 + 手写 IK → `JointTrajectoryController` + MoveIt2 `FollowJointTrajectory` action
- 手写 waypoint → MoveIt2 L1 规划 (RRT*/OMPL)
- L3→L4 端到端验证 (joint trajectory → ros2_control → MuJoCo)

6 周路线图见 [`p5_moveit2_learning/`](../../p5_moveit2_learning/).
