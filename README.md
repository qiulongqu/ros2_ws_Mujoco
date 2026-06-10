# MuJoCo + ROS2 Learning Project · MuJoCo + ROS2 学习项目

**Mastering physics simulation and robot control, one phase at a time.**
**循序渐进掌握物理仿真与机器人控制.**

![status](https://img.shields.io/badge/status-active-brightgreen) ![p1--p4](https://img.shields.io/badge/P1--P4-passed-blue) ![p5](https://img.shields.io/badge/P5-in_progress-yellow) ![ros2](https://img.shields.io/badge/ROS2-Humble-orange) ![mujoco](https://img.shields.io/badge/MuJoCo-3.x-green)

> **Latest · 最新**: 2026-06-10 — Root README rewritten as GitHub-homepage-quality bilingual doc.
> **Full launch commands · 完整启动命令**: [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md)
> **Lessons learned · 经验教训**: [`CLAUDE.md`](CLAUDE.md)

---

## Project Background | 项目背景

| Aspect / 维度 | Detail / 详情 |
|---|---|
| Goal / 目标 | Robotics simulation + ROS2 (job-seeking direction) / 机器人仿真 + ROS2 (求职方向) |
| Focus / 偏好 | Robotic arm + demo/theory balance / 机械臂 + 演示/原理兼顾 |
| Time budget / 时间 | 5–15 h/week / 每周 5~15 小时 |
| Background / 基础 | Python/C++ OK, ROS2 beginner, MuJoCo from zero / Python/C++ 语法 OK, ROS2 入门, MuJoCo 零基础起步 |
| Hardware / 硬件 | **ABB GoFa CRB15000-10/1.52** (6-DOF, 1.52 m reach) / ABB GoFa 6 轴机械臂 |

---

## Learning Path (5 Phases) | 学习路径 (5 阶段)

| # | Phase / 阶段 | Status / 状态 | Acceptance / 验收 |
|---|---|---|---|
| **P1** | MuJoCo Python API intro / MuJoCo Python API 入门 | ✅ Done / 完成 | Free fall 6mm error, pendulum 0J energy drift, spring 0.4% peak error |
| **P2** | ROS2 + MuJoCo bridge / ROS2 + MuJoCo 桥接 | ✅ Done / 完成 | Open-loop + sub/pub + full 2-way bridge, 4/4 steps |
| **P3** | Arm model (GoFa CRB15000) / 机械臂模型 | ✅ Done / 完成 | 6-DOF XML + PTP 5 targets, steady-state < 2° |
| **P4** | Pick-and-Place / 拣放操作 | ✅ Done / 完成 | 8-phase FSM + IK, place error 7mm, max joint error 1.7° |
| **P5** | MoveIt2 learning / MoveIt2 学习 | 🟡 In progress / 进行中 | JTC baseline + A2 e2e closed loop (2026-06-09), concept notes + L1 planning pending |

---

## Repository Structure | 目录结构

```
mujocoONLY/
├── tutorials/                          # 4 phases (MuJoCo-focused) | 4 阶段教程
│   ├── p1_hello_mujoco/                ✅ 3 physics experiments + README
│   ├── p2_ros2_bridge/                 ✅ 4-step ROS2 bridge + README
│   ├── p3_arm_model/                   ✅ 6-DOF model + 3 steps + README
│   └── p4_pick_place/                  ✅ Scene + 8-phase FSM + README
├── src/                                # ROS2 packages (colcon workspace) | ROS2 包
│   ├── gofa_mujoco_bringup/            # MuJoCo + ros2_control bridge | 桥接
│   └── gofa_moveit_config/             # MoveIt2 planning config | MoveIt2 配置
├── p5_moveit2_learning/                # P5: MoveIt2 standalone / P5 独立目录
│   ├── docs/  src/  launch/  config/  scripts/  notes/
│   └── README.md                       # 6-week roadmap / 6 周路线图
├── resources/                          # Rendered output images | 渲染输出
│   ├── p1/  p2/  p3/  p4/  p5/         # Per-phase outputs | 各阶段输出
│   └── _debug/                         # Debug scratch / 调试临时
├── docs/                               # Documentation hub | 文档枢纽
│   ├── plans/                          # Implementation plans / 实施计划
│   ├── references/                     # Reference materials / 参考资料
│   └── reports/                        # Reports, archived READMEs, launch commands / 工作报告
├── logs/                               # Work logs / 工作日志
├── CLAUDE.md                           # Lessons learned (mandatory read) / 经验教训
├── README.md                           # This file / 本文件
└── .gitignore                          # Ignore build/install/log / 忽略构建产物
```

---

## P1–P4 Results Snapshot | P1~P4 成果快照

### P1 — Fundamental Physics / 基础物理

| Script / 脚本 | Content / 内容 | Acceptance / 验收 |
|------|------|------|
| `01_hello_mujoco.py` | Free fall + bouncing / 自由落体 + 弹跳 | Air error 6mm / 空中误差 6mm |
| `02_pendulum.py` | Pendulum (large angle) / 单摆 (大角度) | Energy drift 0J / 能量漂移 0J |
| `03_spring_damper.py` | Spring-damper (ζ=0.1) / 弹簧阻尼 | Peak err 0.4%, period err 0.0005s / 峰值误差 0.4% |

### P2 — ROS2 Bridge / ROS2 桥接

| Script / 脚本 | Content / 内容 | Mode / 模式 |
|------|------|------|
| `step1_openloop_control.py` | Open-loop torque ctrl / 开环力矩 | Pure MuJoCo / 纯 MuJoCo |
| `step2_ros2_subscriber.py` | ROS2 sub-driven / 订阅驱动 | `--test` / ROS2 |
| `step3_joint_publisher.py` | Publish /joint_state / 发布 | `--test` / ROS2 |
| `step4_full_bridge.py` | Full 2-way bridge / 双向桥接 | `--test` / ROS2 |

### P3 — Arm Model / 机械臂

| Script / 脚本 | Content / 内容 | Acceptance / 验收 |
|------|------|------|
| `step1_verify_model.py` | Load + 5 tests / 加载 + 5 项 | 6 body / 6 joint / 6 actuator |
| `step2_interactive_viewer.py` | Official viewer / 官方查看器 | Mouse + Space pause / 鼠标 + Space 暂停 |
| `step3_ptp_move.py` | PTP (S-curve) / 关节空间 PTP | 5 targets < 2° / 5 目标 < 2° |
| `step3_trajectory_control.py` | Traj tracking (qfrc_bias FF) / 轨迹跟踪 | Error < 5° / 误差 < 5° |

### P4 — Pick-and-Place (8-phase FSM) / 拣放 (8 阶段状态机)

| # | Phase / 阶段 | Trigger / 触发 |
|---|---|---|
| 1 | `PRE_GRASP` | Move above pick point +10cm / 移动到拣取位上方 10cm |
| 2 | `APPROACH`  | Descend to pick point / 下降到拣取位 |
| 3 | `GRASP`     | Activate teleport suction / 激活 teleport 吸附 |
| 4 | `LIFT`      | Move to safe altitude / 提升到安全高度 |
| 5 | `PRE_PLACE` | Move above place point / 移动到放置位上方 |
| 6 | `DESCEND`   | Descend to place point / 下降到放置位 |
| 7 | `RELEASE`   | Deactivate suction / 停用吸附 |
| 8 | `RETREAT`   | Return to safe altitude / 撤退到安全高度 |

**Acceptance (2026-06-09) / 验收**: cycle 3.04s, place error 7.0mm, max joint tracking 1.7°, quintic S-curve eliminates phase-transition shake / quintic S 曲线消除阶段间抖动.

---

## Quick Start | 快速开始

### 0. Install MuJoCo env (once) / 安装 MuJoCo 环境 (一次)

```bash
conda create -n mujoco -y
conda activate mujoco
pip install mujoco numpy glfw pillow matplotlib
```

### 1. P1 physics in 10 s / 10 秒跑通 P1

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p1_hello_mujoco/01_hello_mujoco.py
```

### 2. P4 full pipeline / 跑 P4 拣放看完整效果

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p4_pick_place/step1_build_scene.py       # 7 scene tests
python3 tutorials/p4_pick_place/step2_manual_pick_place.py # 8-phase run + plot
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view  # viewer, auto-loop
```

### 3. ROS2 + MoveIt2 joint launch / ROS2 + MoveIt2 联合启动

```bash
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
colcon build --packages-select gofa_mujoco_bringup gofa_moveit_config
source install/setup.bash

# MuJoCo + ros2_control baseline / 基础
ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py

# Full MoveIt2 + MuJoCo + RViz / 完整 MoveIt2
ros2 launch gofa_moveit_config demo.launch.py
# 4 GUI combos: see docs/reports/脚本启动命令.md §5.3
```

### 4. Send position cmd (ForwardCommandController) / 发送位置命令

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states
```

> **Full launch commands** (with `--test` / `--view` / `--demo` / `--joints` flags) see [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md)

---

## Environment Configuration | 环境配置详解

### Why MuJoCo and ROS2 need separate envs / 为什么需要独立环境

- **EN** — ROS2's `PYTHONPATH` includes `/opt/ros/humble/...`, conflicting with MuJoCo's Python 3.13 env. Use `conda activate mujoco` for MuJoCo scripts and `source /opt/ros/humble/setup.bash` for ROS2 launch. **Don't mix.**
- **CN** — ROS2 的 `PYTHONPATH` 包含 `/opt/ros/humble/...`, 与 MuJoCo 的 Python 3.13 环境冲突. 跑 MuJoCo 时 `conda activate mujoco`, 跑 ROS2 时 `source /opt/ros/humble/setup.bash`, **不要混用**.

### WSL2 display issue / WSL2 显示问题

- **EN** — Under WSL2 (WSLg or VcXsrv) the viewer window may be black, but saved PNGs are normal. **Trust the PNGs.** Don't try `MUJOCO_GL=egl` — ineffective under WSL2. See `CLAUDE.md` "WSL2 + Mujoco 显示问题" section.
- **CN** — WSL2 下 viewer 窗口可能黑屏, 但 PNG 正常. **以 PNG 为准**. 不要尝试 `MUJOCO_GL=egl`, WSL2 下不生效. 详见 `CLAUDE.md`.

---

## Documentation Index | 文档索引

| Doc / 文档 | Purpose / 用途 |
|---|---|
| [`README.md`](README.md) | This file / 本文件 — project top-level entry / 项目顶层入口 |
| [`CLAUDE.md`](CLAUDE.md) | Lessons learned, must-read / 经验教训, 必读 |
| [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md) | All launch commands (mandatory archive) / 全部启动命令 |
| [`docs/reports/root_README_原版.md`](docs/reports/root_README_原版.md) | Archived pre-bilingual root README / 归档的中文版根 README |
| [`docs/reports/p4_pick_place_技术详解.md`](docs/reports/p4_pick_place_技术详解.md) | P4 full technical notes (XML, IK, FSM) / P4 技术详解 |
| [`docs/plans/`](docs/plans/) | Implementation plans / 实施计划 |
| [`docs/references/`](docs/references/) | Reference materials (GoFa config) / 参考资料 |
| [`p5_moveit2_learning/README.md`](p5_moveit2_learning/README.md) | P5 6-week roadmap / P5 6 周路线图 |

**Per-phase sub-READMEs / 各阶段子 README (bilingual):**
- [`tutorials/p1_hello_mujoco/README.md`](tutorials/p1_hello_mujoco/README.md)
- [`tutorials/p2_ros2_bridge/README.md`](tutorials/p2_ros2_bridge/README.md)
- [`tutorials/p3_arm_model/README.md`](tutorials/p3_arm_model/README.md)
- [`tutorials/p4_pick_place/README.md`](tutorials/p4_pick_place/README.md) (bilingual ✅)

---

## Current Tasks & TODO | 当前任务与待办

> Full task list via Claude Code `TaskList` tool. Core items: / 完整 TaskList 通过工具查看, 核心项:

- [x] **P5 baseline closed-loop (2026-06-09)** / P5 baseline 闭环 — JTC + move_group + start validation
- [x] **P5 A2 end-to-end (2026-06-09)** / P5 A2 端到端 — L3→L4 direct test PASS (5 waypoints, 7.5s, 0.43° err)
- [ ] **P5 L1 planning fix** / P5 L1 规划修复 — `/plan_kinematic_path` "Skipping invalid start state", use `/move_action`
- [ ] **P5 concept notes** / P5 概念笔记 — Read MoveIt2 official 5 concept docs, write to `p5_moveit2_learning/docs/concept_notes/`
- [ ] **P5 RViz end-to-end** / P5 RViz 端到端 — Plan + Execute once in RViz, screenshot to `resources/p5/`
- [ ] **P4 end-effector shake fix** / P4 末端抖动修复 — quintic interpolation not yet validated in `--view` mode (test mode verified)

---

## Dependencies | 依赖

| Package / 包 | Version / 版本 | Purpose / 用途 |
|---|---|---|
| mujoco | >= 3.0 | Physics engine / 物理引擎 |
| numpy | latest | Numerical computing / 数值计算 |
| glfw | latest | Window/viewer / 窗口 |
| PIL | latest | Image save / 图像保存 |
| matplotlib | latest | Trajectory plots / 轨迹图 |
| ROS2 Humble | LTS | Middleware (P2+) / 中间件 |
| MoveIt2 Humble | LTS | Motion planning (P5) / 运动规划 |

---

## Remote Push | 远程推送

- **EN** — Local git is committed. GitHub remote push pending due to network issues. When network is restored:
- **CN** — 本地 git 已 commit, GitHub 远程因网络问题未推送. 网络恢复后:

```bash
git push -u origin main   # first time / 首次
git push                  # subsequent / 之后
```

See `docs/reports/脚本启动命令.md` §8 for details.

---

## License & Credits | 许可与致谢

- **EN** — Learning project, free to fork. ABB GoFa URDF/XACRO sources from `abb_omnicore_ros2`. MuJoCo engine: Google DeepMind.
- **CN** — 学习项目, 欢迎 fork. ABB GoFa URDF/XACRO 源文件来自 `abb_omnicore_ros2`. MuJoCo 引擎: Google DeepMind.
