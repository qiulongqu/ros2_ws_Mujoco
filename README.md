# Mujoco 学习项目

循序渐进学习 **MuJoCo 物理仿真 + ROS2 机器人控制**, 围绕 ABB GoFa CRB15000 机械臂展开.

> **最新更新**: 2026-06-09 — P4 step2 8 阶段状态机跑通, 终点误差 15mm; README 拉通到 P5.
> **完整启动命令**: 详见 [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md)
> **经验教训**: 详见 [`CLAUDE.md`](CLAUDE.md)

---

## 项目背景

- **目标**: 找工作 (机器人仿真/ROS2 方向)
- **偏好**: 机械臂 + 演示/原理兼顾
- **时间**: 每周 5~15 小时
- **基础**: Python/C++ 语法 OK, ROS2 入门, Mujoco 零基础起步

## 学习路径 (5 阶段) 与当前进度

| 阶段 | 内容 | 状态 | 验收 |
|------|------|------|------|
| **P1** | MuJoCo Python API 入门 | ✅ 完成 | 自由落体空中误差 0.006m, 单摆能量漂移 0J, 弹簧峰值误差 0.4% |
| **P2** | ROS2 + MuJoCo 桥接 | ✅ 完成 | 开环 + 订阅/发布 + 双向桥接 4 步全过 |
| **P3** | 机械臂模型 (GoFa CRB15000) | ✅ 完成 | 6-DOF XML + PTP 5 目标稳态误差 < 2° |
| **P4** | 拣放操作 (Pick-and-Place) | ✅ 完成 | 8 阶段状态机 + IK, 终点 15mm, 关节最大跟踪 1.7° |
| **P5** | MoveIt2 学习 | 🟡 规划中 | 概念笔记 + 路线图已写, 集成 demo 待做 |

---

## 目录结构

```
mujocoONLY/
├── tutorials/                          # 4 阶段教程 (MuJoCo 为主)
│   ├── p1_hello_mujoco/                ✅ 3 个物理实验 + README
│   ├── p2_ros2_bridge/                 ✅ 4 步 ROS2 桥接 + README
│   ├── p3_arm_model/                   ✅ 6-DOF 模型 + 3 个 step + README
│   └── p4_pick_place/                  ✅ 场景 + 8 阶段状态机 + README
├── src/                                # ROS2 包 (colcon workspace)
│   ├── gofa_mujoco_bringup/            # MuJoCo + ros2_control 桥接
│   └── gofa_moveit_config/             # MoveIt2 规划配置
├── p5_moveit2_learning/                # P5: MoveIt2 独立学习目录
│   ├── docs/  src/  launch/  config/  scripts/  notes/
│   └── README.md                       # 6 周路线图
├── resources/                          # 渲染输出图
│   ├── p1/  p2/  p3/  p4/              # 各阶段输出
│   └── _debug/                         # 调试临时
├── docs/
│   ├── plans/                          # 实施计划
│   ├── references/                     # 参考资料 (GoFa 配置文件说明)
│   └── reports/                        # 工作报告 + 启动命令
├── logs/                               # 工作日志
├── CLAUDE.md                           # 经验教训记录 (强制阅读)
├── README.md                           # 本文件
└── .gitignore                          # 忽略 build/install/log
```

---

## P1~P4 成果快照

### P1 基础物理

| 脚本 | 内容 | 验收 |
|------|------|------|
| `01_hello_mujoco.py` | 自由落体 + 弹跳 | 空中误差 0.006m |
| `02_pendulum.py` | 单摆 (大角度) | 能量漂移 0J |
| `03_spring_damper.py` | 弹簧阻尼 (ζ=0.1) | 峰值误差 0.4%, 周期误差 0.0005s |

### P2 ROS2 桥接

| 脚本 | 内容 | 模式 |
|------|------|------|
| `step1_openloop_control.py` | 开环力矩控制 | 纯 MuJoCo |
| `step2_ros2_subscriber.py` | ROS2 订阅驱动 | `--test` / ROS2 |
| `step3_joint_publisher.py` | 发布 /joint_state | `--test` / ROS2 |
| `step4_full_bridge.py` | 完整双向桥接 | `--test` / ROS2 |

### P3 机械臂

| 脚本 | 内容 | 验收 |
|------|------|------|
| `step1_verify_model.py` | 加载 + 5 项测试 | 6 body / 6 joint / 6 actuator |
| `step2_interactive_viewer.py` | 官方 viewer | 鼠标交互, Space 暂停 |
| `step3_ptp_move.py` | PTP (S-curve 插值) | 5 目标稳态 < 2° |
| `step3_trajectory_control.py` | 轨迹跟踪 (qfrc_bias FF) | 跟踪误差 < 5° |

### P4 拣放 (8 阶段状态机)

| 阶段 | 名称 | 触发 |
|------|------|------|
| 1 | PRE_GRASP | 移动到拣取位上方 10cm |
| 2 | APPROACH | 下降到拣取位 |
| 3 | GRASP | 激活 teleport 吸附 |
| 4 | LIFT | 提升到安全高度 |
| 5 | PRE_PLACE | 移动到放置位上方 |
| 6 | DESCEND | 下降到放置位 |
| 7 | RELEASE | 停用吸附 |
| 8 | RETREAT | 撤退到安全高度 |

**验收 (2026-06-09)**: 全程 3.04s, 终点误差 15.0mm, 关节最大跟踪 1.7°, quintic S-curve 插值消除阶段间抖动.

---

## 快速开始

### 0. 安装 MuJoCo 环境 (一次)

```bash
conda create -n mujoco -y
conda activate mujoco
pip install mujoco numpy glfw pillow matplotlib
```

### 1. 跑最简单的 P1 验证物理

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p1_hello_mujoco/01_hello_mujoco.py
```

### 2. 跑 P4 拣放看完整效果

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p4_pick_place/step1_build_scene.py       # 7 项场景测试
python3 tutorials/p4_pick_place/step2_manual_pick_place.py # 8 阶段跑通 + 轨迹图
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view  # 交互查看器 (自动循环)
```

### 3. ROS2 + MoveIt2 联合启动

```bash
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
colcon build --packages-select gofa_mujoco_bringup gofa_moveit_config
source install/setup.bash

# MuJoCo + ros2_control 基础
ros2 launch gofa_mujoco_bringup gofa_mujoco.launch.py

# 完整 MoveIt2 + MuJoCo + RViz
ros2 launch gofa_moveit_config demo.launch.py
# 4 种 GUI 组合见 docs/reports/脚本启动命令.md §5.3
```

### 4. 发送位置命令 (ForwardCommandController)

```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states
```

> **完整启动命令** (含 `--test` / `--view` / `--demo` / `--joints` 等参数) 见 [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md)

---

## 环境配置详解

### 为什么 Mujoco 和 ROS2 需要独立环境

ROS2 的 `PYTHONPATH` 包含 `/opt/ros/humble/...`, 与 MuJoCo 的 Python 3.13 环境冲突. 解决:
- 跑 MuJoCo 脚本时: `conda activate mujoco`
- 跑 ROS2 launch 时: `source /opt/ros/humble/setup.bash`
- **不要混用**

### WSL2 下的显示问题

WSL2 (无论 WSLg 还是 VcXsrv) 下 MuJoCo viewer 窗口可能黑屏, 但保存的 PNG 正常. **以 PNG 为准**. 不要尝试 `MUJOCO_GL=egl` 等方案, WSL2 下不生效. 详见 `CLAUDE.md` 的 "WSL2 + Mujoco 显示问题" 章节.

---

## 文档索引

| 文档 | 用途 |
|------|------|
| [`README.md`](README.md) | 本文件, 项目顶层入口 |
| [`CLAUDE.md`](CLAUDE.md) | 经验教训, 必读 |
| [`docs/reports/脚本启动命令.md`](docs/reports/脚本启动命令.md) | 所有脚本的启动命令, 强制落库 |
| [`docs/plans/2026-06-06_step3_implementation_plan_v2.md`](docs/plans/) | Step3 实施计划 |
| [`docs/references/GoFa_CRB15000_Config_Files.md`](docs/references/) | GoFa 配置文件说明 |
| [`p5_moveit2_learning/README.md`](p5_moveit2_learning/README.md) | P5 6 周路线图 |

各阶段子目录还有独立 README:
- `tutorials/p1_hello_mujoco/README.md`
- `tutorials/p2_ros2_bridge/README.md`
- `tutorials/p3_arm_model/README.md`
- `tutorials/p4_pick_place/README.md`

---

## 当前任务与待办

> 完整 TaskList 通过 Claude Code `TaskList` 工具查看. 核心待办:

- [ ] **P4 viewer 模式实测** — `--view` 模式下 quintic 插值未人工确认 (test 模式已验)
- [ ] **CLAUDE.md P2-P5 实战教训补全** — IK / quintic / teleport / 工业 4 层架构沉淀
- [ ] **P5 集成 demo** — MoveIt2 → JointTrajectoryController → MuJoCo 闭环

---

## 依赖

- mujoco >= 3.0
- numpy
- glfw (可视化窗口)
- PIL (图像保存)
- matplotlib (轨迹图)
- ROS2 Humble + MoveIt2 Humble (P5 起)

---

## 远程推送

本地 git 已 commit, GitHub 远程因网络问题未推送. 网络恢复后:

```bash
git push -u origin main   # 首次
git push                  # 之后
```

详见 `docs/reports/脚本启动命令.md` §8.
