# Mujoco 学习项目

循序渐进学习 MuJoCo 物理仿真 + ROS2 机器人控制。

## 项目背景

- **目标**：找工作（机器人仿真/ROS2方向）
- **偏好**：机械臂 + 演示+原理兼顾
- **时间**：每周 5~15 小时
- **基础**：Python/C++ 语法OK，ROS2 入门，Mujoco 零基础

## 学习路径（5阶段）

| 阶段 | 内容 | 目标产出 |
|------|------|---------|
| **P1** | Mujoco Python API 入门 | ✅ 完成 — 自由落体、单摆、弹簧阻尼 |
| **P2** | ROS2 + Mujoco 桥接 | ✅ 完成 — 开环控制、ROS2 订阅/发布、完整双向桥接 |
| **P3** | 机械臂模型构建 | ✅ 完成 — MuJoCo XML 模型 + ros2_control 桥接 |
| **P4** | 拣放操作 | ✅ 完成 — 场景构建 + 手动 pick-and-place |
| **P5** | MoveIt2 学习 | 🚧 进行中 — 运动规划 + 轨迹执行 |

## 目录结构

```
mujocoONLY/
├── tutorials/
│   ├── p1_hello_mujoco/       ✅
│   │   ├── 01_hello_mujoco.py
│   │   ├── 02_pendulum.py
│   │   ├── 03_spring_damper.py
│   │   └── README.md
│   ├── p2_ros2_bridge/        ✅
│       ├── models/
│       ├── scripts/
│       └── ros2_pkg/
│   ├── p3_arm_model/          ✅
│       ├── gofa_crb15000.xml   # MuJoCo 模型
│       ├── step1_verify_model.py
│       ├── urdf/              # URDF/XACRO 源文件
│       ├── meshes/            # STL + DAE
│       └── config/
│   └── p4_pick_place/         🚧
│       ├── gofa_table_block.xml
│       ├── step1_build_scene.py
│       ├── step2_manual_pick_place.py
│       └── README.md
├── src/                       # ROS2 包 (colcon workspace)
│   ├── gofa_mujoco_bringup/   # MuJoCo + ros2_control 桥接
│   └── gofa_moveit_config/    # MoveIt2 规划配置
├── p5_moveit2_learning/       # P5: MoveIt2 学习项目
│   ├── docs/                  # 学习文档、概念笔记
│   ├── src/                   # 教程练习代码
│   ├── launch/                # Launch 文件
│   ├── config/                # 配置文件
│   ├── scripts/               # 辅助脚本
│   └── notes/                 # 个人学习笔记
├── resources/                 # 渲染输出图
│   ├── p1/ p2/ p3/ p4/
│   └── _debug/                # 调试临时文件
├── docs/                      # 项目文档
│   ├── plans/                 # 实施计划
│   ├── reports/               # 工作报告
│   └── references/            # 参考资料
├── logs/                      # 工作日志
├── CLAUDE.md                  # 学习经验教训记录
└── README.md                  # 本文件
```

## P1 成果

- **P1-01** 自由落体：空中误差 0.006m，弹跳物理正确
- **P1-02** 单摆：能量漂移 0.0 J，能量守恒完美
- **P1-03** 弹簧阻尼：峰值包络误差 0.4%，周期误差 0.0005s

## 环境配置

Mujoco 和 ROS2 需要独立环境，避免包冲突。

### 创建 Mujoco 环境

```bash
conda create -n mujoco -y
conda activate mujoco
pip install mujoco numpy glfw pillow
```

以后所有 Mujoco 脚本用 `mujoco` 环境：

```bash
conda activate mujoco
python3 tutorials/p1_hello_mujoco/01_hello_mujoco.py
```

### ROS2 + Mujoco 联合仿真（P2 起）

两个 terminal 分别跑：

```
# Terminal 1 — ROS2 环境
conda deactivate
source /opt/ros/humble/setup.bash
ros2 launch ...

# Terminal 2 — Mujoco 环境
conda activate mujoco
python3 scripts/mujoco_bridge.py
```

## 运行方式

### P1~P4 纯 MuJoCo 脚本

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p1_hello_mujoco/01_hello_mujoco.py
python3 tutorials/p1_hello_mujoco/02_pendulum.py
python3 tutorials/p1_hello_mujoco/03_spring_damper.py
```

### P4 Pick-and-Place

```bash
# 场景验证
python3 tutorials/p4_pick_place/step1_build_scene.py
# 交互查看器
python3 tutorials/p4_pick_place/step1_build_scene.py --view

# 手动 pick-and-place (8 阶段状态机 + IK)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py
# 交互查看器 (自动循环)
python3 tutorials/p4_pick_place/step2_manual_pick_place.py --view
```

### ROS2 + MuJoCo + ros2_control + RViz 联合启动

**构建：**

```bash
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
source /opt/ros/humble/setup.bash
colcon build --packages-select gofa_mujoco_bringup gofa_moveit_config
source install/setup.bash
```

**启动（支持 MuJoCo 和 RViz 独立开关）：**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 模式 1: 两个窗口都开（默认）
ros2 launch gofa_moveit_config demo.launch.py

# 模式 2: 只开 MuJoCo 窗口，不开 RViz
ros2 launch gofa_moveit_config demo.launch.py rviz_gui:=false

# 模式 3: 只开 RViz，MuJoCo 在后台运行（headless）
ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false

# 模式 4: 两个都不开（纯后台运行）
ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false
```

**发送位置控制命令：**

```bash
# 设置 6 个关节的目标角度（弧度）
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]"
```

**查看关节状态：**

```bash
ros2 topic echo /joint_states
```

## 依赖

- mujoco >= 3.0
- numpy
- glfw（可视化窗口）
- PIL（图像保存）