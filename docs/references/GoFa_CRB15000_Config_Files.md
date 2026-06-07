# GoFa CRB 15000-10/1.52 运动仿真配置文件清单

> 源码根路径: `/home/yunhao2204/ros2_ws_abb/abb_gofa_battery/src/abb_omnicore_ros2`

---

## Package 1: `abb_resources` — 底层公共资源

| # | 文件路径 | 作用 |
|---|---------|------|
| 1 | `abb_resources/package.xml` | ROS2 package 元信息 |
| 2 | `abb_resources/CMakeLists.txt` | 编译配置 |
| 3 | `abb_resources/urdf/common_materials.xacro` | 定义 ABB 标准材质（黄色涂装等） |
| 4 | `abb_resources/urdf/common_colours.xacro` | 定义 ABB 标准颜色 |

---

## Package 2: `robot_specific_config/abb_gofa_crb15000_support` — 机械臂描述包（核心）

### URDF/XACRO 文件

| # | 文件路径 | 作用 |
|---|---------|------|
| 5 | `robot_specific_config/abb_gofa_crb15000_support/package.xml` | ROS2 package 元信息 |
| 6 | `robot_specific_config/abb_gofa_crb15000_support/CMakeLists.txt` | 编译配置（安装 mesh/urdf/launch 目录） |
| 7 | `robot_specific_config/abb_gofa_crb15000_support/urdf/gofa_crb15000_10_152.xacro` | **顶层入口文件** — 声明参数（use_fake_hardware/rws_ip），include macro + ros2_control |
| 8 | `robot_specific_config/abb_gofa_crb15000_support/urdf/gofa_crb15000_10_152_macro.xacro` | **核心 URDF 宏** — 6 个 link（质量+惯量+碰撞模型+视觉模型）+ 6 个 revolute joint（角度限位+速度限制）+ base/flange/tool0 固定坐标系 |
| 9 | `robot_specific_config/abb_gofa_crb15000_support/urdf/gofa_crb15000.ros2_control.xacro` | **ros2_control 硬件接口** — 6 关节 position/velocity command_interface + state_interface；fake_hardware 用 mock_components，真机用 abb_hardware_interface + EGM(port 6511) |
| 10 | `robot_specific_config/abb_gofa_crb15000_support/urdf/gofa_crb15000.muJoCo.ros2_control.xacro` | **MuJoCo 仿真硬件接口** — 使用 mujoco_ros2_control/MujocoSystemInterface 插件替代真实硬件 |

### 碰撞模型（STL 网格）

| # | 文件路径 | 作用 |
|---|---------|------|
| 11 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/base_link.stl` | 底座碰撞模型 |
| 12 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_1.stl` | 关节1碰撞模型 |
| 13 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_2.stl` | 关节2碰撞模型 |
| 14 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_3.stl` | 关节3碰撞模型 |
| 15 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_4.stl` | 关节4碰撞模型 |
| 16 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_5.stl` | 关节5碰撞模型 |
| 17 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/collision/link_6.stl` | 关节6碰撞模型 |

### 视觉模型（DAE/STL 网格，用于 RViz 渲染）

| # | 文件路径 | 作用 |
|---|---------|------|
| 18 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/base_link.dae` | 底座视觉模型（DAE） |
| 19 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_1.dae` | 关节1视觉模型 |
| 20 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_2.dae` | 关节2视觉模型 |
| 21 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_3.dae` | 关节3视觉模型 |
| 22 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_4.dae` | 关节4视觉模型 |
| 23 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_5.dae` | 关节5视觉模型 |
| 24 | `robot_specific_config/abb_gofa_crb15000_support/meshes/gofa_crb15000_10_152/visual/link_6.dae` | 关节6视觉模型 |

### 配置

| # | 文件路径 | 作用 |
|---|---------|------|
| 25 | `robot_specific_config/abb_gofa_crb15000_support/config/joint_names_gofa_crb15000_10_152.yaml` | 关节名称列表（joint_1 ~ joint_6） |

---

## Package 3: `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config` — MoveIt2 配置包

### 机器人描述文件

| # | 文件路径 | 作用 |
|---|---------|------|
| 26 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/package.xml` | ROS2 package 元信息 |
| 27 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/CMakeLists.txt` | 编译配置（安装 config/launch 目录） |
| 28 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/gofa_crb15000_10_152.urdf.xacro` | MoveIt 用 xacro 入口，include abb_gofa_crb15000_support 的顶层 xacro |
| 29 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/gofa_crb15000_10_152.mujoco.urdf.xacro` | MuJoCo 仿真用 xacro 入口 |
| 30 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/gofa_crb15000_10_152.srdf` | **语义描述文件** — 定义 manipulator 规划组（base_link→tool0），home 位姿，碰撞禁用对（10 对），虚拟关节（world→base_link） |

### 运动规划配置

| # | 文件路径 | 作用 |
|---|---------|------|
| 31 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/kinematics.yaml` | **IK 求解器** — 配置 kinematics_solver（默认 KDL），求解超时/尝试次数 |
| 32 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/joint_limits.yaml` | **关节限位** — 6 关节的 max_position/min_position + max_velocity/max_acceleration |
| 33 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/ompl_planning.yaml` | **运动规划** — OMPL 规划器参数（RRT/RRTConnect/RRTstar/PRM 等） |
| 34 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/pilz_cartesian_limits.yaml` | **笛卡尔限位** — 末端在 XYZ 方向的平移/旋转速度限制 |
| 35 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/initial_positions.yaml` | **初始关节值** — 启动时各关节的默认位置 |

### 控制器配置

| # | 文件路径 | 作用 |
|---|---------|------|
| 36 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/moveit_controllers.yaml` | MoveIt 控制器接口（FollowJointTrajectory action 映射） |
| 37 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/ros2_controllers.yaml` | ros2_control 控制器 spawn 配置（joint_state_broadcaster + abb_controller） |

### 传感器配置

| # | 文件路径 | 作用 |
|---|---------|------|
| 38 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/config/sensors_3d.yaml` | 3D 传感器配置（深度相机等，可选） |

### Launch 文件

| # | 文件路径 | 作用 |
|---|---------|------|
| 39 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/rsp.launch.py` | 启动 robot_state_publisher（解析 xacro → 发布 TF） |
| 40 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/move_group.launch.py` | 启动 move_group 节点 |
| 41 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/moveit.launch.py` | **MoveIt2 主启动文件** — 组合 rsp + move_group + RViz |
| 42 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/demo.launch.py` | **演示启动** — 一键启动仿真（fake_hardware + moveit + RViz） |
| 43 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/mujoco_demo.launch.py` | MuJoCo 仿真演示启动 |
| 44 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/spawn_controllers.launch.py` | 启动 ros2_control 控制器 |
| 45 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/static_virtual_joint_tfs.launch.py` | 发布 world→base_link 虚拟关节 TF |
| 46 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/moveit_rviz.launch.py` | 启动 MoveIt2 的 RViz 配置 |

### RViz 配置

| # | 文件路径 | 作用 |
|---|---------|------|
| 47 | `robot_specific_config/abb_gofa_crb15000_10_152_moveit_config/launch/moveit.rviz` | RViz 面板布局和显示配置 |

---

## 附: abb_bringup 总启动文件（参考，非必要）

| # | 文件路径 | 作用 |
|---|---------|------|
| 48 | `abb_bringup/launch/abb_bringup.launch.py` | 项目总启动（组合 control + moveit） |
| 49 | `abb_bringup/launch/abb_control.launch.py` | 用 xacro 生成 robot_description，启动 ros2_control + robot_state_publisher |
| 50 | `abb_bringup/config/abb_controllers.yaml` | 控制器参数配置 |

---

## 依赖关系图

```
abb_resources/
├── common_materials.xacro ───────────────────────────────┐
└── common_colours.xacro                                 │
                                                          ▼
abb_gofa_crb15000_support/
├── gofa_crb15000_10_152_macro.xacro  ← 引用 common_materials
│   ├── mesh 文件 (collision/*.stl + visual/*.dae)  ← 被 macro 引用
│   └── gofa_crb15000_10_152.xacro  ← include macro + ros2_control
│       ├── gofa_crb15000.ros2_control.xacro     (真机)
│       └── gofa_crb15000.muJoCo.ros2_control.xacro  (MuJoCo 仿真)
│
└─→ abb_gofa_crb15000_10_152_moveit_config/
    ├── gofa_crb15000_10_152.urdf.xacro  ← include 上面的顶层 xacro
    ├── gofa_crb15000_10_152.srdf         ← 规划组/碰撞禁用/home位姿
    ├── kinematics.yaml                   ← IK 求解
    ├── joint_limits.yaml                 ← 关节约束
    ├── ompl_planning.yaml                ← 运动规划
    ├── moveit_controllers.yaml           ← 控制接口
    └── launch/demo.launch.py             ← 一键启动仿真
```

---

## 在新项目中最小使用方式

1. 将上述 3 个 package 完整拷贝到新项目的 `src/` 目录下
2. `colcon build` 编译
3. 启动:

```bash
# 纯 fake 仿真（无 MuJoCo，只显示模型 + MoveIt 规划）
ros2 launch abb_gofa_crb15000_10_152_moveit_config demo.launch.py

# MuJoCo 物理仿真
ros2 launch abb_gofa_crb15000_10_152_moveit_config mujoco_demo.launch.py
```
