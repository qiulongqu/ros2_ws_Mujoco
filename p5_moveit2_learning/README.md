# P5: MoveIt2 学习项目

基于官方教程和文档，系统学习 MoveIt2 运动规划框架，并最终集成到 MuJoCo + ros2_control 仿真环境中。

## 项目定位

- **前置条件**：已完成 P1~P4（MuJoCo 基础、ROS2 桥接、机械臂模型、Pick-and-Place）
- **学习目标**：掌握 MoveIt2 核心概念，实现 MuJoCo 仿真环境下的轨迹规划与执行
- **时间投入**：每周 5~10 小时，预计 4~6 周完成
- **参考标准**：优先遵循 ROS2 Humble + MoveIt2 官方文档和教程

## 目录结构

```
p5_moveit2_learning/
├── README.md                 # 本文件
├── docs/                     # 学习文档、笔记、参考资料
│   ├── official_tutorials/   # 官方教程整理
│   ├── concept_notes/        # 概念理解笔记
│   └── troubleshooting/      # 问题排查记录
├── src/                      # 源代码（待编写）
│   ├── moveit_cpp_tutorials/ # MoveIt C++ API 练习
│   ├── moveit_py_tutorials/  # MoveIt Python API 练习
│   └── gofa_moveit_demo/     # GoFa 机械臂 MoveIt2 集成演示
├── launch/                   # Launch 文件（待编写）
├── config/                   # 配置文件（待编写）
├── scripts/                  # 辅助脚本（待编写）
└── notes/                    # 个人学习笔记、心得
```

## 学习路线图

### Phase 1: MoveIt2 基础概念（第 1~2 周）

**目标**：理解 MoveIt2 的核心架构和基本概念，不接触代码。

| 主题 | 内容 | 官方参考 |
|------|------|----------|
| MoveIt2 架构 | Planning Pipeline、MoveGroup、Planning Scene、RobotState | [Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html) |
| 运动学 | FK/IK、Kinematics Plugin、KDL vs TRAC-IK | [Kinematics](https://moveit.picknik.ai/humble/doc/examples/kinematics_model/kinematics_model_tutorial.html) |
| 规划器 | OMPL、规划场景、约束规划 | [Motion Planning](https://moveit.picknik.ai/humble/doc/examples/motion_planning_api/motion_planning_api_tutorial.html) |
| 碰撞检测 | FCL、Planning Scene、Allowed Collision Matrix | [Planning Scene](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html) |
| 控制器集成 | FollowJointTrajectory、ros2_control 衔接 | [Controller](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) |

**产出**：
- `docs/concept_notes/` 下的概念笔记
- 画出 MoveIt2 控制链数据流图

### Phase 2: 官方教程实践（第 2~3 周）

**目标**：跑通官方教程，熟悉 API。

| 教程 | 语言 | 说明 |
|------|------|------|
| MoveIt Quickstart in RViz | RViz | 可视化交互入门 |
| Move Group C++ Interface | C++ | 编程控制机械臂 |
| Move Group Python Interface | Python | Python API 入门 |
| Motion Planning Python API | Python | 高级规划 API |
| Planning with Collision Objects | C++/Python | 障碍物避障规划 |
| Pick and Place | C++/Python | 抓取放置任务 |

**官方教程位置**：
- https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html
- `/opt/ros/humble/share/moveit2_tutorials/`（如已安装）

**产出**：
- `src/moveit_cpp_tutorials/` 和 `src/moveit_py_tutorials/` 下的练习代码
- 每个教程的运行截图/记录

### Phase 3: GoFa + MoveIt2 集成（第 3~4 周）

**目标**：将 MoveIt2 集成到现有的 GoFa MuJoCo 仿真环境中。

**需要完成的工作**：

1. **配置 MoveIt2 包**
   - 使用 `gofa_moveit_config` 包（已有基础配置）
   - 完善 `gofa.srdf`、`kinematics.yaml`、`ompl_planning.yaml`
   - 配置 `moveit_controllers.yaml`（连接 `JointTrajectoryController`）

2. **启动链路**
   ```
   demo_moveit.launch.py
   ├── robot_state_publisher
   ├── ros2_control_node (MuJoCo)
   ├── joint_state_broadcaster
   ├── joint_trajectory_controller  ← 替换 ForwardCommandController
   ├── move_group (MoveIt2)
   └── rviz2 (with MotionPlanning plugin)
   ```

3. **验证功能**
   - RViz 中 MotionPlanning 插件规划轨迹
   - Plan + Execute 到 MuJoCo 仿真
   - 添加碰撞物体进行避障规划

**产出**：
- `src/gofa_moveit_demo/` 下的集成代码
- `launch/demo_moveit.launch.py`
- 集成文档

### Phase 4: 高级主题（第 5~6 周，可选）

| 主题 | 说明 |
|------|------|
| MoveIt Servo | 实时笛卡尔速度控制 |
| MoveIt Task Constructor | 复杂任务编排 |
| Perception + MoveIt | 点云碰撞检测 |
| MoveIt Setup Assistant | 使用 GUI 配置新机械臂 |

## 控制链对比

### 当前模式（P4 完成）：直接控制

```
User → /position_controller/commands → ForwardCommandController
                                           ↓
                                    ros2_control_node → MuJoCo
                                           ↓
                                    /joint_states → RViz (RobotModel)
```

### MoveIt2 模式（P5 目标）：轨迹规划

```
User → RViz MotionPlanning / MoveGroup API
              ↓
        MoveIt2 move_group
              ↓
        /abb_controller/follow_joint_trajectory (action)
              ↓
        JointTrajectoryController
              ↓
        ros2_control_node → MuJoCo
              ↓
        /joint_states → RViz
```

## 关键概念速查

| 术语 | 含义 |
|------|------|
| **MoveGroup** | MoveIt2 的核心节点，提供规划、执行、状态查询的统一接口 |
| **Planning Scene** | 包含机器人状态、碰撞物体、约束的完整场景描述 |
| **SRDF** | Semantic Robot Description Format，定义规划组、末端执行器、碰撞矩阵 |
| **OMPL** | Open Motion Planning Library，MoveIt2 默认的规划算法库 |
| **FollowJointTrajectory** | ROS2 action 接口，MoveIt2 通过它向控制器发送轨迹 |
| **JointTrajectoryController** | ros2_control 的控制器，接收轨迹并按时间插值执行 |

## 官方资源（优先参考）

1. **MoveIt2 官方文档**：https://moveit.picknik.ai/humble/doc/index.html
2. **MoveIt2 Tutorials**：https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html
3. **ros2_control 文档**：https://control.ros.org/humble/doc/ros2_control/doc/index.html
4. **MoveIt2 GitHub**：https://github.com/ros-planning/moveit2
5. **MoveIt Setup Assistant**：`ros2 launch moveit_setup_assistant setup_assistant.launch.py`

## 学习检查清单

- [x] **Phase 3 baseline (2026-06-09)**: JTC + move_group.launch.py 配置闭环, controllers 全 active, action server 在线. 详见 `docs/troubleshooting/0001_JTC_blocking_point.md` 和 CLAUDE.md 5-06~5-10.
- [ ] Phase 1: 阅读完所有概念文档并做笔记
- [ ] Phase 2: 跑通所有官方 C++ 和 Python 教程
- [ ] Phase 3: GoFa + MoveIt2 集成成功，RViz 中可 Plan + Execute
- [ ] Phase 3: 碰撞物体避障规划验证通过
- [ ] Phase 4（可选）: 完成至少一个高级主题

## 注意事项

1. **官方优先**：遇到问题先查官方文档和教程，再查社区
2. **版本匹配**：确保使用 ROS2 Humble + MoveIt2 Humble 分支
3. **与当前项目隔离**：P5 的实验代码放在 `p5_moveit2_learning/` 下，不影响 P1~P4 的运行
4. **记录问题**：所有踩过的坑记录在 `docs/troubleshooting/` 和 `CLAUDE.md` 中

---

## 进度快照 (2026-06-09)

**已完成 (B 阶段 baseline 闭环)**:
- ✅ `abb_controller` (JTC) 配置 + spawn
- ✅ `move_group.launch.py` 加载 ompl_planning.yaml + joint_limits.yaml + 统一 xacro 风格
- ✅ `ros2 control list_controllers` 验证全 active
- ✅ `/joint_states` 50Hz 发布 6 joint
- ✅ `/abb_controller/follow_joint_trajectory` action server 在线

**下一步 (A 阶段概念补全)**:
- 读 MoveIt2 官方 5 篇概念文档, 写 `docs/concept_notes/*.md`
- 启动 `move_group.launch.py` + RViz, 验证 Plan + Execute 一次, 截图存 `resources/p5/`
- Phase 2 官方 tutorial (需 `apt install ros-humble-moveit2-tutorials`)
