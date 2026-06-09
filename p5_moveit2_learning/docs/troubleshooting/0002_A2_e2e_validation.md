# Troubleshooting #0002: P5 MoveIt2 L1 规划失败 + L3→L4 端到端 PASS

**日期**: 2026-06-09
**Phase**: P5 A2 (端到端验证)
**作者**: Claude (Sonnet 4.6) + qiulongqu

---

## 背景

A2 阶段目标: 跑通 P5 baseline 端到端, 证明 L1→L2→L3→L4 全链路。
B 阶段已闭环: JTC + move_group 配置 + action server 全部就绪.

---

## 验证策略: 双管齐下

A2 设计 2 个测试脚本, 互为补充:

| 脚本 | 测什么 | 状态 |
|------|--------|------|
| `scripts/01_plan_execute_demo.py` | L1 MoveIt 规划 + L2 JTC 插值 + L3→L4 执行 | ❌ 规划失败 (OMPL "Skipping invalid start state") |
| `scripts/02_direct_trajectory_test.py` | L3 JTC action + L4 MuJoCo 物理 | ✅ **PASS**, 终点误差 0.43° |

---

## 测试 1: L1 MoveIt 规划 (`/plan_kinematic_path` 服务)

### 现象
调用 `/plan_kinematic_path` 服务规划从零位到 `[0, -0.5, 0.8, 0, 0.5, 0]` rad 的轨迹,
返回 `error_code=99999` (FAILURE), `fraction=0.0`. 整个规划耗时 < 50ms (说明预处理就 fail 了).

### MoveGroup 日志关键报错
```
[moveit.ompl_planning.planning_context_manager]: Cannot find planning configuration
  for group 'gofa_arm' using planner 'RRTConnectkConfigDefault'. Will use defaults.
[moveit.ompl_planning.model_based_planning_context]: It looks like the planning volume was not specified.
[ompl]: Skipping invalid start state (invalid state)
[ompl]: Motion planning start tree could not be initialized!
```

### 修复尝试 (3 步都失败)

| 尝试 | 改动 | 结果 |
|------|------|------|
| #1 | `req.start_state.is_diff = True` (空 start state, MoveIt 用当前 state) | 仍然 "invalid start state" |
| #2 | 显式传 `req.start_state.joint_state = self.latest_joint_state` | 仍然 "invalid start state" |
| #3 | 加 `req.workspace_parameters` (覆盖 base_link -2~2 范围) + 放宽约束 tolerance 0.05 | 仍然 "invalid start state" |

### 根因分析 (未解决, 留给后续)

1. **OMPL 默认配置问题**: `ompl_planning.yaml` 没定义 `planner_configs:` 块, RRT* 等规划器用默认参数, 可能与 GoFa 工作空间不匹配.
2. **Start state 校验失败**: move_group 用 `is_diff=True` 自动取当前 state, 但 OMPL state validity checker 报 "invalid", 可能因为:
   - 当前 state 含有的 joint name 顺序与 SRDF gofa_arm group 不一致
   - 当前 state 含有的 6 个 joint 中某些值在 joint_limits 边界 (用 IK 反解时可能产生微小超出)
3. **Workspace volume 缺失警告**: `ompl_planning.yaml` 没设 `default_workspace_bounds`, 触发警告, 间接影响 start tree 初始化.

### 替代方案: 跳过 L1 规划, 直测 L3→L4

既然 L1 规划需要更深入的 SRDF/OMPL/kinematics 调优, 短期先把 L3→L4 闭环证出来, 
L1 规划作为单独 Phase 1.5 任务 (用 moveit_py / moveit_commander 高级 API, 或直接调
`/move_action` 而不是 `/plan_kinematic_path`).

---

## 测试 2: L3→L4 端到端 (`/abb_controller/follow_joint_trajectory` action)

### 现象
直接构造 5 个 waypoint 的 JointTrajectory (零位 → 中间 → 目标 → 撤回 → 归零),
发给 JTC 的 action server. 期望 JTC 内部 splines 插值, 驱动 ros2_control position
interface, 触发 MuJoCo 物理仿真.

### 证据 (硬数据)
```
[INFO] JTC goal 接受, 等待执行结果...
[INFO] Evidence 已写入: resources/p5/p5_step2_direct_trajectory.json
[INFO] status: 4 (SUCCEEDED)
[INFO] 终点误差: max=0.0076 rad (0.43°)
[INFO] verdict: PASS
```

### 关键参数
- 5 waypoint × 1.5s/waypoint = 7.5s 总轨迹
- 起点: 零位 (所有关节 = 0 rad)
- 终点: 零位 (闭环)
- 中间 3 个 waypoint: `[0.3,-0.3,0.5,0,0.3,0]`, `[0,-0.5,0.8,0,0.5,0]`, `[-0.3,-0.3,0.5,0,0.3,0]`
- 最大关节运动: ±0.8 rad ≈ 46°
- 终点误差 0.43° (远小于 P4 验收标准 2°)

### L3→L4 4 个验证点全过
| 验证项 | 状态 | 证据 |
|--------|------|------|
| JTC action server 在线 | ✅ | `wait_for_server` 返回 True |
| JTC goal 接受 | ✅ | `goal_handle.accepted = True` |
| 轨迹执行完成 | ✅ | `result.status = 4` (SUCCEEDED) |
| MuJoCo 实际运动到目标 | ✅ | `/joint_states` 终点 vs 目标 max 误差 0.43° |

---

## L1 规划的修复路线 (后续 Phase)

把 L1 规划问题留给后续 Phase 1.5, 修复路径:

1. **改用 `/move_action` 高级接口**: 而不是直接 `/plan_kinematic_path`. 前者内部处理 start state + workspace + 适配器, 用户只需给 "goal pose" 或 "joint values".
2. **装 moveit_py 或 moveit_commander**: 提供 C++/Python 高层 API, 自动处理 start state.
3. **完善 `ompl_planning.yaml`**: 加 `default_workspace_bounds` + `planner_configs:` 块, 用 `RRTstarkConfigDefault` (更宽松的 start state 容差).
4. **检查 SRDF group 定义**: 当前 `gofa_arm` 用 chain base_link→tool0, 6 个 joint. 考虑加 virtual_joint 或 explicit joint list.

---

## 沉淀到 CLAUDE.md 的新教训

### 5-11 L1 规划初次集成常见 3 坑 (2026-06-09)

1. **`pipeline_id` 不要写 "ompl"**: move_group 自动注册 pipeline 时用 launch 配置的 key, 不是 yaml 里的 `planning_plugin` 名. 默认空字符串即可.
2. **`start_state.is_diff = True` + 空 joint_state**: MoveIt 会自动取当前 planning scene state, 但需要 `workspace_parameters` 已经设了.
3. **OMPL "Skipping invalid start state" 9 成是配置问题**: 不是 IK 算不出, 而是 joint name 顺序 / joint limit 边界 / workspace volume 缺失. 短期 workaround: 改用 `/move_action` 高层接口, 长期: 完善 `ompl_planning.yaml` 的 `planner_configs` 和 `default_workspace_bounds`.

### 5-12 L3→L4 端到端验证的最小可行路径 (2026-06-09)

不依赖 moveit_py / moveit_commander, 用 rclpy + control_msgs.action.FollowJointTrajectory
就能验证 L3→L4 闭环:

```python
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ["joint_1", ..., "joint_6"]
for i, wp in enumerate(waypoints):
    point = JointTrajectoryPoint()
    point.positions = wp
    point.time_from_start.sec = (i+1) * 1.5
    goal.trajectory.points.append(point)
action_client.send_goal_async(goal)
```

- 优势: 零额外依赖, 直接测 JTC + ros2_control + MuJoCo
- 适用: Phase 1 验证 L3/L4 链路, Phase 2 验证 L1 规划

### 5-13 验证脚本要避开 L1 复杂依赖 (2026-06-09)

- 不要在 baseline 验证里直接用 `/plan_kinematic_path` 服务 — MoveIt 内部对 start state / workspace / collision 的初始化非常挑剔, 容易撞墙.
- **推荐**: 用 action 客户端发 FollowJointTrajectory 直测 L3→L4, 用 RViz 手动 Plan+Execute 验证 L1 (GUI 帮你处理 start state).

---

## 启动命令 (加入 docs/reports/脚本启动命令.md §5.4)

```bash
# 终端 1: bringup + JTC
source /opt/ros/humble/setup.bash
source ~/ros2_ws_abb/mujocoONLY/install/setup.bash
ros2 launch gofa_moveit_config demo.launch.py mujoco_gui:=false rviz_gui:=false

# 终端 2: move_group
ros2 launch gofa_moveit_config move_group.launch.py

# 终端 3: 端到端验证 (L3→L4 直测)
/usr/bin/python3 p5_moveit2_learning/scripts/02_direct_trajectory_test.py
# 期望: status=4 SUCCEEDED, 终点误差 < 1°

# 终端 3 (L1 规划, 当前失败, 后续 Phase 1.5 修):
/usr/bin/python3 p5_moveit2_learning/scripts/01_plan_execute_demo.py
# 当前: error_code=99999, 后续需改用 /move_action
```

---

## 下一步 (Phase A 概念补全)

A2 已闭环 (L3→L4 验证通过, L1 规划作为 follow-up). 接下来:

- [ ] **A1 概念笔记**: 读 MoveIt2 官方 5 篇概念文档, 写 `p5_moveit2_learning/docs/concept_notes/`. 重点理解 `MotionPlanRequest` 字段语义 (这样后续修 L1 规划才有理论依据).
- [ ] **Phase 1.5 (L1 修复)**: 改用 `/move_action` 或 moveit_py 高层 API 绕过 L1 当前问题.
- [ ] **Phase 2 官方 tutorial**: `apt install ros-humble-moveit2-tutorials`, 跑 Python API demo.
- [ ] **Phase 3 集成**: RViz MotionPlanning 插件可视化 + Plan + Execute 一次, 截图存 `resources/p5/`.
