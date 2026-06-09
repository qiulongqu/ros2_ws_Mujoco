# CLAUDE.md — Mujoco 学习经验教训记录

> 每次实验的错误和教训都需要记录，避免重蹈覆辙。

---

## P1 实验教训

### P1-01 自由落体

**错误1：球的位置从 xpos 取值，但 xpos 初始化为 [0,0,0]**

- **原因**：`xpos` 是全局坐标，`mj_step()` 后 Mujoco 不会自动更新 `xpos`（需要调用 `mj_forward()`），而 `qpos[:3]` 才是 freejoint 的真实位置。
- **教训**：对于 freejoint 物体，取位置用 `data.qpos[:3]`，不要用 `data.xpos[id]`（除非你确定调用了 forward）。
- **错误2：仿真结束球落在 -2.89m，说明地面没有接触碰撞**
- **原因**： Mujoco 默认 plane geom 没有配置 solref/solimp，导致球直接穿透地面。
- **教训**：接触物理必须关注 `solref`（接触刚度）和 `solimp`（穿透抑制参数），Mujoco 默认参数对某些场景不够硬。

---

### P1-02 单摆

**错误1：周期无法检测**

- **原因**：60° 大角度下单摆不是简谐运动，零点交叉比简化解复杂。
- **教训**：小角度近似（θ₀ < 15°）才能用 `θ(t) = θ₀·cos(ωt)`；大角度要验证能量守恒而不是解析解。
- **错误2：Mujoco 内置关节阻尼≠物理阻尼**
- **原因**：`<joint damping="2">` 是物理引擎粘性阻尼，不是弹簧刚度。`stiffness` 属性在某些Mujoco版本/配置下不能直接用。
- **教训**：用能量守恒验证关节行为，而不是依赖关节阻尼直接计算。Mujoco 内置阻尼是数值阻尼。

---

### P1-03 弹簧阻尼

**错误1：Mujoco 的 `<joint stiffness="k">` 属性不存在**

- **原因**：Mujoco XML schema 中 joint 没有 `stiffness` 属性。
- **教训**：`stiffness` 是 tendon 的属性，不是 joint 的。弹簧需要通过 actuator 手动施加控制力实现。
- **错误2：能量不守恒（隐式积分器特性）**
- **原因**：Mujoco 默认使用 `Euler` 或 `implicit` 积分器，具有数值阻尼，会额外耗散能量。能量漂移 12J 说明数值阻尼很大。
- **教训**：隐式积分器 + 物理阻尼 → 双重阻尼。验证弹簧阻尼用峰值包络衰减（ζ 验证）和周期（ω_d 验证），不要用能量守恒。
- **错误3：峰值包络误差达 73%（错误对比方式）**
- **原因**：对比方式错误——应该对比每个峰值与其理论包络值，而不是全时间序列与包络线对比。
- **教训**：峰值包络验证应该找局部最大值（peak indices），逐点对比峰值，而不是全数组对比。

---

## 通用教训

### 环境配置

- **Mujoco 和 ROS2 必须使用独立 conda 环境**，不可混用
- ROS2 的 `PYTHONPATH` 包含 `/opt/ros/humble/...`，与 Mujoco 的 Python 3.13 环境冲突
- 解决：创建独立 `mujoco` 环境，跑 Mujoco 脚本时 `conda activate mujoco`

### MuJoCo Python API 关键点

1. `MjModel.from_xml_string(xml)` — 从 XML 字符串创建模型（不是 from_xml_path）
2. `MjData(model)` — 每次 reset 需要新建 MjData，或者 reset 现有 data
3. `mj_step(model, data)` — 注意这是 `mujoco.mj_step`，不是 `data.step()`
4. `data.qpos[:3]` — freejoint 前3个元素是位置（单位时间步后记录的是上一步的位置）
5. `Renderer.render()` — 返回 numpy array，不是直接渲染到窗口；需要先 `update_scene(data)` 再 `render()`
6. `glfw.init()` / `glfw.create_window()` — 无显示器环境会失败，需要做 fallback

### 官方优先原则

**遇到问题时，优先采用官方的、标准的解决方法和流程。如果官方方案不存在或确实不可行，再考虑替代方案。**

- MuJoCo 官方文档：https://mujoco.readthedocs.io/
- MuJoCo 官方 GitHub 示例：https://github.com/google-deepmind/mujoco/tree/main/python
- 优先查官方文档、官方示例、官方 tutorial，再查社区论坛/博客
- 不要一开始就自己发明方案——官方方案通常已被广泛验证

### 不要重复造轮子

**在动手实现之前，先搜索是否有现成的、已验证成功的实现案例。**

- GitHub 上有大量 MuJoCo 机械臂控制的开源项目，已经解决了相同的问题
- 照着已验证成功的方案做，比从零摸索高效得多
- 搜关键词：`mujoco robot arm control` / `mujoco position control` / `mujoco trajectory tracking`
- 先读懂别人的代码再动手，不要自己想当然

---

### 调试方法

1. **先跑纯数据输出**（不启动渲染窗口），验证物理正确性
2. **能量守恒**是验证物理仿真的第一原则（单摆验证成功）
3. **理论解对比**要分阶段（空中 vs 触地后），不能混在一起比较
4. **小步数调试**：先用 10 步验证行为，再用 500 步做完整验证
5. **打印 qpos / qvel / qfrc** 是调试关节行为最快的方式

### Mujoco vs 物理教材的差异

- **隐式积分**：数值稳定但有额外阻尼，能量不守恒是正常的
- **穿透抑制**：地面必须有 solref/solimp 才能阻止穿透
- **关节刚度**：不是 joint 属性，而是 tendon 的 `<passive>` 或 actuator 控制力
- **四元数**：`qpos[3:7]` 是 freejoint 的旋转（四元数 w,x,y,z）

---

### WSL2 + Mujoco 显示问题

**现象**：窗口始终黑色，但保存的 PNG 图片颜色正常。

**原因**：WSL2 的 X server（无论 WSLg 还是 VcXsrv）在 GPU 渲染帧和窗口显示层之间的同步存在兼容性问题。Mujoco GPU 渲染正常，但显示层没有推送帧到窗口。

**解决**：接受现状，以保存的 PNG 为准。图片渲染正常 = 物理正确。

**不要尝试的方案**：
- MUJOCO_GL=egl — 在 WSL2 下仍然无效
- 修改 glfw 配置 — 治标不治本

---

## 脚本落库 (强制规则)

**每次新增/修改任何可执行脚本 (Python / launch), owner 必须同步把启动命令追加到:**

> `docs/reports/脚本启动命令.md`

**Why**: 项目有 16+ 脚本, 跨 P1~P4 教程 + ROS2 launch. 命令散落在各 README/文件头容易丢失, 未来换机器/换环境时, 启动成本指数级上升. 一个集中的"活文档"是闭环的抓手.

**How to apply**:
1. 新增脚本后, 先跑一次确认命令可执行 (无 ROS2 脚本用 `conda run -n mujoco python3 ...`, ROS2 脚本先 `source` 再跑)
2. 把命令按所属阶段 (P1/P2/P3/P4/ROS2) 追加到 `脚本启动命令.md` 对应章节
3. 提交时一并 commit 本文件和 `脚本启动命令.md`
4. 如果脚本的 `argparse` 参数变了, 同步更新命令

**禁止**:
- "代码改了但忘记更新启动命令" — 等于没改
- 把命令散落在多个 README — 统一到 `脚本启动命令.md`
- 写完不跑就记录 — 必须先验证再记录

---

## 后续 P2~P5 注意事项 (实战沉淀)

### P2 (ROS2 + MuJoCo 桥接)

#### 2-01 频率匹配: 仿真 500Hz / ROS2 100Hz

- **现象**: ROS2 spinner 一次, MuJoCo 应该跑 250 步 (dt=2ms) 才能跟上 100Hz 发布频率.
- **教训**: `for _ in range(steps_per_cycle): mj.mj_step(...)` 中的 `steps_per_cycle = int(1.0/100/0.002) = 5` (不是 250). 100Hz 是发布频率, 不是仿真步频率 — 仿真一直是 500Hz, 每 5 步发布一次 state.

#### 2-02 Standalone + ROS2 双模式

- **现象**: 没装 ROS2 (或没 source) 的机器上, 纯 `import rclpy` 失败.
- **教训**: 脚本开头 try-except `import rclpy`, 失败时降级到 `--test` standalone 模式. 这样一份脚本既能生产用, 又能在没装 ROS2 的开发机跑通.

#### 2-03 Float64MultiArray 是常用扭矩指令格式

- **教训**: 力矩/速度等数值指令用 `std_msgs/Float64MultiArray` 即可, 没必要用 JointTrajectory. trajectory 消息是 MoveIt2 的配套, 不是给简单控制用的.

#### 2-04 /joint_states 必须配 joint_state_broadcaster

- **现象**: 只 spawn position_controller 不 spawn joint_state_broadcaster, `/joint_states` topic 没人发布.
- **教训**: 至少两个 controller — `joint_state_broadcaster` (发布 state) + `position_controller` (接收 cmd). 顺序: 先 state 再 controller, 否则 TF 没数据.

---

### P3 (机械臂模型 + PTP 运动)

#### 3-01 Position Actuator (Menagerie 模式) 是黄金组合

```xml
<position name="act_joint_1" joint="joint_1"
          kp="4000" kv="400"
          ctrlrange="-4.7124 4.7124"
          forcerange="-400 400"/>
```

- **原理**: MuJoCo 内置 PD 伺服 `torque = kp*(ctrl - qpos) - kv*qvel`. Python 侧 `data.ctrl[:] = target_q` 即可, 不用自己写 PD.
- **Why**: 手写 PD (motor actuator) 容易被动力学非线性 (重力 + Coriolis) 干扰. Position actuator 让 MuJoCo 帮你做所有伺服.
- **How to apply**: 中等精度 (稳态 1-2°) 需求用 position; 高精度 (亚度) 才考虑 motor + qfrc_bias 前馈.

#### 3-02 kp/kv 比例关系

- **现象**: kv=kp/10 是个经验值, kp=4000 时 kv=400. 偏离这个比例会有震荡或响应迟钝.
- **教训**: 高 kv (k_damping 强) → 快速稳定但超调; 低 kv → 平滑但慢. 调试时优先调 kp (响应), kv 保持 1/10 比例.

#### 3-03 implicit 积分器是高 kp 数值稳定的必要条件

- **现象**: kp=4000 + Euler 积分器, dt=2ms 时手腕关节爆炸性震荡.
- **原因**: 显式积分器稳定条件是 `dt × ω_n < 2`, 高 kp → 大 ω_n → 越界.
- **教训**: `<option integrator="implicit"/>` 是高 kp + 轻量化手腕关节的标配.

#### 3-04 Quintic S-curve 插值是消除抖动的关键

```python
def quintic_interp(t, duration):
    frac = t / duration
    return 6*frac**5 - 15*frac**4 + 10*frac**3  # 6f^5-15f^4+10f^3

def ptp_target(start, goal, t, duration):
    return start + (goal - start) * quintic_interp(t, duration)
```

- **Why**: 多项式 `6f^5-15f^4+10f^3` 在 `f=0, f=1` 处速度/加速度都是 0, 阶段切换时无突变 → PD 伺服不震荡.
- **What not**: 线性插值 `start + (goal-start)*f` 在边界 f=0,1 速度非零, 切换时冲击大, 高 kp 下明显抖动.
- **How to apply**: PTP 移动、状态机阶段切换、轨迹跟踪都该用. 工业 S-curve (速度/加速度可设上限) 是更完整版.

#### 3-05 工具函数库 (P3+P4 复用)

沉淀在 `tutorials/p3_arm_model/step3_ptp_move.py`:

- `quintic_interp(t, duration)` — 0/1 边界 vel/accel 都为 0
- `ptp_target(start, goal, t, duration)` — 关节空间插值
- `run_ptp_sequence(model, data, targets_deg, ...)` — 一组 PTP 移动 + 跟踪误差统计

---

### P4 (Pick-and-Place 8 阶段状态机)

#### 4-01 数值 IK (有限差分 Jacobian + DLS)

```python
J = np.zeros((3, 6))
for j in range(6):
    q_plus = q.copy()
    q_plus[j] += eps  # eps=0.0005 rad
    data.qpos[:6] = q_plus
    mj.mj_forward(model, data)
    J[:, j] = (get_pos(data) - current_pos) / eps

dq = J.T @ np.linalg.solve(J @ J.T + 0.05 * np.eye(3), error)
q += 0.5 * dq  # learning_rate=0.5
```

- **Why not analytical IK**: 6-DOF 通用解析解依赖关节构型, GoFa 这种非球腕 (joint_6 之前没汇聚到一点) 不标准. 数值法通用, 100-500 次迭代 ~ 1ms 出解.
- **DLS 阻尼 λ=0.05**: 避免 Jacobian 奇异时 dq 爆炸. λ 太小 → 奇异点震荡; 太大 → 收敛慢.
- **关节限位 clamp**: 迭代完每步 `np.clip(q[j], lo, hi)`, 防止解超出物理可达域.
- **收敛阈值 1mm**: 工业精度一般 0.1-1mm, 学习阶段 1mm 已经够用.

#### 4-02 Teleport 吸附替代 weld constraint (MuJoCo 3.8.1)

- **现象**: `<weld body1="link_6" body2="block" active="false"/>` 在 free body (方块) 和 kinematic body (机械臂) 之间不可靠激活. 查 mujoco 论坛也有同样问题.
- **替代方案**: 在抓取阶段每步 `data.qpos[6:9] = data.site('gripper_site').xpos.copy()`, `data.qvel[6:12] = 0.0`. 等效于把方块"焊"到吸盘, 但用代码控制.
- **Why**: 工业上 suction gripper (吸盘) 本身就靠气压维持, 真掉下来不是仿真 bug. teleport 是物理正确的近似.
- **Release**: 停止 teleport, 方块自然掉落 (有重力), 落到桌面停稳.

#### 4-03 安全高度 (safe_z) 防止 LIFT/PRE_PLACE 撞桌

```python
safe_z = max(pick_tool0[2], place_tool0[2]) + 0.50  # 50cm above highest
lift_xyz = np.array([pick_tool0[0], pick_tool0[1], safe_z])
```

- **Why**: 工业机器人抬升量不能简单加 10cm, 必须算"全程最高点 + 余量". 否则 6 自由度臂在奇异点附近抬升会撞桌面.
- **50cm 是 GoFa 臂展 1.5m 的 1/3**: 余量充足但不浪费 cycle time.

#### 4-04 8 阶段状态机 + phase_time + ramp_start 三件套

| 变量 | 作用 |
|------|------|
| `phase_idx` | 当前阶段 (0-8) |
| `phase_time` | 当前阶段已运行时间 (DT 累加) |
| `q_ramp_start` | 当前阶段起始关节角 (S-curve 的 start) |
| `q_ramp_dur` | 当前阶段 ramp 持续时间 (默认 0.4s) |
| `q_target` | 当前阶段目标关节角 |

- **关键**: `_next(phase_name)` 切换阶段时, **必须** 把 `q_ramp_start = data.qpos[:6].copy()` (实际当前位置, 不是上一阶段目标). 否则 S-curve 起点跳变.
- **arrived 判定**: `ramp_done (phase_time >= dur) AND joints_near (max |err| < 0.03 rad)`. 必须等 ramp 完成, 不能光看关节误差 — 早期 ramp 中关节接近目标但还没到位.

#### 4-05 交互查看器自动循环模式

工业开发不可能每次跑都手点 reset. 标准做法:

```python
_state = {'phase': 'running', 'timer': 0.0, 'cycle': 0, 'reset_start': np.zeros(6)}

def controller(m, d):
    s = _state
    if s['phase'] == 'running':
        sm.step()
        set_ctrl(m, d, sm.get_ctrl())
        if sm.phase == 'COMPLETE':
            s['phase'] = 'complete_hold'
            s['cycle'] += 1
    elif s['phase'] == 'complete_hold':
        s['timer'] += DT
        if s['timer'] >= 2.0:  # hold 2s 看效果
            s['phase'] = 'resetting_arm'
            s['reset_start'] = d.qpos[:6].copy()
    elif s['phase'] == 'resetting_arm':
        interp = ptp_target(s['reset_start'], np.zeros(6), s['timer'], 0.8)
        set_ctrl(m, d, interp)
        d.qpos[6:9] = INIT_BLOCK_QPOS[:3]  # 复位方块
        s['timer'] += DT
        if s['timer'] >= 0.8 + 0.5:  # ramp + hold
            s['phase'] = 'reset_hold'
    elif s['phase'] == 'reset_hold':
        s['timer'] += DT
        if s['timer'] >= 0.5:
            sm.phase_idx = 0  # 状态机复位
            sm.phase_time = 0.0
            s['phase'] = 'running'
            s['timer'] = 0.0

mj.set_mjcb_control(controller)
viewer.launch(model, data)
```

- **Why**: 4 状态机 (running/hold/resetting/reset_hold) 比简单的 1 次性跑强 10 倍, 调试时不用手动复位.
- **Space 暂停**: viewer 内置, 可暂停看关节角度; Tab 打开面板检查 ctrl 状态.

#### 4-06 工业 4 层控制架构 (沉淀认知)

| 层级 | 任务 | P4 实现 | 工业真实 |
|------|------|---------|---------|
| L1 规划 | MoveIt2 / RRT* | ❌ (手写 IK) | OMPL, CHOMP, STOMP |
| L2 轨迹 | 时间参数化 S-curve | ✅ (quintic) | 梯形/S 形/ jerk-limited |
| L3 伺服 | 位置/速度/力控 | ✅ (position actuator) | PID + FF + 摩擦补偿 |
| L4 物理 | 接触/碰撞/动力学 | ✅ (MuJoCo 引擎) | ODE / MuJoCo / Bullet |

- **Why 写出来**: 用户最初问"工业上是这么开发的吗", 答: 工业上每层有现成商业库 (MoveIt2 / Reflexxes / Orocos / Bullet). 学习阶段用 MuJoCo 一把梭哈, 但要清楚每层职责, 别把所有逻辑都堆在 step() 里.
- **How to apply**: 未来上 MoveIt2 (L1) 后, 4 层会真正分清. 现在 (P4) 用 state machine 是因为还没学 L1, 不是"工业就该这么写".

#### 4-07 端执行器抖动 = L2 缺 S-curve (不是 L3 PD 调参)

- **现象**: 用户发现 P4 step2 末端 tool0 抖动.
- **根因**: 状态机阶段切换时 `q_target` 瞬变, L3 PD 直接对突变响应 → 高 kp 下震荡.
- **解决**: 加 L2 quintic S-curve, `ptp_target(start, goal, t, dur)`. 不需要改 kp/kv.
- **教训**: 看到"末端抖"先怀疑 L1/L2 (规划/轨迹), 不要先去 L3/L4 (PD/物理) 调参. 80% 的"调不好"其实是上层没规划好.

---

### P5 (MoveIt2 学习)

#### 5-01 整体路径

- **Phase 1 (1-2 周)**: 概念笔记, 不写代码
- **Phase 2 (2-3 周)**: 跑官方 tutorial
- **Phase 3 (3-4 周)**: GoFa + MoveIt2 集成 (`JointTrajectoryController` 替换 `ForwardCommandController`)
- **Phase 4 (5-6 周, 可选)**: MoveIt Servo / Task Constructor / Setup Assistant

详见 `p5_moveit2_learning/README.md`.

#### 5-02 ForwardCommandController vs JointTrajectoryController

- `ForwardCommandController` — 接收单个 `Float64MultiArray`, 直接下发位置. P2-P4 基础学习用.
- `JointTrajectoryController` — 接收 `trajectory_msgs/JointTrajectory`, 内部做时间参数化 + 插值. MoveIt2 通过 `FollowJointTrajectory` action 发送.
- **切换**: MoveIt2 集成时必须替换. controller 名字在 `gofa_controllers.yaml` 改, launch 文件中 spawner 列表也要改.

#### 5-03 启动链差异 (P4 vs P5)

```
P4 (当前):                                  P5 (目标):
  robot_state_publisher                       robot_state_publisher
  ros2_control_node (mujoco_ros2_control)    ros2_control_node (mujoco_ros2_control)
  joint_state_broadcaster                     joint_state_broadcaster
  position_controller ← ForwardCommandCtrl    joint_trajectory_controller ← FollowJointTrajectory
                                              move_group (MoveIt2)
                                              rviz2 (MotionPlanning 插件)
```

#### 5-04 /joint_states 时间戳必须用仿真时间

- **现象**: 控制器报 "timestamp mismatch" 或 TF 警告
- **原因**: `use_sim_time` 没统一 — `/clock` 是仿真时间, `/joint_states` 用了 wall time, 差几个 ms TF 就跳.
- **教训**: 所有节点 (ros2_control, rsp, move_group) 都设 `use_sim_time:=True`, launch 顶层加 `use_sim_time` 节点级覆盖.

#### 5-05 RViz 必须配 RobotModel + MotionPlanning 双显示

- **现象**: 配了 `MotionPlanning` 但模型不显示.
- **原因**: MoveIt2 用 planning frame 算碰撞, 但 RViz 视觉化要 `RobotModel` display 显式订阅 `robot_description`.
- **教训**: `RobotModel` 和 `MotionPlanning` 是两个独立 display, 都需要. `MotionPlanning` 调 move_group 服务, `RobotModel` 单纯可视化.

#### 5-06 ros2_control controller 资源互斥 (踩坑 2026-06-09)
- **现象**: `abb_controller` (JTC) 和 `position_controller` (ForwardCommandController) 同时 spawn 报 "Resource conflict for controller 'abb_controller'. Command interface 'joint_1/position' is already claimed."
- **根因**: ros2_control 硬约束 — 同一 hardware interface (joint_X/position) 只能被一个 controller 持有.
- **解决**: 用不同 launch 文件分流:
  - P4 baseline `gofa_mujoco_bringup/gofa_mujoco.launch.py` — 只 spawn `position_controller` (topic pub)
  - P5 MoveIt2 `gofa_moveit_config/demo.launch.py` — 只 spawn `abb_controller` (JTC + action)
- **Why**: 这是架构选择, 不是 bug。同一个 controller_manager 进程里不能并存两个 claim 同一接口的 controller。
- **How to apply**: 后续如果需要 P4+P5 共存 (比如先 MoveIt2 规划, 再 ForwardCommand 接管), 必须先 `unload` 一个再 `load` 另一个, 不能同时 active。

#### 5-07 JointTrajectoryController 类名位置 (踩坑 2026-06-09)
- **错误**: 第一版 yaml 写 `type: position_controllers/JointTrajectoryController`
- **报错**: `Loader for controller 'abb_controller' (type 'position_controllers/JointTrajectoryController') not found.`
- **正解**: `type: joint_trajectory_controller/JointTrajectoryController` (包名=joint_trajectory_controller, 不是 position_controllers)
- **验证方法**: `ros2 control list_controller_types | grep Trajectory` 是唯一可靠方式。不要靠记忆, 记忆容易和 `position_controllers/GripperActionController` 混。

#### 5-08 move_group 启动顺序 (新增 2026-06-09)
1. `ros2 launch gofa_moveit_config demo.launch.py` (启动 controllers + JTC)
2. 验证 `ros2 control list_controllers` 看到 `abb_controller active` (JTC)
3. 验证 `/abb_controller/follow_joint_trajectory` action server 在线: `ros2 action list | grep trajectory`
4. `ros2 launch gofa_moveit_config move_group.launch.py` (启动规划节点)
5. `rviz2 -d <path>/moveit.rviz` (可视化 + Plan)
- **Why**: move_group 不负责 spawn controllers, 它假设 controllers 已经在 active 状态。

#### 5-09 move_group 配置文件全量加载 (新增 2026-06-09)
- **现象**: 漏加载 `ompl_planning.yaml` → 规划超时或路径差
- **现象**: 漏加载 `joint_limits.yaml` → 关节限位没传给 move_group, 可能规划到物理不可达区域
- **正确模板**:
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
- **教训**: moveit_config 包下 5 个 yaml 缺一不可, launch 文件不加载某个 = 走默认值, 大概率 fail。

#### 5-10 P5 baseline 验证清单 (闭环 2026-06-09)
- [x] colcon build 通过 (`gofa_mujoco_bringup`, `gofa_moveit_config`)
- [x] `demo.launch.py mujoco_gui:=false rviz_gui:=false` 启动 25s+ 无 ERROR
- [x] `ros2 control list_controllers`: `abb_controller` + `joint_state_broadcaster` 都 active
- [x] `/joint_states` 50Hz 发布 6 个 joint
- [x] `/abb_controller/follow_joint_trajectory` action server 在线
- [x] MuJoCo 6 joint position control enabled
- [ ] **未验证**: 完整 `move_group.launch.py` + RViz 中 Plan + Execute (A 阶段补概念后再做)

---

## 实战原则 (跨阶段)

1. **能用现成库, 就不自己造轮子** (MoveIt2 替代手写 IK, position actuator 替代手写 PD)
2. **每层职责清晰** (L1 规划 / L2 轨迹 / L3 伺服 / L4 物理), 不要堆在一个 step() 里
3. **可视化 + 数据双输出** (viewer 看实时 + 轨迹图看分析), 不要只看终端打印
4. **闭环测试** (state machine + viewer 自动循环), 跑通一次不够, 跑 10 次看稳定性
5. **错误信息第一原则** (MuJoCo 报 `Error opening file` → 先看路径; 报 `integrator unstable` → 换 implicit)

---

## 资源

- **MuJoCo 官方文档**: https://mujoco.readthedocs.io/
- **MuJoCo 官方示例**: https://github.com/google-deepmind/mujoco/tree/main/python
- **mujoco_ros2_control**: https://github.com/PickNikRobotics/mujoco_ros2_control
- **MoveIt2 Humble**: https://moveit.picknik.ai/humble/
- **GoFa URDF 源**: `abb_omnicore_ros2/robot_specific_config/abb_gofa_crb15000_support/`