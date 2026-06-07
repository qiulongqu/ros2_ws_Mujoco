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

## 后续 P2~P5 注意事项（待补充）

### P2（ROS2 + Mujoco 桥接）

- ROS2 的 `/joint_states` 和 Mujoco 的 `data.qpos` 需要通过 `ezomq` 或自定义桥接节点
- 控制频率不匹配：Mujoco 建议 1000Hz+，ROS2 一般 100Hz，需插值
- **官方 mujoco_ros2_control 不需要 MoveIt2**：官方 demo 只用 `ForwardCommandController` + RViz `RobotModel` 即可显示和控制
- **控制器名称必须一致**：`gofa_controllers.yaml` 中定义的控制器名必须和 launch 文件中 spawn 的名字一致
- **使用 `ForwardCommandController` 而非 `JointTrajectoryController`**：前者直接接收位置命令，适合基础学习；后者需要 trajectory 消息，是 MoveIt2 的配套控制器
- **所有节点必须统一 `use_sim_time=True`**，否则会出现 TF 时间回跳警告
- **RViz 必须配置 `RobotModel` display**：只配 `MotionPlanning` 不够，需要 `RobotModel` 来显示 URDF 模型

### P3（机械臂模型）

**P3-01 机械臂模型创建**

### P3（机械臂模型）

**P3-01 机械臂模型创建**

- URDF → MuJoCo 需要用 `mujoco-webcas` 或手写 XML
- 关节命名要和 URDF 一致
- **渲染全黑**：必须在 rendering 前调用 `mj_forward(model, data)` 计算 xpos，否则所有几何体位置为 0，渲染结果全黑
- **XML fixed camera**：`xyaxes` 定义 camera 的 X/Y 轴方向，Z 轴（forward）= cross(X, Y)。手动计算容易出错，推荐脚本中使用 `MjvCamera` 设置 `lookat` + `distance` + `azimuth` + `elevation`
- **STL mesh 加载**：MuJoCo 直接支持 STL mesh，路径用 `<compiler meshdir="..."/>` 指定相对路径基准
- **运动学链转换**：URDF joint `<origin xyz>` → MuJoCo body `pos`；URDF joint `<axis xyz>` → MuJoCo joint `axis`；URDF `<inertial>` → MuJoCo `<inertial>` (fullinertia 格式: ixx iyy izz ixy ixz iyz)
- **离线渲染**：`conda run -n mujoco` + `glfw.window_hint(glfw.VISIBLE, 0)` 可在无显示器环境渲染并保存 PNG
- **地面**：`<geom type="plane" size="2 2 0.01"/>` 提供基准地面，否则机械臂在重力下无限坠落