# P3 Step3 关节轨迹跟踪控制 — 实现方案文档

> **文档时间戳**: 2026-05-27 23:59 CST
> **状态**: 方案评审阶段 (待用户审核后实施)

---

## 1. 背景与目标

### 1.1 任务

实现 ABB GoFa CRB15000 机械臂在 MuJoCo 中的关节空间轨迹跟踪控制。

- 输入: 6 个关节的目标角度 `target(t)` (正弦轨迹)
- 输出: 执行器控制信号 `ctrl(t)`，使 `qpos(t)` 跟踪 `target(t)`
- 验收: 最大跟踪误差 < 5°

### 1.2 当前状态

| 脚本 | 状态 | 说明 |
|------|------|------|
| `step1_verify_model.py` | ✅ 5/5 通过 | 模型加载、运动学、重力仿真正常 |
| `step2_interactive_viewer.py` | ✅ 可用 | viewer.launch() 交互查看 |
| `step3_trajectory_control.py` | ❌ 全部发散 | 7 种方案均失败 |

---

## 2. 已失败的方案及根因

### 方案1-7 回顾

| # | 方案 | 失败现象 | 根因 |
|---|------|---------|------|
| 1 | PD + 固定重力补偿 (零位) | 预热OK，跟踪发散 795°+ | 重力随关节角剧变，固定补偿只在零位有效 |
| 2 | PD + 动态 qfrc_bias | 预热阶段发散 200°+ | qfrc_bias 在偏离位产生正反馈 |
| 3 | `<position>` + gear | QACC instability at DOF 4 | gear² 放大使远端轻量关节刚度过高 |
| 4 | `<position>` 无 gear, kp=500 | arm 掉到地面以下 | kp 太低不足以对抗重力 |
| 5 | motor + 高 KP(=3000) + 固定补偿 | 科氏力爆炸 40000+ Nm | 高增益→过冲→高速→科氏力发散 |
| 6 | motor + 动态重力(target位置) + PD | 立即发散 | 补偿在 target 位与零位差异不够大 |
| 7 | 计算力矩控制 (M·PD + qfrc_bias) | 发散 | qM packed format 提取 M 矩阵有误 |

### 2.2 共同根因

1. **分散 PD 不适用于多体系统**: 惯性耦合使 joint_1 力矩在 joint_4 产生更大加速度
2. **重力梯度极陡**: joint_2 从 0°→15°，重力扭矩 -22.5→-54.2 Nm (2.4x)
3. **MuJoCo motor actuator 是纯力矩驱动**: 无内置位置伺服，需自己实现全部控制律

---

## 3. 官方已验证方案: MuJoCo Menagerie 模式

### 3.1 调研来源

MuJoCo Menagerie (github.com/google-deepmind/mujoco_menagerie) 中多个机械臂模型使用统一控制模式:

- `franka_emika_panda/panda.xml`
- `universal_robots/ur5e/ur5e.xml`
- `kinova_gen3/gen3.xml`

### 3.2 官方执行器配置

```xml
<!-- 核心: position actuator, 无 gear, 高 kp, forcerange 限力矩 -->
<position name="joint_1" joint="joint_1"
          kp="3500"
          kv="350"
          ctrlrange="-2.967 2.967"
          forcerange="-87 87"/>
```

**关键参数解释**:

| 参数 | 典型值 | 作用 |
|------|--------|------|
| `kp` | 2000-4500 | 位置刚度 (Nm/rad)，越高跟踪越紧 |
| `kv` | kp/10 (200-450) | 速度阻尼 (Nms/rad)，抑制振荡 |
| `ctrlrange` | = joint range | ctrl 直接代表目标角度 (rad) |
| `forcerange` | 关节力矩极限 | 防止数值爆炸，保护仿真稳定性 |
| `gear` | **不设 (默认1)** | 避免 gear² 放大导致远端关节不稳 |

### 3.3 position actuator 内置方程

```
torque = kp * (ctrl - qpos) - kv * qvel
```

- `ctrl` = 目标关节角度 (rad)，直接传 `target`
- 无需手动计算 PD，无需手写重力补偿
- `forcerange` 夹紧力矩，防止溢出
- `kp` 足够大 (>2000) 即可对抗重力

### 3.4 为什么这个方案能工作而我们的失败了

| 维度 | 我们的失败方案 | 官方方案 |
|------|--------------|---------|
| 执行器类型 | motor (纯力矩,gear≠1) | position (内置PD,gear=1) |
| PD 实现 | Python 手动 (每步) | C 引擎内置 (高效稳定) |
| gear | 100/50 | 无 (gear=1) |
| kp | 300-500 (太低) | 2000-4500 (足够对抗重力) |
| forcerange | 无 | 有 (防止数值爆炸) |
| 重力补偿 | 手动计算(常有误) | 不需要 (kp 够大直接抵消) |
| 惯量解耦 | 手动(未成功) | 不需要 (kp 够大压倒耦合) |

---

## 4. 实现计划

### 4.1 修改 `gofa_crb15000.xml` — 替换 actuator

**当前** (motor + gear):
```xml
<actuator>
  <motor name="motor_1" joint="joint_1" gear="100"
         ctrllimited="true" ctrlrange="-500 500"/>
  <!-- ... -->
</actuator>
```

**目标** (position, 无 gear, 高 kp, forcerange):
```xml
<actuator>
  <position name="act_joint_1" joint="joint_1"
            kp="3500" kv="350"
            ctrlrange="-4.7124 4.7124"
            forcerange="-330 330"/>
  <position name="act_joint_2" joint="joint_2"
            kp="3500" kv="350"
            ctrlrange="-2.4435 1.5708"
            forcerange="-330 330"/>
  <position name="act_joint_3" joint="joint_3"
            kp="2500" kv="250"
            ctrlrange="-3.9270 1.4835"
            forcerange="-200 200"/>
  <position name="act_joint_4" joint="joint_4"
            kp="1500" kv="150"
            ctrlrange="-3.4907 3.4907"
            forcerange="-50 50"/>
  <position name="act_joint_5" joint="joint_5"
            kp="1000" kv="100"
            ctrlrange="-2.2689 2.2689"
            forcerange="-30 30"/>
  <position name="act_joint_6" joint="joint_6"
            kp="500"  kv="50"
            ctrlrange="-4.7124 4.7124"
            forcerange="-15 15"/>
</actuator>
```

**设计原则**:
- kp 从近端(大关节,大重力)到远端(小关节,小重力)递减
- forcerange 参考关节最大力矩,近端大远端小
- kv = kp/10 (标准经验值)
- ctrlrange = joint range (ctrl 即目标角度 rad)

### 4.2 修改 `step3_trajectory_control.py` — 简化控制

```python
def set_ctrl(model, data, target: np.ndarray):
    """position actuator: ctrl = target angle directly"""
    data.ctrl[:] = target  # 就这么简单
```

- 无需手动 PD 计算
- 无需重力补偿
- 无需 gear 换算
- 无需 M 矩阵

### 4.3 验证流程

1. **预热验证**: 保持零位 200 步，确认 qpos 不偏离
2. **保持零位**: 持续 500 步，确认稳定
3. **小幅度轨迹**: 10% amplitude, no ramp, 确认跟踪
4. **全幅度轨迹**: 完整正弦轨迹, ramp 1.0s, 确认误差 < 5°

---

## 5. 风险评估

| 风险 | 概率 | 缓解 |
|------|------|------|
| kp 选择不合适 | 中 | 从保守值开始,逐步调高; 参考 Menagerie 类似尺寸臂 |
| forcerange 太小 | 低 | 先用大范围, 确认工作后用仿真数据收紧 |
| WSL2 数值差异 | 低 | 先用 --test 模式验证, 确认无误后再 viewer |
| 远端关节振荡 | 中 | kv = kp/10 标准阻尼; 必要时增大 kv |

---

## 6. 参考资料

- [MuJoCo Menagerie - Panda](https://github.com/google-deepmind/mujoco_menagerie/blob/main/franka_emika_panda/panda.xml)
- [MuJoCo Menagerie - UR5e](https://github.com/google-deepmind/mujoco_menagerie/blob/main/universal_robots/ur5e/ur5e.xml)
- [MuJoCo XML Reference - position actuator](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position)
- [MuJoCo Documentation - Actuator models](https://mujoco.readthedocs.io/en/stable/modeling.html#actuator)

---

> **文档作者**: Claude Code (PUA P8 Agent)
> **审核状态**: 待用户确认
> **下一步**: 用户审核方案 → 修改 XML → 修改 Python → 运行验证
