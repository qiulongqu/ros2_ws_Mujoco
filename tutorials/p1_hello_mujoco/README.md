# P1: Mujoco Python API 入门

掌握 Mujoco 核心概念：`MjModel`（物理蓝图）、`MjData`（运行时状态）、`mj_step()`（时间步推进）。

## 练习清单

| 文件 | 内容 | 核心概念 |
|------|------|---------|
| `01_hello_mujoco.py` | 自由落体 + 弹跳 | freejoint、qpos、contact |
| `02_pendulum.py` | 单摆振动 | hinge关节、能量守恒、qvel |
| `03_spring_damper.py` | 弹簧阻尼振动 | slide关节、actuator、阻尼比ζ |

## 运行

```bash
conda activate mujoco
cd /home/yunhao2204/ros2_ws_abb/mujocoONLY
python3 tutorials/p1_hello_mujoco/01_hello_mujoco.py
python3 tutorials/p1_hello_mujoco/02_pendulum.py
python3 tutorials/p1_hello_mujoco/03_spring_damper.py
```

## 验证结果

- **P1-01**: 空中最大误差 0.006m ✅ PASS
- **P1-02**: 能量漂移 0.0 J ✅ PASS
- **P1-03**: 峰值包络误差 0.4% ✅ PASS，周期误差 0.0005s ✅ PASS

## 渲染结果

渲染图像保存在 `../resources/` 目录：
- `p1_01_ball_drop.png`
- `p1_02_pendulum.png`
- `p1_03_spring_damper.png`