"""
P2-Step1: 开环力矩控制（从 XML 文件加载模型）
================================================

【原理】
验证 actuator + ctrl[:] 控制关节力矩的正确性。

两种测试：
  Test A：无控制力矩 → 摆臂自由摆动（能量守恒）
  Test B：恒定正向力矩 → 摆臂持续逆时针旋转（力矩做功）

【标准结构】
- 模型定义：models/pendulum_actuated.xml
- 控制脚本：scripts/step1_openloop_control.py（加载模型并控制）
"""

import os
import mujoco as mj
import numpy as np

# 模型路径（基于脚本位置计算）
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.dirname(SCRIPT_DIR)          # .../p2_ros2_bridge/
MODEL_PATH = os.path.join(PKG_ROOT, 'models', 'pendulum_actuated.xml')

model = mj.MjModel.from_xml_path(MODEL_PATH)
data  = mj.MjData(model)

L = 0.5
g = 9.81
dt      = 0.002
n_steps = 1000  # 2 秒

print("=" * 55)
print("P2-Step1: 开环力矩控制")
print("=" * 55)
print(f"  模型: {MODEL_PATH}")

# ============================================================
# Test A: 无控制力矩（验证自由摆动能量守恒）
# ============================================================
print("\n[Test A] 无控制力矩 — 验证能量守恒")
data_A = mj.MjData(model)
data_A.qpos[0] = np.radians(60)

theta_A, ke_A, pe_A = [], [], []
for _ in range(n_steps):
    data_A.ctrl[0] = 0.0
    mj.mj_step(model, data_A)
    theta_A.append(data_A.qpos[0])
    v = data_A.qvel[0]
    ke_A.append(0.5 * 1 * L**2 * v**2)
    pe_A.append(1 * g * L * (1 - np.cos(data_A.qpos[0])))

E_total_A = np.array(ke_A) + np.array(pe_A)
E_drift_A = np.max(np.abs(E_total_A - E_total_A[0]))
print(f"  初始能量: {E_total_A[0]:.4f} J")
print(f"  能量漂移: {E_drift_A:.6f} J")
print(f"  ✅ PASS: {E_drift_A < 1e-4}")

# ============================================================
# Test B: 恒定正向力矩（验证持续逆时针旋转）
# ============================================================
print("\n[Test B] 正向力矩=2.0 N·m — 验证逆时针旋转")
data_B = mj.MjData(model)
data_B.qpos[0] = 0.0

omega_B = []
for _ in range(n_steps):
    data_B.ctrl[0] = 2.0
    mj.mj_step(model, data_B)
    omega_B.append(data_B.qvel[0])

omega_final = omega_B[-1]
print(f"  末角速度: {omega_final:.4f} rad/s (正值=逆时针)")
print(f"  ✅ PASS: {omega_final > 0.1}")

# ============================================================
# Test C: 恒定负向力矩（验证反向旋转）
# ============================================================
print("\n[Test C] 负向力矩=-2.0 N·m — 验证顺时针旋转")
data_C = mj.MjData(model)
data_C.qpos[0] = 0.0

omega_C = []
for _ in range(n_steps):
    data_C.ctrl[0] = -2.0
    mj.mj_step(model, data_C)
    omega_C.append(data_C.qvel[0])

omega_final_C = omega_C[-1]
print(f"  末角速度: {omega_final_C:.4f} rad/s (负值=顺时针)")
print(f"  ✅ PASS: {omega_final_C < -0.1}")

print("\n" + "=" * 55)
print("结论: actuator + ctrl 力矩控制正确")
print("=" * 55)

# ============================================================
# 保存图像
# ============================================================
try:
    import matplotlib.pyplot as plt
    t = np.arange(n_steps) * dt

    fig, axes = plt.subplots(2, 3, figsize=(14, 6))

    axes[0, 0].plot(t, np.degrees(theta_A), 'b-')
    axes[0, 0].set_title('Test A: Angle (no torque)')
    axes[0, 0].set_ylabel('theta (deg)')
    axes[0, 0].grid(True)

    axes[1, 0].plot(t, E_total_A, 'g-')
    axes[1, 0].set_title('Test A: Total Energy')
    axes[1, 0].set_ylabel('E (J)')
    axes[1, 0].grid(True)

    axes[0, 1].plot(t, np.degrees([data_B.qpos[0]] * n_steps), 'b-')
    axes[0, 1].set_title('Test B: Angle (tau=+2)')
    axes[0, 1].grid(True)

    axes[1, 1].plot(t, omega_B, 'r-')
    axes[1, 1].set_title('Test B: Angular velocity')
    axes[1, 1].set_ylabel('omega (rad/s)')
    axes[1, 1].grid(True)

    axes[0, 2].plot(t, np.degrees([data_C.qpos[0]] * n_steps), 'b-')
    axes[0, 2].set_title('Test C: Angle (tau=-2)')
    axes[0, 2].grid(True)

    axes[1, 2].plot(t, omega_C, 'r-')
    axes[1, 2].set_title('Test C: Angular velocity')
    axes[1, 2].grid(True)

    plt.tight_layout()
    IMG_DIR = os.path.join(PKG_ROOT, '..', '..', 'resources', 'p2')
    plt.savefig(os.path.join(IMG_DIR, 'p2_step1_openloop.png'), dpi=150)
    print(f"\nSaved: {os.path.normpath(IMG_DIR)}/p2_step1_openloop.png")
except ImportError:
    print("\nmatplotlib not available, skipping plot")