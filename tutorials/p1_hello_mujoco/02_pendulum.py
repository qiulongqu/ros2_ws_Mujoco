"""
P1-02: 单摆运动
========================

【原理】
单摆的物理方程：θ''(t) = -(g/L)·sin(θ)

两种验证方式：
  1. 小角度近似 → θ(t) = θ₀·cos(√(g/L)·t)，适用 θ₀ < 15°
  2. 数值积分 + 能量守恒验证，适用任意角度

本练习用 60° 大角度，观察线性近似失效的物理含义，同时验证能量守恒。

【关键概念】
- <joint type="hinge">  — 铰接关节（1D旋转）
- data.qpos[0]          — 关节角度 θ（弧度）
- data.qvel[0]          — 关节角速度 ω（弧度/秒）
- 能量守恒：KE + PE = const
"""

import mujoco as mj
import numpy as np
import glfw
from mujoco import Renderer

# ============================================================
# 1. 创建模型（单摆）
# ============================================================
L = 0.5
g = 9.81
theta0 = np.radians(60)  # 60°（大角度，观察线性近似失效）

xml = f"""
<mujoco model="pendulum">
  <option gravity="0 0 -{g}"/>

  <worldbody>
    <body name="pivot" pos="0 0 0">
      <joint name="pivot_joint" type="hinge" axis="0 0 1"/>
      <body name="arm" pos="0 0 -{L}">
        <geom type="sphere" size="0.08" mass="1" rgba="0 1 0.8 1"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.001 0.001 0.001"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
model = mj.MjModel.from_xml_string(xml)
data  = mj.MjData(model)
data.qpos[0] = theta0

dt      = 0.002
n_steps = 2000

# ============================================================
# 2. 仿真 + 记录
# ============================================================
theta_arr = []
omega_arr = []
ke_arr    = []
pe_arr    = []

for _ in range(n_steps):
    mj.mj_step(model, data)
    theta_arr.append(data.qpos[0])
    omega_arr.append(data.qvel[0])
    ke = 0.5 * 1 * L**2 * data.qvel[0]**2
    pe = 1 * g * L * (1 - np.cos(data.qpos[0]))
    ke_arr.append(ke)
    pe_arr.append(pe)

theta_arr = np.array(theta_arr)
omega_arr = np.array(omega_arr)
ke_arr    = np.array(ke_arr)
pe_arr    = np.array(pe_arr)
t         = np.arange(n_steps) * dt

# ============================================================
# 3. 验证1: 能量守恒（主要验证）
# ============================================================
total_energy = ke_arr + pe_arr
energy_drift = np.max(np.abs(total_energy - total_energy[0]))

omega_natural = np.sqrt(g / L)

print("=" * 55)
print("P1-02: 单摆运动验证")
print("=" * 55)
print(f"  摆臂长度:       L={L} m")
print(f"  初始角度:       {np.degrees(theta0):.1f}°")
print(f"  理论固有频率:   ω=√(g/L)={omega_natural:.4f} rad/s")
print(f"  理论周期:       T=2π/ω={2*np.pi/omega_natural:.4f} s")
print()
print(f"  【能量守恒验证】")
print(f"  初始总能量:      {total_energy[0]:.4f} J")
print(f"  最大能量漂移:    {energy_drift:.6f} J")
print(f"  ✅ PASS (<1e-4): {energy_drift < 1e-4}")

# ============================================================
# 4. 验证2: 周期验证
# ============================================================
print()
print(f"  【周期验证】")
zero_crossings = np.where(np.diff(np.sign(theta_arr)) != 0)[0]
if len(zero_crossings) >= 2:
    T实测 = t[zero_crossings[1]] - t[zero_crossings[0]]
    T理论 = 2 * np.pi / omega_natural
    print(f"  实测半周期:      {T实测:.4f} s")
    print(f"  理论半周期:      {T理论:.4f} s")
    print(f"  周期误差:        {abs(T实测 - T理论):.4f} s")
    print(f"  ✅ PASS (<0.1s): {abs(T实测 - T理论) < 0.1}")
else:
    print("  零点检测不足，跳过周期验证")

# ============================================================
# 5. 渲染可视化（实时 + 截图保存）
# ============================================================
import glfw
from mujoco import Renderer
import time

if not glfw.init():
    print("GLFW 初始化失败，跳过渲染")
else:
    window = glfw.create_window(1200, 900, "P1-02: 单摆", None, None)
    glfw.make_context_current(window)
    renderer = Renderer(model)

    print("\n实时渲染中（关闭窗口或等待 5 秒后保存截图）...")
    data = mj.MjData(model)
    data.qpos[0] = theta0
    start_time = time.time()
    frame_count = 0

    saved_frame = None
    while not glfw.window_should_close(window) and (time.time() - start_time) < 5:
        mj.mj_step(model, data)
        renderer.update_scene(data)
        renderer.render()
        frame_count += 1
        if saved_frame is None and frame_count == 1:
            saved_frame = renderer.render()
        glfw.poll_events()

    try:
        import PIL.Image
        if saved_frame is not None:
            PIL.Image.fromarray(saved_frame).save("p1_02_pendulum.png")
        else:
            PIL.Image.fromarray(renderer.render()).save("p1_02_pendulum.png")
        print(f"已保存: p1_02_pendulum.png ({frame_count} 帧)")
    except ImportError:
        print("未安装 PIL，跳过图像保存")

    renderer.close()
    glfw.terminate()
    print("程序结束")