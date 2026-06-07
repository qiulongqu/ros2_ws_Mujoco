"""
P1-03: 弹簧阻尼振动
========================

【原理】
弹簧阻尼系统：m·x'' + c·x' + k·x = 0
参数：m=1kg, k=100 N/m, c=2 Ns/m（ζ=0.1，欠阻尼）

Mujoco 使用隐式积分器（implicit），具有数值阻尼，会额外耗散能量。
这是数值稳定积分器的固有特性，不是bug。

【验证策略】
  1. 峰值包络衰减速率  — 验证 ζ=0.1 阻尼比正确
  2. 振动周期         — 验证 ω_d = ω₀√(1-ζ²) 正确
  （能量守恒不适用：隐式积分 + 物理阻尼 → 能量自然耗散）

【关键概念】
- integrator="implicit"  — 隐式积分器（数值稳定，适合弹簧系统）
- actuator + ctrl        — 电机施加弹簧阻尼力
- 欠阻尼：ζ < 1，振动衰减包络 x(t) = x₀·e^(-ζω₀t)
"""

import mujoco as mj
import numpy as np
import glfw
from mujoco import Renderer

# ============================================================
# 1. 参数
# ============================================================
m   = 1.0
k   = 100.0
c   = 2.0
x0  = 0.5

xml = """
<mujoco model="spring_damper">
  <option gravity="0 0 0" integrator="implicit"/>

  <worldbody>
    <body name="wall" pos="-1 0 0">
      <geom type="box" size="0.5 0.5 0.5" rgba="0.1 0.1 0.1 1"/>
    </body>
    <body name="mass" pos="0 0 0">
      <joint name="slider" type="slide" axis="1 0 0"/>
      <geom type="box" size="0.1 0.1 0.1" mass="1" rgba="1 0.4 0 1"/>
    </body>
  </worldbody>

  <actuator>
    <motor joint="slider" ctrlrange="-200 200"/>
  </actuator>
</mujoco>
"""
model = mj.MjModel.from_xml_string(xml)
data  = mj.MjData(model)
data.qpos[0] = x0

dt      = 0.002
n_steps = 1000

omega0  = np.sqrt(k / m)
zeta    = c / (2 * np.sqrt(k * m))
omega_d = omega0 * np.sqrt(1 - zeta**2)

print("=" * 55)
print("P1-03: 弹簧阻尼振动")
print("=" * 55)
print(f"  质量 m:          {m} kg")
print(f"  刚度 k:          {k} N/m")
print(f"  阻尼 c:          {c} Ns/m")
print(f"  阻尼比 ζ:        {zeta:.4f} (欠阻尼)")
print(f"  有阻尼频率 ω_d:  {omega_d:.4f} rad/s")
print(f"  理论周期 T:       {2*np.pi/omega_d:.4f} s")

# ============================================================
# 2. 仿真
# ============================================================
x_arr = []
for _ in range(n_steps):
    x  = data.qpos[0]
    v  = data.qvel[0]
    data.ctrl[0] = -k * x - c * v
    mj.mj_step(model, data)
    x_arr.append(x)

x_arr = np.array(x_arr)
t     = np.arange(n_steps) * dt

# ============================================================
# 3. 峰值包络验证（核心）
# ============================================================
envelope = x0 * np.exp(-zeta * omega0 * t)

# 检测峰值（局部最大值）
peak_idx = np.where(np.diff(np.sign(np.diff(x_arr))) < 0)[0] + 1
peak_values  = np.abs(x_arr[peak_idx])
peak_times   = t[peak_idx]
peak_envelope_theory = x0 * np.exp(-zeta * omega0 * peak_times)

relative_error = np.max(np.abs(peak_values - peak_envelope_theory) / peak_envelope_theory)

print()
print(f"  【峰值包络验证 - 阻尼比 ζ】")
print(f"  检测峰值数:      {len(peak_idx)} 个")
for i, (pv, pe) in enumerate(zip(peak_values, peak_envelope_theory)):
    print(f"    峰值{i+1}: 实测={pv:.5f} 理论={pe:.5f} 误差={abs(pv-pe)/pe*100:.1f}%")
print(f"  最大相对误差:    {relative_error:.4f}")
print(f"  ✅ PASS (<0.15): {relative_error < 0.15}")

# ============================================================
# 4. 振动周期验证
# ============================================================
if len(peak_idx) >= 2:
    T实测 = np.mean(np.diff(peak_times))
    print()
    print(f"  【振动周期验证】")
    print(f"  理论周期:        {2*np.pi/omega_d:.4f} s")
    print(f"  实测周期:        {T实测:.4f} s")
    print(f"  周期误差:        {abs(T实测 - 2*np.pi/omega_d):.4f} s")
    print(f"  ✅ PASS (<0.1s): {abs(T实测 - 2*np.pi/omega_d) < 0.1}")

# ============================================================
# 5. 过阻尼/欠阻尼/临界阻尼特性验证
# ============================================================
print()
print(f"  【系统特性】")
print(f"  系统类型:        欠阻尼（ζ<1），有振荡衰减")
print(f"  是否过阻尼:      {zeta > 1}  |  是否欠阻尼: {zeta < 1}")
print(f"  临界阻尼系数:    {2*np.sqrt(k*m):.2f} Ns/m")
print("=" * 55)

# ============================================================
# 6. 渲染可视化（实时 + 截图保存）
# ============================================================
import glfw
from mujoco import Renderer
import time

if not glfw.init():
    print("GLFW 初始化失败，跳过渲染")
else:
    window = glfw.create_window(1200, 900, "P1-03: 弹簧阻尼", None, None)
    glfw.make_context_current(window)
    renderer = Renderer(model)

    print("\n实时渲染中（关闭窗口或等待 5 秒后保存截图）...")
    data = mj.MjData(model)
    data.qpos[0] = x0
    start_time = time.time()
    frame_count = 0

    saved_frame = None
    while not glfw.window_should_close(window) and (time.time() - start_time) < 5:
        x = data.qpos[0]
        v = data.qvel[0]
        data.ctrl[0] = -k * x - c * v
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
            PIL.Image.fromarray(saved_frame).save("p1_03_spring_damper.png")
        else:
            PIL.Image.fromarray(renderer.render()).save("p1_03_spring_damper.png")
        print(f"已保存: p1_03_spring_damper.png ({frame_count} 帧)")
    except ImportError:
        print("未安装 PIL，跳过图像保存")

    renderer.close()
    glfw.terminate()
    print("程序结束")