"""
P1-01: 最简单的自由落体
========================

【原理】
Mujoco 核心三步：
  1. mjModel  — 物理世界蓝图（物体、关节、重力）
  2. mjData   — 每个时间步的运行时状态
  3. mj.mj_step() — 推进一个时间步

【练习内容】
球从 height=2 自由下落，加入地面后弹跳。
分析两部分：
  Phase 1（空中）：自由落体，h(t) = 2 - 0.5*g*t^2
  Phase 2（弹跳）：接触地面后的弹跳运动

【关键概念】
- mj.MjModel.from_xml_string() — XML → 模型
- mj.MjData(model)            — 创建运行时数据
- mj.mj_step()                — 仿真一步
- freejoint                   — 自由移动关节（7维：3D位置 + 4元数朝向）
- data.qpos[:3]               — 取位置（freejoint 的前3个值）
- geom sphere/plane            — 几何体类型
- contact                      — 接触模型（弹跳、穿透抑制）
"""

import mujoco as mj
import numpy as np

# ============================================================
# 1. 创建模型（球 + 地面）
# ============================================================
xml = """
<mujoco model="ball_drop">
  <option gravity="0 0 -9.81" timestep="0.002"/>

  <worldbody>
    <body name="floor" pos="0 0 0">
      <geom type="plane" size="2 2 0.1" rgba="0.1 0.1 0.1 1"/>
    </body>
    <body name="ball" pos="0 0 2">
      <freejoint/>
      <geom type="sphere" size="0.2" rgba="0 1 0.6 1"
            solref="3000 1.0" solimp="0.9 0.95 0.001"/>
    </body>
  </worldbody>
</mujoco>
"""
model = mj.MjModel.from_xml_string(xml)
data  = mj.MjData(model)

# ============================================================
# 2. 仿真参数
# ============================================================
dt      = 0.002          # 时间步长 (s)
n_steps = 500            # 总步数
duration = dt * n_steps  # 总时长

# ============================================================
# 3. 前向仿真 + 记录数据
# ============================================================
time_arr   = []
height_arr = []

for _ in range(n_steps):
    mj.mj_step(model, data)
    time_arr.append(data.time)
    height_arr.append(data.qpos[2])  # freejoint z轴高度

time_arr   = np.array(time_arr)
height_arr = np.array(height_arr)

# ============================================================
# 4. 分阶段分析
# ============================================================
g = 9.81
t = time_arr

# 理论解（不考虑地面）
h_theory = 2 - 0.5 * g * t**2

# 找第一次触地（z <= 0.2 + 小阈值），球半径 0.2，中心 z=0.2 时触地
ground_z = 0.2  # 球心触地的临界值
contact_idx = np.where(height_arr <= ground_z)[0]
first_contact = contact_idx[0] if len(contact_idx) > 0 else n_steps

# Phase 1: 空中阶段（触地前）
air_t   = t[:first_contact]
air_h   = height_arr[:first_contact]
air_theory = h_theory[:first_contact]
air_error = np.max(np.abs(air_h - air_theory))

# Phase 2: 触地后弹跳
bounce_t   = t[first_contact:]
bounce_h   = height_arr[first_contact:]

print("=" * 55)
print("P1-01: 自由落体 + 弹跳分析")
print("=" * 55)
print(f"  触地时刻:        t={air_t[-1]:.4f}s (第{first_contact}步)")
print(f"  空中最大误差:    {air_error:.6f} m")
print(f"  ✅ 空中阶段 PASS (<0.01): {air_error < 0.01}")
print(f"  弹跳次数(估计):  ~{np.sum(np.diff(bounce_h) > 0)} 次")
print(f"  弹跳最大高度:    {np.max(bounce_h):.4f} m")
print("=" * 55)

# ============================================================
# 5. 渲染可视化（实时 + 截图保存）
# ============================================================
import glfw
from mujoco import Renderer
import time

if not glfw.init():
    print("GLFW 初始化失败（无显示器环境，跳过渲染）")
else:
    window = glfw.create_window(1200, 900, "P1-01: 自由落体", None, None)
    glfw.make_context_current(window)
    renderer = Renderer(model)

    # 实时渲染循环（边仿真边显示）
    print("\n实时渲染中（关闭窗口或等待 5 秒后保存截图）...")
    data = mj.MjData(model)
    start_time = time.time()
    frame_count = 0

    saved_frame = None
    while not glfw.window_should_close(window) and (time.time() - start_time) < 5:
        mj.mj_step(model, data)
        renderer.update_scene(data)
        renderer.render()
        frame_count += 1
        # 保存第一帧（有物体颜色，不是全黑）
        if saved_frame is None and frame_count == 1:
            saved_frame = renderer.render()
        glfw.poll_events()

    # 保存截图
    print(f"\n已渲染 {frame_count} 帧，保存截图 ...")
    try:
        import PIL.Image
        if saved_frame is not None:
            PIL.Image.fromarray(saved_frame).save("p1_01_ball_drop.png")
        else:
            renderer.update_scene(data)
            PIL.Image.fromarray(renderer.render()).save("p1_01_ball_drop.png")
        print("已保存: p1_01_ball_drop.png")
    except ImportError:
        print("未安装 PIL，跳过图像保存")

    renderer.close()
    glfw.terminate()
    print("程序结束")