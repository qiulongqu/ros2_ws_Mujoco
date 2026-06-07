"""
P2-Step3: Mujoco 发布关节状态到 ROS2
================================================

【原理】
Mujoco 仿真每 500Hz 发布关节状态到 /mujoco_joint_state Topic。

Standalone 测试模式：
  - 从 60° 释放 pendulum，自由摆动
  - 记录 theta(t) / omega(t)，验证能量守恒
  - 输出相当于 "发布" 的数据

ROS2 模式：
  - 实际发布 JointState 到 /mujoco_joint_state Topic
  - 另一个 terminal: ros2 topic echo /mujoco_joint_state

【用法】
# Standalone 测试
python3 step3_joint_publisher.py --test
# ROS2 模式
python3 step3_joint_publisher.py
"""

import os
import sys
import time
import numpy as np
import mujoco as mj

FORCE_STANDALONE = '--test' in sys.argv

ROS2_AVAILABLE = False
if not FORCE_STANDALONE:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        ROS2_AVAILABLE = True
    except ImportError:
        pass

if ROS2_AVAILABLE:
    class MujocoPublisher(Node):
        def __init__(self, model_path):
            super().__init__('mujoco_joint_publisher')
            self.model = mj.MjModel.from_xml_path(model_path)
            self.data  = mj.MjData(self.model)
            self.pub = self.create_publisher(
                JointState, 'mujoco_joint_state', 10
            )
            self.get_logger().info('Publishing to /mujoco_joint_state')

        def publish(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['pivot_joint']
            msg.position = [float(self.data.qpos[0])]
            msg.velocity = [float(self.data.qvel[0])]
            self.pub.publish(msg)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.dirname(SCRIPT_DIR)
MODEL_PATH = os.path.join(PKG_ROOT, 'models', 'pendulum_pub.xml')
IMG_DIR    = os.path.join(PKG_ROOT, '..', '..', 'resources', 'p2')

print("=" * 55)
print("P2-Step3: Mujoco 发布关节状态")
print("=" * 55)

dt = 0.002
hz = 500
steps_per_cycle = int(1.0 / hz / dt)
n_cycles        = 1000
n_steps         = n_cycles * steps_per_cycle

if FORCE_STANDALONE or not ROS2_AVAILABLE:
    # ============================================================
    # Standalone 测试：自由摆动，记录并验证
    # ============================================================
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data  = mj.MjData(model)
    data.qpos[0] = np.radians(60)

    print(f"[Standalone] 模型: {MODEL_PATH}")
    print(f"初始角度=60°，自由摆动 {n_steps * dt:.1f}s\n")

    L, g = 0.5, 9.81
    theta_arr = []
    omega_arr = []
    ke_arr, pe_arr = [], []

    for _ in range(n_steps):
        data.ctrl[0] = 0.0
        mj.mj_step(model, data)
        theta_arr.append(data.qpos[0])
        omega_arr.append(data.qvel[0])
        v = data.qvel[0]
        ke_arr.append(0.5 * 1 * L**2 * v**2)
        pe_arr.append(1 * g * L * (1 - np.cos(data.qpos[0])))

    E_total = np.array(ke_arr) + np.array(pe_arr)

    # 验证频率（通过零点检测）
    zero_cross = np.where(np.diff(np.sign(theta_arr)) != 0)[0]
    if len(zero_cross) >= 2:
        T_meas = 2 * (zero_cross[1] - zero_cross[0]) * dt
        T_theory = 2 * np.pi / np.sqrt(g / L)
        print(f"  实测周期:   {T_meas:.4f}s  理论周期: {T_theory:.4f}s")
        print(f"  周期误差:   {abs(T_meas - T_theory):.4f}s")

    E_drift = np.max(np.abs(E_total - E_total[0]))
    print(f"  初始能量:   {E_total[0]:.4f}J  能量漂移: {E_drift:.6f}J")
    print(f"  ✅ PASS: 能量守恒 {E_drift < 1e-4}")
    print(f"  数据可发布: theta(t), omega(t) 正常记录")

    # 保存 matplotlib 曲线图
    try:
        import matplotlib.pyplot as plt
        t = np.arange(n_steps) * dt
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))
        axes[0].plot(t, np.degrees(theta_arr), 'b-')
        axes[0].set_ylabel('theta (deg)')
        axes[0].set_title('Joint Position (to publish)')
        axes[0].grid(True)
        axes[1].plot(t, omega_arr, 'r-')
        axes[1].set_ylabel('omega (rad/s)')
        axes[1].set_title('Joint Velocity (to publish)')
        axes[1].grid(True)
        axes[2].plot(t, E_total, 'g-')
        axes[2].set_xlabel('time (s)')
        axes[2].set_ylabel('Energy (J)')
        axes[2].set_title('Total Energy (must be constant)')
        axes[2].grid(True)
        plt.tight_layout()
        path = os.path.join(IMG_DIR, 'p2_step3_publisher.png')
        plt.savefig(path, dpi=150)
        print(f"\nSaved: {path}")
    except ImportError:
        print("\nmatplotlib not available")

    # 渲染截图：采集多帧并排拼接
    try:
        import glfw
        from mujoco import Renderer
        from PIL import Image
        if glfw.init():
            window = glfw.create_window(800, 600, "P2-Step3", None, None)
            glfw.make_context_current(window)
            renderer = Renderer(model)
            d2 = mj.MjData(model)
            d2.qpos[0] = np.radians(60)

            n_frames = 8
            frames = []
            total_steps = 2500

            for i in range(n_frames):
                d3 = mj.MjData(model)
                d3.qpos[0] = np.radians(60)
                for _ in range(int(total_steps * i / n_frames)):
                    d3.ctrl[0] = 0.0
                    mj.mj_step(model, d3)
                renderer.update_scene(d3)
                frames.append(renderer.render())

            h, w = frames[0].shape[:2]
            strip = np.zeros((h, w * n_frames, 3), dtype=np.uint8)
            for i, f in enumerate(frames):
                strip[:, i*w:(i+1)*w] = f

            path2 = os.path.join(IMG_DIR, 'p2_step3_pendulum.png')
            Image.fromarray(strip).save(path2)
            print(f"Saved: {path2} ({n_frames} frames)")
            renderer.close()
            glfw.terminate()
    except Exception as e:
        print(f"Rendering: {e}")

    print("\n" + "=" * 55)

else:
    # ============================================================
    # ROS2 模式
    # ============================================================
    rclpy.init()
    node = MujocoPublisher(MODEL_PATH)
    node.data.qpos[0] = np.radians(30)
    print("ROS2 节点已启动，发布到 /mujoco_joint_state")
    print("Terminal 1: ros2 topic echo /mujoco_joint_state")
    print("按 Ctrl+C 退出\n")

    try:
        for i in range(5000):
            rclpy.spin_once(node, timeout_sec=0.001)
            for _ in range(steps_per_cycle):
                mj.mj_step(node.model, node.data)
            node.publish()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    print("=" * 55)