"""
P2-Step2: ROS2 订阅者驱动 Mujoco
================================================

【原理】
ROS2 节点订阅 /mujoco_joint_cmd，接收力矩指令，驱动 Mujoco。

Standalone 测试模式（无 ROS2 环境）：
  - 发送恒定力矩 2.0 N·m，验证 pendulum 响应

ROS2 模式（有 ROS2 环境）：
  - 等待 /mujoco_joint_cmd Topic
  - 无 Topic 时力矩=0，pendulum 静止

【用法】
# Standalone 测试（推荐先用这个验证物理）
python3 step2_ros2_subscriber.py --test

# ROS2 模式
source /opt/ros/humble/setup.bash
ros2 topic pub /mujoco_joint_cmd std_msgs/Float64MultiArray "{data: [2.0]}" --once
python3 step2_ros2_subscriber.py
"""

import os
import sys
import numpy as np
import mujoco as mj
import time

# 强制 standalone 测试模式（无 ROS2 依赖）
FORCE_STANDALONE = '--test' in sys.argv

# ROS2 导入
ROS2_AVAILABLE = False
if not FORCE_STANDALONE:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
        ROS2_AVAILABLE = True
    except ImportError:
        pass

# ============================================================
# ROS2 节点（仅 ROS2 可用时）
# ============================================================
if ROS2_AVAILABLE:
    class MujocoBridgeNode(Node):
        def __init__(self, model_path):
            super().__init__('mujoco_bridge')
            self.model = mj.MjModel.from_xml_path(model_path)
            self.data  = mj.MjData(self.model)

            self.sub = self.create_subscription(
                Float64MultiArray,
                'mujoco_joint_cmd',
                self.cmd_callback,
                10
            )
            self.latest_torque = [0.0]
            self.get_logger().info('Subscribing to /mujoco_joint_cmd')

        def cmd_callback(self, msg):
            self.latest_torque = list(msg.data)


# ============================================================
# 主程序
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.dirname(SCRIPT_DIR)
MODEL_PATH = os.path.join(PKG_ROOT, 'models', 'pendulum_actuated.xml')
IMG_DIR    = os.path.join(PKG_ROOT, '..', '..', 'resources', 'p2')

print("=" * 55)
print("P2-Step2: ROS2 订阅者驱动 Mujoco")
print("=" * 55)

if FORCE_STANDALONE or not ROS2_AVAILABLE:
    # ============================================================
    # Standalone 测试模式：发送恒定力矩，验证物理响应
    # ============================================================
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data  = mj.MjData(model)

    print(f"[Standalone] 模型: {MODEL_PATH}")
    print("发送恒定力矩 2.0 N·m，验证 Mujoco 响应...\n")

    torque = 2.0
    n_steps = 1000
    dt = 0.002

    # 记录数据
    theta_arr = []
    omega_arr = []
    torque_arr = []

    for _ in range(n_steps):
        data.ctrl[0] = torque
        mj.mj_step(model, data)
        theta_arr.append(data.qpos[0])
        omega_arr.append(data.qvel[0])
        torque_arr.append(data.ctrl[0])

    theta_deg = np.degrees(data.qpos[0])
    omega = data.qvel[0]

    print(f"  末角度:   {theta_deg:.2f}°")
    print(f"  末角速度: {omega:.4f} rad/s")
    print(f"  ✅ PASS: 持续旋转 {abs(omega) > 0.1}")

    # ============================================================
    # 保存曲线图（matplotlib）
    # ============================================================
    try:
        import matplotlib.pyplot as plt
        t = np.arange(n_steps) * dt
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))

        axes[0].plot(t, np.degrees(theta_arr), 'b-', label='theta(t)')
        axes[0].set_ylabel('theta (deg)')
        axes[0].set_title('Joint Angle')
        axes[0].grid(True)

        axes[1].plot(t, omega_arr, 'r-', label='omega(t)')
        axes[1].set_ylabel('omega (rad/s)')
        axes[1].set_title('Joint Angular Velocity')
        axes[1].grid(True)

        axes[2].plot(t, torque_arr, 'g-', label='torque(t)')
        axes[2].set_xlabel('time (s)')
        axes[2].set_ylabel('torque (N·m)')
        axes[2].set_title('Applied Torque')
        axes[2].grid(True)

        plt.tight_layout()
        out_path = os.path.join(IMG_DIR, 'p2_step2_standalone.png')
        plt.savefig(out_path, dpi=150)
        print(f"\nSaved: {out_path}")
    except ImportError:
        print("\nmatplotlib not available, skipping plot")

    # ============================================================
    # 实时渲染 + 保存截图
    # ============================================================
    try:
        import glfw
        from mujoco import Renderer

        if glfw.init():
            window = glfw.create_window(1200, 900, "P2-Step2: Standalone", None, None)
            glfw.make_context_current(window)
            renderer = Renderer(model)

            data = mj.MjData(model)
            start = time.time()
            saved_frame = None
            fc = 0

            while not glfw.window_should_close(window) and (time.time() - start) < 5:
                data.ctrl[0] = torque
                mj.mj_step(model, data)
                renderer.update_scene(data)
                renderer.render()
                fc += 1
                if saved_frame is None and fc >= 10:
                    saved_frame = renderer.render()
                glfw.poll_events()

            if saved_frame is not None:
                from PIL import Image
                img_path = os.path.join(IMG_DIR, 'p2_step2_pendulum.png')
                Image.fromarray(saved_frame).save(img_path)
                print(f"Saved: {img_path}")

            renderer.close()
            glfw.terminate()
    except Exception as e:
        print(f"\nRendering skipped: {e}")

    print("\n" + "=" * 55)

else:
    # ============================================================
    # ROS2 模式
    # ============================================================
    rclpy.init()
    node = MujocoBridgeNode(MODEL_PATH)

    print("ROS2 节点已启动，等待 /mujoco_joint_cmd ...")
    print("Terminal 1: ros2 topic pub /mujoco_joint_cmd "
          "std_msgs/Float64MultiArray '{data: [2.0]}' --once")
    print("按 Ctrl+C 退出\n")

    dt = 0.002
    hz = 500
    steps_per_cycle = int(1.0 / hz / dt)

    try:
        for _ in range(5000):
            rclpy.spin_once(node, timeout_sec=0.001)
            torque = node.latest_torque[0] if node.latest_torque else 0.0
            node.data.ctrl[0] = torque
            for _ in range(steps_per_cycle):
                mj.mj_step(node.model, node.data)
            if _ % 1000 == 0:
                theta_deg = np.degrees(node.data.qpos[0])
                omega = node.data.qvel[0]
                print(f"  t={node.data.time:.2f}s  theta={theta_deg:.1f}°  omega={omega:.2f}")
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    print("\nROS2 节点已关闭")
    print("=" * 55)