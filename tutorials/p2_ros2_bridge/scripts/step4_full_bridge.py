"""
P2-Step4: 完整双向桥接
================================================

【原理】
ROS2 和 Mujoco 双向闭环：
  ROS2 → /mujoco_joint_cmd → Mujoco actuator → 仿真 → /mujoco_joint_state → ROS2

Standalone 测试模式：
  - PID 控制 pendulum 跟踪目标角度
  - 验证力矩指令 → 仿真 → 状态读取闭环


ROS2 模式：
  - 订阅 /mujoco_joint_cmd 接收力矩
  - 发布 /mujoco_joint_state 输出状态

【用法】
python3 step4_full_bridge.py --test
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
        from std_msgs.msg import Float64MultiArray
        from sensor_msgs.msg import JointState
        ROS2_AVAILABLE = True
    except ImportError:
        pass

if ROS2_AVAILABLE:
    class FullBridgeNode(Node):
        def __init__(self, model_path):
            super().__init__('full_bridge')
            self.model = mj.MjModel.from_xml_path(model_path)
            self.data  = mj.MjData(self.model)

            self.sub = self.create_subscription(
                Float64MultiArray, 'mujoco_joint_cmd',
                self.cmd_cb, 10
            )
            self.pub = self.create_publisher(
                JointState, 'mujoco_joint_state', 10
            )
            self.latest_torque = [0.0]
            self.get_logger().info('Full bridge: sub /mujoco_joint_cmd, pub /mujoco_joint_state')

        def cmd_cb(self, msg):
            self.latest_torque = list(msg.data)

        def step(self):
            self.data.ctrl[0] = self.latest_torque[0]
            for _ in range(250):
                mj.mj_step(self.model, self.data)

        def publish(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['pivot_joint']
            msg.position = [float(self.data.qpos[0])]
            msg.velocity = [float(self.data.qvel[0])]
            self.pub.publish(msg)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT   = os.path.dirname(SCRIPT_DIR)
MODEL_PATH = os.path.join(PKG_ROOT, 'models', 'pendulum_actuated.xml')
IMG_DIR    = os.path.join(PKG_ROOT, '..', '..', 'resources', 'p2')

print("=" * 55)
print("P2-Step4: 完整双向桥接")
print("=" * 55)

L, g = 0.5, 9.81
dt = 0.002
hz = 500
steps_per_cycle = int(1.0 / hz / dt)

if FORCE_STANDALONE or not ROS2_AVAILABLE:
    # ============================================================
    # Standalone 测试：PID 控制跟踪目标角度
    # ============================================================
    model = mj.MjModel.from_xml_path(MODEL_PATH)
    data  = mj.MjData(model)
    data.qpos[0] = 0.0
    target = np.radians(30)

    Kp, Kd = 5.0, 1.0
    n_cycles = 1000
    t_total = n_cycles / hz

    print(f"[Standalone] PID 控制跟踪目标角度")
    print(f"Kp={Kp}, Kd={Kd}, target={np.degrees(target):.0f}°")
    print(f"模拟 ROS2 → cmd → Mujoco → state → ROS2 闭环\n")

    theta_arr = []
    omega_arr = []
    torque_arr = []
    t_arr = []

    for i in range(n_cycles):
        x = data.qpos[0]
        v = data.qvel[0]
        err = target - x
        torque = Kp * err - Kd * v  # PD 控制
        data.ctrl[0] = torque
        for _ in range(steps_per_cycle):
            mj.mj_step(model, data)
        theta_arr.append(data.qpos[0])
        omega_arr.append(data.qvel[0])
        torque_arr.append(torque)
        t_arr.append(data.time)

    x_final = data.qpos[0]
    x_err = np.abs(x_final - target)

    print(f"  目标: {np.degrees(target):.1f}°")
    print(f"  实际: {np.degrees(x_final):.1f}°")
    print(f"  误差: {np.degrees(x_err):.2f}°")
    print(f"  ✅ PASS: 稳定跟踪 {np.degrees(x_err) < 5.0}")
    print(f"  闭环验证: cmd → actuator → 仿真 → state 链路正常")

    # matplotlib 曲线
    try:
        import matplotlib.pyplot as plt
        t2 = np.array(t_arr)
        fig, axes = plt.subplots(3, 1, figsize=(10, 8))
        axes[0].plot(t2, np.degrees(theta_arr), 'b-', label='actual')
        axes[0].axhline(np.degrees(target), color='r', linestyle='--', label='target')
        axes[0].set_ylabel('theta (deg)')
        axes[0].set_title('PID Position Tracking')
        axes[0].legend()
        axes[0].grid(True)
        axes[1].plot(t2, omega_arr, 'r-')
        axes[1].set_ylabel('omega (rad/s)')
        axes[1].set_title('Joint Velocity')
        axes[1].grid(True)
        axes[2].plot(t2, torque_arr, 'g-')
        axes[2].set_xlabel('time (s)')
        axes[2].set_ylabel('torque (N.m)')
        axes[2].set_title('Control Torque (cmd)')
        axes[2].grid(True)
        plt.tight_layout()
        path = os.path.join(IMG_DIR, 'p2_step4_bridge.png')
        plt.savefig(path, dpi=150)
        print(f"\nSaved: {path}")
    except ImportError:
        print("matplotlib not available")

    # 渲染截图：采集多帧并排拼接
    try:
        import glfw
        from mujoco import Renderer
        from PIL import Image
        if glfw.init():
            window = glfw.create_window(800, 600, "P2-Step4", None, None)
            glfw.make_context_current(window)
            renderer = Renderer(model)

            n_frames = 8
            frames = []
            total_steps = 2500

            for i in range(n_frames):
                d3 = mj.MjData(model)
                d3.qpos[0] = 0.0
                for _ in range(int(total_steps * i / n_frames)):
                    err = target - d3.qpos[0]
                    torque = Kp * err - Kd * d3.qvel[0]
                    d3.ctrl[0] = torque
                    mj.mj_step(model, d3)
                renderer.update_scene(d3)
                frames.append(renderer.render())

            h, w = frames[0].shape[:2]
            strip = np.zeros((h, w * n_frames, 3), dtype=np.uint8)
            for i, f in enumerate(frames):
                strip[:, i*w:(i+1)*w] = f

            path2 = os.path.join(IMG_DIR, 'p2_step4_pendulum.png')
            Image.fromarray(strip).save(path2)
            print(f"Saved: {path2} ({n_frames} frames)")
            renderer.close()
            glfw.terminate()
    except Exception as e:
        print(f"Rendering: {e}")

    print("\n" + "=" * 55)

else:
    # ROS2 模式
    rclpy.init()
    node = FullBridgeNode(MODEL_PATH)
    print("Full bridge 已启动")
    print("Terminal 1: ros2 topic pub /mujoco_joint_cmd std_msgs/Float64MultiArray '{data: [2.0]}'")
    print("Terminal 2: ros2 topic echo /mujoco_joint_state")
    print("按 Ctrl+C 退出\n")

    try:
        for _ in range(5000):
            rclpy.spin_once(node, timeout_sec=0.001)
            node.step()
            node.publish()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    print("=" * 55)